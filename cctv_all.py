# -*- coding: utf-8 -*-
"""서울 25개 구 방범 CCTV 위치(행정안전부 CCTV 정보)를 구별 파일로 저장 → GitHub Pages 에서 지도가 바로 읽음
원본 API는 100곳씩만 주고 동시에 여러 번 부르면 멈춰서, 여기서 천천히(2개씩) 받아 둠. 일주일에 한 번만 실행(워크플로가 캐시로 관리)
사용: python cctv_all.py --out cctv"""
import os, sys, json, time, argparse, urllib.request
from concurrent.futures import ThreadPoolExecutor
API = 'https://seoul3d-api.jinwonchoi.workers.dev'
GU_OPN = {'11110':'3000000','11140':'3010000','11170':'3020000','11200':'3030000','11215':'3040000','11230':'3050000','11260':'3060000','11290':'3070000','11305':'3080000','11320':'3090000','11350':'3100000','11380':'3110000','11410':'3120000','11440':'3130000','11470':'3140000','11500':'3150000','11530':'3160000','11545':'3170000','11560':'3180000','11590':'3190000','11620':'3200000','11650':'3210000','11680':'3220000','11710':'3230000','11740':'3240000'}
F = ['WGS84_LAT','WGS84_LOT','INSTL_PRPS_SE_NM','CAM_CNTOM','LCTN_ROAD_NM_ADDR','LCTN_LOTNO_ADDR','CAM_PIXEL_CNT','SHT_ANGLE_INFO','INSTL_YM','KPNG_DAY_CNT','MNG_INST_NM','MNG_INST_TELNO','DAT_CRTR_YMD']


def page(opn, pg):
    for k in range(4):
        try:
            req = urllib.request.Request('%s/api/cctv?code=%s&rows=100&page=%d' % (API, opn, pg), headers={'User-Agent': 'seoul3d-cctv'})
            with urllib.request.urlopen(req, timeout=25) as r:
                b = json.loads(r.read().decode('utf-8'))['response']['body']
            it = (b.get('items') or {}).get('item') or []
            if not isinstance(it, list):
                it = [it]
            return int(b.get('totalCount') or 0), it
        except Exception:
            time.sleep(2 + 3 * k)
    return None


def one(code, out):
    opn = GU_OPN[code]
    first = page(opn, 1)
    if not first:
        print(code, '실패(첫 쪽)'); return False
    total, rows = first[0], list(first[1])
    pages = (total + 99) // 100
    with ThreadPoolExecutor(2) as ex:
        got = list(ex.map(lambda p: page(opn, p), range(2, pages + 1)))
    miss = sum(1 for g in got if g is None)
    for g in got:
        if g:
            rows += g[1]
    data = {'code': opn, 'gu': code, 'total': total, 'fields': F, 'rows': [[x.get(k) if x.get(k) is not None else '' for k in F] for x in rows],
            'gen': time.strftime('%Y-%m-%d', time.gmtime(time.time() + 9 * 3600))}
    if miss == 0:
        json.dump(data, open(os.path.join(out, code + '.json'), 'w', encoding='utf-8'), ensure_ascii=False, separators=(',', ':'))
    print(code, '%d/%d곳' % (len(rows), total), '빠진 쪽 %d' % miss if miss else '완료')
    return miss == 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='cctv')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    ok = 0
    for code in GU_OPN:
        if os.path.exists(os.path.join(a.out, code + '.json')):   # 이번 주에 이미 받은 구는 건너뜀
            ok += 1; continue
        ok += one(code, a.out)
    print('구 %d/25 완료' % ok)
    if ok == 25:
        open(os.path.join(a.out, 'done'), 'w').write(time.strftime('%Y-%m-%d'))


if __name__ == '__main__':
    main()
