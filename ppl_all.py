# -*- coding: utf-8 -*-
"""서울 실시간 도시데이터 121곳의 지금 인구를 한 번에 받아 ppl_all.json 으로 저장 (시뮬레이션 수요에 사용)"""
import os, sys, json, argparse, datetime
from concurrent.futures import ThreadPoolExecutor
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim_auto as A


def one(api, cd):
    for _ in range(2):
        try:
            a = A.get_json('%s/api/seoul-ppltn?area=%s' % (api, cd))['SeoulRtd.citydata_ppltn'][0]
            return cd, {'nm': a['AREA_NM'], 'p': (float(a['AREA_PPLTN_MIN']) + float(a['AREA_PPLTN_MAX'])) / 2,
                        'lvl': a['AREA_CONGEST_LVL'], 't': a.get('PPLTN_TIME')}
        except Exception:
            pass
    return cd, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--api', default=A.API_DEFAULT)
    ap.add_argument('--hotspots', default='hotspots.json')
    ap.add_argument('--out', default='ppl_all.json')
    a = ap.parse_args()
    codes = [f['properties']['AREA_CD'] for f in json.load(open(a.hotspots, encoding='utf-8'))['features']]
    with ThreadPoolExecutor(6) as ex:
        got = dict(ex.map(lambda c: one(a.api, c), codes))
    areas = {k: v for k, v in got.items() if v}
    t = max([v['t'] for v in areas.values() if v.get('t')] or [(datetime.datetime.utcnow() + datetime.timedelta(hours=9)).strftime('%Y-%m-%d %H:%M')])
    json.dump({'t': t, 'areas': areas}, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False)
    print('인구 %d/%d곳 (%s 기준) → %s' % (len(areas), len(codes), t, a.out))


if __name__ == '__main__':
    main()
