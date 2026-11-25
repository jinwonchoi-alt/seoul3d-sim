# -*- coding: utf-8 -*-
"""서울 실시간 도시데이터 121곳의 지금 인구 + 12시간 예측 인구를 한 번에 받아 ppl_all.json 으로 저장 (시뮬레이션 수요에 사용)
   예측: areas[코드]["f"] = {"YYYY-MM-DD HH:00": 예상 인구(가운데 값), ...},  fc = 예측 시각 목록"""
import os, sys, json, argparse, datetime
from concurrent.futures import ThreadPoolExecutor
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim_auto as A


def one(api, cd):
    for _ in range(2):
        try:
            a = A.get_json('%s/api/seoul-ppltn?area=%s' % (api, cd))['SeoulRtd.citydata_ppltn'][0]
            f = {}
            for x in a.get('FCST_PPLTN') or []:
                try:
                    f[x['FCST_TIME']] = [(float(x['FCST_PPLTN_MIN']) + float(x['FCST_PPLTN_MAX'])) / 2, x.get('FCST_CONGEST_LVL')]
                except Exception:
                    pass
            return cd, {'nm': a['AREA_NM'], 'p': (float(a['AREA_PPLTN_MIN']) + float(a['AREA_PPLTN_MAX'])) / 2,
                        'lvl': a['AREA_CONGEST_LVL'], 't': a.get('PPLTN_TIME'), 'f': f}
        except Exception:
            pass
    return cd, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--api', default=A.API_DEFAULT)
    ap.add_argument('--hotspots', default='hotspots.json')
    ap.add_argument('--out', default='ppl_all.json')
    ap.add_argument('--targets', default='', help='예측 시뮬레이션 시각 고르기 (예측 목록 순번, 예: 1,2,12)')
    ap.add_argument('--targets-out', default='fc_targets.txt')
    a = ap.parse_args()
    codes = [f['properties']['AREA_CD'] for f in json.load(open(a.hotspots, encoding='utf-8'))['features']]
    with ThreadPoolExecutor(6) as ex:
        got = dict(ex.map(lambda c: one(a.api, c), codes))
    areas = {k: v for k, v in got.items() if v}
    t = max([v['t'] for v in areas.values() if v.get('t')] or [(datetime.datetime.utcnow() + datetime.timedelta(hours=9)).strftime('%Y-%m-%d %H:%M')])
    fc = sorted({k for v in areas.values() for k in (v.get('f') or {})})
    json.dump({'t': t, 'fc': fc, 'areas': areas}, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False)
    print('인구 %d/%d곳 (%s 기준, 예측 %s ~ %s) → %s' % (len(areas), len(codes), t, fc[0] if fc else '-', fc[-1] if fc else '-', a.out))
    if a.targets:
        # 예측 시뮬레이션을 돌릴 시각: 예측 목록에서 몇 번째(1부터)를 고를지. 예) 1,2,12 → 가까운 2개는 매시간 새로, 12시간 뒤는 처음 만듦
        pick = []
        for k in a.targets.split(','):
            k = int(k)
            if 1 <= k <= len(fc) and fc[k - 1] not in pick:
                pick.append(fc[k - 1])
        open(a.targets_out, 'w').write('\n'.join(pick) + ('\n' if pick else ''))
        print('예측 시뮬레이션 시각:', ', '.join(pick) or '없음')


if __name__ == '__main__':
    main()
