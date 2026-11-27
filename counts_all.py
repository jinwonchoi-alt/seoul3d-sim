# -*- coding: utf-8 -*-
"""서울 25개 구의 교통량 검지기 지점 + 최근 1시간 교통량을 한 번에 받아 counts_all.json 으로 저장
   --at "YYYY-MM-DD HH:00" 이면 예측용: 그 시각과 같은 요일·같은 시간의 지난주(없으면 2주 전, 전날) 실측 교통량"""
import os, sys, json, argparse, datetime
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim_auto as A


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--api', default=A.API_DEFAULT)
    ap.add_argument('--gu-json', default='seoul_gu.json')
    ap.add_argument('--out', default='counts_all.json')
    ap.add_argument('--at', default=None, help='예측 시각 "YYYY-MM-DD HH:00" (지난주 같은 요일·시각 교통량 사용)')
    a = ap.parse_args()
    from shapely.geometry import shape, Point
    from pyproj import Transformer
    polys = {}
    for f in json.load(open(a.gu_json, encoding='utf-8'))['features']:
        c = str(f['properties'].get('sig'))
        g = shape(f['geometry']).buffer(0.002)
        polys[c] = polys[c].union(g) if c in polys else g
    tr = Transformer.from_crs('EPSG:5181', 'EPSG:4326', always_xy=True)
    spots = []
    for r in A.get_json(a.api + '/api/vol?list=1')['rows']:
        lon, lat = tr.transform(r['x'], r['y'])
        spots.append({'id': r['id'], 'nm': r['nm'], 'lon': round(lon, 6), 'lat': round(lat, 6)})
    now = datetime.datetime.utcnow() + datetime.timedelta(hours=9)
    vol, ymd, hh = {}, '', ''
    if a.at:
        at = datetime.datetime.strptime(a.at[:13], '%Y-%m-%d %H')
        backs = [at - datetime.timedelta(days=d) for d in (7, 14, 1)]
    else:
        backs = [now - datetime.timedelta(hours=b) for b in (1, 2, 3, 24)]
    # 시점마다 자료가 있는 지점 수를 세고, 가장 많은 시점의 80% 이상 채워진 시점 중 가장 최근 것을 씀
    # (1시간 전 자료가 일부 지점만 올라온 상태에서 그 시점을 고르면 나머지 지점이 빠지기 때문)
    cands = []
    for t in backs:
        y, h = t.strftime('%Y%m%d'), t.strftime('%H')
        got, fails = {}, 0
        for i in range(0, len(spots), 40):
            ids = ','.join(s['id'] for s in spots[i:i + 40])
            try:
                rows = A.get_json('%s/api/vol?spots=%s&ymd=%s&hh=%s' % (a.api, ids, y, h))['rows']
            except Exception as e:
                fails += 1
                print('  교통량 조회 실패 %s %s시 (%d번째 묶음): %s' % (y, h, i // 40 + 1, e))
                continue
            for r in rows:
                if not r.get('error'):
                    got[r['id']] = r
        n = sum(1 for r in got.values() if r['in'] + r['out'] > 0)
        print('  %s %s시: 자료 있는 지점 %d곳%s' % (y, h, n, (' (조회 실패 %d번)' % fails) if fails else ''))
        cands.append((n, got, y, h))
    best = max([c[0] for c in cands] or [0])
    for n, got, y, h in cands:
        if best and n >= best * 0.8:
            vol, ymd, hh = got, y, h
            break
    out = {}
    for code, poly in polys.items():
        ss = []
        for s in spots:
            v = vol.get(s['id'])
            if v and v['in'] + v['out'] > 0 and poly.contains(Point(s['lon'], s['lat'])):
                ss.append(dict(s, **{'in': v['in'], 'out': v['out']}))
        out[code] = {'ymd': ymd, 'hh': hh, 'spots': ss}
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False)
    print('교통량 %s %s시 기준 / 지점 %d곳 / 구별:' % (ymd, hh, len(vol)),
          ' '.join('%s:%d' % (c, len(v['spots'])) for c, v in sorted(out.items())))


if __name__ == '__main__':
    main()
