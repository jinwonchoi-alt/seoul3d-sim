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
    for t in backs:
        y, h = t.strftime('%Y%m%d'), t.strftime('%H')
        got = {}
        for i in range(0, len(spots), 40):
            ids = ','.join(s['id'] for s in spots[i:i + 40])
            for r in A.get_json('%s/api/vol?spots=%s&ymd=%s&hh=%s' % (a.api, ids, y, h))['rows']:
                if not r.get('error'):
                    got[r['id']] = r
        if sum(r['in'] + r['out'] for r in got.values()) > 0:
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
