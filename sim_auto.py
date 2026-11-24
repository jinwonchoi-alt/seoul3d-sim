# -*- coding: utf-8 -*-
"""
서울 3D 지도 - SUMO 교통 시뮬레이션 자동 생성
사용법:  python sim_auto.py 강남구        (또는 구코드 11680)
옵션:    --minutes 10  --density 30  --nodelink <MOCT_LINK 폴더>  --site <site 폴더>

하는 일 (사람 손 안 댐)
 1) 표준노드링크(MOCT_LINK.shp)에서 해당 구 도로만 잘라냄
 2) netconvert 로 SUMO 도로망 생성 (차로수·제한속도·신호 자동 추정)
 3) 도로 등급/차로수 비례로 교통 수요 생성 (randomTrips)
 4) SUMO 실행 → 차량 궤적을 위경도로 저장
 5) site/sim/<구코드>.json 으로 압축 저장 → 3D 지도 '시뮬레이션' 레이어가 읽음
"""
import os, sys, json, argparse, subprocess, shutil, glob, math
import xml.etree.ElementTree as ET

GU = {'종로구':'11110','중구':'11140','용산구':'11170','성동구':'11200','광진구':'11215','동대문구':'11230',
      '중랑구':'11260','성북구':'11290','강북구':'11305','도봉구':'11320','노원구':'11350','은평구':'11380',
      '서대문구':'11410','마포구':'11440','양천구':'11470','강서구':'11500','구로구':'11530','금천구':'11545',
      '영등포구':'11560','동작구':'11590','관악구':'11620','서초구':'11650','강남구':'11680','송파구':'11710','강동구':'11740'}
TM = '+proj=tmerc +lat_0=38 +lon_0=127 +k=1 +x_0=200000 +y_0=600000 +ellps=GRS80 +units=m +no_defs'
HERE = os.path.dirname(os.path.abspath(__file__))
FCD_DT = 3   # 차량 위치 기록 간격(초): 지도에서 사이를 부드럽게 이어 그림


def log(*a):
    print('[sim]', *a, flush=True)


def fix_proj():
    try:
        import pyproj
        os.environ.setdefault('PROJ_DATA', pyproj.datadir.get_data_dir())
    except Exception:
        pass


def sumo_home():
    h = os.environ.get('SUMO_HOME')
    if h and os.path.isdir(h):
        return h
    try:
        import sumo
        return sumo.SUMO_HOME
    except Exception:
        pass
    sys.exit('SUMO를 찾을 수 없습니다. "pip install eclipse-sumo" 후 다시 실행하세요.')


def tool(home, name):
    exe = os.path.join(home, 'bin', name + ('.exe' if os.name == 'nt' else ''))
    return exe if os.path.exists(exe) else name


def run(cmd):
    log(' '.join(os.path.basename(c) if i == 0 else c for i, c in enumerate(cmd))[:200])
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout[-3000:], r.stderr[-3000:])
        sys.exit('실패: ' + os.path.basename(cmd[0]))
    return r


def cut_links(nodelink, gu_json, code, work, buffer_m):
    import shapefile
    from shapely.geometry import shape, LineString
    from shapely.ops import transform  # noqa
    from pyproj import Transformer
    to_tm = Transformer.from_crs('EPSG:4326', TM, always_xy=True).transform
    to_ll = Transformer.from_crs(TM, 'EPSG:4326', always_xy=True).transform
    feats = json.load(open(gu_json, encoding='utf-8'))['features']
    poly = None
    for f in feats:
        if str(f['properties'].get('sig')) == code:
            poly = transform(to_tm, shape(f['geometry'])).buffer(buffer_m)
    if poly is None:
        sys.exit('구 경계를 찾지 못했습니다: ' + code)
    minx, miny, maxx, maxy = poly.bounds
    src = shapefile.Reader(os.path.join(nodelink, 'MOCT_LINK'), encoding='cp949')
    out = os.path.join(work, 'links')
    w = shapefile.Writer(out, shapeType=shapefile.POLYLINE, encoding='utf-8')
    keep = ['LINK_ID', 'F_NODE', 'T_NODE', 'LANES', 'ROAD_RANK', 'ROAD_NAME', 'MAX_SPD']
    names = [f[0] for f in src.fields[1:]]
    idx = [names.index(k) for k in keep]
    for k in keep:
        fd = src.fields[1:][names.index(k)]
        w.field(k, 'N' if fd[1] == 'N' else 'C', size=fd[2] if k != 'ROAD_NAME' else 60)
    n = 0
    for i in range(len(src)):
        shp = src.shape(i)
        b = shp.bbox
        if b[2] < minx or b[0] > maxx or b[3] < miny or b[1] > maxy:
            continue
        line = LineString(shp.points)
        if not poly.intersects(line):
            continue
        rec = src.record(i)
        vals = [rec[j] for j in idx]
        if not vals[3] or vals[3] < 1:
            vals[3] = 1
        if not vals[6] or vals[6] < 10:
            vals[6] = 50
        w.line([[to_ll(x, y) for x, y in shp.points]])
        w.record(*vals)
        n += 1
    w.close()
    with open(out + '.prj', 'w') as f:
        f.write('GEOGCS["WGS 84",DATUM["WGS_1984",SPHEROID["WGS 84",6378137,298.257223563]],PRIMEM["Greenwich",0],UNIT["degree",0.0174532925199433]]')
    log('도로 링크', n, '개 추출')
    return out


def build_net(home, prefix, work):
    net = os.path.join(work, 'net.net.xml')
    run([tool(home, 'netconvert'), '--shapefile-prefix', prefix,
         '--shapefile.street-id', 'LINK_ID', '--shapefile.from-id', 'F_NODE', '--shapefile.to-id', 'T_NODE',
         '--shapefile.laneNumber', 'LANES', '--shapefile.speed', 'MAX_SPD', '--speed-in-kmh',
         '--shapefile.name', 'ROAD_NAME', '--shapefile.type-id', 'ROAD_RANK',
         '--shapefile.traditional-axis-mapping', '--proj.utm',
         '--geometry.remove', '--junctions.join', '--junctions.join-dist', '15',
         '--tls.guess', '--tls.guess.threshold', '40', '--tls.join', '--tls.default-type', 'actuated', '--tls.cycle.time', '150',
         '--roundabouts.guess', '--ramps.guess', '--no-turnarounds.except-deadend',
         '--keep-edges.components', '1', '--remove-edges.isolated',
         '--output.street-names', '--ignore-errors', '-o', net])
    return net


def make_demand(home, net, work, minutes, density, seed):
    # minutes = 워밍업 포함 전체 시간
    trips = os.path.join(work, 'trips.trips.xml')
    rou = os.path.join(work, 'routes.rou.xml')
    rt = os.path.join(home, 'tools', 'randomTrips.py')
    run([sys.executable, rt, '-n', net, '-o', trips, '-r', rou, '-b', '0', '-e', str(minutes * 60),
         '--insertion-density', str(density), '--lanes', '--speed-exponent', '1.5',
         '--fringe-factor', '8', '--min-distance', '800', '--seed', str(seed), '--validate',
         '--trip-attributes', 'departLane="best" departSpeed="max"',
         '--remove-loops'])
    return rou


def simulate(home, net, rou, work, minutes, warmup):
    fcd = os.path.join(work, 'fcd.xml')
    add = os.path.join(work, 'meandata.add.xml')
    with open(add, 'w') as f:
        f.write('<additional><edgeData id="md" file="meandata.xml" begin="%d" end="%d"/></additional>' % (warmup * 60, (warmup + minutes) * 60))
    run([tool(home, 'sumo'), '-n', net, '-r', rou, '--begin', '0', '--end', str((warmup + minutes) * 60),
         '--fcd-output', fcd, '--fcd-output.geo', '--device.fcd.period', str(FCD_DT), '--device.fcd.begin', str(warmup * 60),
         '--time-to-teleport', '120', '--ignore-route-errors', '--no-step-log', '--no-warnings',
         '--step-length', '1', '-a', add])
    return fcd


def pack(fcd, out_json, code, minutes, net, warmup, meta=None, gu_shape=None):
    """궤적 압축: 차량별 [시작초, lon*1e5, lat*1e5, 이후 (dlon,dlat,속도km/h) 반복], 2초 간격"""
    tracks = {}
    for ev, el in ET.iterparse(fcd, events=('end',)):
        if el.tag != 'timestep':
            continue
        t = int(float(el.get('time'))) - warmup * 60
        if t < 0:
            el.clear()
            continue
        for v in el:
            vid = v.get('id')
            tr = tracks.get(vid)
            if tr is None:
                tracks[vid] = tr = {'t0': t, 'pts': []}
            tr['pts'].append((round(float(v.get('x')) * 1e5), round(float(v.get('y')) * 1e5),
                              round(float(v.get('speed')) * 3.6)))
        el.clear()
    # 구 경계(+약 300m) 안에 있는 구간만 저장 (주변 도로는 계산에만 쓰고 파일은 작게)
    inside = None
    if gu_shape is not None:
        from shapely.prepared import prep
        from shapely.geometry import Point
        pg = prep(gu_shape.buffer(0.003))
        inside = lambda x, y: pg.contains(Point(x / 1e5, y / 1e5))
    vehs, npts = [], 0
    for tr in tracks.values():
        runs, cur, t = [], [], tr['t0']
        for i, q in enumerate(tr['pts']):
            if inside is None or inside(q[0], q[1]):
                if not cur:
                    cur_t0 = tr['t0'] + i * FCD_DT
                cur.append(q)
            elif cur:
                runs.append((cur_t0, cur)); cur = []
        if cur:
            runs.append((cur_t0, cur))
        for t0, p in runs:
            if len(p) < 2:
                continue
            arr = [t0, p[0][0], p[0][1], p[0][2]]
            for a, b in zip(p, p[1:]):
                arr += [b[0] - a[0], b[1] - a[1], b[2]]
            vehs.append(arr)
            npts += len(p)
    # 도로망 경계 (카메라 이동용)
    xs = [v[1] for v in vehs] or [0]
    ys = [v[2] for v in vehs] or [0]
    data = {'gu': code, 'dt': FCD_DT, 'dur': minutes * 60, 'scale': 1e5,
            'bbox': [min(xs) / 1e5, min(ys) / 1e5, max(xs) / 1e5, max(ys) / 1e5], 'v': vehs}
    if meta:
        data.update(meta)
    os.makedirs(os.path.dirname(out_json), exist_ok=True)
    with open(out_json, 'w') as f:
        json.dump(data, f, separators=(',', ':'))
    log('차량', len(vehs), '대 / 위치점', npts, '개 →', out_json, '(%.1f MB)' % (os.path.getsize(out_json) / 1e6))


# ---------- 실제 교통량 보정 (서울시 TOPIS 검지기 교통량) ----------
API_DEFAULT = 'https://seoul3d-api.jinwonchoi.workers.dev'


def get_json(url):
    import urllib.request
    req = urllib.request.Request(url, headers={'User-Agent': 'seoul3d-sim'})
    with urllib.request.urlopen(req, timeout=60) as r:
        return json.loads(r.read().decode('utf-8'))


def fetch_counts(api, gu_json, code, ymd=None, hh=None):
    """구 안의 교통량 지점과 최근 1시간 교통량. 반환: {'ymd','hh','spots':[{id,nm,lon,lat,in,out}]}"""
    from shapely.geometry import shape, Point
    from pyproj import Transformer
    import datetime
    poly = None
    for f in json.load(open(gu_json, encoding='utf-8'))['features']:
        if str(f['properties'].get('sig')) == code:
            poly = shape(f['geometry']).buffer(0.002)
    lst = get_json(api + '/api/vol?list=1')['rows']
    tr = Transformer.from_crs('EPSG:5181', 'EPSG:4326', always_xy=True)
    spots = []
    for r in lst:
        lon, lat = tr.transform(r['x'], r['y'])
        if poly.contains(Point(lon, lat)):
            spots.append({'id': r['id'], 'nm': r['nm'], 'lon': round(lon, 6), 'lat': round(lat, 6)})
    if not spots:
        return {'ymd': '', 'hh': '', 'spots': []}
    tries = [(ymd, hh)] if ymd else []
    now = datetime.datetime.utcnow() + datetime.timedelta(hours=9)
    for back in (1, 2, 3, 24):
        t = now - datetime.timedelta(hours=back)
        tries.append((t.strftime('%Y%m%d'), t.strftime('%H')))
    for y, h in tries:
        res = get_json('%s/api/vol?spots=%s&ymd=%s&hh=%s' % (api, ','.join(s['id'] for s in spots), y, h))
        vol = {r['id']: r for r in res['rows'] if not r.get('error')}
        if sum(v['in'] + v['out'] for v in vol.values()) > 0:
            for s in spots:
                v = vol.get(s['id'], {})
                s['in'], s['out'] = v.get('in', 0), v.get('out', 0)
            spots = [s for s in spots if s['in'] + s['out'] > 0]
            return {'ymd': y, 'hh': h, 'spots': spots}
    return {'ymd': '', 'hh': '', 'spots': []}


def sumolib_of(home):
    sys.path.insert(0, os.path.join(home, 'tools'))
    import sumolib
    return sumolib


def map_spots(home, net_file, spots):
    """지점마다 가까운 도로(양방향 2개 edge)를 찾음"""
    sumolib = sumolib_of(home)
    net = sumolib.net.readNet(net_file)
    for s in spots:
        x, y = net.convertLonLat2XY(s['lon'], s['lat'])
        cand = [(e, d) for e, d in net.getNeighboringEdges(x, y, 60) if e.getFunction() != 'internal']
        cand.sort(key=lambda t: t[1])
        s['edges'] = []
        if not cand:
            continue
        a = cand[0][0]
        s['edges'].append(a.getID())
        fa, ta = a.getFromNode().getID(), a.getToNode().getID()
        for e, d in cand[1:]:
            # 반대 방향: 출발/도착 노드가 뒤바뀐 도로, 없으면 가까운 다른 방향 도로
            if (e.getFromNode().getID() == ta and e.getToNode().getID() == fa) or d < 40:
                if e.getID() not in s['edges']:
                    s['edges'].append(e.getID())
                    break
    return [s for s in spots if s['edges']]


def route_edge_counts(rou):
    cnt = {}
    for ev, el in ET.iterparse(rou, events=('end',)):
        if el.tag == 'route' and el.get('edges'):
            for e in set(el.get('edges').split()):
                cnt[e] = cnt.get(e, 0) + 1
        if el.tag == 'vehicle':
            el.clear()
    return cnt


def calibrate(home, net, work, minutes, density, seed, counts, boost=None):
    """후보 경로를 넉넉히 만든 뒤, 관측 교통량에 맞게 경로를 골라냄 (SUMO routeSampler)"""
    spots = counts['spots']
    T = minutes * 60
    base = os.path.join(work, 'routes.rou.xml')
    if not os.path.exists(base):
        base = make_demand(home, net, work, minutes, density, seed)
    bc = route_edge_counts(base)
    nbase = sum(1 for ev, el in ET.iterparse(base, events=('end',)) if el.tag == 'vehicle')
    obs_rows, ratios = [], []
    for s in spots:
        e = s['edges']
        v = [s['in'], s['out']]
        if len(e) == 2:   # 유입/유출 중 어느 쪽이 어느 방향인지 → 기본 흐름 비율과 더 비슷한 쪽으로 배정
            b0, b1 = bc.get(e[0], 0), bc.get(e[1], 0)
            if (v[0] - v[1]) * (b0 - b1) < 0:
                v = [v[1], v[0]]
            pairs = [(e[0], v[0]), (e[1], v[1])]
        else:
            pairs = [(e[0], v[0] + v[1])]
        for eid, vh in pairs:
            c = vh * T / 3600.0 * ((boost or {}).get(s['id'], 1.0))
            obs_rows.append((eid, c))
            if bc.get(eid, 0) > 5:
                ratios.append(c / bc[eid])
    if not ratios:
        log('보정 지점이 도로와 연결되지 않아 보정 생략')
        return base
    ratios.sort()
    r = ratios[len(ratios) // 2]
    log('관측/기본 교통량 비율 중앙값 %.2f (지점 %d곳)' % (r, len(ratios)))
    cand_density = density * 2   # 후보 경로는 넉넉히, 실제 투입량은 기본 수준 유지 (전체를 늘리면 망 전체가 막힘)
    cdir = os.path.join(work, 'cand')
    os.makedirs(cdir, exist_ok=True)
    cand = os.path.join(cdir, 'routes.rou.xml')
    if not os.path.exists(cand):
        cand = make_demand(home, net, cdir, minutes, cand_density, seed + 1)
    cnt_file = os.path.join(work, 'counts.xml')
    with open(cnt_file, 'w') as f:
        f.write('<data>\n  <interval id="obs" begin="0" end="%d">\n' % T)
        for eid, c in obs_rows:
            f.write('    <edge id="%s" entered="%d"/>\n' % (eid, round(c)))
        f.write('  </interval>\n</data>\n')
    out = os.path.join(work, 'calibrated.rou.xml')
    run([sys.executable, os.path.join(home, 'tools', 'routeSampler.py'), '-r', cand, '-d', cnt_file,
         '-o', out, '--total-count', str(int(nbase * min(1.2, max(0.8, r)))), '--seed', str(seed),
         '-a', 'departLane="best" departSpeed="max"', '--mismatch-output', os.path.join(work, 'mismatch.xml')])
    return out


def measure_spots(meandata, spots, warmup, minutes):
    """시뮬레이션 기록 구간의 지점별 시간당 통과 대수"""
    tot = {}
    for ev, el in ET.iterparse(meandata, events=('end',)):
        if el.tag == 'edge':
            tot[el.get('id')] = tot.get(el.get('id'), 0) + float(el.get('entered', 0))
    res = []
    for s in spots:
        simv = sum(tot.get(e, 0) for e in s['edges']) * 60.0 / minutes
        res.append({'nm': s['nm'], 'lon': s['lon'], 'lat': s['lat'], 'obs': s['in'] + s['out'], 'sim': round(simv)})
    return res


def ask_gu(allow_all=False):
    names = list(GU.keys())
    print('\n어느 구를 만들까요? 번호를 입력하세요.')
    for i, n in enumerate(names, 1):
        print('%2d. %-6s' % (i, n), end='\n' if i % 5 == 0 else '   ')
    if allow_all:
        print(' 0. 서울 전체 (25개 구, 구마다 2~3분)')
    while True:
        t = input('번호: ').strip()
        if allow_all and t == '0':
            return 'all'
        if t.isdigit() and 1 <= int(t) <= len(names):
            return names[int(t) - 1]
        print('1~25 사이 번호를 입력하세요.')


def gu_shape_of(gu_json, code):
    try:
        from shapely.geometry import shape
        from shapely.ops import unary_union
        return unary_union([shape(f['geometry']) for f in json.load(open(gu_json, encoding='utf-8'))['features'] if str(f['properties'].get('sig')) == code])
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('gu', nargs='?', default=None, help='구 이름(강남구), 구코드(11680) 또는 all')
    ap.add_argument('--minutes', type=int, default=10, help='시뮬레이션 시간(분)')
    ap.add_argument('--density', type=float, default=30, help='도로 1km당 시간당 차량 투입 수')
    ap.add_argument('--warmup', type=int, default=5, help='기록 전 워밍업(분) - 도로에 차가 찬 상태부터 기록')
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--nodelink', default=None, help='MOCT_LINK.shp 가 있는 폴더')
    ap.add_argument('--site', default=os.path.join(HERE, '..', 'site'))
    ap.add_argument('--api', default=API_DEFAULT, help='중계 서버 주소 (교통량 받기)')
    ap.add_argument('--no-calib', action='store_true', help='실제 교통량 보정 안 함')
    ap.add_argument('--counts-file', default=None, help='교통량 JSON 파일 (인터넷 없이 보정할 때)')
    ap.add_argument('--ymd', default=None)
    ap.add_argument('--hh', default=None)
    ap.add_argument('--net-dir', default=None, help='미리 만든 도로망 폴더 (<구코드>.net.xml.gz)')
    ap.add_argument('--buffer', type=float, default=2000, help='구 경계 바깥 도로 포함 범위(m) - 다리·경계 교통 반영')
    a = ap.parse_args()
    if not a.gu:
        a.gu = ask_gu(True)
    codes = list(GU.values()) if a.gu.lower() == 'all' else [GU.get(a.gu, a.gu)]
    if codes[0] not in GU.values():
        sys.exit('구 이름/코드를 확인하세요: ' + a.gu)
    if not a.nodelink and not a.net_dir:
        c = glob.glob(os.path.join(HERE, '..', 'raw', 'nodelink', '*', 'MOCT_LINK.shp'))
        if not c:
            sys.exit('MOCT_LINK.shp 를 찾지 못했습니다. --nodelink 로 폴더를 지정하세요.')
        a.nodelink = os.path.dirname(sorted(c)[-1])
    fix_proj()
    home = sumo_home()
    for code in codes:
        work = os.path.join(HERE, 'work', code)
        shutil.rmtree(work, ignore_errors=True)
        os.makedirs(work)
        log('구', code, '/ 노드링크', a.nodelink)
        pre = os.path.join(a.net_dir, code + '.net.xml.gz') if a.net_dir else None
        if pre and os.path.exists(pre):
            net = pre
        else:
            prefix = cut_links(a.nodelink, os.path.join(a.site, 'seoul_gu.json'), code, work, a.buffer)
            net = build_net(home, prefix, work)
        import datetime
        meta = {'calib': None, 'gen': (datetime.datetime.utcnow() + datetime.timedelta(hours=9)).strftime('%Y-%m-%d %H:%M')}
        counts = None
        if not a.no_calib:
            try:
                if a.counts_file:
                    counts = json.load(open(a.counts_file, encoding='utf-8'))
                    if code in counts:
                        counts = counts[code]
                else:
                    counts = fetch_counts(a.api, os.path.join(a.site, 'seoul_gu.json'), code, a.ymd, a.hh)
            except Exception as e:
                log('교통량 받기 실패 → 보정 없이 진행:', e)
                counts = None
        spots = map_spots(home, net, counts['spots']) if counts and counts.get('spots') else []
        if spots:
            log('교통량 지점 %d곳 (%s %s시 기준)으로 보정' % (len(spots), counts['ymd'], counts['hh']))
            rou = calibrate(home, net, work, a.warmup + a.minutes, a.density, a.seed, dict(counts, spots=spots))
        else:
            log('이 구에는 교통량 지점이 없거나 받지 못함 → 도로 규모 기준 수요만 사용')
            rou = make_demand(home, net, work, a.warmup + a.minutes, a.density, a.seed)
        fcd = simulate(home, net, rou, work, a.minutes, a.warmup)
        if spots:
            res = measure_spots(os.path.join(work, 'meandata.xml'), spots, a.warmup, a.minutes)
            # 2차 보정: 1차 결과에서 모자란 지점은 목표를 높여 한 번 더 (최대 2배)
            boost = {}
            for s_, r in zip(spots, res):
                if r['sim'] > 0:
                    boost[s_['id']] = min(2.0, max(0.7, r['obs'] / float(r['sim'])))
            tot1 = sum(abs(r['sim'] - r['obs']) for r in res)
            log('1차 보정 오차 %d%% → 2차 보정' % (100 * tot1 / max(1, sum(r['obs'] for r in res))))
            rou2 = calibrate(home, net, work, a.warmup + a.minutes, a.density, a.seed, dict(counts, spots=spots), boost)
            fcd2 = simulate(home, net, rou2, work, a.minutes, a.warmup)
            res2 = measure_spots(os.path.join(work, 'meandata.xml'), spots, a.warmup, a.minutes)
            if sum(abs(r['sim'] - r['obs']) for r in res2) < tot1:
                res, fcd = res2, fcd2
            else:
                log('2차 보정이 더 나쁘지 않게 1차 결과 유지')
                fcd = simulate(home, net, rou, work, a.minutes, a.warmup)
                res = measure_spots(os.path.join(work, 'meandata.xml'), spots, a.warmup, a.minutes)
            meta['calib'] = {'ymd': counts['ymd'], 'hh': counts['hh'], 'spots': res}
            for r in res:
                log('  %-16s 관측 %5d대/h  시뮬 %5d대/h' % (r['nm'], r['obs'], r['sim']))
            log('보정 결과: 시뮬/관측 합계 %d%%' % (100 * sum(r['sim'] for r in res) / max(1, sum(r['obs'] for r in res))))
        pack(fcd, os.path.join(a.site, 'sim', code + '.json'), code, a.minutes, net, a.warmup, meta, gu_shape_of(os.path.join(a.site, 'seoul_gu.json'), code))
        os.remove(fcd)
    log('완료. 3D 지도에서 [시뮬레이션] 레이어를 켜세요.')


if __name__ == '__main__':
    main()
