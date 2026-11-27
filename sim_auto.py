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
TLS_ACT = False
MEAS = None   # 교통량 재는 시간(분)
PASSES = 3    # 새 보정 반복 횟수
TARGET = {}   # 지점 도로별 관측 교통량(대/시) — 2차 보정용
TTT = 120
CALIB_GAIN = 1.25
TR_DMIN = 600
TR_DMAX = 2500
JB = []
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


# 실시간 인구(서울 도시데이터 121곳) → 사람이 많은 곳일수록 차가 더 많이 출발·도착
PPL = None        # [(shapely 폴리곤, 인구), ...]  main 에서 채움
PPL_REF = 20000.0  # 인구 2만 명 = 출발·도착 확률 +1배
PPL_CAP = 8.0
PPL_HIT = {}      # 이번 구에서 반영된 인구 구역 {이름: 인구}


def load_ppl(ppl_file, hotspots_file, at=None):
    """at 을 주면 그 시각의 예측 인구(ppl_all.json 의 f)를 씀. 그 시각 예측이 없는 구역은 가장 가까운 예측 시각(90분 안), 그것도 없으면 지금 인구"""
    from shapely.geometry import shape
    d = json.load(open(ppl_file, encoding='utf-8'))
    areas = d.get('areas', d)
    out = []
    for f in json.load(open(hotspots_file, encoding='utf-8'))['features']:
        a = areas.get(f['properties'].get('AREA_CD'))
        if not a:
            continue
        if at:
            f_ = a.get('f') or {}
            v = f_.get(at)
            if not v and f_:
                # 지역마다 갱신 시각이 달라 그 시각 예측이 없으면 가장 가까운 예측 시각(90분 안) 값을 씀
                try:
                    import datetime as _dt
                    t0 = _dt.datetime.strptime(at[:16], '%Y-%m-%d %H:%M')
                    k = min(f_, key=lambda x: abs((_dt.datetime.strptime(x[:16], '%Y-%m-%d %H:%M') - t0).total_seconds()))
                    if abs((_dt.datetime.strptime(k[:16], '%Y-%m-%d %H:%M') - t0).total_seconds()) <= 5400:
                        v = f_[k]
                except Exception:
                    v = None
            p = float(v[0]) if v else float(a.get('p', 0))   # 그래도 없으면 지금 인구
        else:
            p = float(a.get('p', 0))
        if p > 0:
            out.append((shape(f['geometry']).buffer(0.0015), p, f['properties'].get('AREA_NM')))
    return out, (at or d.get('t'))


def ppl_weights(home, net, work, base_args):
    """randomTrips 기본 확률(차로·속도·외곽 유입)을 뽑은 뒤, 인구 많은 구역 도로에 배수를 곱해 다시 씀"""
    import sumolib
    from shapely.geometry import Point
    rt = os.path.join(home, 'tools', 'randomTrips.py')
    w0 = os.path.join(work, 'w0')
    run([sys.executable, rt, '-n', net, '-o', os.path.join(work, 'w0.trips.xml'), '-b', '0', '-e', '1',
         '--weights-output-prefix', w0] + base_args)
    n = sumolib.net.readNet(net)
    fac, hit = {}, {}
    for e in n.getEdges():
        sh = e.getShape()
        x, y = sh[len(sh) // 2]
        lon, lat = n.convertXY2LonLat(x, y)
        pt, f = Point(lon, lat), 1.0
        for poly, pop, nm in PPL:
            if poly.contains(pt):
                f += pop / PPL_REF
                hit[nm] = pop
        fac[e.getID()] = min(PPL_CAP, f)
    w1 = os.path.join(work, 'w1')
    for suf in ('.src.xml', '.dst.xml'):
        tree = ET.parse(w0 + suf)
        for el in tree.iter('edge'):
            el.set('value', '%.6g' % (float(el.get('value', 0)) * fac.get(el.get('id'), 1.0)))
        tree.write(w1 + suf)
    log('실시간 인구 반영: 인구 구역 %d곳 (%s)' % (len(hit), ', '.join('%s %d' % (k, v) for k, v in sorted(hit.items(), key=lambda t: -t[1])[:5])))
    return w1, hit


def make_demand(home, net, work, minutes, density, seed):
    # minutes = 워밍업 포함 전체 시간
    trips = os.path.join(work, 'trips.trips.xml')
    rou = os.path.join(work, 'routes.rou.xml')
    rt = os.path.join(home, 'tools', 'randomTrips.py')
    base = ['--lanes', '--speed-exponent', '1.5', '--fringe-factor', '8', '--seed', str(seed)]
    extra = []
    if PPL:
        try:
            w1, hit = ppl_weights(home, net, work, base)
            PPL_HIT.clear()
            PPL_HIT.update(hit)
            extra = ['--weights-prefix', w1]
        except Exception as e:
            log('인구 반영 실패 → 도로 규모 기준만 사용:', e)
    run([sys.executable, rt, '-n', net, '-o', trips, '-r', rou, '-b', '0', '-e', str(minutes * 60),
         '--insertion-density', str(density), '--min-distance', '800', '--validate',
         '--trip-attributes', 'departLane="best" departSpeed="max"',
         '--remove-loops'] + base + extra)
    return rou


def tls_actuated(home, net_file, out_file, max_green=60):
    """모든 신호를 '감응식'(차가 많은 방향에 초록불을 더 주는 방식)으로 바꾸는 추가 파일.
       도로망을 만들 때 생긴 기본 신호는 방향마다 초록불을 똑같이 나눠 큰길이 막혔음 (관측의 60%밖에 못 흘림)"""
    sumolib = sumolib_of(home)
    net = sumolib.net.readNet(net_file, withPrograms=True)
    n = 0
    with open(out_file, 'w') as f:
        f.write('<additional>\n')
        for tl in net.getTrafficLights():
            for pid, prog in tl.getPrograms().items():
                f.write('  <tlLogic id="%s" programID="act" type="actuated" offset="0">\n' % tl.getID())
                for ph in prog.getPhases():
                    st = ph.state
                    green = ('G' in st or 'g' in st) and 'y' not in st
                    if green:
                        f.write('    <phase duration="%d" minDur="6" maxDur="%d" state="%s"/>\n' % (max(10, ph.duration), max_green, st))
                    else:
                        f.write('    <phase duration="%d" state="%s"/>\n' % (ph.duration, st))
                f.write('  </tlLogic>\n')
                n += 1
                break
        f.write('</additional>\n')
    return n

def simulate(home, net, rou, work, minutes, warmup):
    """교통량은 워밍업 뒤 MEAS분 동안 재고(짧게 재면 우연에 흔들림), 지도용 차량 위치는 마지막 minutes분만 기록"""
    M = max(minutes, MEAS or minutes)
    fcd = os.path.join(work, 'fcd.xml')
    add = os.path.join(work, 'meandata.add.xml')
    with open(add, 'w') as f:
        f.write('<additional><edgeData id="md" file="meandata.xml" begin="%d" end="%d"/></additional>' % (warmup * 60, (warmup + M) * 60))
    if TLS_ACT:
        tla = os.path.join(work, 'tls_act.add.xml')
        if not os.path.exists(tla):
            log('신호 %d곳을 감응식으로' % tls_actuated(home, net, tla))
        add = add + ',' + tla
    run([tool(home, 'sumo'), '-n', net, '-r', rou, '--begin', '0', '--end', str((warmup + M) * 60),
         '--fcd-output', fcd, '--fcd-output.geo', '--device.fcd.period', str(FCD_DT), '--device.fcd.begin', str((warmup + M - minutes) * 60),
         '--time-to-teleport', str(TTT), '--ignore-route-errors'] + JB + [ '--no-step-log', '--no-warnings',
         '--step-length', '1', '-a', add])
    return fcd


def pack(fcd, out_json, code, minutes, net, warmup, meta=None, gu_shape=None):
    """궤적 압축: 차량별 [시작초, lon*1e5, lat*1e5, 이후 (dlon,dlat,속도km/h) 반복], FCD_DT 간격(지금 3초, 예측 6초)"""
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
    os.makedirs(os.path.dirname(os.path.abspath(out_json)), exist_ok=True)
    if out_json.endswith('.gz'):
        import gzip
        with gzip.open(out_json, 'wt', compresslevel=9) as f:
            json.dump(data, f, separators=(',', ':'))
    else:
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
    # 시점마다 자료가 있는 지점 수를 세고, 가장 많은 시점의 80% 이상 채워진 시점 중 가장 최근 것을 씀
    cands = []
    for y, h in tries:
        try:
            res = get_json('%s/api/vol?spots=%s&ymd=%s&hh=%s' % (api, ','.join(s['id'] for s in spots), y, h))
        except Exception as e:
            log('교통량 조회 실패 %s %s시: %s' % (y, h, e)); continue
        vol = {r['id']: r for r in res['rows'] if not r.get('error')}
        cands.append((sum(1 for v in vol.values() if v['in'] + v['out'] > 0), vol, y, h))
    best = max([c[0] for c in cands] or [0])
    for n, vol, y, h in cands:
        if best and n >= best * 0.8:
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
    """지점마다 해당 도로(양방향 2개 edge)를 찾음.
       예전엔 '가장 가까운 도로'라서 큰길 옆 골목(시흥대로4길)이나 끊긴 지하도로가 잡혀 교통량이 크게 모자랐음
       → 지점 이름의 도로명과 같은 도로를 먼저, 없으면 차로 많은 도로를 고르고, 너무 짧거나 어디에도 안 이어진 도로는 뺌"""
    import re
    sumolib = sumolib_of(home)
    net = sumolib.net.readNet(net_file)
    for s in spots:
        x, y = net.convertLonLat2XY(s['lon'], s['lat'])
        road = re.sub(r'\(.*', '', s['nm']).replace(' ', '')
        key = road[:4] if len(road) >= 4 else road
        def usable(e):
            return e.getFunction() != 'internal' and e.allows('passenger') and e.getLength() >= 8 and (e.getIncoming() or e.getOutgoing())
        cand = [(e, d) for e, d in net.getNeighboringEdges(x, y, 150) if usable(e)]
        named = [(e, d) for e, d in cand if key and key in (e.getName() or '').replace(' ', '')]
        if named:
            pool = named
        else:
            near = [(e, d) for e, d in cand if d <= 60]
            big = [(e, d) for e, d in near if e.getLaneNumber() >= 2]
            pool = big or near
        if not pool:
            s['edges'] = []
            continue
        # 가까우면서 큰 도로 (거리 1m = 차로 0.05개)
        pool.sort(key=lambda t: t[1] - 20 * min(t[0].getLaneNumber(), 5))
        a = pool[0][0]
        s['edges'] = [a.getID()]
        fa, ta = a.getFromNode().getID(), a.getToNode().getID()
        ax, ay = a.getToNode().getCoord()[0] - a.getFromNode().getCoord()[0], a.getToNode().getCoord()[1] - a.getFromNode().getCoord()[1]
        best = None
        for e, d in pool[1:] + [t for t in cand if t not in pool]:
            if e.getID() == a.getID():
                continue
            if e.getFromNode().getID() == ta and e.getToNode().getID() == fa:
                best = e
                break
            bx, by = e.getToNode().getCoord()[0] - e.getFromNode().getCoord()[0], e.getToNode().getCoord()[1] - e.getFromNode().getCoord()[1]
            if best is None and d < 60 and ax * bx + ay * by < 0 and (not named or (e, d) in named):   # 반대 방향
                best = e
        if best is not None:
            s['edges'].append(best.getID())
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


MAX_SCALE = 1.2   # 관측/기본 비율을 투입 총량에 반영하는 상한 (--max-scale)


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
         '-o', out, '--total-count', str(int(nbase * min(MAX_SCALE, max(0.8, r)))), '--seed', str(seed),
         '-a', 'departLane="best" departSpeed="max"', '--mismatch-output', os.path.join(work, 'mismatch.xml')])
    return out


def through_routes(home, net_file, edges, out_file, per_edge=80, seed=1, dmin=None, dmax=None):
    dmin = dmin or TR_DMIN; dmax = dmax or TR_DMAX
    """관측 지점 도로를 반드시 지나는 후보 경로: 지점 위쪽 0.6~2.5km 에서 출발해 지점을 지나 아래쪽 0.6~2.5km 로 (큰길 교통량이 모자라던 문제 보완)"""
    import random
    from collections import deque
    sumolib = sumolib_of(home)
    net = sumolib.net.readNet(net_file)
    rnd = random.Random(seed)
    ok0 = lambda e: e.getFunction() != 'internal' and e.allows('passenger')
    def bfs(start, back):
        prev, dist, q = {start: None}, {start: 0.0}, deque([start])
        while q:
            e = q.popleft()
            if dist[e] > dmax:
                continue
            nb = list(e.getIncoming().keys()) if back else list(e.getOutgoing().keys())
            for n in nb:
                if n in prev or not ok(n):
                    continue
                prev[n] = e
                dist[n] = dist[e] + n.getLength()
                q.append(n)
        return prev, dist
    n = 0
    with open(out_file, 'w') as f:
        f.write('<routes>\n')
        for eid in edges:
            try:
                e = net.getEdge(eid)
            except Exception:
                continue
            vmin = e.getSpeed() * 0.75
            ok = lambda x: ok0(x) and (x.getSpeed() >= vmin or x.getLaneNumber() >= max(2, e.getLaneNumber() - 1))   # 지점과 비슷한 큰길로만 이어서
            pu, du = bfs(e, True)
            pd, dd = bfs(e, False)
            ups = [x for x in pu if dmin <= du[x] <= dmax] or [x for x in pu if x is not e] or [e]   # 도로망 끝에서 잘린 도로는 그 도로에서 출발
            dns = [x for x in pd if dmin <= dd[x] <= dmax] or [x for x in pd if x is not e] or [e]
            for k in range(per_edge):
                o, d = rnd.choice(ups), rnd.choice(dns)
                a = []
                x = o
                while x is not None:
                    a.append(x.getID()); x = pu[x]
                b = []
                x = d
                while x is not None and x is not e:
                    b.append(x.getID()); x = pd[x]
                route = a + b[::-1]
                f.write('  <route id="t%d" edges="%s"/>\n' % (n, ' '.join(route)))
                n += 1
        f.write('</routes>\n')
    return n


def merge_routes(files, out):
    """여러 경로 파일의 차량을 출발 시각 순으로 하나로 (id 겹치지 않게)"""
    items = []
    for i, fn in enumerate(files):
        for ev, el in ET.iterparse(fn, events=('end',)):
            if el.tag in ('vehicle', 'trip', 'flow'):
                el.set('id', 'm%d_%s' % (i, el.get('id')))
                items.append((float(el.get('depart', 0) or 0), ET.tostring(el, encoding='unicode')))
                el.clear()
    items.sort(key=lambda t: t[0])
    with open(out, 'w') as f:
        f.write('<routes>\n')
        for _, x in items:
            f.write('  ' + x.strip() + '\n')
        f.write('</routes>\n')
    return out


def calibrate2(home, net, work, minutes, density, seed, counts, boost=None, gain=None, eboost=None, tag=''):
    gain = gain or CALIB_GAIN
    """배경 교통(randomTrips) + 지점별 부족분만 '지점을 지나는 경로'로 채움.
       예전 방식(총량 상한 안에서 경로 고르기)은 큰길(시간당 2천 대 이상)에서 관측의 60%밖에 못 채웠음."""
    spots = counts['spots']
    T = minutes * 60
    base = os.path.join(work, 'routes.rou.xml')
    if not os.path.exists(base):
        base = make_demand(home, net, work, minutes, density, seed)
    bc = route_edge_counts(base)
    rows, edges = [], []
    for s in spots:
        e = s['edges']
        v = [s['in'], s['out']]
        if len(e) == 2:
            b0, b1 = bc.get(e[0], 0), bc.get(e[1], 0)
            if (v[0] - v[1]) * (b0 - b1) < 0:
                v = [v[1], v[0]]
            pairs = [(e[0], v[0]), (e[1], v[1])]
        else:
            pairs = [(e[0], v[0] + v[1])]
        for eid, vh in pairs:
            need = vh * T / 3600.0 * ((boost or {}).get(s['id'], 1.0)) * ((eboost or {}).get(eid, 1.0)) * gain - bc.get(eid, 0)
            TARGET[eid] = vh
            rows.append((eid, max(0, need)))
            edges.append(eid)
    cand = os.path.join(work, 'through.rou.xml')
    if os.path.exists(cand):
        nr = 1; log('지점 통과 후보 경로: 앞 회차 것 다시 씀')
    else:
        nr = through_routes(home, net, edges, cand, seed=seed); log('지점 통과 후보 경로 %d개' % nr)
    if not nr:
        return base
    cnt_file = os.path.join(work, 'counts%s.xml' % tag)
    with open(cnt_file, 'w') as f:
        f.write('<data>\n  <interval id="obs" begin="0" end="%d">\n' % T)
        for eid, c in rows:
            f.write('    <edge id="%s" entered="%d"/>\n' % (eid, round(c)))
        f.write('  </interval>\n</data>\n')
    extra = os.path.join(work, 'extra%s.rou.xml' % tag)
    run([sys.executable, os.path.join(home, 'tools', 'routeSampler.py'), '-r', cand, '-d', cnt_file,
         '-o', extra, '--seed', str(seed), '-a', 'departLane="best" departSpeed="max"', '--prefix', 'x'])
    out = os.path.join(work, 'calibrated%s.rou.xml' % tag)
    return merge_routes([base, extra], out)

def edge_rates(meandata, minutes):
    tot = {}
    for ev, el in ET.iterparse(meandata, events=('end',)):
        if el.tag == 'edge':
            tot[el.get('id')] = tot.get(el.get('id'), 0) + float(el.get('entered', 0)) + float(el.get('departed', 0))   # 도로망 끝에서 잘려 들어오는 길이 없는 도로(관악터널 등)는 차가 그 도로에서 출발하므로 출발 대수도 셈
    return {k: v * 60.0 / minutes for k, v in tot.items()}


def measure_spots(meandata, spots, warmup, minutes):
    """시뮬레이션 기록 구간의 지점별 시간당 통과 대수"""
    tot = {}
    for ev, el in ET.iterparse(meandata, events=('end',)):
        if el.tag == 'edge':
            tot[el.get('id')] = tot.get(el.get('id'), 0) + float(el.get('entered', 0)) + float(el.get('departed', 0))   # 도로망 끝에서 잘려 들어오는 길이 없는 도로(관악터널 등)는 차가 그 도로에서 출발하므로 출발 대수도 셈
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
    ap.add_argument('--density', type=float, default=None, help='도로 1km당 시간당 배경 차량 투입 수 (기본: 새 보정 12, 예전 보정 30)')
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
    ap.add_argument('--ppl-file', default=None, help='실시간 인구 JSON (ppl_all.py 결과) - 사람 많은 곳에서 차가 더 출발·도착')
    ap.add_argument('--hotspots', default=None, help='인구 구역 경계 (hotspots.json)')
    ap.add_argument('--buffer', type=float, default=2000, help='구 경계 바깥 도로 포함 범위(m) - 다리·경계 교통 반영')
    ap.add_argument('--ppl-at', default=None, help='예측 시뮬레이션: 이 시각("YYYY-MM-DD HH:00")의 예측 인구 사용')
    ap.add_argument('--out', default=None, help='결과 파일 경로 (기본 <site>/sim/<구코드>.json, .gz 로 끝나면 압축 저장)')
    ap.add_argument('--fast', action='store_true', help='2차 보정 생략 (예측 시뮬레이션용, 시간 절반)')
    ap.add_argument('--fcd-dt', type=int, default=3, help='차량 위치 기록 간격(초). 크게 하면 파일이 작아짐')
    ap.add_argument('--fast-boost', type=float, default=1.8, help='--fast 일 때 관측 목표 배수')
    ap.add_argument('--measure', type=int, default=None, help='교통량 재는 시간(분, 기본 10 · 예측(--fast) 8). 지도 기록(--minutes)보다 길면 앞부분은 재기만 함')
    ap.add_argument('--passes', type=int, default=None, help='새 보정 반복 횟수(1차 포함, 기본 3 · 예측(--fast) 1). 4번째부터는 거의 안 줄어듦')
    ap.add_argument('--gain', type=float, default=1.6, help='새 보정: 지점 목표 배수 (출발~지점 도착 지연 보정)')
    ap.add_argument('--tr-min', type=float, default=300, help='새 보정: 지점 통과 경로 출발 거리 최소(m)')
    ap.add_argument('--tr-max', type=float, default=1500, help='새 보정: 지점 통과 경로 출발 거리 최대(m)')
    ap.add_argument('--no-unjam', dest='unjam', action='store_false', help='교차로 꽉 막힘 풀기 끄기 (기본: 15초 막으면 무시, 60초 정체 시 순간이동)')
    ap.add_argument('--tls-act', action='store_true', help='신호를 감응식으로 (큰길 흐름 개선)')
    ap.add_argument('--old-calib', dest='calib2', action='store_false', help='예전 보정 방식(총량 상한 안에서 경로 고르기)으로')
    ap.add_argument('--max-scale', type=float, default=1.2, help='투입 총량 상한 배수 (출퇴근 등 교통량 많은 시각은 크게)')
    a = ap.parse_args()
    if a.density is None:
        a.density = 12.0 if a.calib2 else 30.0
    global TLS_ACT
    TLS_ACT = a.tls_act
    global TTT, JB, CALIB_GAIN, TR_DMIN, TR_DMAX, MEAS, PASSES
    MEAS, PASSES = ((a.measure or (8 if a.fast else 10)) if a.calib2 else None), max(1, a.passes or (1 if a.fast else 3))
    CALIB_GAIN, TR_DMIN, TR_DMAX = a.gain, a.tr_min, a.tr_max
    if a.unjam:
        TTT = 60
        JB = ['--ignore-junction-blocker', '15']
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
    global PPL, MAX_SCALE, FCD_DT
    MAX_SCALE = a.max_scale
    FCD_DT = max(1, a.fcd_dt)
    ppl_t = None
    if a.ppl_file:
        try:
            PPL, ppl_t = load_ppl(a.ppl_file, a.hotspots or os.path.join(a.site, 'hotspots.json'), a.ppl_at)
            log('실시간 인구 %d곳 불러옴 (%s)' % (len(PPL), ppl_t))
        except Exception as e:
            log('실시간 인구 불러오기 실패 → 인구 반영 없이:', e)
            PPL = None
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
        meta = {'ppl': ppl_t if PPL else None, 'calib': None, 'gen': (datetime.datetime.utcnow() + datetime.timedelta(hours=9)).strftime('%Y-%m-%d %H:%M')}
        if a.ppl_at:
            meta['fc'] = a.ppl_at   # 예측 시뮬레이션 대상 시각
        PPL_HIT.clear()
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
        M = max(a.minutes, MEAS or a.minutes) if a.calib2 else a.minutes   # 교통량 재는 시간(분)
        if spots:
            log('교통량 지점 %d곳 (%s %s시 기준)으로 보정' % (len(spots), counts['ymd'], counts['hh']))
            # --fast: 2차 보정 대신, 1차 보정에서 보통 모자라는 만큼(약 0.55배) 미리 목표를 높여 한 번에 맞춤
            pre = {s_['id']: a.fast_boost for s_ in spots} if a.fast else None
            rou = (calibrate2 if a.calib2 else calibrate)(home, net, work, a.warmup + (M if a.calib2 else a.minutes), a.density, a.seed, dict(counts, spots=spots), pre if not a.calib2 else None)
        else:
            log('이 구에는 교통량 지점이 없거나 받지 못함 → 도로 규모 기준 수요만 사용')
            rou = make_demand(home, net, work, a.warmup + M, a.density, a.seed)
        fcd = simulate(home, net, rou, work, a.minutes, a.warmup)
        if spots and a.calib2:
            # 반복 보정: 도로마다 (관측 ÷ 지난번 시뮬)를 목표 배수에 곱해 다시 → 가장 오차 작은 결과를 씀
            obs_sum = max(1, sum(s_['in'] + s_['out'] for s_ in spots))
            res = measure_spots(os.path.join(work, 'meandata.xml'), spots, a.warmup, M)
            best_err = sum(abs(r['sim'] - r['obs']) for r in res)
            best_fcd = os.path.join(work, 'fcd_best.xml'); shutil.copy(fcd, best_fcd)
            errs = [100 * best_err / obs_sum]
            eb = {}
            for k in range(2, PASSES + 1):
                er = edge_rates(os.path.join(work, 'meandata.xml'), M)
                for e in TARGET:
                    if er.get(e, 0) > 30 and TARGET[e] > 0:
                        eb[e] = min(3.0, max(0.3, eb.get(e, 1.0) * min(1.8, max(0.55, TARGET[e] / er[e]))))
                rou_k = calibrate2(home, net, work, a.warmup + M, a.density, a.seed, dict(counts, spots=spots), None, eboost=eb, tag=str(k))
                fcd = simulate(home, net, rou_k, work, a.minutes, a.warmup)
                res_k = measure_spots(os.path.join(work, 'meandata.xml'), spots, a.warmup, M)
                err_k = sum(abs(r['sim'] - r['obs']) for r in res_k)
                errs.append(100 * err_k / obs_sum)
                if err_k < best_err:
                    best_err, res = err_k, res_k
                    shutil.copy(fcd, best_fcd)
            fcd = best_fcd
            log('새 보정 오차 %s %%' % ' → '.join('%d' % e for e in errs))
            meta['calib'] = {'ymd': counts['ymd'], 'hh': counts['hh'], 'spots': res}
            log('보정 결과: 시뮬/관측 합계 %d%%' % (100 * sum(r['sim'] for r in res) / max(1, sum(r['obs'] for r in res))))
        elif spots and a.fast:
            res = measure_spots(os.path.join(work, 'meandata.xml'), spots, a.warmup, a.minutes)
            meta['calib'] = {'ymd': counts['ymd'], 'hh': counts['hh'], 'spots': res}
            log('보정 결과(1차만): 시뮬/관측 합계 %d%%' % (100 * sum(r['sim'] for r in res) / max(1, sum(r['obs'] for r in res))))
        elif spots:
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
        if PPL_HIT:
            meta['pplTop'] = [[k, int(v)] for k, v in sorted(PPL_HIT.items(), key=lambda t: -t[1])[:6]]
        out = a.out.replace('{gu}', code) if a.out else os.path.join(a.site, 'sim', code + '.json')
        pack(fcd, out, code, a.minutes, net, a.warmup + (M - a.minutes), meta, gu_shape_of(os.path.join(a.site, 'seoul_gu.json'), code))
        os.remove(fcd)
    log('완료. 3D 지도에서 [시뮬레이션] 레이어를 켜세요.')


if __name__ == '__main__':
    main()
