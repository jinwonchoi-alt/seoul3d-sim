"""도로망 자동 보정 (구별 net.xml.gz 를 고쳐서 다시 만듦)

1) 끊긴 도로 잇기: 같은 이름의 도로 끝점이 40m 안에 있고 같은 방향으로 이어지면 연결
   (예: 서부간선지하도로가 양쪽 끝에서 지상 서부간선도로와 끊겨 있어 차가 한 대도 못 들어갔음)
2) 차로 수 빈칸 메우기: 앞뒤가 같은 도로인데 중간 짧은 구간(200m 미만)만 차로가 2개 이상 적으면
   (원본 자료에 차로 수가 비어 1차로로 들어간 경우) 앞뒤 중 작은 쪽 차로 수로 맞춤

사용: python fix_net.py <입력 net.xml.gz> <출력 net.xml.gz>
"""
import os, sys, math, gzip, shutil, subprocess, tempfile
import xml.etree.ElementTree as ET

SNAP_M = 40.0
GAP_LEN = 200.0


def sumo_bin(name):
    try:
        import sumo
        p = os.path.join(sumo.SUMO_HOME, 'bin', name + ('.exe' if os.name == 'nt' else ''))
        if os.path.exists(p):
            return p
    except Exception:
        pass
    return name


def heading(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def angdiff(x, y):
    d = abs(x - y) % (2 * math.pi)
    return min(d, 2 * math.pi - d)


def fix(net_in, net_out, log=print):
    tmp = tempfile.mkdtemp(prefix='fixnet_')
    pre = os.path.join(tmp, 'p')
    subprocess.run([sumo_bin('netconvert'), '-s', net_in, '--plain-output-prefix', pre, '--no-warnings'],
                   check=True, stdout=subprocess.DEVNULL)
    nt = ET.parse(pre + '.nod.xml'); et = ET.parse(pre + '.edg.xml')
    nodes = {n.get('id'): n for n in nt.getroot().iter('node')}
    xy = {k: (float(n.get('x')), float(n.get('y'))) for k, n in nodes.items()}
    edges = [e for e in et.getroot().iter('edge')]
    E = {e.get('id'): e for e in edges}

    def shape(e):
        s = e.get('shape')
        if s:
            return [tuple(map(float, p.split(',')[:2])) for p in s.split()]
        return [xy[e.get('from')], xy[e.get('to')]]

    def build_deg():
        din, dout = {}, {}
        for e in edges:
            dout.setdefault(e.get('from'), []).append(e)
            din.setdefault(e.get('to'), []).append(e)
        return din, dout

    # ---- 1) 끊긴 도로 잇기: '들어오기만 하는 점'(막다른 끝)과 '나가기만 하는 점'(시작점)을
    #         40m 안의 같은 이름·같은 방향 도로가 이어지는 점으로 옮겨 붙임
    def ang_in(e):
        p = shape(e); return heading(p[-2], p[-1])
    def ang_out(e):
        p = shape(e); return heading(p[0], p[1])
    def move_end(e, old, new_, which):
        e.set(which, new_)
        if e.get('shape'):
            pts = shape(e)
            if which == 'to': pts[-1] = xy[new_]
            else: pts[0] = xy[new_]
            e.set('shape', ' '.join('%.2f,%.2f' % p for p in pts))
    snapped = 0
    for _ in range(2):
        din, dout = build_deg()
        for k in list(xy):
            ins, outs = din.get(k, []), dout.get(k, [])
            if ins and not outs:          # 막다른 끝 → 같은 이름 도로가 '나가는' 점에 붙임
                named = [e for e in ins if (e.get('name') or '') not in ('', '-')]
                if not named: continue
                best = None
                for j, o2 in dout.items():
                    if j == k or not o2: continue
                    d = math.dist(xy[k], xy[j])
                    if d > SNAP_M or (best and d >= best[0]): continue
                    if any((n.get('name') or '') == (e.get('name') or '') and angdiff(ang_out(n), ang_in(e)) < math.radians(50) for e in named for n in o2):
                        best = (d, j)
                if best:
                    for e in ins: move_end(e, k, best[1], 'to')
                    snapped += 1; log('  잇기 %s(%s) 끝 → 노드 %s (%.0fm)' % (named[0].get('name'), k, best[1], best[0]))
                    din, dout = build_deg()
            elif outs and not ins:        # 시작점 → 같은 이름 도로가 '들어오는' 점에 붙임
                named = [e for e in outs if (e.get('name') or '') not in ('', '-')]
                if not named: continue
                best = None
                for j, i2 in din.items():
                    if j == k or not i2: continue
                    d = math.dist(xy[k], xy[j])
                    if d > SNAP_M or (best and d >= best[0]): continue
                    if any((p.get('name') or '') == (e.get('name') or '') and angdiff(ang_in(p), ang_out(e)) < math.radians(50) for e in named for p in i2):
                        best = (d, j)
                if best:
                    for e in outs: move_end(e, k, best[1], 'from')
                    snapped += 1; log('  잇기 %s(%s) 시작 → 노드 %s (%.0fm)' % (named[0].get('name'), k, best[1], best[0]))
                    din, dout = build_deg()

    # ---- 2) 차로 수 빈칸 메우기
    din, dout = build_deg()
    filled = 0
    for e in edges:
        nm = e.get('name') or ''
        if not nm or nm == '-':
            continue
        L = int(e.get('numLanes', '1'))
        ln = sum(math.dist(a, b) for a, b in zip(shape(e), shape(e)[1:]))
        if ln >= GAP_LEN:
            continue
        pre_ = [p for p in din.get(e.get('from'), []) if (p.get('name') or '') == nm and p is not e]
        nxt = [n for n in dout.get(e.get('to'), []) if (n.get('name') or '') == nm and n is not e]
        if not pre_ or not nxt:
            continue
        a = max(int(p.get('numLanes', '1')) for p in pre_); b = max(int(n.get('numLanes', '1')) for n in nxt)
        want = min(a, b)
        if L <= want - 2:
            e.set('numLanes', str(want))
            for ln_el in list(e.findall('lane')):   # 차로별 설정이 있으면 지움 (새 차로 수로 다시 만들게)
                e.remove(ln_el)
            filled += 1; log('  차로 %s %s %d → %d' % (e.get('id'), nm, L, want))

    # 안 쓰게 된 노드 지우기
    used = set()
    for e in edges:
        used.add(e.get('from')); used.add(e.get('to'))
    for k, n in list(nodes.items()):
        if k not in used:
            nt.getroot().remove(n)
    nt.write(pre + '.nod.xml', encoding='utf-8'); et.write(pre + '.edg.xml', encoding='utf-8')
    if not snapped and not filled:
        shutil.copy(net_in, net_out); shutil.rmtree(tmp, ignore_errors=True)
        return 0, 0
    out_plain = os.path.join(tmp, 'out.net.xml')
    subprocess.run([sumo_bin('netconvert'), '-n', pre + '.nod.xml', '-e', pre + '.edg.xml', '--ignore-errors.edge-type',
                    '--tls.default-type', 'actuated', '--tls.cycle.time', '150', '--no-turnarounds.except-deadend',
                    '--output.street-names', '--ignore-errors', '--no-warnings', '-o', out_plain],
                   check=True, stdout=subprocess.DEVNULL)
    if net_out.endswith('.gz'):
        with open(out_plain, 'rb') as fi, gzip.open(net_out, 'wb', compresslevel=9) as fo:
            shutil.copyfileobj(fi, fo)
    else:
        shutil.copy(out_plain, net_out)
    shutil.rmtree(tmp, ignore_errors=True)
    return snapped, filled


if __name__ == '__main__':
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    s, f = fix(sys.argv[1], sys.argv[2])
    print('연결 %d곳, 차로 보정 %d곳' % (s, f))
