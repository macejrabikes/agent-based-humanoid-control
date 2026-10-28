#!/usr/bin/env python3
"""gen_world.py: generator svetov pre Gazebo (Chappie) z ASCII mapy alebo nahodne.

Pouzitie (z korena repozitara):
  python3 tools/gen_world.py --ascii maps/izba.txt --name izba          # svet z ASCII mapy
  python3 tools/gen_world.py --random rooms --seed 3 --name rooms_3     # nahodne izby s dverami
  python3 tools/gen_world.py --random maze --seed 5 --name maze_5       # bludisko
  python3 tools/gen_world.py --random warehouse --seed 1 --name sklad   # sklad s regalmi
Potom spusti simulaciu so svetom:
  ros2 launch humanoid_gazebo walk_demo.launch.py world:=rooms_3.sdf

Subor sa zapise do src/humanoid_gazebo/worlds/<name>.sdf (+ <name>.txt, ASCII mapa na upravu/znovu pouzitie) a
automaticky aj do install/humanoid_gazebo/share/humanoid_gazebo/worlds/ (ak existuje), takze netreba colcon build.

Znaky mapy (1 znak = 0.5 x 0.5 m, horny riadok = sever):
  #  stena (vyska 1.5 m)        =  nizka stena (0.6 m)       .  volne (aj medzera = dvere)
  S  start robota (musi byt prave jeden; svet sa posunie tak, ze S je v pocitku suradnic, kde robota drzi zaves)
  B  oranzova skrina 0.9 m      R  cervena kocka 0.6 m       C  zeleny valec (r 0.45, v 1.5 m)
  P  ruzovy valec (r 0.45, v 1.0 m)   T  siv stlp 0.4 m (v 1.5 m)
  p  tlacitelna modra skrina 0.6x0.6x0.9 m, 3 kg (dynamicka)
Mena modelov: wall_*, lowwall_*, box_B_*, ... (podla znaku) - agent ich vidi len cez kameru/lidar.
"""
import argparse
import os
import random
import re
import sys
from collections import deque

CELL = 0.5
WALL_H, LOW_H = 1.5, 0.6
COL = {"wall": (0.85, 0.85, 0.88), "low": (0.6, 0.6, 0.2), "B": (0.9, 0.4, 0.1), "R": (0.85, 0.15, 0.15),
       "C": (0.3, 0.7, 0.3), "P": (0.7, 0.2, 0.5), "T": (0.4, 0.4, 0.45), "p": (0.2, 0.5, 0.8)}


def mat(c):
    s = "%.2f %.2f %.2f 1" % c
    return "<material><ambient>%s</ambient><diffuse>%s</diffuse></material>" % (s, s)


def static_model(name, x, y, z, geom, colour):
    return ('    <model name="%s"><static>true</static><pose>%.3f %.3f %.3f 0 0 0</pose><link name="link">'
            '<collision name="collision"><geometry>%s</geometry></collision>'
            '<visual name="visual"><geometry>%s</geometry>%s</visual></link></model>\n' % (name, x, y, z, geom, geom, mat(colour)))


def box(sx, sy, sz):
    return "<box><size>%.3f %.3f %.3f</size></box>" % (sx, sy, sz)


def cyl(r, h):
    return "<cylinder><radius>%.3f</radius><length>%.3f</length></cylinder>" % (r, h)


def dyn_box(name, x, y, sx, sy, sz, mass, colour):
    ix, iy, iz = [mass / 12.0 * (a * a + b * b) for a, b in ((sy, sz), (sx, sz), (sx, sy))]
    g = box(sx, sy, sz)
    return ('    <model name="%s"><pose>%.3f %.3f %.3f 0 0 0</pose><link name="link">'
            '<inertial><mass>%g</mass><inertia><ixx>%.5f</ixx><iyy>%.5f</iyy><izz>%.5f</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
            '<collision name="collision"><geometry>%s</geometry><surface><friction><ode><mu>0.4</mu><mu2>0.4</mu2></ode></friction></surface></collision>'
            '<visual name="visual"><geometry>%s</geometry>%s</visual></link></model>\n' % (name, x, y, sz / 2 + 0.001, mass, ix, iy, iz, g, g, mat(colour)))


# ---------------------------------------------------------------- ASCII -> SDF
def parse_map(text):
    rows = [r.rstrip("\n") for r in text.split("\n")]
    while rows and not rows[-1].strip():
        rows.pop()
    w = max(len(r) for r in rows)
    grid = [list(r.ljust(w, ".")) for r in rows]
    starts = [(r, c) for r, row in enumerate(grid) for c, ch in enumerate(row) if ch == "S"]
    if len(starts) != 1:
        sys.exit("mapa musi mat prave jeden znak S (nasiel som %d)" % len(starts))
    return grid, starts[0]


def build_models(grid, start):
    sr, sc = start
    X = lambda c: (c - sc) * CELL
    Y = lambda r: (sr - r) * CELL
    out, n = [], {}
    for r, row in enumerate(grid):
        c = 0
        while c < len(row):
            ch = row[c]
            if ch in "#=":                       # zluci vodorovne rady do jednej steny
                c2 = c
                while c2 + 1 < len(row) and row[c2 + 1] == ch:
                    c2 += 1
                length = (c2 - c + 1) * CELL
                x = (X(c) + X(c2)) / 2.0
                h = WALL_H if ch == "#" else LOW_H
                key = "wall" if ch == "#" else "lowwall"
                n[key] = n.get(key, 0) + 1
                out.append(static_model("%s_%d" % (key, n[key]), x, Y(r), h / 2, box(length, CELL, h), COL["wall" if ch == "#" else "low"]))
                c = c2 + 1
                continue
            if ch in "BRCPTp":
                n[ch] = n.get(ch, 0) + 1
                name, x, y = "obj_%s_%d" % (ch, n[ch]), X(c), Y(r)
                if ch == "B":
                    out.append(static_model(name, x, y, 0.45, box(0.9, 0.9, 0.9), COL[ch]))
                elif ch == "R":
                    out.append(static_model(name, x, y, 0.3, box(0.6, 0.6, 0.6), COL[ch]))
                elif ch == "C":
                    out.append(static_model(name, x, y, 0.75, cyl(0.45, 1.5), COL[ch]))
                elif ch == "P":
                    out.append(static_model(name, x, y, 0.5, cyl(0.45, 1.0), COL[ch]))
                elif ch == "T":
                    out.append(static_model(name, x, y, 0.75, box(0.4, 0.4, 1.5), COL[ch]))
                else:
                    out.append(dyn_box(name, x, y, 0.6, 0.6, 0.9, 3.0, COL[ch]))
            c += 1
    return "".join(out), n


def load_template(path):
    s = open(path, encoding="utf-8").read()
    head = s[:s.index("    <!-- obvodove steny")]
    gantry = re.search(r'    <!-- zaves.*?</model>\n', s, re.S)
    if not gantry:
        sys.exit("v sablone nie je blok zaves (gantry): %s" % path)
    return head, gantry.group(0)


def write_world(name, grid, start, template, ws):
    models, counts = build_models(grid, start)
    head, gantry = load_template(template)
    sdf = head + "\n" + gantry + "\n    <!-- vygenerovane: gen_world.py -->\n" + models + "  </world>\n</sdf>\n"
    txt = "\n".join("".join(r) for r in grid) + "\n"
    dirs = [os.path.join(ws, "src/humanoid_gazebo/worlds"),
            os.path.join(ws, "install/humanoid_gazebo/share/humanoid_gazebo/worlds")]
    for d in dirs:
        if os.path.isdir(d) and not os.path.islink(d):
            with open(os.path.join(d, name + ".sdf"), "w", encoding="utf-8") as f:
                f.write(sdf)
            if d == dirs[0]:
                with open(os.path.join(d, name + ".txt"), "w", encoding="utf-8") as f:
                    f.write(txt)
            print("zapisane:", os.path.join(d, name + ".sdf"))
    return counts


# ---------------------------------------------------------------- nahodne mapy
def empty_grid(w, h):
    g = [["." for _ in range(w)] for _ in range(h)]
    for c in range(w):
        g[0][c] = g[h - 1][c] = "#"
    for r in range(h):
        g[r][0] = g[r][w - 1] = "#"
    return g


def wall_dist(g):
    """Chebyshev vzdialenost (v bunkach) od najblizsej steny/nizkej steny"""
    h, w = len(g), len(g[0])
    d = [[99] * w for _ in range(h)]
    q = deque()
    for r in range(h):
        for c in range(w):
            if g[r][c] in "#=":
                d[r][c] = 0
                q.append((r, c))
    while q:
        r, c = q.popleft()
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                rr, cc = r + dr, c + dc
                if 0 <= rr < h and 0 <= cc < w and d[rr][cc] > d[r][c] + 1:
                    d[rr][cc] = d[r][c] + 1
                    q.append((rr, cc))
    return d


def place_objects(g, rng, start, count, kinds, min_wall=3, min_start=6, min_gap=5, avoid=()):
    """polozi `count` objektov na volne bunky ďaleko od stien, startu, dveri a navzajom"""
    d = wall_dist(g)
    h, w = len(g), len(g[0])
    cand = [(r, c) for r in range(h) for c in range(w)
            if g[r][c] == "." and d[r][c] >= min_wall and max(abs(r - start[0]), abs(c - start[1])) >= min_start
            and all(max(abs(r - a), abs(c - b)) >= 4 for a, b in avoid)]
    rng.shuffle(cand)
    placed = []
    for r, c in cand:
        if len(placed) >= count:
            break
        if all(max(abs(r - a), abs(c - b)) >= min_gap for a, b in placed):
            g[r][c] = rng.choice(kinds)
            placed.append((r, c))
    return placed


def gen_rooms(rng, size_m):
    n = int(size_m / CELL)
    g = empty_grid(n, n)
    doors = []

    def split(r0, c0, r1, c1, depth):               # obdlznik vnutri [r0..r1] x [c0..c1]
        hh, ww = r1 - r0 + 1, c1 - c0 + 1
        if (hh < 18 and ww < 18) or depth > 4:
            return
        horiz = hh >= ww if rng.random() < 0.7 else ww > hh
        if horiz and hh >= 18:
            r = rng.randint(r0 + 8, r1 - 8)
            for c in range(c0, c1 + 1):
                g[r][c] = "#"
            dc = rng.randint(c0 + 2, c1 - 4)
            for c in range(dc, dc + 3):
                g[r][c] = "."
            doors.append((r, dc + 1))
            split(r0, c0, r - 1, c1, depth + 1)
            split(r + 1, c0, r1, c1, depth + 1)
        elif not horiz and ww >= 18:
            c = rng.randint(c0 + 8, c1 - 8)
            for r in range(r0, r1 + 1):
                g[r][c] = "#"
            dr = rng.randint(r0 + 2, r1 - 4)
            for r in range(dr, dr + 3):
                g[r][c] = "."
            doors.append((dr + 1, c))
            split(r0, c0, r1, c - 1, depth + 1)
            split(r0, c + 1, r1, c1, depth + 1)
    split(1, 1, n - 2, n - 2, 0)
    d = wall_dist(g)
    spots = [(r, c) for r in range(n) for c in range(n) if g[r][c] == "." and d[r][c] >= 4
             and all(max(abs(r - a), abs(c - b)) >= 5 for a, b in doors)]
    start = rng.choice(spots)
    g[start[0]][start[1]] = "S"
    kinds = ["B", "B", "R", "C", "P", "T", "p"]
    place_objects(g, rng, start, max(6, n // 4), kinds, avoid=doors)
    return g, start


def gen_maze(rng, size_m):
    m = max(3, int(size_m / (4 * CELL)))             # m x m buniek, chodba 3 bunky (1.5 m) + stena 1 bunka
    n = m * 4 + 1
    g = [["#"] * n for _ in range(n)]

    def carve(i, j):
        for r in range(i * 4 + 1, i * 4 + 4):
            for c in range(j * 4 + 1, j * 4 + 4):
                g[r][c] = "."
    seen, stack, deadend = set(), [(0, 0)], {}
    seen.add((0, 0))
    carve(0, 0)
    order = []
    while stack:
        i, j = stack[-1]
        nb = [(i + a, j + b) for a, b in ((1, 0), (-1, 0), (0, 1), (0, -1))
              if 0 <= i + a < m and 0 <= j + b < m and (i + a, j + b) not in seen]
        if not nb:
            stack.pop()
            continue
        ni, nj = rng.choice(nb)
        carve(ni, nj)
        # prerazi stenu medzi (i,j) a (ni,nj)
        for r in range(min(i, ni) * 4 + 1, max(i, ni) * 4 + 4):
            for c in range(min(j, nj) * 4 + 1, max(j, nj) * 4 + 4):
                g[r][c] = "."
        deadend[(i, j)] = deadend.get((i, j), 0) + 1
        deadend[(ni, nj)] = deadend.get((ni, nj), 0) + 1
        seen.add((ni, nj))
        order.append((ni, nj))
        stack.append((ni, nj))
    start = (2, 2)
    g[2][2] = "S"
    ends = [k for k, v in deadend.items() if v == 1 and k != (0, 0)]
    rng.shuffle(ends)
    kinds = ["B", "R", "P", "T", "p"]
    if ends:                                       # cielovy zeleny valec v najvzdialenejsej slepej uličke
        far = max(ends, key=lambda k: k[0] + k[1])
        g[far[0] * 4 + 2][far[1] * 4 + 2] = "C"
        ends.remove(far)
    for (i, j) in ends[:max(2, len(ends) // 2)]:
        g[i * 4 + 2][j * 4 + 2] = rng.choice(kinds)
    return g, start


def gen_warehouse(rng, size_m):
    w, h = int(size_m * 1.5 / CELL), int(size_m / CELL)
    g = empty_grid(w, h)
    for c in range(10, w - 8, 7):                  # rady regalov (nizke steny) s uzkymi uličkami 3 bunky = 1.5 m
        for r in range(4, h - 4):
            g[r][c] = "="
            g[r][c + 1] = "="
        for _ in range(2):                          # prechody medzi radmi
            gap = rng.randint(6, h - 10)
            for r in range(gap, gap + 3):
                g[r][c] = g[r][c + 1] = "."
    start = (h // 2, 4)
    g[start[0]][start[1]] = "S"
    place_objects(g, rng, start, max(8, w // 6), ["B", "R", "C", "P", "T", "p"], min_wall=2, min_start=5, min_gap=4)
    return g, start


GEN = {"rooms": gen_rooms, "maze": gen_maze, "warehouse": gen_warehouse}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--ascii", metavar="SUBOR", help="ASCII mapa")
    src.add_argument("--random", choices=sorted(GEN), help="nahodny svet: rooms | maze | warehouse")
    ap.add_argument("--name", required=True, help="nazov sveta (bez .sdf)")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--size", type=float, default=20.0, help="velkost nahodneho sveta v metroch (predvolene 20)")
    ap.add_argument("--ws", default=os.environ.get("CHAPPIE_WS", "~/agent-based-humanoid-control"))
    ap.add_argument("--template", default=None, help="svet, z ktoreho sa vezme hlavicka (predvolene arena_big_walk.sdf)")
    a = ap.parse_args()
    ws = os.path.expanduser(a.ws)
    template = a.template or os.path.join(ws, "src/humanoid_gazebo/worlds/arena_big_walk.sdf")
    if a.ascii:
        grid, start = parse_map(open(a.ascii, encoding="utf-8").read())
    else:
        seed = a.seed if a.seed is not None else random.randrange(10 ** 6)
        print("seed:", seed)
        grid, start = GEN[a.random](random.Random(seed), a.size)
    counts = write_world(a.name, grid, start, template, ws)
    print("\n".join("".join(r) for r in grid))
    print("objekty:", counts)


if __name__ == "__main__":
    main()
