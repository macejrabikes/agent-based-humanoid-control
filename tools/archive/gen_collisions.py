#!/usr/bin/env python3
"""
Vygeneruje jednoduche collision tvary (box / valec) z vizualnych meshov
v humanoid.urdf.xacro.

Pouzitie:
  python gen_collisions.py            # dry-run: vypise tabulku, nic nemeni
  python gen_collisions.py --apply    # zaloha + prepis collision blokov
"""
import argparse
import os
import shutil
import sys

import numpy as np
import trimesh
from lxml import etree

# linky, ktore nechavame na pokoji (senzory, virtualne linky)
SKIP = ('imu', 'logo', 'd435', 'mid360', 'base_link', 'world')
MIN_SIZE = 0.01  # minimalny rozmer v metroch


def rpy_to_matrix(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def floats(text, default):
    return [float(v) for v in text.split()] if text else default


def fmt(values):
    return " ".join(f"{v:.4f}" for v in values)


def resolve_mesh(filename, pkg_dir):
    path = filename
    if path.startswith('package://'):
        path = path[len('package://'):]
        path = path.split('/', 1)[1]  # zahod nazov balika
    return os.path.join(pkg_dir, path)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_pkg = os.path.join(
        here, '..', 'src', 'humanoid_description')
    ap = argparse.ArgumentParser()
    ap.add_argument('--pkg', default=default_pkg,
                    help='cesta k baliku humanoid_description')
    ap.add_argument('--xacro', default='urdf/humanoid.urdf.xacro',
                    help='xacro relativne k balíku')
    ap.add_argument('--apply', action='store_true',
                    help='skutocne prepise subor (inak dry-run)')
    args = ap.parse_args()

    pkg_dir = os.path.abspath(args.pkg)
    xacro_path = os.path.join(pkg_dir, args.xacro)
    if not os.path.exists(xacro_path):
        sys.exit(f'Nenasiel som {xacro_path}')

    tree = etree.parse(xacro_path)
    root = tree.getroot()

    print(f"{'link':32s} {'tvar':9s} {'rozmery [cm]':28s}")
    print('-' * 72)
    changed = 0

    for link in root.findall('link'):
        name = link.get('name', '')
        if any(s in name for s in SKIP):
            continue
        visual = link.find('visual')
        if visual is None:
            continue
        mesh_el = visual.find('geometry/mesh')
        if mesh_el is None:
            continue

        mesh_path = resolve_mesh(mesh_el.get('filename'), pkg_dir)
        if not os.path.exists(mesh_path):
            print(f'{name:32s} CHYBA: chyba mesh {mesh_path}')
            continue

        mesh = trimesh.load(mesh_path, force='mesh')
        scale = np.array(floats(mesh_el.get('scale'), [1.0, 1.0, 1.0]))
        verts = np.asarray(mesh.vertices) * scale

        origin = visual.find('origin')
        xyz = np.array(floats(origin.get('xyz') if origin is not None else None,
                              [0.0, 0.0, 0.0]))
        rpy = floats(origin.get('rpy') if origin is not None else None,
                     [0.0, 0.0, 0.0])
        verts = verts @ rpy_to_matrix(*rpy).T + xyz

        lo, hi = verts.min(axis=0), verts.max(axis=0)
        center = (lo + hi) / 2.0
        size = np.maximum(hi - lo, MIN_SIZE)

        axis = int(np.argmax(size))
        others = np.delete(size, axis)

        if size[axis] > 1.6 * others.max():
            radius = 0.5 * float(np.mean(others))
            length = float(size[axis])
            cyl_rpy = {0: (0.0, np.pi / 2, 0.0),
                       1: (-np.pi / 2, 0.0, 0.0),
                       2: (0.0, 0.0, 0.0)}[axis]
            shape = 'valec'
            geom = f'<cylinder radius="{radius:.4f}" length="{length:.4f}"/>'
            col_rpy = cyl_rpy
            dims = f'r={radius*100:.1f}  dlzka={length*100:.1f}  os={"xyz"[axis]}'
        else:
            shape = 'box'
            geom = f'<box size="{fmt(size)}"/>'
            col_rpy = (0.0, 0.0, 0.0)
            dims = f'{size[0]*100:.1f} x {size[1]*100:.1f} x {size[2]*100:.1f}'

        print(f'{name:32s} {shape:9s} {dims:28s}')

        if not args.apply:
            continue

        frag = etree.fromstring(
            '<collision>\n'
            f'      <origin xyz="{fmt(center)}" rpy="{fmt(col_rpy)}"/>\n'
            '      <geometry>\n'
            f'        {geom}\n'
            '      </geometry>\n'
            '    </collision>')

        old = link.findall('collision')
        if old:
            idx = link.index(old[0])
            frag.tail = old[-1].tail
            for c in old:
                link.remove(c)
            link.insert(idx, frag)
        else:
            idx = link.index(visual) + 1
            frag.tail = visual.tail
            visual.tail = '\n    '
            link.insert(idx, frag)
        changed += 1

    if not args.apply:
        print('\nDry-run: nic sa nezmenilo. Ak tabulka vyzera rozumne, '
              'spusti znova s --apply.')
        return

    backup_dir = os.path.expanduser('~/backup_urdf')
    os.makedirs(backup_dir, exist_ok=True)
    backup = os.path.join(backup_dir, 'humanoid.before_collisions.xacro')
    shutil.copy(xacro_path, backup)
    tree.write(xacro_path, xml_declaration=True, encoding='utf-8')
    print(f'\nHotovo: prepisanych {changed} linkov. Zaloha: {backup}')


if __name__ == '__main__':
    main()
