"""
Prevedie Unitree G1 URDF (s rukami + bez rukami) na Xacro rozdelene na:

  humanoid.urdf.xacro  - telo (29 klbov), argumenty with_hands / fixed_base,
                         base_link, nazov robota
  g1.hands.xacro       - ruky s prstami (with_hands:=true) ALEBO
                         gumovy koncovy clanok (with_hands:=false)

Pouzitie:
  python gen_g1_xacro.py
  python gen_g1_xacro.py --name chappie
"""
import argparse
import copy
import os
import sys

from lxml import etree

XACRO_NS = 'http://www.ros.org/wiki/xacro'
PKG = 'humanoid_description'


def X(tag, **attrs):
    """Element v xacro namespace (xacro:if, xacro:arg, ...)."""
    return etree.Element(f'{{{XACRO_NS}}}{tag}', **attrs)


def load(path):
    if not os.path.exists(path):
        sys.exit(f'Nenasiel som {path}')
    return etree.parse(path).getroot()


def drop_mujoco(root):
    for m in root.findall('mujoco'):
        root.remove(m)


def subtree(root, start_link):
    """Vrati (linky, klby) pod danym linkom, vratane start_link."""
    children = {}
    for j in root.findall('joint'):
        children.setdefault(j.find('parent').get('link'), []).append(j)
    link_by_name = {l.get('name'): l for l in root.findall('link')}
    links, joints = [], []
    stack = [start_link]
    while stack:
        name = stack.pop()
        links.append(link_by_name[name])
        for j in children.get(name, []):
            joints.append(j)
            stack.append(j.find('child').get('link'))
    return links, joints


def hand_parts(root, side):
    """Palm joint + vsetko pod nim (linky aj klby) pre jednu ruku."""
    pj_name = f'{side}_hand_palm_joint'
    pj = None
    for j in root.findall('joint'):
        if j.get('name') == pj_name:
            pj = j
            break
    if pj is None:
        sys.exit(f'V URDF chyba klb {pj_name}')
    links, joints = subtree(root, pj.find('child').get('link'))
    return [pj] + joints + links


def count_revolute(elems):
    return sum(1 for e in elems
               if e.tag == 'joint' and e.get('type') == 'revolute')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', default=os.path.expanduser(
        '~/refs/unitree_ros/robots/g1_description'))
    ap.add_argument('--pkg', default=os.path.expanduser(
        '~/agent-based-humanoid-control/src/humanoid_description'))
    ap.add_argument('--with-hand', default='g1_29dof_with_hand_rev_1_0.urdf')
    ap.add_argument('--no-hand', default='g1_29dof_rev_1_0.urdf')
    ap.add_argument('--name', default='chappie')
    args = ap.parse_args()

    with_hand = load(os.path.join(args.src, args.with_hand))
    no_hand = load(os.path.join(args.src, args.no_hand))
    drop_mujoco(with_hand)
    drop_mujoco(no_hand)

    # --- ruky: zober z oboch URDF ---
    parts_true, parts_false = [], []
    for side in ('left', 'right'):
        parts_true += hand_parts(with_hand, side)
        parts_false += hand_parts(no_hand, side)

    xif = X('if', value='$(arg with_hands)')
    for e in parts_true:
        xif.append(copy.deepcopy(e))
        with_hand.remove(e)          # z tela ich vyhod

    xunless = X('unless', value='$(arg with_hands)')
    for e in parts_false:
        xunless.append(copy.deepcopy(e))

    hands = etree.Element('robot', nsmap={'xacro': XACRO_NS})
    hands.append(xif)
    hands.append(xunless)

    # --- koren tela ---
    child_links = {j.find('child').get('link') for j in with_hand.findall('joint')}
    roots = [l.get('name') for l in with_hand.findall('link')
             if l.get('name') not in child_links]
    if len(roots) != 1:
        sys.exit(f'Ocakaval som jeden korenovy link, nasiel som: {roots}')
    root_link = roots[0]

    # --- nove telo: hlavicka + base_link + povodny obsah ---
    body = etree.Element('robot', name=args.name, nsmap={'xacro': XACRO_NS})

    head = [
        X('arg', name='with_hands', default='false'),
        X('arg', name='fixed_base', default='false'),
        X('include', filename='g1.hands.xacro'),
    ]

    base = etree.Element('link', name='base_link')
    bj = etree.Element('joint', name='base_joint', type='fixed')
    etree.SubElement(bj, 'parent', link='base_link')
    etree.SubElement(bj, 'child', link=root_link)

    fx = X('if', value='$(arg fixed_base)')
    etree.SubElement(fx, 'link', name='world')
    wj = etree.SubElement(fx, 'joint', name='world_joint', type='fixed')
    etree.SubElement(wj, 'parent', link='world')
    etree.SubElement(wj, 'child', link='base_link')

    for e in head + [base, bj, fx]:
        body.append(e)
    for e in list(with_hand):
        body.append(e)

    # --- cesty k meshom: meshes/x.STL -> package://humanoid_description/meshes/x.STL ---
    for tree in (body, hands):
        for m in tree.iter('mesh'):
            fn = m.get('filename')
            if fn and not fn.startswith('package://'):
                m.set('filename', f'package://{PKG}/{fn}')

    for tree in (body, hands):
        etree.indent(tree, space='  ')

    urdf_dir = os.path.join(args.pkg, 'urdf')
    os.makedirs(urdf_dir, exist_ok=True)
    body_path = os.path.join(urdf_dir, 'humanoid.urdf.xacro')
    hands_path = os.path.join(urdf_dir, 'g1.hands.xacro')
    etree.ElementTree(body).write(body_path, xml_declaration=True, encoding='utf-8')
    etree.ElementTree(hands).write(hands_path, xml_declaration=True, encoding='utf-8')

    n_body = sum(1 for j in body.findall('joint') if j.get('type') == 'revolute')
    print(f'Korenovy link tela:        {root_link}')
    print(f'Pohyblive klby v tele:     {n_body}   (cakam 29)')
    print(f'Klby rúk (s prstami):      {count_revolute(parts_true)}   (cakam 14)')
    print(f'Klby bez rúk (koncovy clanok): {count_revolute(parts_false)}   (cakam 0)')
    print(f'\nZapisane:\n  {body_path}\n  {hands_path}')


if __name__ == '__main__':
    main()
