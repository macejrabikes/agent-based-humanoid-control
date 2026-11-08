#!/usr/bin/env python3
"""
Dokonci G1 xacro:
  1) prefarbi: 'white' -> chrome (klbove puzdra) / panel (ostatne),
     globalne definicie materialov presunie do materials.xacro
  2) doplni <dynamics damping/friction> vsetkym revolute klbom (telo aj ruky)

Pouzitie:  python g1_finish.py
Zaloha povodnych suborov sa ulozi do ~/backup_urdf/.
"""
import os
import shutil
import sys

from lxml import etree

URDF_DIR = os.path.expanduser(
    '~/agent-based-humanoid-control/src/humanoid_description/urdf')
BODY = os.path.join(URDF_DIR, 'humanoid.urdf.xacro')
HANDS = os.path.join(URDF_DIR, 'g1.hands.xacro')
MATS = os.path.join(URDF_DIR, 'materials.xacro')
XACRO_NS = 'http://www.ros.org/wiki/xacro'

# linky, ktore dostanu chrom (zvysok z 'white' ide na cierne panely)
CHROME_KEYS = ('hip_roll', 'hip_yaw', 'shoulder', 'wrist', 'waist')

MATERIALS_XML = """<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <material name="dark">
    <color rgba="0.1 0.1 0.1 1"/>
  </material>

  <material name="white">
    <color rgba="1 1 1 1"/>
  </material>

  <material name="chrome">
    <color rgba="0.78 0.80 0.84 1"/>
  </material>

  <material name="panel">
    <color rgba="0.13 0.14 0.16 1"/>
  </material>

</robot>
"""


def dynamics_for(effort):
    """(damping, friction) podla sily klbu."""
    if effort >= 80:
        return 2.0, 0.2
    if effort >= 20:
        return 0.5, 0.05
    if effort >= 3:
        return 0.2, 0.02
    return 0.02, 0.002


def retarget_materials(root, report):
    for link in root.iter('link'):
        name = link.get('name', '')
        for vis in link.findall('visual'):
            m = vis.find('material')
            if m is None or m.get('name') != 'white':
                continue
            new = 'chrome' if any(k in name for k in CHROME_KEYS) else 'panel'
            m.set('name', new)
            report.append((name, new))


def add_dynamics(root):
    n = 0
    for j in root.iter('joint'):
        if j.get('type') != 'revolute' or j.find('dynamics') is not None:
            continue
        lim = j.find('limit')
        if lim is None:
            print(f"VAROVANIE: klb {j.get('name')} nema limit, preskakujem")
            continue
        damping, friction = dynamics_for(float(lim.get('effort')))
        dyn = etree.Element('dynamics', damping=str(damping),
                            friction=str(friction))
        dyn.tail = lim.tail
        lim.tail = j.text
        j.insert(j.index(lim) + 1, dyn)
        n += 1
    return n


def ensure_materials_include(body):
    for el in body.iter():
        if isinstance(el.tag, str) and el.tag.endswith('}include') \
                and el.get('filename') == 'materials.xacro':
            return
    first_inc = None
    for el in body:
        if isinstance(el.tag, str) and el.tag.endswith('}include'):
            first_inc = el
    inc = etree.Element(f'{{{XACRO_NS}}}include', filename='materials.xacro')
    if first_inc is None:
        body.insert(0, inc)
    else:
        inc.tail = first_inc.tail
        body.insert(body.index(first_inc) + 1, inc)


def main():
    for p in (BODY, HANDS):
        if not os.path.exists(p):
            sys.exit(f'Nenasiel som {p}')

    backup = os.path.expanduser('~/backup_urdf')
    os.makedirs(backup, exist_ok=True)
    shutil.copy(BODY, os.path.join(backup, 'g1_body.before_finish.xacro'))
    shutil.copy(HANDS, os.path.join(backup, 'g1_hands.before_finish.xacro'))

    body_tree = etree.parse(BODY)
    hands_tree = etree.parse(HANDS)
    body, hands = body_tree.getroot(), hands_tree.getroot()

    # --- materialy ---
    for m in body.findall('material'):      # globalne definicie z povodneho URDF
        body.remove(m)
    ensure_materials_include(body)

    report = []
    retarget_materials(body, report)
    retarget_materials(hands, report)

    with open(MATS, 'w') as f:
        f.write(MATERIALS_XML)

    # --- dynamics ---
    n_body = add_dynamics(body)
    n_hands = add_dynamics(hands)

    for tree in (body, hands):
        etree.indent(tree, space='  ')
    body_tree.write(BODY, xml_declaration=True, encoding='utf-8')
    hands_tree.write(HANDS, xml_declaration=True, encoding='utf-8')

    chrome = sorted({n for n, c in report if c == 'chrome'})
    panel = sorted({n for n, c in report if c == 'panel'})
    print(f'chrome ({len(chrome)}): {", ".join(chrome)}')
    print(f'panel  ({len(panel)}): {", ".join(panel)}')
    print(f'\ndynamics doplnene: telo {n_body}, ruky {n_hands}  (cakam 29 + 14)')
    print(f'zaloha: {backup}')


if __name__ == '__main__':
    main()
