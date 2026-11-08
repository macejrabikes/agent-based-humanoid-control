"""prompt_select: kratsi prompt pre jednoduche ulohy (bez ROS, iba stdlib).

Plny prompt agenta ma ~26 000 znakov (~6 500 tokenov): ~13 000 text s pravidlami + ~13 000 katalog skills. Vacsina uloh
("otoc sa doprava a zamavaj") nepotrebuje popisy dveri, miest, tlacenia, obchadzania ani vsetkych gest.

Bezpecnost (cielom je NEPOKAZIT nic, kratsi prompt je az druhy v poradi):
  1. kratky prompt sa pouzije len ak zadanie obsahuje znamu jednoduchu cinnost A ZAROVEN ziadne slovo zo zoznamu zlozitych veci
     (dvere, miestnost, predmet, farba, hladaj, pozri, stena, miesto, ...). Pri akejkolvek pochybnosti sa pouzije PLNY prompt.
  2. jadro (chodza, otacanie, turn_towards, walk_to_obstacle, walk_to_object, ruky, get_up, wait, look) je vzdy v kazdom prompte.
  3. skills, ktore nie su v zoznamoch skupin nizsie (napr. buduce nove), sa povazuju za jadro, teda sa nikdy nevynechaju.
  4. agent prejde na PLNY prompt, ked: AI vrati neznamy skill, AI skonci bez akcie / napise, ze to neurobi, nejaka akcia
     zlyha, alebo sa uloha tahne na 3. prompt.  Rozhodnutie sa zapise do <run_dir>/prompt_mode.txt.
  5. CHAPPIE_PROMPT=full vypne skracovanie uplne (A/B porovnanie).
"""
import os
import re
import unicodedata

# volitelne skupiny: nazvy skills a odrazky v SYSTEM_PROMPT (zaciatok riadku), ktore sa bez nich vynechaju
GROUPS = {
    "gestures": {
        "skills": ["wave", "wave_left", "wave_both", "point_forward", "thumbs_up", "cheer", "salute", "clap", "bow", "dance"],
        "bullets": ["- wave, point_forward"],
    },
    "places": {
        "skills": ["save_place", "go_to_place", "where_am_i"],
        "bullets": ["- PLACES:"],
    },
    "rooms": {
        "skills": ["look_around", "go_through_opening", "search_room"],
        "bullets": ["- look_around turns", "- go_through_opening finds", "- search_room moves"],
    },
    "push": {
        "skills": ["push_forward"],
        "bullets": ["- push_forward pushes"],
    },
    "avoid": {
        "skills": ["go_around_obstacle", "follow_wall"],
        "bullets": ["- go_around_obstacle:", "- Following a wall:"],
    },
}

# jednoducha cinnost, ktoru prompt bez volitelnych skupin zvladne (po odstraneni diakritiky, zaciatky slov)
SIMPLE = re.compile(
    r"\b(turn|rotate|otoc|obrat|walk|move|go\b|chod|prejdi|pojdi|krok|step|forward|backward|dopredu|dozadu|vpred|vzad|"
    r"dolava|doprava|vlavo|vpravo|left|right|wave|zamav|mavaj|pozdrav|greet|hello|ahoj|dance|tanc|zatanc|bow|poklon|ukloni|"
    r"clap|tlesk|zatlesk|tliesk|zatliesk|potlesk|cheer|jasa|zjasa|salut|thumb|palec|stoj|stop|zastav|wait|pockaj|cakaj|raise|zdvihn|spust|ruk|arm|hand)")

# cokolvek zlozitejsie -> plny prompt (zamerne siroko: radsej zbytocne plny prompt nez chybajuci skill)
COMPLEX = re.compile(
    r"\b(door|dver|room|izb|miestnos|search|hlad|find|najd|look|see|saw|vid|pozri|pozeraj|rozhliad|scan|count|spocit|kolko|"
    r"how many|describe|opis|what|which|ktor|ake\b|aky|ako\b|object|predmet|vec\b|veci|box|skrin|kocka|cube|krabic|skatul|"
    r"stlp|pillar|cylind|valec|ball|lopt|chair|stol|table|person|clovek|osob|human|wall|sten|stien|obstacle|prekaz|around|"
    r"obid|obchad|push|tlac|pull|place|miest|zapam|remember|save|memor|back\b|spat|vrat|return|start|stred|center|centre|"
    r"position|poloh|pozici|where|kde|fall|spadol|get up|vstan|stand up|colou?r|farb|red|blue|green|yellow|cerven|modr|zelen|zlt|"
    r"until|kym|nenaraz|reach|dosiahn|dojdi|go to|towards|toward|smerom|\bk\b|\bku\b|near|blizk|najblizs|follow|sleduj|"
    r"corridor|chodb|exit|vychod|through|cez|pass|map|explor|preskum|navig|bring|prines|pick|zober|take|vezmi|carry|open|otvor|"
    r"close|zatvor|then look|vzdialen|distance|lidar|camera|kamer|image|obraz|picture|snimk|photo|foto)")

GESTURE = re.compile(
    r"\b(wave|zamav|mav|pozdrav|greet|hello|\bhi\b|ahoj|cau|dance|tanc|zatanc|bow|poklon|ukloni|clap|tlesk|tliesk|zatliesk|potlesk|cheer|jasa|zjasa|"
    r"salut|thumb|palec|point|ukaz|gest|ruk|arm|hand|raise|zdvihn)")

# odpoved AI, z ktorej je vidiet, ze skill nema (-> skus znova s plnym promptom)
CANNOT = re.compile(
    r"(not in the catalog|no skill|cannot|can't|can not|unable|not possible|nemozem|nedokazem|neviem|nie je mozne|nemam skill|chyba skill)")


def fold(text):
    """male pismena bez diakritiky"""
    t = unicodedata.normalize("NFD", str(text).lower())
    return "".join(ch for ch in t if unicodedata.category(ch) != "Mn")


def select(task):
    """(rezim, mnozina skupin): rezim 'full' (vsetko) alebo 'lite' (jadro + vybrane skupiny)"""
    if os.environ.get("CHAPPIE_PROMPT", "auto").lower() == "full":
        return "full", set(GROUPS)
    t = fold(task)
    if len(t) > 220 or COMPLEX.search(t) or not SIMPLE.search(t):
        return "full", set(GROUPS)
    groups = set()
    if GESTURE.search(t):
        groups.add("gestures")
    return "lite", groups


def wants_full(reply_text):
    """True, ak z odpovede AI vyplyva, ze v kratkom prompte nieco chybalo"""
    return bool(CANNOT.search(fold(reply_text or "")))


def _drop_bullets(text, prefixes):
    """odstrani odrazky (riadok zacinajuci '- ' + jeho pokracovanie odsadene medzerou) so zadanymi zaciatkami"""
    out, skipping = [], False
    for line in text.split("\n"):
        if line.startswith("- ") or (line and not line.startswith(" ")):
            skipping = any(line.startswith(p) for p in prefixes)
        elif not line.strip():
            skipping = False
        if not skipping:
            out.append(line)
    return "\n".join(out)


def hidden_skills(groups):
    names = set()
    for g, spec in GROUPS.items():
        if g not in groups:
            names.update(spec["skills"])
    return names


def build_system(template, catalog, look, max_actions, mode="full", groups=None):
    """SYSTEM_PROMPT.format(...) pre dany rezim; catalog = zoznam skills (JSON z C++), look = pseudo-skill 'look'"""
    import json
    groups = set(GROUPS) if groups is None else set(groups)
    if mode == "full":
        groups = set(GROUPS)
    drop_bullets = []
    for g, spec in GROUPS.items():
        if g not in groups:
            drop_bullets += spec["bullets"]
    text = _drop_bullets(template, drop_bullets) if drop_bullets else template
    hide = hidden_skills(groups)
    cat = [s for s in catalog if s["name"] not in hide] + [look]
    return text.format(max_actions=max_actions, catalog=json.dumps(cat, ensure_ascii=False))
