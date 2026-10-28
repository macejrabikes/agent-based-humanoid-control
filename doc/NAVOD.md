# Návod na spustenie a použitie

Overené prostredie: Kubuntu 24.04 (vo VMware), ROS 2 Jazzy, Gazebo Harmonic.

## 1. Inštalácia

```bash
# ROS 2 Jazzy + Gazebo Harmonic (podľa oficiálnej dokumentácie)
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-gz-ros2-control python3-colcon-common-extensions

git clone https://github.com/macejrabikes/agent-based-humanoid-control.git ~/agent-based-humanoid-control
cd ~/agent-based-humanoid-control
pip install -r requirements.txt          # pozri poznámku o externom prostredí v Ubuntu 24.04 (venv alebo --break-system-packages)
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 2. API kľúče

Aspoň jeden zo služieb Gemini, Mistral, Groq. Kľúče patria do súboru `~/.chappie_keys` (oprávnenia `chmod 600`), formát `NÁZOV=hodnota`:

```
GEMINI_API_KEY=...
MISTRAL_API_KEY=...
GROQ_API_KEY=...
```

Poradie služieb mení premenná `CHAPPIE_CHAIN` (predvolene `gemini,mistral,groq`). Kľúče nikdy necommitujte.

## 3. Spustenie simulácie

```bash
ros2 launch humanoid_gazebo walk_demo.launch.py
```

Užitočné parametre: `weld_upper:=false` (voľné ruky), `with_hands:=true`, `policy_impl:=py`, `follow:=false`.

## 4. Zadanie úlohy

V druhom termináli (po `source install/setup.bash`):

```bash
ros2 run humanoid_agent task_client "otoč sa doľava o 90 stupňov a povedz, čo vidíš"
ros2 run humanoid_agent task_client "choď k škatuli a posuň ju o meter"
```

Ctrl+C úlohu zruší. Po úlohe sa vypíše výsledok, použitá AI služba a zostávajúce limity.

## 5. Ovládanie z klávesnice (bez AI)

```bash
ros2 run humanoid_teleop wasd_teleop
```

## 6. Skills

Chôdza (`walk_forward`, `walk_backward`, `step_left/right`, `walk_to_obstacle`), otáčanie (`turn_left/right`, `turn_towards`), `get_up`, `look_around`, gestá a pózy rúk, navigácia (`save_place`, `go_to_place`, `where_am_i`) a tlačenie (`push_forward`). Úplný katalóg je v `src/humanoid_skills/include/humanoid_skills/skill_core.hpp`.

## 7. Iné svety (generátor)

Základná aréna je `arena_big_walk.sdf`. Ďalšie svety vygeneruje `tools/gen_world.py` z ASCII mapy (`maps/`) alebo náhodne (`rooms`, `maze`, `warehouse`):

```bash
chappie_world rooms 3                  # náhodné izby so seedom 3 a spustenie simulácie
python3 tools/gen_world.py --ascii maps/izba.txt --name izba
chappie_sim world:=izba.sdf
```

Popis znakov mapy je v hlavičke `tools/gen_world.py`.

## 8. Riešenie problémov

- Robot padá alebo sa nehýbe: reštartujte simuláciu, skontrolujte, že beží `walk_policy_node`.
- Agent odpovedá chybou kvóty: služba sa na čas preskočí, počítadlá sú v `~/.chappie_usage.json`.
- Všetky skratky `chappie_*` (sim, task, box, cam, lidar, cost, world, ...) sú popísané v [`PRIKAZY.md`](PRIKAZY.md). Sú to aliasy a funkcie v `~/.bashrc` autora.
