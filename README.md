# Riadenie humanoidného robota v simulačnom prostredí pomocou agentových systémov

*Controlling a humanoid robot in a simulation environment using agentic systems*

| | |
|---|---|
| **Číslo práce** | 85041 |
| **Druh** | SP |
| **Akademický rok** | 2026/2027 |
| **Vedúci** | doc. Ing. Peter Papcun, PhD. |
| **Študent** | Matej Rabik |
| **Pracovisko** | ÚUI FEI TUKE |

## Abstrakt

Projekt prepája jazykový model (LLM) s humanoidným robotom v simulácii. Upravený model Unitree G1 s názvom **Chappie** beží v Gazebo (ROS 2 Jazzy, Gazebo Harmonic). Používateľ zadá úlohu v prirodzenom jazyku, napríklad `chappie_task "otoč sa k červenej škatuli, choď k nej a posuň ju o meter"`. AI agent dostane snímku z kamery, textový popis okolia z LiDARu, históriu a katalóg hotových funkcií (skills). Vráti plán ako zoznam akcií, ktorý sa skontroluje a potom vykoná. Neplatné akcie robot odmietne.

Prínos projektu:

- **Bezpečné rozhranie medzi AI a robotom:** LLM nevydáva priame povely motorom, vyberá len z overených skills (chôdza, otáčanie, gestá, vstávanie, navigácia, tlačenie). Výstup AI sa validuje.
- **Navigácia a vnímanie:** LiDAR (zastavenie pred prekážkou), pamäť miest (`save_place`, `go_to_place`), natočenie k predmetu podľa pravítka na obraze z kamery (`turn_towards`), tlačenie predmetov (`push_forward`), prechod dverami (`go_through_opening`: otvor v stene nájde LiDAR), prehľadávanie izby (`search_room`).
- **Odolnosť voči výpadkom AI služieb:** reťaz Gemini → Groq → OpenRouter → Mistral, vyčerpanú službu preskočí, vedie si počítadlá limitov a meria spotrebu tokenov (`chappie_cost`, `src/tools/probe_limits.py`).

## Štruktúra repozitára

| Cesta | Obsah |
|---|---|
| `README.md` | tento súbor – všetky informácie o projekte |
| `requirements.txt` | Python závislosti a poznámka k systémovým závislostiam (ROS 2, Gazebo a ďalšie sú popísané v `doc/NAVOD.md`) |
| `src/` | ROS 2 balíky (zdrojový kód, okomentovaný) |
| `src/humanoid_description` | URDF/Xacro model robota Chappie, senzory (kamera, LiDAR Mid-360), meshe |
| `src/humanoid_gazebo` | spúšťacie súbory a svety (`arena.sdf`) pre Gazebo |
| `src/humanoid_walk` | C++ node s natrénovanou politikou chôdze a vstávania |
| `src/humanoid_skills` | C++ knižnica skills (`skill_core.hpp`), node `motion_skills`, pózy rúk |
| `src/humanoid_agent` | Python AI agent: `agent_core.py` (plán), `llm_backends.py` (Gemini/Groq/OpenRouter/Mistral), `task_client.py` (príkaz `chappie_task`) |
| `src/humanoid_interfaces` | ROS 2 akcie `Skill` a `Task` |
| `src/humanoid_teleop` | ovládanie z klávesnice (WASD) |
| `src/humanoid_control` | konfigurácia kontrolérov |
| `src/tools/` | pomocné skripty (generovanie modelu a kolízií, export politiky), generátor svetov `gen_world.py`, meranie limitov AI `probe_limits.py` a test kontroly prechodu dverami `test_door_check.cpp`, webové GUI `chappie_gui.py` |
| `src/maps/` | ASCII mapy pre generátor svetov |
| `src/mujoco_demo/` | záložné demo chôdze v MuJoCo |
| `LICENSE` | licencia projektu (MIT), licencia modelu Unitree je v `doc/UNITREE_LICENSE` |
| `doc/` | technická dokumentácia: návod na spustenie (`doc/NAVOD.md`), prehľad príkazov `chappie_*` (`doc/PRIKAZY.md`), prehľad skillov (`doc/SKILLY.md`), literatúra (`doc/LITERATURA.md`) |

## Architektúra

```
chappie_task "úloha"
   └─> /agent/task ──> agent_node (Agent.run: kamera + LiDAR + história + katalóg skills ─> LLM ─> JSON plán ─> validácia)
                                 └─> /skills/execute ──> motion_skills (C++, SkillExecutor)
                                          └─> /cmd_vel a trajektórie rúk ──> walk_policy ──> Gazebo
```

## Návod na spustenie a použitie

Podrobný návod (inštalácia ROS 2 Jazzy a Gazeba Harmonic, kľúče k AI službám, spustenie) je v [`doc/NAVOD.md`](doc/NAVOD.md). Skrátene:

```bash
git clone https://gitlab.uui.fei.tuke.sk/studentprojects/2026/bp/riadenie-humanoidneho-robota-v-simulacnom-prostredi-pomocou-agentovych-systemov.git ~/agent-based-humanoid-control
cd ~/agent-based-humanoid-control
colcon build --symlink-install
source install/setup.bash
ros2 launch humanoid_gazebo walk_demo.launch.py        # simulácia + agent
ros2 run humanoid_agent task_client "otoč sa doľava o 90 stupňov"
```

### Svety, GUI a testy

```bash
python3 src/tools/gen_world.py --random rooms --seed 3 --name rooms_3     # náhodné izby s dverami (ďalej maze, warehouse alebo --ascii src/maps/izba.txt)
ros2 launch humanoid_gazebo walk_demo.launch.py world:=rooms_3.sdf        # simulácia s týmto svetom
python3 src/tools/chappie_gui.py                       # webové GUI (lokálny server)
# test kontroly prechodu dverami na simulovanom LiDARe bez ROS:
g++ -std=c++17 -O1 -I src/humanoid_skills/include src/tools/test_door_check.cpp -o /tmp/test_dc && /tmp/test_dc
python3 src/tools/probe_limits.py --service gemini     # zmeria limity AI služby
python3 src/humanoid_agent/humanoid_agent/cost_report.py   # spotreba tokenov a nákladov AI (v návode ako príkaz chappie_cost)
```

API kľúče sa nikdy neukladajú do repozitára. Čítajú sa z premenných prostredia alebo zo súboru `~/.chappie_keys`.

## Príručky a podporné dokumenty

Uložené v `doc/`: [`NAVOD.md`](doc/NAVOD.md) (inštalácia a spustenie), [`PRIKAZY.md`](doc/PRIKAZY.md) (príkazy `chappie_*`), [`SKILLY.md`](doc/SKILLY.md) (prehľad skills), [`LITERATURA.md`](doc/LITERATURA.md) (literatúra s poznámkami).

## Použitie umelej inteligencie

V súlade s pokynmi ÚUI uvádzam, kde a ako bola AI použitá.

1. **AI ako súčasť systému (runtime):** modely Gemini, Groq, OpenRouter a Mistral rozhodujú, ktoré skills sa použijú. Ide o samotný predmet práce.
2. **AI ako pomocník pri vývoji:** asistent Claude (Anthropic) pomáhal pri návrhu a písaní častí kódu (napr. LiDAR filter, navigačné skills, reťaz LLM služieb) a pri formulácii dokumentácie. Autor kód a texty skontroloval, otestoval v simulácii a zodpovedá za ne.

Použité cudzie zdroje: model Unitree G1 a jeho licencia (`doc/UNITREE_LICENSE`), politika chôdze `walk_mjlab` a ďalšie zdroje uvedené nižšie.

## Referencie

- Unitree G1 (URDF/MJCF): <https://github.com/unitreerobotics/unitree_ros>, <https://github.com/unitreerobotics/unitree_mujoco>
- ROS 2 Jazzy: <https://docs.ros.org/en/jazzy/>
- Gazebo Harmonic: <https://gazebosim.org/docs/harmonic/>
- MuJoCo: <https://mujoco.readthedocs.io/>
- mjlab (tréning politiky chôdze): <https://github.com/mujocolab/mjlab>
- Google Gemini API: <https://ai.google.dev/gemini-api/docs>
- Mistral API: <https://docs.mistral.ai/>
- Groq API: <https://console.groq.com/docs>
- OpenRouter API: <https://openrouter.ai/docs>
- Ahn, M. et al.: *Do As I Can, Not As I Say: Grounding Language in Robotic Affordances* (SayCan), CoRL 2022. <https://arxiv.org/abs/2204.01691>
- Yao, S. et al.: *ReAct: Synergizing Reasoning and Acting in Language Models*, ICLR 2023. <https://arxiv.org/abs/2210.03629>
- Huang, W. et al.: *Inner Monologue: Embodied Reasoning through Planning with Language Models*, CoRL 2022. <https://arxiv.org/abs/2207.05608>
- Liang, J. et al.: *Code as Policies: Language Model Programs for Embodied Control*, ICRA 2023. <https://arxiv.org/abs/2209.07753>
- Macenski, S. et al.: *Robot Operating System 2: Design, architecture, and uses in the wild*, Science Robotics 7(66), 2022. <https://arxiv.org/abs/2211.07752>
- Koenig, N., Howard, A.: *Design and use paradigms for Gazebo, an open-source multi-robot simulator*, IROS 2004. <https://ieeexplore.ieee.org/document/1389727>
- Anguelov, D. et al.: *Detecting and Modeling Doors with Mobile Robots*, ICRA 2004. <https://dblp.org/rec/conf/icra/AnguelovKPT04.html>
- Yamauchi, B.: *A frontier-based approach for autonomous exploration*, CIRA 1997. <https://ieeexplore.ieee.org/document/613851>
- Úplný zoznam s poznámkami: [`doc/LITERATURA.md`](doc/LITERATURA.md)

## Veľké súbory a datasety

Veľké súbory a datasety sa do gitu nenahrávajú. Výnimkou sú meshe robota v `src/humanoid_description/meshes/` (cca 116 MB, formát STL) a natrénované politiky chôdze (`src/humanoid_walk/config/*.bin`), ktoré sú nevyhnutné na spustenie modelu v simulácii. Zdroj meshov a ich licencia: model Unitree G1 (pozri Referencie a `doc/UNITREE_LICENSE`).

## Licencia

Pozri `LICENSE` a `doc/UNITREE_LICENSE`.
