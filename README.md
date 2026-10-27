# Agent-Based Humanoid Control

Bakalársky projekt: **Riadenie humanoidného robota v simulačnom prostredí pomocou agentových systémov**

- **Univerzita:** Technická univerzita v Košiciach (TUKE), Fakulta elektrotechniky a informatiky
- **Študijný program:** Inteligentné systémy
- **Pracovisko:** Ústav umelej inteligencie (ÚUI)
- **Vedúci práce:** doc. Ing. Peter Papcun, PhD.
- **Autor:** Matej Rabik

## Abstrakt

Projekt prepája jazykový model (LLM) s humanoidným robotom v simulácii. Upravený model Unitree G1 s názvom **Chappie** beží v Gazebo (ROS 2 Jazzy, Gazebo Harmonic). Používateľ zadá úlohu v prirodzenom jazyku, napríklad `chappie_task "otoč sa k červenej škatuli, choď k nej a posuň ju o meter"`. AI agent dostane snímku z kamery, textový popis okolia z LiDARu, históriu a katalóg hotových funkcií (skills). Vráti plán ako zoznam akcií, ktorý sa skontroluje a potom vykoná. Neplatné akcie robot odmietne.

Prínos projektu:

- **Bezpečné rozhranie medzi AI a robotom:** LLM nevydáva priame povely motorom, vyberá len z overených skills (chôdza, otáčanie, gestá, vstávanie, navigácia, tlačenie). Výstup AI sa validuje.
- **Navigácia a vnímanie:** LiDAR (zastavenie pred prekážkou), pamäť miest (`save_place`, `go_to_place`), natočenie k predmetu podľa pravítka na obraze z kamery (`turn_towards`), tlačenie predmetov (`push_forward`).
- **Odolnosť voči výpadkom AI služieb:** reťaz Gemini → Mistral → Groq, vyčerpanú službu preskočí a vedie si počítadlá limitov.

## Štruktúra repozitára

| Cesta | Obsah |
|---|---|
| `README.md` | tento súbor |
| `requirements.txt` | Python závislosti a poznámka k systémovým závislostiam |
| `src/` | ROS 2 balíky (zdrojový kód, okomentovaný) |
| `src/humanoid_description` | URDF/Xacro model robota Chappie, senzory (kamera, LiDAR Mid-360), meshe |
| `src/humanoid_gazebo` | spúšťacie súbory a svety (`arena.sdf`) pre Gazebo |
| `src/humanoid_walk` | C++ node s natrénovanou politikou chôdze a vstávania |
| `src/humanoid_skills` | C++ knižnica skills (`skill_core.hpp`), node `motion_skills`, pózy rúk |
| `src/humanoid_agent` | Python AI agent: `agent_core.py` (plán), `llm_backends.py` (Gemini/Mistral/Groq), `task_client.py` (príkaz `chappie_task`) |
| `src/humanoid_interfaces` | ROS 2 akcie `Skill` a `Task` |
| `src/humanoid_teleop` | ovládanie z klávesnice (WASD) |
| `src/humanoid_control` | konfigurácia kontrolérov |
| `tools/` | pomocné skripty (generovanie modelu a kolízií, export politiky) |
| `mujoco_demo/` | záložné demo chôdze v MuJoCo |
| `doc/` | technická dokumentácia a návod na spustenie (`doc/NAVOD.md`) |

## Architektúra

```
chappie_task "úloha"
   └─> /agent/task ──> agent_node (Agent.run: kamera + LiDAR + história + katalóg skills ─> LLM ─> JSON plán ─> validácia)
                                 └─> /skills/execute ──> motion_skills (C++, SkillExecutor)
                                          └─> /cmd_vel a trajektórie rúk ──> walk_policy ──> Gazebo
```

## Spustenie

Podrobný návod je v [`doc/NAVOD.md`](doc/NAVOD.md). Skrátene:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch humanoid_gazebo walk_demo.launch.py        # simulácia + agent
ros2 run humanoid_agent task_client "otoč sa doľava o 90 stupňov"
```

API kľúče sa nikdy neukladajú do repozitára. Čítajú sa z premenných prostredia alebo zo súboru `~/.chappie_keys`.

## Použitie umelej inteligencie

V súlade s pokynmi ÚUI uvádzam, kde a ako bola AI použitá.

1. **AI ako súčasť systému (runtime):** modely Gemini, Mistral a Groq rozhodujú, ktoré skills sa použijú. Ide o samotný predmet práce.
2. **AI ako pomocník pri vývoji:** asistent Claude (Anthropic) pomáhal pri návrhu a písaní častí kódu (napr. LiDAR filter, navigačné skills, reťaz LLM služieb) a pri formulácii dokumentácie. Autor kód a texty skontroloval, otestoval v simulácii a zodpovedá za ne.
3. **Konzultácia so školiteľom:** *doplniť dátum a súhlas školiteľa s použitím AI.*

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
- *Doplniť odbornú literatúru (agentové systémy, LLM pre robotiku, napr. SayCan, Code as Policies) podľa konzultácie so školiteľom.*

## Licencia

Pozri `LICENSE` a `doc/UNITREE_LICENSE`.
