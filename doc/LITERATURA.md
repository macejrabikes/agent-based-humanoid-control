# Literatúra k bakalárskej práci (návrh)

Položky 1 až 8 som overil vyhľadávaním (existuje práca s týmto názvom a konferenciou/časopisom). Zoznam autorov je skrátený na prvého ("et al."), strany a DOI si doplň z odkazov. Formát uprav podľa šablóny fakulty (ISO 690). Odkazy v časti Dokumentácia som nekontroloval, over ich a doplň dátum prístupu.

## Agent s jazykovým modelom a robot

1. AHN, M. et al. *Do As I Can, Not As I Say: Grounding Language in Robotic Affordances.* In: Proceedings of the 6th Conference on Robot Learning (CoRL), PMLR 205, 2022. arXiv:2204.01691. https://arxiv.org/abs/2204.01691
   - Kde použiť: základná myšlienka, že jazykový model plánuje z množiny skillov, ktoré robot reálne vie vykonať. U nás sú to skilly (`walk_forward`, `go_through_opening`, ...).
2. YAO, S. et al. *ReAct: Synergizing Reasoning and Acting in Language Models.* In: International Conference on Learning Representations (ICLR), 2023. arXiv:2210.03629. https://arxiv.org/abs/2210.03629
   - Kde použiť: cyklus "rozmýšľaj, konaj, pozri sa na výsledok", ktorý používa náš agent (pole `Úvaha`, akcie, výsledok skillu v histórii).
3. HUANG, W. et al. *Inner Monologue: Embodied Reasoning through Planning with Language Models.* In: Conference on Robot Learning (CoRL), 2022. arXiv:2207.05608. https://arxiv.org/abs/2207.05608
   - Kde použiť: spätná väzba zo senzorov a kamery sa vracia do jazykového modelu (náš LIDAR a `look`).
4. LIANG, J. et al. *Code as Policies: Language Model Programs for Embodied Control.* In: IEEE International Conference on Robotics and Automation (ICRA), 2023. arXiv:2209.07753. https://arxiv.org/abs/2209.07753
   - Kde použiť: porovnanie, AI generuje kód, my AI nechávame vyberať z hotových overených skillov (bezpečnosť, predvídateľnosť).

## Robotický softvér a simulácia

5. MACENSKI, S., FOOTE, T., GERKEY, B., LALANCETTE, C., WOODALL, W. *Robot Operating System 2: Design, architecture, and uses in the wild.* Science Robotics, 2022, 7(66), eabm6074. arXiv:2211.07752. https://arxiv.org/abs/2211.07752
   - Kde použiť: ROS 2 (uzly, topicy, akcie), na ktorom beží `motion_skills` a agent.
6. KOENIG, N., HOWARD, A. *Design and use paradigms for Gazebo, an open-source multi-robot simulator.* In: IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2004. https://ieeexplore.ieee.org/document/1389727
   - Kde použiť: Gazebo ako simulátor (u nás novšia verzia Gazebo Harmonic).

## Hľadanie dverí a prehľadávanie izby

7. ANGUELOV, D., KOLLER, D., PARKER, E., THRUN, S. *Detecting and Modeling Doors with Mobile Robots.* In: IEEE International Conference on Robotics and Automation (ICRA), 2004. https://dblp.org/rec/conf/icra/AnguelovKPT04.html
   - Kde použiť: dvere ako medzera v stene nájdená zo senzorov mobilného robota (náš `go_through_opening` hľadá medzeru podľa skoku vzdialenosti v LiDAR skene).
8. YAMAUCHI, B. *A frontier-based approach for autonomous exploration.* In: IEEE International Symposium on Computational Intelligence in Robotics and Automation (CIRA), 1997. https://ieeexplore.ieee.org/document/613851
   - Kde použiť: prehľadávanie neznámeho priestoru presúvaním na nové miesta (náš `search_room` je zjednodušená verzia bez mapy, vyberá najvolnejší smer, kde robot ešte nebol).

## Dokumentácia (online zdroje, doplň dátum prístupu)

- Open Robotics. *ROS 2 Jazzy Jalisco Documentation.* https://docs.ros.org/en/jazzy/
- Open Robotics. *Gazebo Harmonic Documentation.* https://gazebosim.org/docs/harmonic/
- Unitree Robotics. *Unitree G1 – technická dokumentácia.* https://www.unitree.com/g1
- Livox. *Mid-360 User Manual.* https://www.livoxtech.com/mid-360
- Google, Groq, OpenRouter, Mistral AI: dokumentácia API, ktoré používa `llm_backends.py` (uveď dátum, kedy si ich čítal, a že limity sa menia).

## Poznámka k transparentnosti AI

Do práce patrí aj veta, že kód a časť textu vznikli s pomocou AI asistenta (Claude) a že použitie bolo konzultované s vedúcim (to si už povedal, že máš). Presné znenie podľa pokynov fakulty.
