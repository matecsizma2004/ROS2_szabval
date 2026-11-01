# Ragasztás KUKA KR 6 R900-2 robottal – dokumentáció

*Robot operációs rendszerek és fejlesztői ökoszisztémák, 2026/27. őszi félév, 7. projektfeladat*

## 1. A feladat

Egy A4-es lapon 10 × 13-as rács van, a cellák 2 × 2 cm-esek. A fekete cellák a ragasztási pontok. A feladat két része:

1. **Mock szenzor:** a ragasztási térképet konfigurációs fájlból kell beolvasni, és `sensor_msgs/msg/Image` üzenetként publikálni egy ROS topicra.
2. **Mozgásprogram:** a robot a kép alapján, sorról sorra haladva, egyenes szakaszokban köti össze a fekete cellák középpontjait. Ahol két fekete cella nem szomszédos, ott a ragasztófejet fel kell emelni.

A megoldás ROS 2 Jazzy alatt készült, Python (rclpy) nyelven. Mozgástervezéshez a MoveIt 2-t használja, a megjelenítés RViz-ben történik. A robot a mock hardverrel futó KUKA KR 6 R900-2 (KR AGILUS) modell.

## 2. Telepítés és futtatás

Előfeltételek: Ubuntu 24.04 és ROS 2 Jazzy.

```bash
# Függőségek
sudo apt install ros-jazzy-moveit ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
                 ros-jazzy-joint-state-publisher-gui python3-pytest

# KUKA robotleírások (a repó nem tartalmazza, a git nem követi)
cd ~/szabval_ros/src
git clone -b master https://github.com/kroshu/kuka_robot_descriptions.git

# Fordítás
cd ~/szabval_ros
rosdep install --from-paths src --ignore-src -y
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# Futtatás
ros2 launch ragasztas ragasztas.launch.py
```

Tesztek: `colcon test --packages-select ragasztas && colcon test-result --verbose`

## 3. Architektúra

```
config/glue_map.yaml
        │ (paraméterek)
        ▼
┌──────────────────┐  /glue_map            ┌──────────────────┐
│ glue_map_sensor  │ ────────────────────► │   glue_motion    │
│  (mock szenzor)  │  sensor_msgs/Image    │ (mozgásvezérlő)  │
└──────────────────┘  mono8, 10x13         └──────────────────┘
                                             │   │        │
                  /compute_cartesian_path ◄──┘   │        │ /glue_markers
                  /execute_trajectory     ◄──────┘        ▼ (MarkerArray)
                  /joint_trajectory_controller/        RViz
                    follow_joint_trajectory
                             │
              ┌──────────────┴───────────────────────────┐
              │ KUKA launch (kuka_kr_moveit_config):     │
              │ move_group, ros2_control_node (mock HW), │
              │ joint_trajectory_controller,             │
              │ robot_state_publisher                    │
              └──────────────────────────────────────────┘
```

| Elem | Típus | Szerep |
|---|---|---|
| `glue_map_sensor` | saját node | a konfig rácsot `mono8` képként publikálja |
| `glue_motion` | saját node | szakaszkeresés, Cartesian tervezés, végrehajtás, markerek |
| `/glue_map` | `sensor_msgs/Image` | ragasztási térkép (reliable, transient_local) |
| `/glue_markers` | `visualization_msgs/MarkerArray` | lap, fekete cellák, tervezett (kék) és kész (zöld) csíkok |
| `/compute_cartesian_path` | `moveit_msgs/srv/GetCartesianPath` | egyenes vonalú pálya tervezése a `tool0` linkre |
| `/execute_trajectory` | `moveit_msgs/action/ExecuteTrajectory` | a tervezett pálya végrehajtása |
| `.../follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | kezdőpózba állás csuklótérben |

A csomag felépítése (`src/ragasztas/`):

- `ragasztas/glue_path.py`: ROS-független logika (rács ↔ kép átalakítás, szakaszkeresés, koordináta-leképezés). Unit tesztek fedik le.
- `ragasztas/glue_map_sensor.py`: a mock szenzor.
- `ragasztas/glue_motion.py`: a mozgásvezérlő.
- `config/glue_map.yaml`: minden paraméter (rács, lappozíció, magasságok, sebesség).
- `launch/ragasztas.launch.py`: a KUKA MoveIt mock launch, a két node és egy saját RViz együtt.
- `rviz/ragasztas.rviz`: RViz konfiguráció (robot, markerek, a szenzor képe).

## 4. Mock szenzor és konfiguráció

A rács a `config/glue_map.yaml` fájlban van, 13 karakterláncként (soronként 10 karakter, `1` = fekete, `0` = üres). Ez a feladatlap ábrájának átirata:

```yaml
grid:
  - "1110011100"   # 0. sor (a lap teteje, a robottól legtávolabb)
  - "0011100000"
  - "0000000000"
  - "0000000011"
  - "0011100000"
  - "0000000000"
  - "0000111111"
  - "0000000000"
  - "1110000000"
  - "0000000000"
  - "0011100000"
  - "0000000000"
  - "0000011111"   # 12. sor
```

A szenzor ebből `mono8` képet készít: szélesség 10, magasság 13, `step` 10. A fekete cella pixelértéke 0, a fehéré 255. A képet `publish_rate` frekvenciával (alapból 1 Hz) küldi a `/glue_map` topicra. A QoS *transient_local*, így a később induló feliratkozó is azonnal megkapja az utolsó képet. A képet `cv_bridge` nélkül, közvetlenül tölti fel.

A mozgásvezérlő **csak a képből** dolgozik, a rácsot nem olvassa a konfigból. A térkép cseréjéhez elég a `grid`-et átírni.

## 5. Pályatervezés

### 5.1 Szakaszkeresés

A `find_strokes()` a képből visszaállított rácson soronként (0 → 12), balról jobbra keresi a maximális összefüggő fekete szakaszokat. „Összefüggő” itt **soron belüli vízszintes szomszédságot** jelent, a feladat „sorról sorra haladva” megfogalmazása alapján. Minden szakasz egy ragasztócsík. A feladat szerint magányos cella nincs; ha mégis lenne, a függvény kihagyja.

A feladatlap térképén **9 csík** van:

| # | sor | oszlopok |
|---|---|---|
| 1 | 0 | 0–2 |
| 2 | 0 | 5–7 |
| 3 | 1 | 2–4 |
| 4 | 3 | 8–9 |
| 5 | 4 | 2–4 |
| 6 | 6 | 4–9 |
| 7 | 8 | 0–2 |
| 8 | 10 | 2–4 |
| 9 | 12 | 5–9 |

### 5.2 Koordináták

A lap pozíciója fix, a `world` frame-ben adjuk meg (`world` = a robot talpa). A paraméterek a rács bal felső sarkát adják meg: `sheet_x = 0.82`, `sheet_y = 0.10`, `sheet_z = 0.0`. A lap a padlón fekszik, a robot előtt középen. A (sor, oszlop) cella középpontja:

```
x = sheet_x − (sor + 0.5) · 0.02     (a sorok a robot felé haladnak)
y = sheet_y − (oszlop + 0.5) · 0.02  (az oszlopok a robotból nézve balról jobbra)
z = sheet_z + glue_height            (ragasztás: 5 mm a lap felett)
```

A rács így x ∈ [0.56; 0.82] m, y ∈ [−0.10; 0.10] m tartományba esik. A lap helyét előzetes numerikus inverz kinematikai számítással választottuk ki (az URDF alapján):

- lefelé néző szerszámmal minden cella elérhető;
- minden csukló a határain belül marad;
- az A5 csukló végig legalább kb. 0.2 rad (≈ 11°) marad: a közeli sarokban 0.2, a távoliban 0.87 rad. Így a pálya elkerüli a csuklószingularitást (A5 = 0).

Közelebbi lapnál (x ≈ 0.45 m) az A5 előjelet váltana a pálya közben.

### 5.3 Mozgássorrend

1. **Kezdőpóz:** a mock hardver a KUKA „home” pózában indul ([0, −90°, 90°, 0, 0, 0]). Ebben a pózban A5 = 0, ami szinguláris. Ezért a robot először csuklótérben, a controlleren keresztül átáll a `ready_joints` pózba ([0, −0.30, 1.32, 0, 0.55, 0] rad). Itt a `tool0` lefelé néz, kb. 10 cm-rel a lap közepe felett.
2. **Csíkonként egy Cartesian kérés** a `/compute_cartesian_path` szolgáltatásnak, a következő waypointokkal:
   `kezdőcella felett (lift_height = 5 cm)` → `kezdőcella` → `közbenső cellák középpontjai` → `zárócella` → `zárócella felett`.
   - A szerszám orientációja végig állandó: a `tool0` z tengelye függőlegesen lefelé mutat, ami a (0, 1, 0, 0) kvaternió.
   - Két csík között a robot az előző csík feletti pontból egyenes vonalban, biztonsági magasságban halad a következő feletti pontba. Ez a feladat szerinti **emelés**.
   - Interpolációs lépésköz `max_step = 5 mm`. Az időparaméterezést a MoveIt végzi `velocity_scaling = acceleration_scaling = 0.2` mellett.
   - Ha a `fraction < 1.0` (a pálya nem teljesen tervezhető), a node hibával leáll.
3. **Végrehajtás** az `/execute_trajectory` actionnel. A kész csík az RViz-ben zöldre vált.
4. A végén a robot visszaáll a kezdőpózba, a node pedig logolja a csíkok számát és a ragasztási ciklusidőt.

A tényleges ragasztást (például CollisionObject-eket a planning scene-ben) a feladat nem kéri, ezért nem modellezzük.

## 6. Vizualizáció

A `ragasztas.launch.py` két RViz ablakot nyit:

- a KUKA MoveIt RViz-ét, amelyet a csomag maga indít (ezt nem kell használni);
- a saját `ragasztas.rviz` ablakot.

A saját ablak tartalma:

- a robotmodell;
- `/glue_markers`: fehér A4-es lap, fekete cellák, a tervezett pálya vékony kék vonalakkal, az elkészült csíkok vastag zöld vonalakkal;
- `/glue_map`: a szenzor képe (Image panel). Ennek egyeznie kell a feladatlap ábrájával.

## 7. Szimulációs teszt eredménye

*(A futtatás után kitöltendő.)*

- Unit tesztek (`test/test_glue_path.py`): rácsméret, a 9 elvárt csík, a kép ↔ rács oda-vissza átalakítás, a hibás rács elutasítása, a cellaközéppontok. Eredmény: 6/6 sikeres.
- Szenzor: `ros2 topic echo /glue_map --once` → `height: 13`, `width: 10`, `encoding: mono8`, `step: 10`, 130 bájt, a minta helyes.
- Teljes futás: képernyőképek ide: a robot a kezdőpózban, egy csík ragasztás közben, a kész állapot.
- Mért értékek: csíkok száma: 9; `fraction` csíkonként: …; ragasztási ciklusidő: … s.

## 8. Korlátok, továbbfejlesztés

- A fej-emelés csak csíkok között történik; a függőlegesen szomszédos cellák nem alkotnak közös csíkot.
- A szakaszok mindig balról jobbra haladnak. Kígyózó (boustrophedon) bejárással rövidebb lenne az üresjárat.
- A lap pozíciója fix (a feladat ezt megengedi), és csak markerként jelenik meg, a planning scene-ben nincs ütközési objektum.
- Valós roboton a `tool0` helyett egy ragasztófej TCP-t kellene definiálni, és a sebességet a ragasztóanyaghoz kellene hangolni.
