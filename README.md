# Clearance-Aware A\* Path Planner (ROS 2)
---
## Daftar Isi
- [Gambaran Umum](#gambaran-umum)
- [Arsitektur Sistem](#arsitektur-sistem)
- [Struktur Direktori](#struktur-direktori)
- [Dependensi](#dependensi)
- [Prasyarat & Setup Workspace](#prasyarat-&-setup-workspace)
- [Cara Menjalankan](#cara-menjalankan)
- [Cara Menggunakan (di RViz)](#cara-menggunakan-di-rviz)
- [Konfigurasi Parameter](#konfigurasi-parameter)
- [Analisis Perbandingan](#analisis-perbandingan)
- [Topik ROS 2](#topik-ros-2)
---
## Gambaran Umum
**vis_astar** adalah implementasi sistem navigasi robot 2D berbasis ROS 2 Humble yang terdiri dari:
| Komponen | Deskripsi |
|---|---|
| **Clearance A\*** | Modifikasi A\* yang menghindari jalur terlalu dekat dengan obstacle |
| **Fake Odometry** | Simulasi gerak robot tanpa Gazebo, lengkap dengan collision detection |
| **Path Follower** | Mengikuti jalur menggunakan algoritma *Pure Pursuit* |
| **Map Generator** | Membuat file peta `.pgm` + `.yaml` dari daftar obstacle |
| **Evaluator** | Benchmark perbandingan A\* biasa vs Clearance A\* pada berbagai skenario |
---
## Arsitektur Sistem
```
RViz (2D Pose Estimate)          RViz (2D Goal Pose / Nav2 Goal)
         │                                    │
         ▼                                    ▼
  /initialpose                          /goal_pose
         │                                    │
         └──────────────┬─────────────────────┘
                        ▼
               [ planner_node ]
               (ClearanceAStar)
                        │
                        ▼ /vis_path
               [ path_follower_node ]
               (Pure Pursuit Controller)
                        │
                        ▼ /cmd_vel
                 [ odom_node ]
               (Fake Odometry + Collision)
                        │
                        ▼ /odom + TF
                  Robot bergerak
```
---
## Struktur Direktori
```
vis_astar/
├── vis_astar/                   
│   ├── astar.py                 
│   ├── clearance_astar.py       # A* dengan penalty clearance terhadap obstacle
│   ├── utils.py                 # Konversi OccupancyGrid menjadi grid, BFS clearance map
│   ├── planner_node.py          # Membentuk path berwarna hijau di RViz
│   ├── path_follower_node.py    # Path Follower (Pure Pursuit)
│   ├── odom_node.py             # Odometry + collision detection
│   ├── joint_state_node.py      
│   └── evaluate.py              
├── maps/
│   ├── map_generator.py         # Generator peta alternatif
│   ├── map.pgm / map.yaml       # Peta default yang digunakan saat launch
│   ├── narrow_map.*             # Peta koridor sempit
│   ├── ushape_map.*             # Peta bentuk U
│   └── dense_map.*              # Peta banyak obstacle
├── launch/
│   └── planner.launch.py        # Main launch file
├── config/
│   ├── obstacles.py             # Posisi obstacle
│   └── vis_astar.rviz           # Tampilan RViz
├── urdf/
│   └── robot.urdf               # Deskripsi model robot
└── worlds/
    └── world.world              
```
---
## Dependensi
Pastikan sudah terinstall:
- **ROS 2 Humble** (atau Foxy/Iron, sesuaikan jika perlu)
- Package ROS 2 berikut:
  ```
  nav2_map_server
  nav2_lifecycle_manager
  robot_state_publisher
  tf2_ros
  rviz2
  ```
---
## Prasyarat & Setup Workspace

Jika belum memiliki *workspace* ROS 2 untuk projek ini, ikuti langkah-langkah di bawah ini untuk membuat struktur direktori dari awal:

```bash
# 1. Buat direktori workspace dan folder source (src)
mkdir -p ~/vis_astar_ws/src

# 2. Masuk ke folder source
cd ~/vis_astar_ws/src

# 3. Letakkan atau clone package 'vis_astar' ke dalam direktori src/ ini

# 4. Install dependensi Python yang dibutuhkan
pip install numpy

# 5. Jalankan rosdep dari root workspace untuk memastikan dependensi sistem terpenuhi
cd ~/vis_astar_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
---
## Cara Build
```bash
# Masuk ke workspace
cd ~/vis_astar_ws
# Source ROS 2
source /opt/ros/humble/setup.bash
# Build package
colcon build --packages-select vis_astar
# Source workspace hasil build
source install/setup.bash
```
---
## Cara Menjalankan
### 1. Build package
```bash
colcon build
source install/setup.bash
```
### 2. Jalankan semua node sekaligus
```bash
ros2 launch vis_astar planner.launch.py
```
Perintah ini akan menjalankan secara otomatis:
- `map_server` — memuat peta 2D
- `lifecycle_manager` — mengelola lifecycle map_server
- `robot_state_publisher` — mempublikasikan TF dari URDF
- `joint_state_node` — publisher joint state
- `odom_node` — fake odometry dengan collision detection
- `planner_node` — perencana jalur Clearance A\*
- `path_follower_node` — pengikut jalur Pure Pursuit
- `rviz2` — visualisasi (terbuka otomatis)
---
## Cara Menggunakan (di RViz)
Setelah semua node berjalan dan RViz terbuka:
### Langkah 1 — Set Posisi Awal Robot
1. Klik tombol **"2D Pose Estimate"** di toolbar RViz
2. Klik dan drag di peta untuk menentukan posisi + orientasi awal robot
3. Robot akan berpindah ke posisi tersebut
### Langkah 2 — Set Tujuan Robot
1. Klik tombol **"2D Goal Pose"** (atau **"Nav2 Goal"**)  
2. Klik titik tujuan di peta
3. Planner otomatis menghitung jalur menggunakan Clearance A\*
4. Jalur ditampilkan di RViz (topic `/vis_path`)
5. Robot bergerak mengikuti jalur secara otomatis
---
## Konfigurasi Parameter
### `odom_node` — Odometry
| Parameter | Default | Keterangan |
|---|---|---|
| `publish_rate` | `20.0` Hz | Frekuensi publish odometry |
| `initial_x` | `0.0` m | Posisi awal X robot |
| `initial_y` | `0.0` m | Posisi awal Y robot |
| `initial_yaw` | `0.0` rad | Orientasi awal robot |
| `collision_enabled` | `True` | Aktifkan deteksi tabrakan dengan obstacle |
Parameter ini bisa diubah di `launch/planner.launch.py`.
### `path_follower_node` — Pure Pursuit Controller
| Parameter | Nilai | Keterangan |
|---|---|---|
| `lookahead` | `0.6` m | Jarak lookahead Pure Pursuit |
| `linear_speed` | `0.6` m/s | Kecepatan maju robot |
| `goal_tol` | `0.35` m | Toleransi mencapai tujuan |
| `max_angular` | `2.0` rad/s | Kecepatan rotasi maksimum |
### `planner_node` — Clearance A\*
| Parameter | Nilai | Keterangan |
|---|---|---|
| `lambda_score` | `8.0` | Bobot penalty clearance (makin besar = makin menjauhi dinding) |
| `min_clearance` | `10` cells | Jarak minimum ke obstacle yang boleh dilalui |
Parameter ini bisa diubah langsung di `vis_astar/planner_node.py` pada baris:
```python
self.planner = ClearanceAStar(lambda_score=8.0, min_clearance=10)
```
---
## Analisis Perbandingan
Script `evaluate.py` membandingkan performa **A\* biasa** vs **Clearance A\*** pada beberapa skenario:
```bash
cd ~/vis_astar_ws
source install/setup.bash
python3 src/vis_astar/vis_astar/evaluate.py
```
Skenario yang diuji:
- **Narrow Corridor** — koridor sempit
- **Dense Obstacles** — obstacle padat
- **U-Shape Trap** — jebakan berbentuk U
- **Open World (100×100)** — dunia terbuka besar
Output yang ditampilkan:
| Kolom | Keterangan |
|---|---|
| `Length` | Panjang jalur total (dalam grid cells) |
| `MinClr` | Clearance minimum sepanjang jalur |
| `AvgClr` | Rata-rata clearance sepanjang jalur |
| `Safe%` | Persentase waypoint dengan clearance ≥ 3 cells |
| `Time(ms)` | Waktu komputasi planning |
| `dLen` | Selisih panjang jalur vs A\* (+ = lebih panjang, lebih aman) |
| `dMinClr` | Selisih clearance min vs A\* (+ = lebih jauh dari obstacle) |
---
## Topik ROS 2
| Topik | Tipe | Arah | Keterangan |
|---|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | Subscribe | Peta dari map_server |
| `/odom` | `nav_msgs/Odometry` | Publish | Odometri robot |
| `/cmd_vel` | `geometry_msgs/Twist` | Subscribe/Publish | Perintah gerak robot |
| `/vis_path` | `nav_msgs/Path` | Publish | Jalur hasil perencanaan |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | Subscribe | Posisi awal dari RViz |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Subscribe | Posisi tujuan dari RViz |
---