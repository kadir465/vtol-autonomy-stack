<p align="center">
  <img src="https://img.shields.io/badge/ROS%202-Humble-3498db?style=for-the-badge&logo=ros&logoColor=white" alt="ROS 2 Humble"/>
  <img src="https://img.shields.io/badge/PX4-Autopilot%20v1.14+-eb5424?style=for-the-badge&logo=px5&logoColor=white" alt="PX4 Autopilot"/>
  <img src="https://img.shields.io/badge/YOLOv8-Target%20Acquisition-111111?style=for-the-badge&logo=yolo&logoColor=white" alt="YOLOv8"/>
  <img src="https://img.shields.io/badge/Python-3.10+-3776ab?style=for-the-badge&logo=python&logoColor=white" alt="Python 3.10+"/>
  <img src="https://img.shields.io/badge/Platform-Linux%20%2F%20Ubuntu%2022.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white" alt="Ubuntu 22.04"/>
  <img src="https://img.shields.io/badge/License-MIT-success?style=for-the-badge" alt="License"/>
</p>

<h1 align="center">🛩️ VTOL Autonomy Stack</h1>
<h3 align="center">Autonomous Mission Management & Visual Servoing for Tactical Loitering Munition</h3>

<p align="center">
  <b>Taktik VTOL Dolanan Mühimmat (Kamikaze İHA) Tam Otonom Görev Yönetim Sistemi</b><br/>
  <i>ROS 2 Humble · PX4 Offboard Control · YOLOv8 Computer Vision · Circular Orbit Loiter · Precision Dive Terminal Guidance</i>
</p>

<p align="center">
  <a href="#-overview">Overview</a> •
  <a href="#-system-architecture">Architecture</a> •
  <a href="#-flight-phases-state-machine">Flight Phases</a> •
  <a href="#-ground-control-station-gcs">GCS Terminal</a> •
  <a href="#-installation--build">Installation</a> •
  <a href="#-quickstart">Quickstart</a> •
  <a href="#-simulation">Simulation</a> •
  <a href="#-configuration">Configuration</a>
</p>

---

## 🔭 Overview / Genel Bakış

**VTOL Autonomy Stack**, dikey kalkış ve iniş (VTOL) yapabilen sabit kanatlı taktik insansız hava araçları ve dolanan mühimmatlar (loitering munitions) için geliştirilmiş, yüksek güvenilirlikli bir **ROS 2** uçuş yönetim sistemidir.

Sistem, pist bağımsız dikey kalkıştan (multikopter modu) başlayıp, sabit kanada geçiş, yüksek hızlı intikal, hedef üzerinde dairesel gözetleme (loiter), derin öğrenme tabanlı hedef tespiti ve hedefe kilitlenip terminal dalış (kamikaze dalış) gerçekleştirmeye kadar olan tüm taktik uçuş profilini otonom olarak icra eder.

### ✨ Key Capabilities / Temel Özellikler

- 🚀 **Vertical Takeoff & Transition:** Multikopter modunda 30m dikey tırmanış ve tam otomatik sabit kanat (FW) aerodinamik geçişi.
- ⚡ **High-Speed Cruise & Glide:** 150m seyir irtifasında hibrit 3D pozisyon+hız vektörü ile 200 km/h intikal.
- 🔄 **Dynamic Aerodynamic Orbit (Search):** Hedef alanı üzerinde $R = \frac{V^2}{g \cdot \tan(\phi)}$ formülüyle hıza ve yatış limitine göre dinamik hesaplanan 70m yarıçaplı dairesel arama.
- 🎯 **AI-Powered Target Acquisition:** Ultralytics YOLOv8 ile gerçek zamanlı optik hedef tespiti, bounding box çıkarımı ve piksel hedef merkezleme.
- 👤 **Human-in-the-Loop (HITL) Safety:** Hedefe kilitlenildiğinde uçak otonom taarruza geçmez; hedefe bakarak pozisyon korur (`hover/loiter track`) ve yer istasyonundan operatörün **ENGAGE** taarruz onayını bekler.
- 🦅 **Visual Servo Precision Dive:** Operatör onayıyla devreye giren, hıza göre kazancı dinamik ölçeklenen (Gain Scheduling) P-kontrolcü ile 150 km/h terminal dalış.
- 🖥️ **Tactical Terminal GCS:** Curses tabanlı, düşük kaynak tüketimli, yapay ufuk ve canlı telemetri sunan yer istasyonu arayüzü.

---

## 🏗 System Architecture / Sistem Mimarisi

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                         UAV COMPANION COMPUTER (Jetson / RPi)               │
│                                                                              │
│  ┌──────────────────┐       ┌─────────────────┐    ┌──────────────────────┐  │
│  │   stream_node    │       │   vision_node   │    │    state_machine     │  │
│  │ (Camera Reader / │──────►│    (YOLOv8 AI   │───►│   (Mission Manager / │  │
│  │  JPEG Publisher) │       │ Target Tracker) │    │     Flight Brain)    │  │
│  └──────────────────┘       └─────────────────┘    └──────────┬───────────┘  │
│                                                               │ (On ENGAGE)  │
│                                                    ┌──────────▼───────────┐  │
│                                                    │     visual_servo     │  │
│                                                    │   (Terminal Dive /   │  │
│                                                    │   Balistik Pilotaj)  │  │
│                                                    └──────────┬───────────┘  │
│                             ▲                                 │              │
│                             │     uXRCE-DDS Bridge (10-50Hz)  ▼              │
│                      ┌──────┴───────────────────────────────────────┐        │
│                      │                 PX4 AUTOPILOT                │        │
│                      │       (Flight Controller / Pixhawk / SITL)   │        │
│                      └──────────────────────────────────────────────┘        │
└──────────────────────────────────────────────────────────────────────────────┘
                                      ▲
                             WiFi / Telemetry Link
                                      ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│                         GROUND CONTROL STATION (GCS)                         │
│                                                                              │
│  ┌───────────────────────────────────┐    ┌───────────────────────────────┐  │
│  │             gcs_panel             │    │         gcs_listener          │  │
│  │   (Curses-based Tactical UI &     │◄──►│ (Watchdog, Telemetry Bridge & │  │
│  │        Mission Commands)          │    │     Coordinate Validator)     │  │
│  └───────────────────────────────────┘    └───────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## 🔄 Flight Phases (State Machine)

Uçuş görevi 9 sıralı ve güvenli durumdan (FSM) oluşur:

```
  [IDLE] ──► [PREFLIGHT] ──► [TAKEOFF] ──► [TRANSITION] ──► [SLANTED_CLIMB]
                                                                  │
  [ENGAGE] ◄── [SEARCH (Lock)] ◄── [SEARCH (Orbit)] ◄── [APPROACH] ◄── [CRUISE]
 (Kamikaze     (HITL Onay        (70m Dairesel         (Kademeli
   Dalış)       Bekleme)            Arama)             Yavaşlama)
```

| Faz # | Faz Adı | Tipik İrtifa | Hedef Hız | Açıklama |
|---|---|---|---|---|
| 0 | `IDLE` | 0m | 0 km/h | Sistem başlangıcı; Home ve Hedef GPS koordinat girişi beklenir. |
| 1 | `PREFLIGHT` | 0m | 0 km/h | Sensör kontrolleri, PX4 Offboard moduna geçiş ve motor arm etme. |
| 2 | `TAKEOFF` | 0 → 30m | 50 km/h | Dikey tırmanış (Multicopter VTOL modu). |
| 3 | `TRANSITION` | 30m | Geçiş | Döner kanattan sabit kanada aerodinamik geçiş (5 sn bekleme). |
| 4 | `SLANTED_CLIMB`| 30 → 150m| 120 km/h | Hedefe doğru yönelerek tırmanış. |
| 5 | `CRUISE` | 150m | 200 km/h | Hedefe intikal (hedefe 950 metreye kadar tam hız seyir). |
| 6 | `APPROACH` | 150 → 70m | 90 → 60 km/h| Kademeli yavaşlama ve operasyon irtifasına alçalış. |
| 7 | `SEARCH` | 70m | 60 km/h | Hedef üzerinde 70m yarıçaplı dairesel orbit uçuşu ve yapay zeka araması. |
| 8 | `ENGAGE` | 70 → 0m | 150 km/h | Operatör onayı sonrası görsel servo güdümlü terminal kamikaze dalış. |

---

## 📦 Package Organization / Paket Yapısı

```bash
vtol-autonomy-stack/
├── src/
│   ├── vtol_control/              # Görev yönetimi, uçuş FSM ve yer istasyonu
│   │   ├── state_machine.py       # Ana uçuş yöneticisi (9 fazlı durum makinesi)
│   │   ├── visual_servo.py        # Görsel servo terminal dalış pilotajı
│   │   ├── gcs_panel.py           # Curses tabanlı interaktif operatör arayüzü
│   │   └── gcs_listener.py        # Watchdog güvenlik monitörü ve köprü
│   │
│   ├── vtol_vision/               # Görüntü işleme ve yapay zeka paketi
│   │   ├── vision_node.py         # YOLOv8 hedef tespit, hata kestirimi & HUD
│   │   ├── stream_node.py         # Kamera yayınlayıcı (Lazy-init, CPU dostu)
│   │   └── models/
│   │       └── best.pt            # Eğitilmiş PyTorch YOLO hedef modeli
│   │
│   ├── vtol_bringup/              # Konfigürasyon ve başlatma paketleri
│   │   ├── launch/
│   │   │   ├── uav.launch.py      # İHA tümleşik başlatıcı
│   │   │   ├── gcs.launch.py      # Yer istasyonu başlatıcı
│   │   │   └── simulation.launch.py # Gazebo & PX4 SITL köprü başlatıcı
│   │   └── config/
│   │       ├── mission_params.yaml# Uçuş hızları, irtifalar, PID kazançları
│   │       └── camera_params.yaml # Kamera çözünürlüğü, FPS ve model eşikleri
│   │
│   └── px4_msgs/                  # PX4 ROS 2 arayüz mesaj tanımları
│
├── rebuild_clean.sh               # Otomatik önbellek temizleme & derleme scripti
└── README.md
```

---

## 🖥 Ground Control Station (GCS)

Yer İstasyonu (`gcs_panel.py`), harici GUI kütüphanelerine bağımlı olmadan saf Python `curses` ile her türlü terminalde (SSH oturumları dahil) çalışan taktik bir arayüzdür:

```text
╔═══════════════════════════════════════════════════════════════════════╗
║       ██╗   ██╗████████╗ ██████╗ ██╗          ██████╗  ██████╗███████ ║
║       ██║   ██║╚══██╔══╝██╔═══██╗██║         ██╔════╝ ██╔════╝██╔════ ║
║       ██║   ██║   ██║   ██║   ██║██║         ██║  ███╗██║     ███████ ║
║       ╚██╗ ██╔╝   ██║   ██║   ██║██║         ██║   ██║██║     ╚════██ ║
║        ╚████╔╝    ██║   ╚██████╔╝███████╗    ╚██████╔╝╚██████╗███████ ║
║         ╚═══╝     ╚═╝    ╚═════╝ ╚══════╝     ╚═════╝  ╚═════╝╚══════ ║
║              GROUND  CONTROL  STATION  —  TACTICAL  TERMINAL          ║
╚═══════════════════════════════════════════════════════════════════════╝
═══════════════════════════════════════════════════════════════════════
  ◆ VTOL GCS v2.0  │  İRTİFA: 70.0m  │  HIZ: 16.7 m/s  │  LINK: ONLINE
  ╔════════════════════════════════════════════════════════════════╗
  ║      SEARCH       —  Dairesel Arama Aktif (70m Yarıçap)      ║
  ╚════════════════════════════════════════════════════════════════╝
  ⚡ ████ HEDEF KİTLİ ████  |  MOD: SABİT/HOVER  |  ENGAGE EMRİ BEKLENİYOR
```

### Klavye Kısayolları (Hotkeys)
- <kbd>T</kbd> : **TAKEOFF** — Motorları arm eder ve dikey kalkışı başlatır (Sadece `IDLE` fazında).
- <kbd>G</kbd> : **TARGET GPS** — Hedef enlem/boylam koordinatlarını ayarlar.
- <kbd>H</kbd> : **HOME GPS** — Kalkış / üs koordinatlarını ayarlar.
- <kbd>E</kbd> : **ENGAGE** — Kilitlenilen hedefe taarruz/kamikaze dalış yetkisi verir.
- <kbd>R</kbd> : **RTL** — Return to Launch (Üsse acil geri dönüş).
- <kbd>A</kbd> : **ABORT** — Görevi iptal et ve güvenli bekleme moduna geç.
- <kbd>Q</kbd> : Panelden çıkış.

---

## 🔧 Installation & Build / Kurulum

### Gereksinimler
- **İşletim Sistemi:** Ubuntu 22.04 LTS
- **ROS 2:** Humble Hawksbill
- **PX4 Autopilot:** v1.14 veya üstü
- **Python:** 3.10+
- **uXRCE-DDS Agent:** PX4 ↔ ROS 2 haberleşmesi için

### 1. Python Bağımlılıkları
```bash
pip install ultralytics opencv-python-headless torch torchvision numpy pyyaml
```

### 2. Workspace Kurulumu ve Derleme
```bash
# Workspace kök dizinine geçin
cd vtol-autonomy-stack

# ROS 2 ortamını yükleyin
source /opt/ros/humble/setup.bash

# Projeyi derleyin (veya bash rebuild_clean.sh kullanın)
colcon build --symlink-install

# Ortamı yükleyin
source install/setup.bash
```

---

## 🚀 Quickstart / Kullanım

### Gerçek Uçuş (UAV + GCS)

**1. Uçak Tarafında (Companion Computer):**
```bash
# uXRCE-DDS Agent başlatın
MicroXRCEAgent udp4 -p 8888

# Tüm uçak otonomi stack'ini başlatın
ros2 launch vtol_bringup uav.launch.py
```

**2. Yer İstasyonunda (GCS Bilgisayarı):**
```bash
ros2 launch vtol_bringup gcs.launch.py
```

---

## 🎮 Simulation (SITL & Gazebo)

Gazebo ve PX4 SITL ile tam yazılım simülasyonu (Software-in-the-Loop):

```bash
# Terminal 1: PX4 SITL Başlatma
cd ~/PX4-Autopilot
make px4_sitl gz_standard_vtol

# Terminal 2: Otonomi Stack & Bridge
ros2 launch vtol_bringup simulation.launch.py

# Terminal 3: Taktik Operatör Paneli
ros2 launch vtol_bringup gcs.launch.py
```

---

## ⚙️ Configuration / Konfigürasyon

Tüm uçuş, hız, aerodinamik limitler ve yapay zeka parametreleri koda dokunmadan YAML dosyaları üzerinden yönetilebilir:

- [`mission_params.yaml`](file:///src/vtol_bringup/config/mission_params.yaml):
  - `takeoff_altitude_m`: Kalkış irtifası (Varsayılan: `150.0 m`)
  - `loiter_altitude_m`: Arama irtifası (Varsayılan: `50.0 m`)
  - `max_bank_angle_deg`: İzin verilen maksimum yatış açısı (Varsayılan: `30.0°`)
  - `search_speed_kmh`: Dairesel arama hızı (Varsayılan: `60.0 km/h`)
  - `cruise_speed_ratio`: Seyir hızı katsayısı (Varsayılan: `3.33` $\to$ 200 km/h)
  - `engage_speed_ratio`: Dalış hızı katsayısı (Varsayılan: `2.5` $\to$ 150 km/h)

- [`camera_params.yaml`](file:///src/vtol_bringup/config/camera_params.yaml):
  - `confidence_threshold`: YOLO hedef güvenilirlik eşiği (`0.60`)
  - `image_width` / `image_height`: Çözünürlük (`640x480`)
  - `fps`: Kare hızı (`30 FPS`)

---

## 📡 ROS 2 Topic Interface

| Topic Adı | Mesaj Tipi | Yön | Açıklama |
|---|---|---|---|
| `/fmu/in/trajectory_setpoint` | `TrajectorySetpoint` | Çıkış | PX4 pozisyon, hız ve açı setpoint'leri |
| `/fmu/in/vehicle_command` | `VehicleCommand` | Çıkış | Mod değiştirme, ARM komutları |
| `/fmu/out/vehicle_local_position_v1` | `VehicleLocalPosition` | Giriş | Uçak NED koordinat ve hız telemetrisi |
| `/vision/target_error` | `Point` | Çıkış | Hedef piksel hatası ($x, y$) ve kilit durumu ($z$) |
| `/vtol/current_state` | `String` | Çıkış | Anlık durum makinesi fazı |
| `/control/operator_cmd` | `String` | Giriş | GCS operatör komutları (`TAKEOFF`, `ENGAGE`, `RTL`) |

---

## 🛡️ License
Bu proje açık kaynak topluluğu ve savunma/robotik araştırmaları için geliştirilmiştir. Detaylar için [LICENSE](LICENSE) dosyasına bakabilirsiniz.