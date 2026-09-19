#!/usr/bin/env python3
"""
VTOL Taktiksel Görev Yöneticisi - ADIM 1: Telemetri ve Dikey Kalkış (Agile Phase 1)
=============================================================================
Sadece gerçek sensör okumalarıyla (VehicleLocalPosition) çalışan, 
Offboard modda Position+Velocity maskesiyle katı kontrollü kalkış.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import Point
from px4_msgs.msg import (
    VehicleCommand, OffboardControlMode, TrajectorySetpoint,
    VehicleStatus, VehicleLocalPosition, HomePosition,
    BatteryStatus, Airspeed,
)
from enum import Enum, auto
import time
import math
import json

CODE_VERSION = "Phase_Full_Tactical_v4"

class FlightState(Enum):
    IDLE          = auto()
    PREFLIGHT     = auto()
    TAKEOFF       = auto()          # Dikey Kalkış (30m)
    TRANSITION    = auto()          # Sabit Kanata Geçiş
    SLANTED_CLIMB = auto()          # Eğimli Tırmanış (150m)
    CRUISE        = auto()          # Seyir
    APPROACH      = auto()          # Yaklaşma
    SEARCH        = auto()          # Arama
    ENGAGE        = auto()          # Dalış / Kamikaze

class VTOLMissionManager(Node):
    def __init__(self):
        super().__init__('vtol_mission_manager')

        # Irtifa ve Hız Parametreleri (Phase 1)
        self.declare_parameter('takeoff_altitude_m', 30.0)
        self.declare_parameter('cruise_altitude_m',  150.0)
        self.declare_parameter('mission_loop_hz',     10.0)
        self.declare_parameter('search_speed_kmh',     60.0)

        self.declare_parameter('loiter_k_r', 0.5)            # Yarıçapa itme/çekme agresifliği
        self.declare_parameter('loiter_lookahead_rad', 0.5)

        # ── DİNAMİK AERODİNAMİK MODEL PARAMETRELERİ ─────────────────
        self.declare_parameter('max_bank_angle_deg', 30.0)    # Maks bank açısı (derece)
        self.declare_parameter('min_loiter_radius_m', 40.0)   # Güvenlik alt sınırı (metre)
        self.declare_parameter('max_loiter_radius_m', 200.0)  # Güvenlik üst sınırı (metre)
        self.declare_parameter('altitude_radius_ref_m', 70.0) # İrtifa referans noktası (metre)
        self.declare_parameter('altitude_radius_scale', 0.3)  # İrtifa-yarıçap ölçekleme oranı

        # ── DİNAMİK HIZ KADEME ORANLARI ───────────────────────────────
        self.declare_parameter('approach_speed_ratio', 1.5)    # Yaklaşma = search × 1.5
        self.declare_parameter('cruise_speed_ratio', 3.33)     # Seyir = search × 3.33
        self.declare_parameter('engage_speed_ratio', 2.5)      # Dalış = search × 2.5
        self.declare_parameter('climb_speed_ratio', 2.0)       # Tırmanış = search × 2.0

        # ── DİNAMİK YAVAŞLAMA MESAFESİ PARAMETRELERİ ─────────────────
        self.declare_parameter('decel_rate_mps2', 1.5)         # Yavaşlama ivmesi (m/s²)
        self.declare_parameter('approach_entry_ratio', 1.3)    # CRUISE→APPROACH çarpanı
        self.declare_parameter('min_search_entry_m', 80.0)     # Min APPROACH→SEARCH mesafesi

        self._cfg_tkf_alt = 30.0
        self._cfg_crz_alt = 150.0
        self._cfg_srch_alt = 70.0  # Arama irtifasi (Dolanan Muhimmat profili)
        self._cfg_loop_hz = self.get_parameter('mission_loop_hz').value
        self._search_speed_kmh = self.get_parameter('search_speed_kmh').value

        # Hiz Limitleri: search_speed'e oransal olarak hesaplanır
        self._spd_takeoff  = 50.0 * 1000.0 / 3600.0  # Kalkış hızı (sabit, multikopter modu)
        self._spd_search   = self._search_speed_kmh * 1000.0 / 3600.0
        self._spd_climb, self._spd_cruise, self._spd_approach, self._spd_engage = \
            self._compute_dynamic_speeds(self._spd_search)

        # Dinamik faz geçiş mesafeleri (başlangıç hesaplaması)
        self._cruise_to_approach_dist = 950.0   # CRUISE→APPROACH geçiş (dinamik güncellenir)
        self._approach_decel_dist     = 650.0   # Kademe yavaşlama mesafesi (dinamik güncellenir)
        self._approach_to_search_dist = 150.0   # APPROACH→SEARCH geçiş (dinamik güncellenir)
        self._update_phase_distances()          # İlk hesaplama

        self.get_logger().info(f"VTOL Mission Manager Başlatıldı - {CODE_VERSION}")

        self._state            = FlightState.IDLE
        
        # Gerçek Telemetri (Sensör) Verileri
        self._current_x  = 0.0
        self._current_y  = 0.0
        self._current_z  = 0.0
        self._current_vx = 0.0
        self._current_vy = 0.0
        self._current_vz = 0.0

        # ── YENİ TELEMETRİ VERİLERİ ────────────────────────────────
        # Batarya
        self._battery_percent = 0.0   # 0.0 - 1.0 (volt_based_soc_estimate)
        self._battery_volt    = 0.0   # Toplam voltaj (V)
        self._battery_valid    = False
        self._last_battery_time = 0.0
        # Pusula / Heading
        self._heading_deg     = 0.0   # 0-360° (kuzey=0, saat yönü)
        self._heading_valid    = False
        # Hava Hızı
        self._airspeed_mps    = 0.0   # indicated_airspeed (m/s)
        self._airspeed_valid   = False
        self._last_airspeed_time = 0.0
        # Crab Angle (rüzgar sürüklenme açısı)
        self._crab_angle_deg  = 0.0   # heading vs ground track farkı
        # Bingo Battery (otonom RTL eşiği)
        self._bingo_battery_pct = 0.20  # Dinamik hesaplanacak min batarya
        self._bingo_rtl_triggered = False

        # State-specific flag'ları __init__'te başlat
        self._search_init_done      = False
        self._search_fw_locked      = False
        self._approach_decel_triggered = False
        self._engage_fw_speed_set   = False
        self._last_speed_cmd_t      = 0.0
        self._transition_start_t    = None
        
        self._home_x = None
        self._home_y = None
        self._home_lat = None
        self._home_lon = None
        self._target_wp = None
        self._pending_target_gps = None
        self._target_locked = False
        
        self._px4_nav_state = 0
        self._px4_arming_state = 0
        self._px4_connected = False
        self._last_px4_msg_time = 0.0
        self._arm_retry_count = 0
        self._takeoff_requested = False
        self._engage_authorized = False  # Operator ROE onay bayragi
        
        #  LOITER (SEARCH) PARAMETRELERI 
        # Yarıçap artık dinamik hesaplanıyor: R = V²/(g·tan(φ)) + irtifa ölçekleme
        self._loiter_radius = self._compute_dynamic_radius(self._spd_search, self._cfg_srch_alt)
        self._loiter_direction = 1.0 # 1: Saat yönü, -1: Ters saat yönü
        self._loiter_start_time = None
        self._search_hold_x = None  # Hedefe kitlenince sabit kalınacak pozisyon
        self._search_hold_y = None
        self._search_orbit_count = 0.0  # Toplam tur sayısı
        self._search_last_angle = None  # Açı takibi (tur sayacı)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        #  Subscribers 
        self.create_subscription(String, '/control/operator_cmd', self._operator_cb, 10)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self._status_cb, qos_profile)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self._local_pos_cb, qos_profile)
        self.create_subscription(HomePosition, '/fmu/out/home_position_v1', self._home_position_cb, qos_profile)
        self.create_subscription(Point, '/vtol/mission_target', self._target_wp_cb, 10)
        self.create_subscription(Point, '/vtol/home_set', self._manual_home_cb, 10)
        self.create_subscription(Point, '/vision/target_error', self._vision_cb, 10)
        self.create_subscription(String, '/vtol/engage_auth', self._engage_auth_cb, 10)

        # ── YENİ PX4 TELEMETRİ ABONELİKLERİ ─────────────────────────
        self.create_subscription(
            BatteryStatus, '/fmu/out/battery_status_v1',
            self._battery_cb, qos_profile)
        self.create_subscription(
            Airspeed, '/fmu/out/airspeed_v1',
            self._airspeed_cb, qos_profile)

        #  Publishers 
        self._status_pub       = self.create_publisher(String, '/vtol/current_state', 10)
        self._log_pub          = self.create_publisher(String, '/vtol/mission_log', 10)
        self._camera_pub       = self.create_publisher(String, '/vision/camera_trigger', 10)
        self._search_info_pub  = self.create_publisher(String, '/vtol/search_info', 10)
        
        self._cmd_pub          = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self._offboard_pub     = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self._traj_pub         = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)

        self._timer = self.create_timer(1.0 / self._cfg_loop_hz, self._mission_loop)

        self.VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
        self.VEHICLE_CMD_DO_SET_MODE = 176

    def _log(self, text, level="INFO", throttle_sec=0.0):
        t = time.time()
        if throttle_sec > 0:
            if not hasattr(self, '_last_log_t'):
                self._last_log_t = {}
            if text in self._last_log_t and (t - self._last_log_t[text] < throttle_sec):
                return
            self._last_log_t[text] = t
            
        self.get_logger().info(text)
        msg = String()
        msg.data = f"[{level}] {text}"
        self._log_pub.publish(msg)

    def _transition(self, new_state: FlightState):
        # Yeni faza girerken bir önceki fazın kalıntılarını temizle
        if new_state == FlightState.SEARCH:
            self._search_init_done      = False
            self._search_fw_locked      = False
            self._last_speed_cmd_t      = 0.0
            self._search_hold_x         = None
            self._search_hold_y         = None
            self._search_orbit_count    = 0.0
            self._search_last_angle     = None
        elif new_state == FlightState.APPROACH:
            self._approach_decel_triggered = False
        elif new_state == FlightState.ENGAGE:
            self._engage_fw_speed_set   = False
        elif new_state == FlightState.TRANSITION:
            self._transition_start_t    = None

        self._state = new_state
        self._log(f"STATE -> {new_state.name}", level="WARN")

    def _status_cb(self, msg: VehicleStatus):
        self._px4_nav_state = msg.nav_state
        self._px4_arming_state = msg.arming_state
        self._px4_connected = True
        self._last_px4_msg_time = time.time()

    def _local_pos_cb(self, msg: VehicleLocalPosition):
        # GERÇEK TELEMETRİ OKUMASI
        self._current_x = msg.x
        self._current_y = msg.y
        self._current_z = msg.z
        self._current_vx = msg.vx
        self._current_vy = msg.vy
        self._current_vz = msg.vz
        
        self._px4_connected = True
        self._last_px4_msg_time = time.time()

        # ── HEADING (PUSULA) GÜNCELLEMESİ ──
        # EKF2 tarafından üretilen VehicleLocalPosition.heading radyan
        if not math.isnan(msg.heading):
            hdg_deg = math.degrees(msg.heading)
            if hdg_deg < 0:
                hdg_deg += 360.0
            self._heading_deg = hdg_deg
            self._heading_valid = True
        else:
            self._heading_valid = False

        # ── CRAB ANGLE HESAPLAMA (Yer Hızı Vektörü vs Heading) ──
        ground_track_rad = math.atan2(msg.vy, msg.vx)  # Yer hızı yön açısı (NED)
        ground_track_deg = math.degrees(ground_track_rad) % 360.0
        if self._heading_valid:
            crab = self._heading_deg - ground_track_deg
            # -180..+180 normalize
            if crab > 180.0:
                crab -= 360.0
            elif crab < -180.0:
                crab += 360.0
            self._crab_angle_deg = crab

    def _battery_cb(self, msg: BatteryStatus):
        """
        BATARYA TELEMETRİSİ
        ════════════════════
        PX4'ten gelen batarya durumunu okur:
          - remaining: 0.0-1.0 arası gerçek kapasite oranı (QGC ile aynı)
          - voltage_v: Toplam paket voltajı
        """
        soc = msg.remaining
        volt = msg.voltage_v
        
        # NaN ve -1 (geçersiz) kontrolü
        if not math.isnan(soc) and soc >= 0.0 and not math.isnan(volt):
            self._battery_percent = max(0.0, min(1.0, soc))
            self._battery_volt = volt
            self._battery_valid = True
            self._last_battery_time = time.time()



    def _airspeed_cb(self, msg: Airspeed):
        """
        HAVA HIZI TELEMETRİSİ
        ══════════════════════
        Pitot tüpünden gelen indicated airspeed (IAS) okunur.
        IAS, aerodinamik kuvvetlerin doğru ölçüsüdür:
          - Stall hızı IAS'a bağlıdır (irtifadan bağımsız)
          - Kontrol yüzeyi verimliliği IAS'a bağlıdır
        
        Geçersiz veri (NaN) durumunda _airspeed_valid = False olur
        ve sistem yer hızına geri döner (fallback).
        """
        ias = msg.indicated_airspeed_m_s
        
        if not math.isnan(ias) and ias > 0.5:  # 0.5 m/s altı gürültü
            self._airspeed_mps = ias
            self._airspeed_valid = True
            self._last_airspeed_time = time.time()
        else:
            # Sensör arızası veya veri yok → fallback
            self._airspeed_valid = False

    def _get_effective_speed_mps(self):
        """
        ETKİN HIZ SEÇİCİ (Airspeed Fallback Mekanizması)
        ═════════════════════════════════════════════════
        Öncelik:
          1. Hava hızı (Pitot sensör) → aerodinamik doğruluk
          2. Yer hızı (GPS/EKF)      → yedek/fallback
          
        Hava hızı 2 saniyeden eski veya geçersizse yer hızına döner.
        
        Returns:
            (speed_mps, source_str): Etkin hız ve kaynak etiketi
        """
        airspeed_age = time.time() - self._last_airspeed_time
        
        if self._airspeed_valid and airspeed_age < 2.0:
            return self._airspeed_mps, "AIR"
        else:
            ground_speed = math.hypot(self._current_vx, self._current_vy)
            return ground_speed, "GND"

    def _compute_bingo_battery(self):
        """
        BINGO BATTERY — DİNAMİK EVE DÖNÜŞ ENERJİSİ HESAPLAYICISI
        ══════════════════════════════════════════════════════════════
        Uçağın anlık Home pozisyonuna olan uzaklığı,
        yer hızı (rüzgar etkisi dahil) ve güvenlik marjını
        hesaba katarak dönüş için gereken minimum batarya yüzdesini hesaplar.
        
        Formül:
          t_return  = dist_to_home / ground_speed     (saniye)
          bingo_pct = (t_return / max_endurance_sec) + safety_margin
        
        Not: max_endurance kabaca 45 dakika (2700s) olarak alınmıştır.
             Gerçek değer batarya kapasitesine göre ayarlanmalıdır.
        """
        if self._home_x is None or self._home_y is None:
            return 0.20  # Home yok → mutlak alt sınır
        
        # Home'a olan mesafe
        dx = self._current_x - self._home_x
        dy = self._current_y - self._home_y
        dist_home = math.sqrt(dx * dx + dy * dy)
        
        # Yer hızı (rüzgar dahil gerçek ilerleme)
        ground_speed = math.hypot(self._current_vx, self._current_vy)
        ground_speed = max(5.0, ground_speed)  # Min 5 m/s (en kötü rüzgar)
        
        # Tahmini dönüş süresi (saniye)
        t_return_sec = dist_home / ground_speed
        
        # Maksimum uçuş süresi (batarya kapasitesine göre ayarlanmalı)
        max_endurance_sec = 2700.0  # ~45 dakika
        
        # Güvenlik marjı (%5 daha fazla tut)
        safety_margin = 0.05
        
        # Dönüş için gereken batarya yüzdesi
        bingo_pct = (t_return_sec / max_endurance_sec) + safety_margin
        
        # Mutlak alt sınır: %20
        return max(0.20, min(0.80, bingo_pct))

    def _operator_cb(self, msg: String):
        cmd = msg.data.strip().upper()
        if cmd == "TAKEOFF" and self._state == FlightState.IDLE:
            self._takeoff_requested = True
            self._log("Operator TAKEOFF komutu alindi.", level="WARN")
        elif cmd in ["ENGAGE", "E"]:
            if self._state == FlightState.SEARCH:
                self._engage_authorized = True
                self._log("OPERATOR ENGAGE ONAY VERDI! Dalisa geciliyor...", level="ERROR")
            else:
                self._log("ENGAGE komutu reddedildi: Ucak SEARCH modunda degil!", level="WARN")

    def _vision_cb(self, msg: Point):
        # visual_servo kodundan gelen target_error icerisindeki Z degeri Lock bayragidir.
        if msg.z > 0.5:
            self._target_locked = True
        else:
            self._target_locked = False

    def _engage_auth_cb(self, msg: String):
        """GCS panelinden veya CLI'dan gelen ENGAGE yetkilendirmesi."""
        cmd = msg.data.strip().upper()
        if cmd in ["TRUE", "ENGAGE", "1", "E"]:
            if self._state == FlightState.SEARCH:
                self._engage_authorized = True
                self._log("ENGAGE_AUTH ALINDI: Operator dalisa onay verdi!", level="ERROR")
            else:
                self._log("ENGAGE_AUTH reddedildi: Ucak SEARCH modunda degil.", level="WARN")

    def _home_position_cb(self, msg: HomePosition):
        """PX4'ten kalkis noktasinin GPS enlem/boylamini al."""
        if self._home_lat is not None:
            return  # Zaten alindi, tekrar isleme

        self._home_lat = msg.lat
        self._home_lon = msg.lon
        self._log(f"HOME POZISYONU BULUNDU: Lat: {msg.lat:.6f}, Lon: {msg.lon:.6f}", level="WARN")

        # Hafizada bekleyen bir GPS hedefi varsa simdi otomatik cevir
        if self._pending_target_gps is not None:
            self._calculate_ned_target(self._pending_target_gps.x, self._pending_target_gps.y)
            self._pending_target_gps = None

    def _manual_home_cb(self, msg: Point):
        """GCS üzerinden manuel olarak gönderilen home pozisyonunu set eder."""
        self._home_lat = msg.x
        self._home_lon = msg.y
        self._log(f"MANUEL HOME SET EDILDI: Lat: {self._home_lat:.6f}, Lon: {self._home_lon:.6f}", level="WARN")

        # Hafizada bekleyen bir GPS hedefi varsa simdi otomatik cevir
        if self._pending_target_gps is not None:
            self._calculate_ned_target(self._pending_target_gps.x, self._pending_target_gps.y)
            self._pending_target_gps = None

    def _target_wp_cb(self, msg: Point):
        """GCS'den gelen Lat/Lon (x=lat, y=lon) verisini alir. Home varsa cevirir, yoksa hafizada tutar."""
        if self._home_lat is None or self._home_lon is None:
            self._pending_target_gps = msg
            self._log("GPS hedefi hafizaya alindi. Home pozisyonu bekleniyor...", level="WARN", throttle_sec=2.0)
            return

        self._calculate_ned_target(msg.x, msg.y)

    def _calculate_ned_target(self, target_lat, target_lon):
        """Matematiksel GPS -> NED Cevirici."""
        R = 6371000.0
        dlat = math.radians(target_lat - self._home_lat)
        dlon = math.radians(target_lon - self._home_lon)

        north = dlat * R
        east  = dlon * R * math.cos(math.radians(self._home_lat))

        p = Point()
        p.x = float(north)
        p.y = float(east)
        p.z = 0.0
        self._target_wp = p

        gercek_mesafe = math.sqrt(north**2 + east**2)
        self._log(f"GPS->NED Cevrildi! N={north:.1f}m, E={east:.1f}m (Mesafe: {gercek_mesafe:.0f}m)", level="WARN")

    def send_vehicle_command(self, command, p1=0.0, p2=0.0, p3=0.0, p4=0.0, p5=0.0, p6=0.0, p7=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1, msg.param2, msg.param3, msg.param4 = float(p1), float(p2), float(p3), float(p4)
        msg.param5, msg.param6, msg.param7 = float(p5), float(p6), float(p7)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._cmd_pub.publish(msg)

    def set_mode_offboard(self):
        # Base=1, Custom=6, Sub=0 (Offboard)
        self.send_vehicle_command(self.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0, 0.0)

    def arm(self):
        self.send_vehicle_command(self.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0)

    def publish_offboard_heartbeat(self, use_velocity=True, use_position=True):
        msg = OffboardControlMode()
        msg.position = use_position
        msg.velocity = use_velocity
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._offboard_pub.publish(msg)

    def publish_trajectory_takeoff(self, x, y, velocity_z):
        sp = TrajectorySetpoint()
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # Mermi Sabitlemesi: Z konumu yoksayilir, ucak sadece HIZ limitine gore yukselir
        sp.position = [float(x), float(y), float('nan')]
        sp.velocity = [0.0, 0.0, float(velocity_z)]
        sp.yaw = float('nan') 
        
        sp.acceleration = [float('nan')] * 3
        sp.jerk = [float('nan')] * 3
        self._traj_pub.publish(sp)

    def publish_trajectory_hover(self, x, y, target_z):
        sp = TrajectorySetpoint()
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        sp.position = [float(x), float(y), float(target_z)]
        sp.velocity = [float('nan')] * 3
        sp.yaw = float('nan') 
        sp.acceleration = [float('nan')] * 3
        sp.jerk = [float('nan')] * 3
        self._traj_pub.publish(sp)

    def publish_trajectory_hover_track(self, hold_x, hold_y, target_z, track_x, track_y):
        sp = TrajectorySetpoint()
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        sp.position = [float(hold_x), float(hold_y), float(target_z)]
        sp.velocity = [float('nan')] * 3
        dx = track_x - hold_x
        dy = track_y - hold_y
        sp.yaw = float(math.atan2(dy, dx))
        sp.acceleration = [float('nan')] * 3
        sp.jerk = [float('nan')] * 3
        self._traj_pub.publish(sp)

    def publish_trajectory_loiter(self, center_x, center_y, target_z):
        """
        POZİSYON+HIZ HİBRİT DAİRESEL ARAMA
        ═══════════════════════════════════════
        Daire üzerinde bir "lookahead" noktası hesaplayıp hem pozisyon hem hız
        olarak PX4'e gönderir. Saf velocity (NaN pozisyon) yaklaşımı FW modda
        çalışmadığı için bu hibrit yöntem kullanılır.
        
        Algoritma:
          1. Uçağın merkeze olan açısını bul
          2. Açıyı lookahead kadar ilerlet → daire üzerinde hedef noktası
          3. Teğet hız vektörü + radyal düzeltme → velocity komutu
          4. Pozisyon + Velocity birlikte gönder → PX4 hem yönü hem hızı bilir
        """
        sp = TrajectorySetpoint()
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        #  1. Uçağın Merkeze Olan Uzaklığı ve Açısı 
        dx = self._current_x - center_x
        dy = self._current_y - center_y
        current_radius = math.sqrt(dx**2 + dy**2)
        
        # Merkeze çok yakınsa (başlangıç anı) → güvenli açı
        if current_radius < 1.0:
            radial_angle = 0.0
        else:
            radial_angle = math.atan2(dy, dx)

        #  2. LOOKAHEAD NOKTASI (Daire Üzerinde İleri Hedef) 
        lookahead_rad = self.get_parameter('loiter_lookahead_rad').value  # 0.5 rad (~29°)
        target_angle = radial_angle + (self._loiter_direction * lookahead_rad)
        
        # Daire üzerindeki hedef pozisyon
        orbit_target_x = center_x + self._loiter_radius * math.cos(target_angle)
        orbit_target_y = center_y + self._loiter_radius * math.sin(target_angle)

        #  3. TEĞET HIZ VEKTÖRÜ (Dönüş Yönünde) 
        tangent_angle = radial_angle - (math.pi / 2.0 * self._loiter_direction)
        v_tangent_x = self._spd_search * math.cos(tangent_angle)
        v_tangent_y = self._spd_search * math.sin(tangent_angle)

        #  4. RADYAL DÜZELTME (Yarıçap Hata Telafisi) 
        radius_error = current_radius - self._loiter_radius
        k_r = self.get_parameter('loiter_k_r').value  # Agresiflik katsayısı
        v_radial_mag = -1.0 * k_r * radius_error
        v_radial_mag = max(min(v_radial_mag, 8.0), -8.0)  # Maks ±8 m/s düzeltme

        vx_total = v_tangent_x + v_radial_mag * math.cos(radial_angle)
        vy_total = v_tangent_y + v_radial_mag * math.sin(radial_angle)

        #  5. HİBRİT GÖNDER: Pozisyon (lookahead) + Hız (teğet+radyal) 
        sp.position = [float(orbit_target_x), float(orbit_target_y), float(target_z)]
        sp.velocity = [float(vx_total), float(vy_total), float('nan')]
        
        # Yaw: Hız vektörü yönüne bak (FW doğal uçuş yönü)
        sp.yaw = float(math.atan2(vy_total, vx_total))

        sp.acceleration = [float('nan')] * 3
        sp.jerk         = [float('nan')] * 3
        self._traj_pub.publish(sp)

    def _compute_dynamic_speeds(self, search_speed_mps):
        """
        DİNAMİK HIZ KADEME HESAPLAYICISI
        ════════════════════════════════════════
        Tüm faz hızlarını search_speed'e oransal hesaplar:
          climb    = search × climb_ratio
          cruise   = search × cruise_ratio
          approach = search × approach_ratio
          engage   = search × engage_ratio
        
        Returns:
            (spd_climb, spd_cruise, spd_approach, spd_engage) — m/s cinsinden
        """
        r_climb    = self.get_parameter('climb_speed_ratio').value
        r_cruise   = self.get_parameter('cruise_speed_ratio').value
        r_approach = self.get_parameter('approach_speed_ratio').value
        r_engage   = self.get_parameter('engage_speed_ratio').value

        return (
            search_speed_mps * r_climb,
            search_speed_mps * r_cruise,
            search_speed_mps * r_approach,
            search_speed_mps * r_engage,
        )

    def _compute_decel_distance(self, v_high_mps, v_low_mps):
        """
        DİNAMİK YAVAŞLAMA MESAFESİ HESAPLAYICISI
        ══════════════════════════════════════════
        Kinetik enerji farkına dayalı frenleme mesafesi:
          D = (V_high² - V_low²) / (2 × a_decel)
        
        Args:
            v_high_mps: Başlangıç hızı (m/s)
            v_low_mps:  Hedef hız (m/s)
        Returns:
            Gerekli yavaşlama mesafesi (metre)
        """
        a_decel = self.get_parameter('decel_rate_mps2').value
        a_decel = max(0.1, a_decel)  # Sıfıra bölme koruması
        
        d = (v_high_mps ** 2 - v_low_mps ** 2) / (2.0 * a_decel)
        return max(50.0, d)  # Minimum 50m güvenlik

    def _update_phase_distances(self):
        """
        FAZ GEÇİŞ MESAFELERİNİ GÜNCELLE
        ════════════════════════════════════════
        CRUISE→APPROACH ve APPROACH→SEARCH mesafelerini
        mevcut hız profiline göre yeniden hesaplar.
        """
        entry_ratio = self.get_parameter('approach_entry_ratio').value
        min_search_entry = self.get_parameter('min_search_entry_m').value

        # APPROACH→SEARCH: approach hızından search hızına yavaşlama
        d_approach_to_search = self._compute_decel_distance(
            self._spd_approach, self._spd_search
        )
        self._approach_to_search_dist = max(min_search_entry, d_approach_to_search)

        # Kademe yavaşlama noktası: daha uzaktan başla
        self._approach_decel_dist = self._approach_to_search_dist * 1.8

        # CRUISE→APPROACH: cruise hızından approach hızına yavaşlama × giriş oranı
        d_cruise_to_approach = self._compute_decel_distance(
            self._spd_cruise, self._spd_approach
        )
        self._cruise_to_approach_dist = d_cruise_to_approach * entry_ratio

    def _compute_dynamic_radius(self, speed_mps, altitude_m):
        """
        DİNAMİK DAİRESEL DÖNÜŞ YARICAPI HESAPLAYICISI
        ═══════════════════════════════════════════════
        Aerodinamik Temel:
          R_aero = V² / (g × tan(φ))
        
        İrtifa-FOV Ölçeklemesi:
          İrtifa referanstan yüksekse yarıçap büyür (kamera daha geniş görür),
          alçaksa yarıçap küçülür (daha hassas arama).
          R_final = R_aero × (1 + scale × (alt - ref) / ref)
        
        Güvenlik: Sonuç min/max sınırları arasında kelepçelenir.
        
        Args:
            speed_mps:  Arama hızı (m/s)
            altitude_m: Mevcut arama irtifası (metre AGL, pozitif)
        Returns:
            Kelepçelenmiş dinamik yarıçap (metre)
        """
        g = 9.80665  # Standart yerçekimi ivmesi (m/s²)
        
        # Parametreleri oku
        phi_deg = self.get_parameter('max_bank_angle_deg').value
        r_min   = self.get_parameter('min_loiter_radius_m').value
        r_max   = self.get_parameter('max_loiter_radius_m').value
        alt_ref = self.get_parameter('altitude_radius_ref_m').value
        alt_scale = self.get_parameter('altitude_radius_scale').value
        
        # Bank açısını radyana çevir (güvenlik: 5°-60° arası)
        phi_deg = max(5.0, min(60.0, phi_deg))
        phi_rad = math.radians(phi_deg)
        
        # ── ADIM 1: Aerodinamik Yarıçap ──
        # R = V² / (g × tan(φ))
        tan_phi = math.tan(phi_rad)
        if tan_phi < 0.01:  # Sıfıra bölme koruması
            tan_phi = 0.01
        r_aero = (speed_mps ** 2) / (g * tan_phi)
        
        # ── ADIM 2: İrtifa-FOV Ölçeklemesi ──
        # İrtifa referanstan farklıysa yarıçapı ölçekle
        if alt_ref > 1.0 and altitude_m > 1.0:
            alt_ratio = (altitude_m - alt_ref) / alt_ref
            altitude_factor = 1.0 + (alt_scale * alt_ratio)
            # Ölçekleme faktörünü 0.5-2.0 arasında tut
            altitude_factor = max(0.5, min(2.0, altitude_factor))
        else:
            altitude_factor = 1.0
        
        r_final = r_aero * altitude_factor
        
        # ── ADIM 3: Güvenlik Kelepçeleme ──
        r_clamped = max(r_min, min(r_max, r_final))
        
        return r_clamped

    def _get_real_speed_kmh(self):
        """Gercek 3D hiz vektorunun buyuklugunu km/h cinsinden dondurur."""
        return math.sqrt(self._current_vx**2 + self._current_vy**2 + self._current_vz**2) * 3.6

    def _publish_search_info(self, alt, speed_kmh, dist, mode_str):
        """Panel'e JSON formatında arama durumu bilgisi yayınlar.
        Telemetri verilerini (batarya, heading, airspeed, crab angle) içerir."""
        effective_spd, spd_src = self._get_effective_speed_mps()
        info = {
            "target_locked": self._target_locked,
            "engage_authorized": self._engage_authorized,
            "mode": mode_str,
            "altitude": round(alt, 1),
            "speed_kmh": round(speed_kmh, 1),
            "distance_m": round(dist, 1),
            "orbit_radius_m": self._loiter_radius,
            "orbit_count": round(self._search_orbit_count, 1),
            "search_speed_target_kmh": self._search_speed_kmh,
            "search_alt_target_m": self._cfg_srch_alt,
            # ── YENİ TELEMETRİ ALANLARI ──
            "battery_percent": round(self._battery_percent * 100, 1),
            "battery_volt": round(self._battery_volt, 2),
            "battery_valid": self._battery_valid,
            "heading_deg": round(self._heading_deg, 1),
            "heading_valid": self._heading_valid,
            "airspeed_mps": round(effective_spd, 1),
            "airspeed_valid": self._airspeed_valid,
            "airspeed_source": spd_src,
            "crab_angle_deg": round(self._crab_angle_deg, 1),
            "bingo_battery_pct": round(self._bingo_battery_pct * 100, 1),
        }
        msg = String()
        msg.data = json.dumps(info)
        self._search_info_pub.publish(msg)

    def publish_trajectory_slanted(self, target_x, target_y, target_z, speed_ms):
        """
        FW Eğimli Tırmanış/Seyir/Yaklaşma: Pozisyon + Hız Vektörü Hibrit Kontrol.
        PX4 Offboard modda hızı sınırlamak için velocity alanı ZORUNLUDUR.
        Sadece pozisyon verilirse otopilot kendi hesapladığı hızla uçar.
        """
        sp = TrajectorySetpoint()
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        #  1. YATAY YÖN VEKTÖRLERİ 
        dx = target_x - self._current_x
        dy = target_y - self._current_y
        dist_xy = math.sqrt(dx**2 + dy**2)

        if dist_xy > 1.0:
            vx = (dx / dist_xy) * speed_ms
            vy = (dy / dist_xy) * speed_ms
        else:
            vx, vy = 0.0, 0.0

        #  2. DİKEY HIZ (İRTİFA ZORLAMASI) 
        alt_error = self._current_z - target_z  # Örn: -35 - (-150) = 115m
        if alt_error > 5.0:
            vz = -15.0  # Hedefin ALTINDA → Tırman (NED'de yukarı = negatif Z)
        elif alt_error < -5.0:
            vz = 15.0   # Hedefin ÜSTÜNDE → İn
        else:
            vz = 0.0    # İrtifaya ulaştı

        #  3. HİBRİT GÖNDER: Pozisyon + Hız 
        sp.position = [float(target_x), float(target_y), float(target_z)]
        sp.velocity = [float(vx), float(vy), float(vz)]
        sp.yaw = math.atan2(dy, dx)

        sp.acceleration = [float('nan')] * 3
        sp.jerk = [float('nan')] * 3
        self._traj_pub.publish(sp)

    def publish_trajectory_engage(self, target_x, target_y, target_z):
        """
        Terminal Dalış: Strict Glideslope & TECS Overpower.
        Hedefi yer altinda (-Z) göstererek uçağın maksimum yunuslama açısıyla dalmasını zorlar.
        """
        sp = TrajectorySetpoint()
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # NED kordinat sisteminde pozitif Z aşağıyı gösterir. 
        # Hedefin 150 metre altı (sert dalış açısı oluşturur).
        deep_target_z = 150.0 

        # Vektör hesaplaması
        dx = target_x - self._current_x
        dy = target_y - self._current_y
        dz = deep_target_z - self._current_z 

        dist_3d = math.sqrt(dx**2 + dy**2 + dz**2)

        if dist_3d > 1.0:
            vx = (dx / dist_3d) * self._spd_engage
            vy = (dy / dist_3d) * self._spd_engage
            vz = (dz / dist_3d) * self._spd_engage 
        else:
            vx = vy = vz = 0.0

        # İki kontrolcüyü de (Pos+Vel) hedefe doğru kilitliyoruz.
        sp.position = [float(target_x), float(target_y), float(deep_target_z)] 
        sp.velocity = [float(vx), float(vy), float(vz)]
        
        sp.yaw = math.atan2(dy, dx)

        sp.acceleration = [float('nan')] * 3
        sp.jerk = [float('nan')] * 3
        self._traj_pub.publish(sp)

    def _mission_loop(self):
        #  DİNAMİK PARAMETRE GÜNCELLEME BLOĞU
        #  Her döngüde tüm hız/yarıçap/mesafe parametreleri yenilenir.
        
        # ── 1. TEMEL HIZ GÜNCELLEME ──
        self._search_speed_kmh = self.get_parameter('search_speed_kmh').value
        self._spd_search = self._search_speed_kmh * 1000.0 / 3600.0
        
        # ── 2. ORANSAL HIZ KADEMELERİ ──
        old_approach = self._spd_approach
        self._spd_climb, self._spd_cruise, self._spd_approach, self._spd_engage = \
            self._compute_dynamic_speeds(self._spd_search)
        
        # ── 3. DİNAMİK FAZ GEÇİŞ MESAFELERİ ──
        self._update_phase_distances()
        
        # Hız kademesi değiştiyse logla
        if abs(self._spd_approach - old_approach) > 0.5:
            self._log(
                f"HIZ KADEMELERİ GÜNCELLENDİ: "
                f"Search={self._search_speed_kmh:.0f}, "
                f"Approach={self._spd_approach*3.6:.0f}, "
                f"Cruise={self._spd_cruise*3.6:.0f}, "
                f"Engage={self._spd_engage*3.6:.0f} km/h | "
                f"Mesafeler: C→A={self._cruise_to_approach_dist:.0f}m, "
                f"A→S={self._approach_to_search_dist:.0f}m",
                throttle_sec=5.0
            )
        
        # ── 4. DİNAMİK YARICAP GÜNCELLEME (AIRSPEED ENDEKSLİ) ──
        current_alt_agl = max(1.0, -self._current_z)  # NED -> AGL (pozitif metre)
        old_radius = self._loiter_radius
        
        # Airspeed Fallback: Pitot sensör verisi varsa onu kullan, yoksa search hızı
        effective_speed, spd_source = self._get_effective_speed_mps()
        # Arama fazında etkin hızı kullan, diğer fazlarda nominal arama hızı
        radius_speed = effective_speed if self._state == FlightState.SEARCH else self._spd_search
        self._loiter_radius = self._compute_dynamic_radius(radius_speed, current_alt_agl)
        
        # Yarıçap önemli ölçüde değiştiyse logla
        if abs(self._loiter_radius - old_radius) > 2.0:
            phi_deg = self.get_parameter('max_bank_angle_deg').value
            self._log(
                f"DİNAMİK YARICAP: {self._loiter_radius:.1f}m "
                f"(Hız={effective_speed*3.6:.0f}km/h [{spd_source}], İrtifa={current_alt_agl:.0f}m, "
                f"Bank={phi_deg:.0f}°)",
                throttle_sec=3.0
            )
        
        # ── 5. BINGO BATTERY — OTONOM ENERJİ FAİLSAFE ──
        if self._battery_valid:
            self._bingo_battery_pct = self._compute_bingo_battery()
            
            # Batarya bingo eşiğinin altına düştü mü?
            if (self._battery_percent <= self._bingo_battery_pct
                    and not self._bingo_rtl_triggered
                    and self._state not in (FlightState.IDLE, FlightState.PREFLIGHT)):
                self._bingo_rtl_triggered = True
                self._log(
                    f"⚠ BINGO BATTERY! Batarya: %{self._battery_percent*100:.0f} "
                    f"(Eşik: %{self._bingo_battery_pct*100:.0f}). "
                    f"OTONOM RTL BAŞLATILIYOR — Operatör onayı beklenmedi!",
                    level="ERROR"
                )
                # PX4 RTL komutu: NAV_CMD_RETURN_TO_LAUNCH (20)
                self.send_vehicle_command(20)
        
        # ── 6. CRAB ANGLE UYARI ──
        if abs(self._crab_angle_deg) > 15.0 and self._state in (FlightState.SEARCH, FlightState.APPROACH):
            self._log(
                f"⚠ YÜKSEK CRAB ANGLE: {self._crab_angle_deg:+.1f}° — "
                f"Aşırı yan rüzgar! Hassas dalış riski.",
                throttle_sec=10.0, level="WARN"
            )
        
        self._status_pub.publish(String(data=self._state.name))
        
        # Bağlantı koptu mu?
        if self._last_px4_msg_time > 0 and (time.time() - self._last_px4_msg_time) > 2.0:
            self._px4_connected = False

        if self._state == FlightState.IDLE:
            # Sıkı Kontrol: Önce Home, Sonra Hedef, Sonra Takeoff
            if self._home_lat is None:
                self._log("IDLE: GÖREV BLOKE! Lütfen önce GCS üzerinden MANUEL HOME koordinatlarını girin.", throttle_sec=5.0, level="WARN")
            elif self._target_wp is None:
                self._log("IDLE: Home OK. Lütfen GCS üzerinden HEDEF koordinatlarını girin.", throttle_sec=5.0, level="WARN")
            elif self._takeoff_requested:
                # Koordinatlar tam ve operatör TAKEOFF dedi
                self._log("IDLE: Koordinatlar doğrulandı. Kalkış başlatılıyor...", level="WARN")
                self._transition(FlightState.PREFLIGHT)
            else:
                if self._px4_connected:
                    self._log("IDLE: Sistem hazır (Home & Hedef Mevcut). Operatör TAKEOFF bekliyor.", throttle_sec=10.0)
                else:
                    self._log("IDLE: PX4 Telemetri bekleniyor...", throttle_sec=5.0)

        elif self._state == FlightState.PREFLIGHT:
            if not self._px4_connected: return
            
            # Offboard Heartbeat baslamadan once stream isitilmalidir.
            self.publish_offboard_heartbeat(use_velocity=False, use_position=True)
            # Ucak buludugu yeri merkez seciyor
            if self._home_x is None:
                self._home_x = self._current_x
                self._home_y = self._current_y

            self.publish_trajectory_takeoff(self._home_x, self._home_y, -self._spd_takeoff)

            is_armed = self._px4_arming_state == 2
            if not is_armed:
                self._arm_retry_count += 1
                if self._arm_retry_count > 30:
                    self._transition(FlightState.IDLE)
                    return
                # ARM atmadan once limiti bastiralim (PX4 uzerinde param1=2 Climb Speed anlamindadir)
                self.send_vehicle_command(178, 2.0, self._spd_takeoff, -1.0)
                self.arm()
                self._log("PREFLIGHT: ARM ediliyor...", throttle_sec=1.0)
            else:
                if self._px4_nav_state != 14: # 14 = OFFBOARD
                    self.set_mode_offboard()
                    self._log("PREFLIGHT: Offboard moduna ayarlanıyor...", throttle_sec=1.0)
                else:
                    self._log("BAŞARILI: Kalkış noktasına kilitlendi, motorlar ARM ve OFFBOARD mod devrede.", level="WARN")
                    self._log("KRİTİK UYARI: Dalış performansı için 'FW_P_LIM_MIN = -60.0' olduğunu doğrulayın!", level="ERROR")
                    self._transition(FlightState.TAKEOFF)

        elif self._state == FlightState.TAKEOFF:
            # Saniyede 10 kere heartbeat ve mermi hizinda setpoint gonderilecek
            self.publish_offboard_heartbeat(use_velocity=True, use_position=True)
            self.publish_trajectory_takeoff(self._home_x, self._home_y, -self._spd_takeoff)
            
            if self._px4_nav_state != 14:
                self.set_mode_offboard()

            # Gercek sensör verisi ile kontrol (-z yukseklik, -vz tırmanış hızımız)
            current_alt = -self._current_z
            current_spd = -self._current_vz 
            
            self._log(f"FAZ 1: Dikey Kalkış Devam Ediyor. Anlık İrtifa: {current_alt:.1f}m, Hız: {(current_spd*3.6):.1f}km/h", throttle_sec=1.0)
            
            # Faz Geçiş Kilidi (Strict 30m lock)
            if current_alt >= self._cfg_tkf_alt - 0.2:
                self._log(f"FAZ 1 TAMAMLANDI! {self._cfg_tkf_alt} Metre irtifaya başarıyla ulaşıldı ve FREN yapılıyor.", level="WARN")
                self._transition(FlightState.TRANSITION)

        elif self._state == FlightState.TRANSITION:
            self.publish_offboard_heartbeat(use_velocity=False, use_position=True)
            self.publish_trajectory_hover(self._home_x, self._home_y, -self._cfg_tkf_alt) 
            
            if self._transition_start_t is None:
                # 3000 = VEHICLE_CMD_DO_VTOL_TRANSITION, Param1 = 4.0 (Fixed-Wing Forward)
                self.send_vehicle_command(3000, 4.0)
                # Hava hizini (Airspeed) ayarla
                self.send_vehicle_command(178, 0.0, self._spd_climb, -1.0)
                self._transition_start_t = time.time()
                self._log("FAZ 2: Geçiş (Transition) komutu gönderildi. Cihaz FW moduna dönüyor...", level="WARN")
                
            if time.time() - self._transition_start_t > 5.0:
                self._transition(FlightState.SLANTED_CLIMB)

        elif self._state == FlightState.SLANTED_CLIMB:
            self.publish_offboard_heartbeat(use_velocity=True, use_position=True)

            if self._px4_nav_state != 14:
                self.set_mode_offboard()

            current_alt = -self._current_z
            gercek_hiz = self._get_real_speed_kmh()
            dist_xy = math.sqrt((self._target_wp.x - self._current_x)**2 + (self._target_wp.y - self._current_y)**2)
            
            self._log(f"FAZ 2: Egimli Tirmanis. Irtifa: {current_alt:.1f}/{self._cfg_crz_alt}m, Hiz: {gercek_hiz:.1f}km/h, Mesafe: {dist_xy:.0f}m", throttle_sec=1.0)
            
            self.publish_trajectory_slanted(self._target_wp.x, self._target_wp.y, -self._cfg_crz_alt, self._spd_climb)

            # Normal gecis: 150m irtifaya ulasti
            if current_alt >= self._cfg_crz_alt - 1.0:
                self._log(f"FAZ 2 TAMAMLANDI! {self._cfg_crz_alt}m irtifaya ulasildi.", level="WARN")
                # CRUISE hızına geç
                self.send_vehicle_command(178, 0.0, self._spd_cruise, -1.0)
                self._transition(FlightState.CRUISE)
            # Erken gecis: Hedef yakinsa 150m'yi bekleme, sarmala girme
            elif dist_xy <= 300.0 and current_alt >= self._cfg_srch_alt:
                self._log(f"FAZ 2 ERKEN GECIS! Hedef {dist_xy:.0f}m mesafede, irtifa {current_alt:.0f}m. APPROACH'a atlaniyor.", level="WARN")
                self._transition(FlightState.APPROACH)
                
        elif self._state == FlightState.CRUISE:
            self.publish_offboard_heartbeat(use_velocity=True, use_position=True)
            self.set_mode_offboard()

            dist_xy = math.sqrt((self._target_wp.x - self._current_x)**2 + (self._target_wp.y - self._current_y)**2)
            gercek_hiz = self._get_real_speed_kmh()
            self._log(f"FAZ 3: Seyir. Mesafe: {dist_xy:.1f}m, Hiz: {gercek_hiz:.1f}km/h", throttle_sec=1.0)
            
            self.publish_trajectory_slanted(self._target_wp.x, self._target_wp.y, -self._cfg_crz_alt, self._spd_cruise)
            
            if dist_xy <= self._cruise_to_approach_dist:
                self._log(
                    f"FAZ 3 TAMAMLANDI: Hedefe {dist_xy:.0f}m kaldi "
                    f"(dinamik eşik: {self._cruise_to_approach_dist:.0f}m). "
                    f"APPROACH fazina geciliyor.",
                    level="WARN"
                )
                # FW hizini dusur: DO_CHANGE_SPEED (178), P1=0 Airspeed, P2=hiz
                self.send_vehicle_command(178, 0.0, self._spd_approach, -1.0)
                self._transition(FlightState.APPROACH)

        elif self._state == FlightState.APPROACH:
            self.publish_offboard_heartbeat(use_velocity=True, use_position=True)
            self.set_mode_offboard()

            dist_xy = math.sqrt((self._target_wp.x - self._current_x)**2 + (self._target_wp.y - self._current_y)**2)
            current_alt = -self._current_z
            gercek_hiz = self._get_real_speed_kmh()
            
            #  KADEMELİ YAVAŞLAMA (DİNAMİK STAGED DECELERATION) 
            # Yavaşlama noktası kinetik enerji farkına göre otomatik hesaplanır.
            if dist_xy <= self._approach_decel_dist and not getattr(self, '_approach_decel_triggered', False):
                self._log(
                    f"KADEMELİ YAVAŞLAMA: Hedefe {dist_xy:.0f}m kaldı "
                    f"(dinamik eşik: {self._approach_decel_dist:.0f}m). "
                    f"Hız sönümleme başlatılıyor ({self._search_speed_kmh:.0f} km/h)...",
                    level="WARN"
                )
                # DO_CHANGE_SPEED ile hava hızını düşür (param2 = _spd_search)
                self.send_vehicle_command(178, 0.0, self._spd_search, 0.0)
                self._approach_decel_triggered = True
            
            # Yörünge hızı: Kademe başladıysa search hızı, başlamadıysa approach hızı.
            current_target_speed = self._spd_search if getattr(self, '_approach_decel_triggered', False) else self._spd_approach

            self._log(
                f"FAZ 4: Yaklasma. Irtifa: {current_alt:.1f}/{self._cfg_srch_alt:.0f}m, "
                f"Hiz: {gercek_hiz:.1f}/{current_target_speed*3.6:.0f}km/h, "
                f"Mesafe: {dist_xy:.1f}m (SEARCH geçiş: {self._approach_to_search_dist:.0f}m)",
                throttle_sec=1.0
            )
            
            # Dinamik hız ile hedefe yaklaş
            self.publish_trajectory_slanted(self._target_wp.x, self._target_wp.y, -self._cfg_srch_alt, current_target_speed)
            
            if dist_xy <= self._approach_to_search_dist:
                self._log(
                    f"FAZ 4 TAMAMLANDI: Hedefe {dist_xy:.0f}m kaldi "
                    f"(dinamik eşik: {self._approach_to_search_dist:.0f}m). "
                    f"Kinetik enerji sönümlendi, SEARCH moduna geciliyor.",
                    level="WARN"
                )
                self._transition(FlightState.SEARCH)

        elif self._state == FlightState.SEARCH:
            #  SEARCH MODU - 70m YARICAPLI DAİRESEL ARAMA
            #  Hedefe kitlenince DÖNÜŞÜ DURDURUP SABİT KALIR, EMİR BEKLERİ
            
            self.publish_offboard_heartbeat(use_velocity=True, use_position=True)
            self.set_mode_offboard()

            #  1. İLK GİRİŞ: Kamera tetikle 
            if not self._search_init_done:
                self._camera_pub.publish(String(data="TRIGGER_ON"))
                self._search_init_done = True
                self._last_speed_cmd_t = 0.0
                self._log(f"SEARCH BASLADI: 70m yarıçaplı dairesel arama başlatıldı. Hız={self._search_speed_kmh:.0f} km/h, İrtifa={self._cfg_srch_alt:.0f}m", level="WARN")

            if time.time() - getattr(self, '_last_speed_cmd_t', 0.0) > 10.0:
                self.send_vehicle_command(178, 0.0, self._spd_search, -1.0)
                self._last_speed_cmd_t = time.time()
                self._log(f"TECS hız yenilendi: {self._search_speed_kmh:.0f} km/h", throttle_sec=5.0)

            #  2. TELEMETRI OKUMA 
            dist_xy = math.sqrt((self._target_wp.x - self._current_x)**2 + (self._target_wp.y - self._current_y)**2)
            current_alt = -self._current_z
            gercek_hiz = self._get_real_speed_kmh()

            #  2.5 TUR SAYACI 
            dx_orbit = self._current_x - self._target_wp.x
            dy_orbit = self._current_y - self._target_wp.y
            current_angle = math.atan2(dy_orbit, dx_orbit)
            if self._search_last_angle is not None:
                delta_angle = current_angle - self._search_last_angle
                # -pi..+pi aralığına normalize et
                if delta_angle > math.pi:
                    delta_angle -= 2.0 * math.pi
                elif delta_angle < -math.pi:
                    delta_angle += 2.0 * math.pi
                self._search_orbit_count += abs(delta_angle) / (2.0 * math.pi)
            self._search_last_angle = current_angle

            #  3. DURUM LOGLAMA 
            lock_str = "KILIT: EVET ██" if self._target_locked else "KILIT: HAYIR"
            auth_str = "ONAY: EVET" if self._engage_authorized else "ONAY: HAYIR"
            mode_str = "SABİT/HOVER" if (self._target_locked and not self._engage_authorized) else "DAİRESEL ARAMA"
            self._log(f"FAZ 5: {mode_str}. İrtifa: {current_alt:.1f}/{self._cfg_srch_alt:.0f}m, Hız: {gercek_hiz:.1f}/{self._search_speed_kmh:.0f}km/h, Mesafe: {dist_xy:.1f}m, Tur: {self._search_orbit_count:.1f} | {lock_str} | {auth_str}", throttle_sec=2.0)


            #  4. DURUM MAKİNESİ: Kilit ve Onay Kontrolü 
            if self._target_locked and not self._engage_authorized:
                #  HEDEFE KİTLENDİ → DÖNÜŞÜ DURDUR, SABİT KAL 
                if not self._search_fw_locked:
                    # İlk kitlenme anı: Mevcut pozisyonu kaydet
                    self._search_hold_x = self._current_x
                    self._search_hold_y = self._current_y
                    self._search_fw_locked = True
                    self._log("████ HEDEF KİTLENDİ! ████ Dönüş durduruldu, pozisyon sabitleniyor. EMİR BEKLENİYOR...", level="ERROR")
                
                self._log("HEDEF KİTLİ - SABİT POZİSYONDA OPERATÖR ENGAGE EMRİ BEKLENİYOR!", throttle_sec=2.0, level="ERROR")
                
                # Sabit pozisyonda tut: Hold pozisyonunu hedefe bakarak bekle
                self.publish_trajectory_hover_track(
                    self._search_hold_x, self._search_hold_y,
                    -self._cfg_srch_alt,
                    self._target_wp.x, self._target_wp.y
                )

            elif self._target_locked and self._engage_authorized:
                self._log("ROE SAGLANDI! Gorsel Kilit + Operator Onay = ENGAGE BASLATILIYOR!", level="ERROR")
                self._transition(FlightState.ENGAGE)
            else:
                if self._search_fw_locked:
                    self._log("KILIT KAYBEDILDI - DAİRESEL ARAMAYA DÖNÜLÜYOR!", level="WARN")
                    self._search_fw_locked = False
                    self._search_hold_x = None
                    self._search_hold_y = None
                
                # 70m yarıçaplı dairesel arama yörüngesi
                self.publish_trajectory_loiter(self._target_wp.x, self._target_wp.y, -self._cfg_srch_alt)
        elif self._state == FlightState.ENGAGE:
            # ENGAGE: Position+Velocity hybrid (deep Z forces pitch down)
            self.publish_offboard_heartbeat(use_velocity=True, use_position=True)
            self.set_mode_offboard()
            
            # Dalista FW maksimum performans emri (DO_CHANGE_SPEED)
            if not self._engage_fw_speed_set:
                self.send_vehicle_command(178, 0.0, self._spd_engage, -1.0)
                self._engage_fw_speed_set = True
                self._log("ENGAGE: Terminal dalis baslatildi! Homing hizi: 150 km/h.", level="ERROR")

            current_alt = -self._current_z
            gercek_hiz = self._get_real_speed_kmh()
            self._log(f"FAZ 6: ENGAGE (DIRECT HOMING)! Irtifa: {current_alt:.1f}m, Hiz: {gercek_hiz:.1f}km/h", throttle_sec=0.5)
            
            # Terminal dalis: Cihaz burnunu ve hizini 3D vektore gore hedefe kilitler.
            self.publish_trajectory_engage(self._target_wp.x, self._target_wp.y, 0.0)

        #  GENEL TELEMETRİ YAYINI (Tüm Uçuş Modlarında)
        #  Vision HUD ve GCS Panel için sürekli veri akışı sağlar.
        
        _telem_alt = max(0.0, -self._current_z)
        _telem_spd = self._get_real_speed_kmh()
        _telem_dist = 0.0
        if self._target_wp is not None:
            _telem_dist = math.hypot(
                self._target_wp.x - self._current_x,
                self._target_wp.y - self._current_y
            )
        if self._state == FlightState.SEARCH:
            if self._target_locked and not self._engage_authorized:
                _telem_mode = "SABİT/HOVER"
            else:
                _telem_mode = "DAİRESEL ARAMA"
        else:
            _telem_mode = self._state.name
        self._publish_search_info(_telem_alt, _telem_spd, _telem_dist, _telem_mode)

def main(args=None):
    rclpy.init(args=args)
    node = VTOLMissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("SIGINT algılandı. Operasyon güvenli şekilde durduruluyor...")
    except Exception as e:
        node.get_logger().error(f"Beklenmeyen Hata: {e}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()