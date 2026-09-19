#!/usr/bin/env python3
"""
Visual Servo — VTOL Kamikaze Dalış Pilotajı (uXRCE-DDS Native)
=============================================
Görev:
  State Machine "ENGAGE" moduna girdiğinde görsel güdüm ile hedefi
  takip eden hız/pozisyon komutlarını Native PX4 (TrajectorySetpoint)
  üzerinden uçağa iletir.
"""

import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition, Airspeed
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class VisualServoNode(Node):

    def __init__(self):
        super().__init__('visual_servo_node')

        self.declare_parameter('kp_yaw',         0.005)
        self.declare_parameter('kp_lateral',     0.02)
        self.declare_parameter('forward_speed',  41.66)
        self.declare_parameter('dive_speed',     50.0)
        self.declare_parameter('servo_hz',       50.0)
        self.declare_parameter('vision_stale_sec', 5.0)
        self.declare_parameter('blind_dive_mode', False)

        # ── DİNAMİK KAZANÇ PLANLAMA (Gain Scheduling) ────────────────
        self.declare_parameter('gain_scheduling_enabled', True)
        self.declare_parameter('gain_ref_speed_mps', 16.67)   # Referans hız (m/s)
        self.declare_parameter('gain_min_ratio', 0.3)         # Minimum kaz. oranı
        self.declare_parameter('gain_max_ratio', 2.0)         # Maksimum kaz. oranı

        # Base kazançlar (referans hızdaki değerler)
        self.kp_yaw_base      = self.get_parameter('kp_yaw').value
        self.kp_lateral_base  = self.get_parameter('kp_lateral').value
        # Aktif kazançlar (her döngüde dinamik güncellenir)
        self.kp_yaw      = self.kp_yaw_base
        self.kp_lateral  = self.kp_lateral_base

        self.forward_spd = self.get_parameter('forward_speed').value
        self.dive_spd    = self.get_parameter('dive_speed').value
        self.servo_hz    = self.get_parameter('servo_hz').value
        self.stale_sec   = self.get_parameter('vision_stale_sec').value
        self._blind_mode = self.get_parameter('blind_dive_mode').value

        self._current_state   = "IDLE"
        self._target_locked   = False
        self._error_x         = 0.0
        self._error_y         = 0.0
        self._last_target_time = 0.0
        
        self._current_x  = 0.0
        self._current_y  = 0.0
        self._current_z  = 0.0 # NED'de Z
        self._current_vx = 0.0
        self._current_vy = 0.0
        self._current_vz = 0.0
        self._target_wp  = None
        self._last_gain_log_time = 0.0  # Gain scheduling log throttle
        # Hava Hızı (Airspeed)
        self._airspeed_mps   = 0.0
        self._airspeed_valid = False
        self._last_airspeed_time = 0.0
        # Heading (pusula — VehicleAttitude quaternion'ünden)
        self._heading_deg   = 0.0
        self._heading_valid = False

        # ── Subscribers ──
        self.create_subscription(String, '/vtol/current_state', self._state_cb, 10)
        self.create_subscription(Point, '/vision/target_error', self._vision_cb, 10)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        # PX4 Native Telemetry
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self._local_pos_cb, sensor_qos)
        self.create_subscription(Airspeed, '/fmu/out/airspeed_v1', self._airspeed_cb, sensor_qos)
        self.create_subscription(Point, '/vtol/local_target', self._target_wp_cb, 10)

        # ── Publishers ──
        # PX4 Native Control Setpoint
        self._setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        
        self._alert_pub = self.create_publisher(String, '/vtol/brain_alerts', 10)
        self._log_pub = self.create_publisher(String, '/vtol/mission_log', 10)
        self._last_gcs_log_time = 0.0

        self.create_timer(1.0 / self.servo_hz, self._control_loop)

        self.get_logger().info("VISUAL SERVO (uXRCE-DDS Native) AKTİF")

    def _state_cb(self, msg: String):
        prev = self._current_state
        self._current_state = msg.data
        if prev == "ENGAGE" and self._current_state != "ENGAGE":
            if hasattr(self, '_locked_dive_x'):
                delattr(self, '_locked_dive_x'); delattr(self, '_locked_dive_y')
                delattr(self, '_locked_dive_z'); delattr(self, '_locked_yaw')
                self.get_logger().warn("ENGAGE kilidi sıfırlandı.")

    def _vision_cb(self, msg: Point):
        if msg.z > 0.5:
            self._target_locked    = True
            self._error_x          = msg.x
            self._error_y          = msg.y
            self._last_target_time = time.time()
        else:
            self._target_locked = False
            self._error_x       = 0.0
            self._error_y       = 0.0

    def _local_pos_cb(self, msg: VehicleLocalPosition):
        self._current_x  = msg.x
        self._current_y  = msg.y
        self._current_z  = msg.z  # NED format (aşağı inildikçe artar)
        self._current_vx = msg.vx
        self._current_vy = msg.vy
        self._current_vz = msg.vz
        
        # ── HEADING (PUSULA) GÜNCELLEMESİ ──
        if not math.isnan(msg.heading):
            hdg_deg = math.degrees(msg.heading)
            if hdg_deg < 0:
                hdg_deg += 360.0
            self._heading_deg = hdg_deg
            self._heading_valid = True
        else:
            self._heading_valid = False
        
        # ── DİNAMİK KAZANÇ GÜNCELLEME (AIRSPEED ÖNCELİKLİ) ──
        if self.get_parameter('gain_scheduling_enabled').value:
            self.kp_yaw, self.kp_lateral = self._compute_scheduled_gains()

    def _airspeed_cb(self, msg: Airspeed):
        """Pitot sensöründen hava hızını okur. Gain scheduling için kullanılır."""
        ias = msg.indicated_airspeed_m_s
        if not math.isnan(ias) and ias > 0.5:
            self._airspeed_mps = ias
            self._airspeed_valid = True
            self._last_airspeed_time = time.time()
        else:
            self._airspeed_valid = False


    def _compute_scheduled_gains(self):
        """
        DİNAMİK KAZANÇ PLANLAMA (Gain Scheduling) — AIRSPEED ÖNCELİKLİ
        ════════════════════════════════════════════════════════════
        Formül:
          Kp = Kp_base × (V_ref / V_current)
        
        Hız kaynağı önceliği:
          1. Hava hızı (Pitot sensör) → aerodinamik doğruluk
          2. Yer hızı (GPS/EKF)      → yedek/fallback
        
        Aerodinamik gerekçe:
          Kontrol yüzeyleri üzerindeki dinamik basınç HAVA HIZINA bağlıdır.
          Rüzgarla karşılaşıldığında yer hızı düşer ama kontrol yüzeyleri
          hala etkilidir (hava hızı aynı kaldığı için). Bu yüzden
          airspeed kullanmak daha doğrudur.
        
        Returns:
            (kp_yaw_scheduled, kp_lateral_scheduled)
        """
        v_ref     = self.get_parameter('gain_ref_speed_mps').value
        ratio_min = self.get_parameter('gain_min_ratio').value
        ratio_max = self.get_parameter('gain_max_ratio').value
        
        # Öncelik: Airspeed (2 saniyeden taze) > Ground Speed
        airspeed_age = time.time() - self._last_airspeed_time
        if self._airspeed_valid and airspeed_age < 2.0:
            v_current = self._airspeed_mps
        else:
            # Fallback: Yer hızı (yatay düzlem)
            v_current = math.sqrt(self._current_vx ** 2 + self._current_vy ** 2)
        
        v_current = max(1.0, v_current)  # Sıfıra bölme koruması
        v_ref     = max(1.0, v_ref)
        
        # Kp = Kp_base × (V_ref / V_current)
        gain_ratio = v_ref / v_current
        
        # Güvenlik kelepçeleme
        gain_ratio = max(ratio_min, min(ratio_max, gain_ratio))
        
        kp_yaw_sched     = self.kp_yaw_base * gain_ratio
        kp_lateral_sched = self.kp_lateral_base * gain_ratio
        
        return kp_yaw_sched, kp_lateral_sched

    def _target_wp_cb(self, msg):
        self._target_wp = msg

    def _control_loop(self):
        if self._current_state != "ENGAGE":
            return

        now = time.time()
        time_since_target = now - self._last_target_time if self._last_target_time > 0 else float('inf')
        
        if not self._blind_mode:
            if not self._target_locked or time_since_target > self.stale_sec:
                return

        if self._target_wp is None:
            return

        # ENGAGE kilidi (dalış endpoint’i sadece ilk seferde hesaplanır)
        if not hasattr(self, '_locked_dive_x'):
            self._locked_yaw = math.atan2(
                self._target_wp.y - self._current_y,
                self._target_wp.x - self._current_x
            )
            
            xy_project_distance = 500.0
            z_dive_depth        = 200.0
            
            self._locked_dive_x = self._current_x + (math.cos(self._locked_yaw) * xy_project_distance)
            self._locked_dive_y = self._current_y + (math.sin(self._locked_yaw) * xy_project_distance)
            self._locked_dive_z = self._current_z + z_dive_depth

        # ══ HDG TABANLI YAW KONTROLÜ (Rüzgar Kompanzasyonu) ══
        # Hedefin GPS azimut açısı (sürekli güncellenir)
        target_bearing = math.atan2(
            self._target_wp.y - self._current_y,
            self._target_wp.x - self._current_x
        )
        
        if self._heading_valid:
            # Heading-based düzeltme: gerçek pusula yönü ile hedef
            # azimutü arasındaki farkı kompanze eder.

            heading_rad = math.radians(self._heading_deg)
            heading_error = target_bearing - heading_rad
            # -pi..+pi normalize
            while heading_error > math.pi:
                heading_error -= 2.0 * math.pi
            while heading_error < -math.pi:
                heading_error += 2.0 * math.pi
            
            # Piksel tabanlı ince düzeltme (İç Döngü)
            # Görsel kilit varsa piksel hatasını da kat
            pixel_correction = 0.0
            if self._target_locked:
                pixel_correction = self.kp_yaw * self._error_x
            
            # Birleşik Yaw: hedef azimutü + piksel ince ayarı
            desired_yaw = target_bearing + pixel_correction
        else:
            # Fallback: Heading verisi yoksa kilitli yaw kullan
            desired_yaw = self._locked_yaw
            heading_error = 0.0

        # TrajectorySetpoint Yayını
        sp = TrajectorySetpoint()
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        sp.position = [float(self._locked_dive_x), float(self._locked_dive_y), float(self._locked_dive_z)]
        sp.yaw = float(desired_yaw)
        
        sp.velocity = [float('nan'), float('nan'), float('nan')]
        sp.acceleration = [float('nan'), float('nan'), float('nan')]
        sp.jerk = [float('nan'), float('nan'), float('nan')]
        sp.yawspeed = float('nan')

        self._setpoint_pub.publish(sp)

        if now - self._last_gcs_log_time >= 1.0:
            v_current = math.hypot(self._current_vx, self._current_vy)
            hdg_src = f"HDG:{self._heading_deg:.0f}" if self._heading_valid else "HDG:N/A"
            bearing_deg = math.degrees(target_bearing) % 360.0
            herr_deg = math.degrees(heading_error) if self._heading_valid else 0.0
            log_text = (
                f"[PILOT-ENGAGE] HDG Tracking | "
                f"{hdg_src} | Bearing: {bearing_deg:.0f} | "
                f"Err: {herr_deg:+.1f} | "
                f"Kp_yaw={self.kp_yaw:.5f} Kp_lat={self.kp_lateral:.4f} "
                f"(V={v_current:.1f}m/s)"
            )
            self.get_logger().info(log_text)
            gcs_msg = String()
            gcs_msg.data = f"[INFO] {log_text}"
            self._log_pub.publish(gcs_msg)
            self._last_gcs_log_time = now

def main(args=None):
    rclpy.init(args=args)
    node = VisualServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()