#!/usr/bin/env python3
"""
GCS Listener — Yer-Uçak Haberleşme Köprüsü (Ground Control Station Bridge)
=============================================================================
Görev:
  1. Vision akışını izler (watchdog beslemesi — kopyalama YAPMAZ)
  2. Operatör komutlarını (ENGAGE/ABORT/RTL) doğrular ve iletir
  3. Haberleşme zaman aşımını (watchdog) izler, kopma halinde RTL basar
  4. Gelen koordinatları mantık kontrolünden (sanity check) geçirir
  5. Uçağın durumunu operatöre geri bildirir (feedback loop)

  Sadece kritik fazlarda (SEARCH, WAIT_FOR_CMD, ENGAGE) aktiftir.
  TAKEOFF/CRUISE gibi modlarda vision akışı olmaması normaldir.
  Bağlantı koptuğunda GCS Listener yönetime el koyar ve RTL emri basar.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Point
from std_msgs.msg import String, Float32



class _Const:
    COMMS_TIMEOUT_SEC       = 2.0      # Bu süre içinde yer verisiz kalırsa alarm
    WATCHDOG_CHECK_HZ       = 2.0      # Saniyede kaç kez kontrol edilecek

    # Koordinat Sınırları (Sanity Check)
    LAT_MIN, LAT_MAX        = -90.0, 90.0
    LON_MIN, LON_MAX        = -180.0, 180.0

    # Geçerli operatör komutları
    VALID_COMMANDS           = {"ENGAGE", "ABORT", "RTL"}

    # Watchdog'un aktif olduğu uçuş fazları
    # Bu fazlar dışında vision akışı olmaması normaldir, alarm verilmez
    WATCHDOG_ACTIVE_STATES   = {"SEARCH", "WAIT_FOR_CMD", "ENGAGE"}

    # Feedback periyodu
    FEEDBACK_HZ             = 1.0      # Saniyede 1 durum mesajı

class GCSListener(Node):

    def __init__(self):
        super().__init__('gcs_listener')

        cb = ReentrantCallbackGroup()

        self._last_vision_time   = None   # Son vision verisi zamanı
        self._last_waypoint_time = None   # Son waypoint verisi zamanı
        self._comms_lost         = False  # Bağlantı kopuk mu?
        self._aircraft_state     = "UNKNOWN"  # Uçağın mevcut durumu

        self.create_subscription(
            Point, '/vision/target_error',
            self._vision_bridge_cb, 10, callback_group=cb
        )

        # Yerden gelen komutları dinle ve Komutan'a (state_machine) ilet
        # Watchdog da bağlantı kopmasında bu kanala RTL emri basar
        self.create_subscription(
            String, '/control/operator_cmd',
            self._operator_cmd_cb, 10, callback_group=cb
        )
        self._operator_pub = self.create_publisher(String, '/control/operator_cmd', 10)


        # 3. HEDEF KOORDİNAT VE HIZ KÖPRÜSÜ             

        self.create_subscription(
            Point, '/control/target_waypoint',
            self._target_callback, 10, callback_group=cb
        )
        self.create_subscription(
            Float32, '/control/cruise_speed',
            self._speed_callback, 10, callback_group=cb
        )
        self._target_pub = self.create_publisher(Point, '/vtol/mission_target', 10)
        self._speed_pub  = self.create_publisher(Float32, '/vtol/target_speed_mps', 10)

        # 4. ALARM VE GERİ BESLEMECİLER                 
        self._alert_pub    = self.create_publisher(String, '/vtol/brain_alerts', 10)
        self._feedback_pub = self.create_publisher(String, '/gcs/status_feedback', 10)

        # 5. UÇAK DURUMU DİNLEYİCİSİ                    
        self.create_subscription(
            String, '/vtol/current_state',
            self._aircraft_state_cb, 10, callback_group=cb
        )

        # Zamanlayıcılar 
        # Watchdog: Haberleşme zaman aşımı kontrolü
        self.create_timer(
            1.0 / _Const.WATCHDOG_CHECK_HZ,
            self._watchdog_tick, callback_group=cb
        )
        # Feedback: Operatöre periyodik durum raporu
        self.create_timer(
            1.0 / _Const.FEEDBACK_HZ,
            self._feedback_tick, callback_group=cb
        )

        self.get_logger().info(
            "═══════════════════════════════════════════════════\n"
            "  GCS LISTENER (Yer-Uçak Haberleşme Köprüsü) AKTİF\n"
            "  Watchdog Süresi  : %.1f saniye\n"
            "  Geçerli Komutlar : %s\n"
            "═══════════════════════════════════════════════════"
            % (_Const.COMMS_TIMEOUT_SEC, ", ".join(_Const.VALID_COMMANDS))
        )

    # 1. VİZYON İZLEYİCİSİ (Watchdog Beslemesi)
    def _vision_bridge_cb(self, msg: Point):
        """
        Vision akışını izler ve watchdog timer'ı sıfırlar.
        Veri kopyalama YAPMAZ — state_machine ve visual_servo zaten
        /vision/target_error'u doğrudan dinler.
        """
        # Watchdog'u besle (bağlantı zamanını güncelle)
        self._last_vision_time = time.time()

        # Hedef bulunduysa logla (debug amaçlı)
        if msg.z == 1.0:
            self.get_logger().info(
                f"[VİZYON] Hedef aktif → ErrX: {msg.x:.1f} px, ErrY: {msg.y:.1f} px",
                throttle_duration_sec=2.0
            )

    # 2. OPERATÖR KOMUT HATTI
    def _operator_cmd_cb(self, msg: String):
        """
        Yerden gelen operatör komutlarını doğrular ve uçağa iletir.
        Tanınmayan komutlar güvenlik gereği iletilmez.
        """
        cmd = msg.data.strip().upper()

        if cmd not in _Const.VALID_COMMANDS:
            self.get_logger().warn(
                f"[KOMUT] Tanınmayan komut reddedildi: '{msg.data}' "
                f"(Geçerli: {_Const.VALID_COMMANDS})"
            )
            self._send_feedback(f"HATA: Tanınmayan komut '{msg.data}'. "
                                f"Geçerli: {', '.join(_Const.VALID_COMMANDS)}")
            return

        # Komutu ilet
        relay_msg = String()
        relay_msg.data = cmd
        self._operator_pub.publish(relay_msg)

        self.get_logger().warn(f"[KOMUT] Operatör komutu iletildi → {cmd}")
        self._send_feedback(f"KOMUT ALINDI ve İLETİLDİ: {cmd}")

    # 3. HEDEF KOORDİNAT KÖPRÜSÜ + SANİTY CHECK
    def _target_callback(self, msg: Point):
        """
        GCS/Terminalden gelen hedef koordinatını doğrular ve uçağa iletir.
        Geçersiz koordinatlar (0,0 veya aralık dışı) reddedilir.
        """
        lat, lon, alt = msg.x, msg.y, msg.z

        # ── Mantık Kontrolü (Sanity Check) ────────────────
        if not (_Const.LAT_MIN <= lat <= _Const.LAT_MAX):
            self.get_logger().error(
                f"[SANİTY] Geçersiz enlem reddedildi: {lat:.6f} "
                f"(Aralık: {_Const.LAT_MIN} ~ {_Const.LAT_MAX})"
            )
            self._send_feedback(f"HATA: Geçersiz enlem değeri: {lat}")
            return

        if not (_Const.LON_MIN <= lon <= _Const.LON_MAX):
            self.get_logger().error(
                f"[SANİTY] Geçersiz boylam reddedildi: {lon:.6f} "
                f"(Aralık: {_Const.LON_MIN} ~ {_Const.LON_MAX})"
            )
            self._send_feedback(f"HATA: Geçersiz boylam değeri: {lon}")
            return

        # Sıfır koordinat kontrolü (GPS hatası veya boş veri)
        if lat == 0.0 and lon == 0.0:
            self.get_logger().error(
                "[SANİTY] Sıfır koordinat (0.0, 0.0) reddedildi — "
                "muhtemel GPS hatası veya boş veri!"
            )
            self._send_feedback("HATA: Koordinat (0,0) — GPS hatası olabilir!")
            return

        # ── Doğrulama Geçti — İlet ───────────────────────
        self._last_waypoint_time = time.time()
        self._target_pub.publish(msg)

        self.get_logger().info(
            f"[HEDEF] Koordinat doğrulandı ve iletildi → "
            f"Lat: {lat:.6f}, Lon: {lon:.6f}, Alt: {alt:.1f}m"
        )
        self._send_feedback(
            f"Hedef koordinat alındı: Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt:.1f}m"
        )

    def _speed_callback(self, msg: Float32):
        """Hız komutunu km/h → m/s çevirip iletir."""
        kmh = msg.data
        if kmh <= 0.0:
            self.get_logger().warn(f"[SANİTY] Geçersiz hız reddedildi: {kmh} km/h")
            self._send_feedback(f"HATA: Geçersiz hız değeri: {kmh} km/h")
            return

        mps = kmh / 3.6
        mps_msg = Float32()
        mps_msg.data = mps
        self._speed_pub.publish(mps_msg)

        self.get_logger().info(f"[HIZ] Hız emri iletildi: {kmh:.0f} km/h → {mps:.2f} m/s")
        self._send_feedback(f"Hız komutu alındı: {kmh:.0f} km/h → {mps:.2f} m/s")

    # 4. WATCHDOG (Haberleşme Zaman Aşımı Kontrolü)
    def _watchdog_tick(self):
        """
        Periyodik olarak son veri zamanını kontrol eder.
        SADECE kritik uçuş fazlarında (SEARCH, WAIT_FOR_CMD, ENGAGE) aktiftir.
        Bağlantı koptuğunda GCS Listener yönetime el koyar ve RTL emri basar.
        """
        # DURUM FİLTRESİ: Yalancı alarm önleme 
        # Watchdog'u sadece hedef arama ve dalış fazlarında aktif et
        if self._aircraft_state not in _Const.WATCHDOG_ACTIVE_STATES:
            # Yalancı alarmı önlemek için zamanı taze tut
            self._last_vision_time = time.time()
            return

        now = time.time()

        if self._last_vision_time is None and self._last_waypoint_time is None:
            return

        # En son veri ne zaman geldi?
        last_data_time = 0.0
        if self._last_vision_time is not None:
            last_data_time = max(last_data_time, self._last_vision_time)
        if self._last_waypoint_time is not None:
            last_data_time = max(last_data_time, self._last_waypoint_time)

        elapsed = now - last_data_time

        if elapsed > _Const.COMMS_TIMEOUT_SEC:
            if not self._comms_lost:
                # ── Bağlantı yeni koptu — YÖNETİME EL KOY ──
                self._comms_lost = True
                self.get_logger().error(
                    f"[WATCHDOG] ⚠ BAĞLANTI KOPTU! "
                    f"Son veri {elapsed:.1f}s önce geldi. "
                    f"GCS Listener yönetime el koyuyor → RTL!"
                )
                self._send_alert("COMMS_LOST")
                self._send_feedback(
                    "UYARI: Yer istasyonu bağlantısı koptu! "
                    "GCS Listener RTL emri verdi."
                )

                # Operatör bağlantısı koptuğu için GCS Listener
                # yönetime el koyar ve Komutan'a eve dön emri basar
                rtl_msg = String()
                rtl_msg.data = "RTL"
                self._operator_pub.publish(rtl_msg)
                self.get_logger().warn(
                    "[WATCHDOG] RTL EMRİ GÖNDERİLDİ → Komutan'a iletildi."
                )
        else:
            if self._comms_lost:
                # ── Bağlantı geri geldi ──
                self._comms_lost = False
                self.get_logger().info(
                    "[WATCHDOG] ✓ Bağlantı yeniden kuruldu!"
                )
                self._send_alert("COMMS_RESTORED")
                self._send_feedback("Bağlantı yeniden kuruldu.")

    
    #  5. DURUM GERİ BİLDİRİMİ (Feedback Loop)
    def _aircraft_state_cb(self, msg: String):
        """State machine'den uçağın mevcut görev durumunu alır."""
        self._aircraft_state = msg.data

    def _feedback_tick(self):
        """
        Operatöre periyodik durum raporu gönderir.
        Bağlantı durumu + uçağın mevcut görev fazı.
        """
        link = "AKTIF" if not self._comms_lost else "KOPUK"
        self._send_feedback(
            f"[HEARTBEAT] Bağlantı: {link} | "
            f"Uçak Durumu: {self._aircraft_state}"
        )

    #  YARDIMCI METOTLAR
    def _send_alert(self, alert_code: str):
        """Brain alerts kanalına sinyal gönderir."""
        msg = String()
        msg.data = alert_code
        self._alert_pub.publish(msg)

    def _send_feedback(self, text: str):
        """Operatöre okunabilir geri bildirim mesajı gönderir."""
        msg = String()
        msg.data = text
        self._feedback_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GCSListener()

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("GCS Listener kapatılıyor...")
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