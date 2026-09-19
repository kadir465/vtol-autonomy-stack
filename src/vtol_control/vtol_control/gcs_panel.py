#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════════════╗
║        VTOL GROUND CONTROL STATION — TACTICAL TERMINAL  v2.0             ║
║                 Askeri Yer Kontrol İstasyonu Paneli                      ║
╠══════════════════════════════════════════════════════════════════════════╣
║  İnsan Operatör (Human-in-the-Loop) ile Otonom Görev Yöneticisi          ║
║  (state_machine.py) arasında çift yönlü iletişim sağlar.                 ║
║                                                                          ║
║  v2.0 Mimari İyileştirmeler:                                             ║
║    • Non-blocking input (getch + input buffer)                           ║
║    • KEY_RESIZE ile dinamik yeniden çizim                                ║
║    • Curses subwindow tabanlı bağımsız panel yönetimi                    ║
║                                                                          ║
║  Telemetri (Dinleme):                                                    ║
║    /vtol/current_state  → Uçuş modu (String)                             ║
║    /vtol/mission_log    → Görev logları (String)                         ║
║                                                                          ║
║  Komuta (Konuşma):                                                       ║
║    /vtol/mission_target → GPS hedef koordinatı (Point)                   ║
║    /control/operator_cmd → ENGAGE / RTL / ABORT (String)                 ║
╚══════════════════════════════════════════════════════════════════════════╝
"""

import curses
import threading
import time
import sys
from datetime import datetime
from enum import Enum, auto
from collections import deque
import json

import rclpy
import math
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition

VERSION = "2.0.0"
MAX_LOG_LINES = 200         # Hafızada tutulan maksimum log
UI_REFRESH_HZ = 4           # Ekran yenileme hızı
MIN_TERM_W = 72             # Minimum terminal genişliği
MIN_TERM_H = 24             # Minimum terminal yüksekliği

LAT_MIN, LAT_MAX = -90.0, 90.0
LON_MIN, LON_MAX = -180.0, 180.0

STATE_THEME = {
    #  state          color_key    türkçe açıklama
    "IDLE":         ("DIM",    "Sistem Boşta"),
    "PREFLIGHT":    ("CYAN",   "Ön Kontroller Yapılıyor"),
    "TAKEOFF":      ("GREEN",  "Dikey Kalkış Devam Ediyor"),
    "TRANSITION":   ("GREEN",  "Sabit Kanat Geçişi"),
    "CRUISE":       ("GREEN",  "Hedefe Seyir Halinde"),
    "SEARCH":       ("YELLOW", "Dairesel Arama Akti̇f (70m Yarıçap)"),
    "WAIT_FOR_CMD": ("YELLOW", "⚡ OPERATÖR ONAYI BEKLENİYOR ⚡"),
    "ENGAGE":       ("RED",    "!!! TAARRUZ AKTİF !!!"),
    "RTL":          ("CYAN",   "Eve Dönüş (RTL)"),
    "LAND":         ("CYAN",   "İniş Manevrası"),
    "FAULT":        ("RED",    "HATA — Manuel Müdahale Gerekli"),
    "UNKNOWN":      ("DIM",    "Bağlantı Bekleniyor..."),
}

# Sabit banner satırları
BANNER = [
    "╔═══════════════════════════════════════════════════════════════════════╗",
    "║       ██╗   ██╗████████╗ ██████╗ ██╗          ██████╗  ██████╗███████ ║",
    "║       ██║   ██║╚══██╔══╝██╔═══██╗██║         ██╔════╝ ██╔════╝██╔════ ║",
    "║       ██║   ██║   ██║   ██║   ██║██║         ██║  ███╗██║     ███████ ║",
    "║       ╚██╗ ██╔╝   ██║   ██║   ██║██║         ██║   ██║██║     ╚════██ ║",
    "║        ╚████╔╝    ██║   ╚██████╔╝███████╗    ╚██████╔╝╚██████╗███████ ║",
    "║         ╚═══╝     ╚═╝    ╚═════╝ ╚══════╝     ╚═════╝  ╚═════╝╚══════ ║",
    "║              GROUND  CONTROL  STATION  —  TACTICAL  TERMINAL          ║",
    "╚═══════════════════════════════════════════════════════════════════════╝",
]

class InputMode(Enum):
    """Operatör arayüz modları — ana döngü her modda
    farklı tuş eşlemesi kullanır."""
    NORMAL        = auto()   # Ana menü — G/E/R/A/Q
    GPS_LAT       = auto()   # Enlem girişi (karakter toplama)
    GPS_LON       = auto()   # Boylam girişi
    GPS_CONFIRM   = auto()   # GPS onay — E/H
    HOME_LAT      = auto()   # Manuel Home Enlem (Düzeltme için)
    HOME_LON      = auto()   # Manuel Home Boylam (Düzeltme için)
    HOME_CONFIRM  = auto()   # Manuel Home Onay
    
    # ── BAŞLATMA SİHİRBAZI MODLARI ────────
    INIT_HOME_LAT   = auto()
    INIT_HOME_LON   = auto()
    INIT_TARGET_LAT = auto()
    INIT_TARGET_LON = auto()
    
    ENGAGE_CONFIRM = auto() 
    ABORT_CONFIRM  = auto()  

#  ROS 2 NODE (Arka Plan Thread)
class GCSPanelNode(Node):
    """Telemetri verilerini alır, operatör komutlarını gönderir."""

    def __init__(self):
        super().__init__('gcs_panel')
        cb = ReentrantCallbackGroup()

        # Thread-safe paylaşım
        self._lock = threading.Lock()
        self._flight_state = "UNKNOWN"
        self._log_buffer: deque[str] = deque(maxlen=MAX_LOG_LINES)
        self._start_time = time.time()
        self._last_state_time = time.time()
        self._state_rx = 0
        self._log_rx = 0
        self._current_alt = 0.0
        self._current_speed = 0.0
        self._search_info = None  # JSON search status data
        self._last_search_info_time = 0.0  # Watchdog: telemetri veri zamanı

        # Subscriber
        self.create_subscription(
            String, '/vtol/current_state',
            self._state_cb, 10, callback_group=cb)
        self.create_subscription(
            String, '/vtol/mission_log',
            self._log_cb, 10, callback_group=cb)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._local_pos_cb, qos_profile_sensor_data, callback_group=cb)
        self.create_subscription(
            String, '/vtol/search_info',
            self._search_info_cb, 10, callback_group=cb)

        # Publisher
        self._target_pub = self.create_publisher(Point, '/vtol/mission_target', 10)
        self._home_pub = self.create_publisher(Point, '/vtol/home_set', 10)
        self._cmd_pub = self.create_publisher(String, '/control/operator_cmd', 10)

        self.get_logger().info("GCS Panel Node v2.0 başlatıldı.")

    #  Callback
    def _state_cb(self, msg: String):
        with self._lock:
            self._flight_state = msg.data.strip()
            self._last_state_time = time.time()
            self._state_rx += 1

    def _log_cb(self, msg: String):
        ts = datetime.now().strftime("%H:%M:%S")
        entry = f"[{ts}] {msg.data}"
        with self._lock:
            self._log_buffer.append(entry)
            self._log_rx += 1

    def _local_pos_cb(self, msg: VehicleLocalPosition):
        with self._lock:
            # NED coord system: Z goes down. We want positive altitude.
            self._current_alt = -msg.z if msg.z < 0 else msg.z
            # V = sqrt(vx^2 + vy^2)
            self._current_speed = math.hypot(msg.vx, msg.vy)

    def _search_info_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self._lock:
                self._search_info = data
                self._last_search_info_time = time.time()
        except json.JSONDecodeError:
            pass

    #  Getter (Thread-Safe)
    def get_state(self) -> str:
        with self._lock:
            return self._flight_state

    def get_logs(self, n: int) -> list[str]:
        with self._lock:
            return list(self._log_buffer)[-n:]

    def get_uptime(self) -> float:
        return time.time() - self._start_time

    def get_link_status(self) -> tuple[bool, float]:
        with self._lock:
            state_elapsed = time.time() - self._last_state_time
            # Telemetri verisi (batarya/airspeed) tazelği de kontrol edilir
            data_elapsed = time.time() - self._last_search_info_time if self._last_search_info_time > 0 else state_elapsed
            worst_elapsed = max(state_elapsed, data_elapsed)
            return (worst_elapsed < 3.0), worst_elapsed

    def get_stats(self) -> tuple[int, int]:
        with self._lock:
            return self._state_rx, self._log_rx

    def get_altitude(self) -> float:
        with self._lock:
            return self._current_alt

    def get_speed(self) -> float:
        with self._lock:
            return self._current_speed

    def get_search_info(self) -> dict:
        with self._lock:
            return self._search_info

    #  Komut Gönderimi
    def send_waypoint(self, lat: float, lon: float, alt: float = 0.0):
        msg = Point()
        msg.x = lat
        msg.y = lon
        msg.z = alt
        self._target_pub.publish(msg)
        self._push_local_log(
            f"[CMD] GPS HEDEFİ GÖNDERİLDİ → Lat: {lat:.6f}, Lon: {lon:.6f}")

    def send_home_pos(self, lat: float, lon: float):
        msg = Point()
        msg.x = lat
        msg.y = lon
        msg.z = 0.0
        self._home_pub.publish(msg)
        self._push_local_log(
            f"[CMD] MANUEL HOME SET EDİLDİ → Lat: {lat:.6f}, Lon: {lon:.6f}")

    def send_command(self, cmd: str):
        msg = String()
        msg.data = cmd
        self._cmd_pub.publish(msg)
        self._push_local_log(f"[CMD] OPERATÖR KOMUTU → {cmd}")

    def _push_local_log(self, text: str):
        ts = datetime.now().strftime("%H:%M:%S")
        with self._lock:
            self._log_buffer.append(f"[{ts}] {text}")


#  TAKTİK ARAYÜZ (Non-Blocking, Subwindow Tabanlı)
class TacticalUI:
    """
    Production-ready curses arayüzü.
      • Tüm girdi non-blocking (getch + input buffer)
      • KEY_RESIZE ile temiz yeniden çizim
      • Curses subwindow ile bağımsız panel bölgeleri
    """

    #  Layout sabitleri (satır sayıları)
    BANNER_H = len(BANNER) + 1          # banner + ayırıcı
    STATUS_H = 12                       # durum panosu (telemetri + search info için genişletildi)
    CMD_H    = 3                        # komut çubuğu (status + sep + prompt)

    def __init__(self, node: GCSPanelNode):
        self._node = node
        self._running = True

        # Input state machine - BAŞLATMA SİHİRBAZI (Step 1/4)
        self._mode = InputMode.INIT_HOME_LAT
        self._input_buf = ""            # Karakter tamponu
        self._pending_lat = 0.0         # GPS onay için geçici
        self._pending_lon = 0.0

        # UI bilgilendirme
        self._status_msg = "GÖREV BAŞLATILDI: 1/4 - LÜTFEN HOME (ÜS) ENLEMİNİ GİRİN."
        self._status_expire = time.time() + 15.0

        self._colors: dict[str, int] = {}

        self._win_banner = None
        self._win_status = None
        self._win_log = None
        self._win_cmd = None

    #  ANA GİRİŞ (curses.wrapper çağırır)
    def run(self, stdscr):
        self._stdscr = stdscr
        self._setup_colors()
        curses.curs_set(0)
        self._stdscr.nodelay(True)
        self._stdscr.timeout(int(1000 / UI_REFRESH_HZ))

        self._rebuild_layout()

        while self._running:
            key = self._stdscr.getch()

            #  Resize sinyali
            if key == curses.KEY_RESIZE:
                self._stdscr.clear()
                self._stdscr.refresh()
                self._rebuild_layout()
                continue

            #  Tuş işleme (non-blocking)
            if key != -1:
                self._handle_key(key)

            #  Ekranı güncelle
            self._refresh_all()

    #  RENK KURULUMU
    def _setup_colors(self):
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_GREEN,  -1)
        curses.init_pair(2, curses.COLOR_YELLOW, -1)
        curses.init_pair(3, curses.COLOR_RED,    -1)
        curses.init_pair(4, curses.COLOR_CYAN,   -1)
        curses.init_pair(5, curses.COLOR_WHITE,  -1)

        self._colors = {
            "GREEN":  curses.color_pair(1) | curses.A_BOLD,
            "YELLOW": curses.color_pair(2) | curses.A_BOLD,
            "RED":    curses.color_pair(3) | curses.A_BOLD,
            "CYAN":   curses.color_pair(4) | curses.A_BOLD,
            "WHITE":  curses.color_pair(5) | curses.A_BOLD,
            "DIM":    curses.color_pair(5),
            "MATRIX": curses.color_pair(1),
        }

    #  DİNAMİK LAYOUT OLUŞTURUCU
    def _rebuild_layout(self):
        h, w = self._stdscr.getmaxyx()

        # Minimum boyut kontrolü
        if h < MIN_TERM_H or w < MIN_TERM_W:
            self._win_banner = None
            self._win_status = None
            self._win_log = None
            self._win_cmd = None
            try:
                self._stdscr.addstr(
                    0, 0,
                    f"Terminal çok küçük! Minimum: {MIN_TERM_W}x{MIN_TERM_H} "
                    f"(Şimdi: {w}x{h})",
                    self._colors["RED"])
                self._stdscr.refresh()
            except curses.error:
                pass
            return

        # Bölge yükseklikleri hesapla
        banner_h = min(self.BANNER_H, h // 4)
        status_h = self.STATUS_H
        cmd_h = self.CMD_H
        log_h = h - banner_h - status_h - cmd_h
        if log_h < 3:
            log_h = 3
            status_h = max(3, h - banner_h - log_h - cmd_h)

        row = 0

        # Eski subwindow'ları temizle
        for win in (self._win_banner, self._win_status, self._win_log, self._win_cmd):
            if win is not None:
                try:
                    win.erase()
                except curses.error:
                    pass

        # Yeni subwindow'lar oluştur
        try:
            self._win_banner = self._stdscr.subwin(banner_h, w, row, 0)
            row += banner_h

            self._win_status = self._stdscr.subwin(status_h, w, row, 0)
            row += status_h

            self._win_log = self._stdscr.subwin(log_h, w, row, 0)
            row += log_h

            self._win_cmd = self._stdscr.subwin(cmd_h, w, row, 0)
        except curses.error:
            pass

    #  TÜM PANELLERİ YENİLE
    def _refresh_all(self):
        h, w = self._stdscr.getmaxyx()
        if h < MIN_TERM_H or w < MIN_TERM_W:
            return  # Layout oluşturulamadı

        try:
            self._draw_banner()
            self._draw_status()
            self._draw_log()
            self._draw_cmd()
        except curses.error:
            pass

    #  PANEL 1: BANNER (Üst Başlık)
    def _draw_banner(self):
        win = self._win_banner
        if win is None:
            return
        win.erase()
        wh, ww = win.getmaxyx()
        attr = self._colors["GREEN"]
        for i, line in enumerate(BANNER):
            if i >= wh - 1:
                break
            self._safe_addstr(win, i, 0, line[:ww - 1], attr)
        # Ayırıcı
        if len(BANNER) < wh:
            self._safe_addstr(win, min(len(BANNER), wh - 1), 0,
                              "═" * (ww - 1), attr)
        win.noutrefresh()

    #  PANEL 2: DURUM PANOSU
    def _draw_status(self):
        win = self._win_status
        if win is None:
            return
        win.erase()
        wh, ww = win.getmaxyx()

        state = self._node.get_state()
        link_ok, link_age = self._node.get_link_status()
        uptime = self._node.get_uptime()
        s_rx, l_rx = self._node.get_stats()
        now_s = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        theme_color, state_desc = STATE_THEME.get(state, ("DIM", state))
        state_attr = self._colors.get(theme_color, self._colors["WHITE"])

        row = 0

        # Saat / versiyon / İrtifa / Hız
        alt = self._node.get_altitude()
        spd = self._node.get_speed()
        self._safe_addstr(win, row, 0,
            f"  ◆ VTOL GCS v{VERSION}  │  {now_s}  │  "
            f"Uptime: {self._fmt_time(uptime)}  │  İRTİFA: {alt:.1f}m | HIZ: {spd:.1f} m/s"[:ww - 1],
            self._colors["CYAN"])
        row += 1

        # Data link
        if link_ok:
            lnk = (f"  ◆ DATA LINK: ████ ONLINE ████  │  "
                    f"Son Veri: {link_age:.1f}s  │  RX: {s_rx} durum, {l_rx} log")
            lnk_attr = self._colors["GREEN"]
        else:
            lnk = (f"  ◆ DATA LINK: ░░░░ OFFLINE ░░░░  │  "
                    f"Son Veri: {link_age:.1f}s  │  BAĞLANTI KOPUK!")
            lnk_attr = self._colors["RED"]
        self._safe_addstr(win, row, 0, lnk[:ww - 1], lnk_attr)
        row += 1

        # ── TELEMETRİ ŞERİDİ (Batarya / Pusula / Hava Hızı / Crab) ──
        search_info = self._node.get_search_info()
        if search_info and row < wh - 1:
            bat_pct  = search_info.get("battery_percent", 0)
            bat_v    = search_info.get("battery_volt", 0)
            bat_ok   = search_info.get("battery_valid", False)
            hdg      = search_info.get("heading_deg", 0)
            hdg_ok   = search_info.get("heading_valid", False)
            airspd   = search_info.get("airspeed_mps", 0)
            air_ok   = search_info.get("airspeed_valid", False)
            air_src  = search_info.get("airspeed_source", "---")
            crab     = search_info.get("crab_angle_deg", 0)
            bingo    = search_info.get("bingo_battery_pct", 20)

            # Batarya renk seçimi
            if not bat_ok:
                bat_str = "BAT: ---"
                bat_attr = self._colors["DIM"]
            elif bat_pct <= bingo:
                bat_str = f"BAT: %{bat_pct:.0f} ({bat_v:.1f}V) ⚠BINGO"
                bat_attr = self._colors["RED"] | curses.A_BLINK
            elif bat_pct <= 30:
                bat_str = f"BAT: %{bat_pct:.0f} ({bat_v:.1f}V)"
                bat_attr = self._colors["RED"]
            elif bat_pct <= 50:
                bat_str = f"BAT: %{bat_pct:.0f} ({bat_v:.1f}V)"
                bat_attr = self._colors["YELLOW"]
            else:
                bat_str = f"BAT: %{bat_pct:.0f} ({bat_v:.1f}V)"
                bat_attr = self._colors["GREEN"]

            # Heading
            hdg_str = f"HDG: {hdg:.0f}°" if hdg_ok else "HDG: ---"

            # Airspeed
            if air_ok:
                air_str = f"AIR: {airspd:.1f} m/s"
            else:
                air_str = f"GND: {airspd:.1f} m/s"

            # Crab Angle renk seçimi
            if abs(crab) > 15:
                crab_str = f"CRAB: {crab:+.0f}° ⚠"
                crab_attr = self._colors["RED"]
            elif abs(crab) > 8:
                crab_str = f"CRAB: {crab:+.0f}°"
                crab_attr = self._colors["YELLOW"]
            else:
                crab_str = f"CRAB: {crab:+.0f}°"
                crab_attr = self._colors["GREEN"]

            telem_line = f"  ◆ {bat_str}  │  {hdg_str}  │  {air_str} [{air_src}]  │  {crab_str}"
            # Tüm şeridi batarya renginde göster (en kritik veri)
            self._safe_addstr(win, row, 0, telem_line[:ww - 1], bat_attr)
            row += 1

            # ── CRAB ANGLE YÜKSEK UYARISI (Hassas Dalış Riski) ──
            if abs(crab) > 15 and row < wh - 1:
                warn_line = "  ⚠ YÜKSEK YAN RÜZGAR — HASSAS DALIŞ RİSKİ! RÜZGAR KOMPANZASYONU GEREKLİ ⚠"
                self._safe_addstr(win, row, 0, warn_line[:ww - 1],
                                  self._colors["RED"] | curses.A_BLINK)
                row += 1

        # Boşluk
        row += 1

        # Uçuş durumu kutusu
        box_w = min(66, ww - 4)
        border = "═" * (box_w - 2)
        self._safe_addstr(win, row, 2, f"╔{border}╗"[:ww - 3], state_attr)
        row += 1
        inner = f"  {state:^16s}  —  {state_desc}"
        pad = max(0, box_w - 4 - len(inner))
        self._safe_addstr(win, row, 2,
                          f"║ {inner}{' ' * pad} ║"[:ww - 3], state_attr)
        row += 1
        self._safe_addstr(win, row, 2, f"╚{border}╝"[:ww - 3], state_attr)
        row += 1

        # ── SEARCH BİLGİ KUTUSU (Sadece SEARCH modunda) ──
        if state == "SEARCH" and search_info and row < wh - 1:
            locked = search_info.get("target_locked", False)
            mode_s = search_info.get("mode", "---")
            orbit_r = search_info.get("orbit_radius_m", 0)
            orbit_c = search_info.get("orbit_count", 0)
            dist_m  = search_info.get("distance_m", 0)
            engage_auth = search_info.get("engage_authorized", False)

            if locked:
                lock_icon = "████ HEDEF KİTLİ ████"
                lock_attr = self._colors["RED"] | curses.A_BLINK if hasattr(curses, 'A_BLINK') else self._colors["RED"]
                if engage_auth:
                    lock_line = f"  ⚡ {lock_icon}  |  MOD: ENGAGE HAZIR  |  Mesafe: {dist_m:.0f}m"
                else:
                    lock_line = f"  ⚡ {lock_icon}  |  MOD: {mode_s}  |  ENGAGE EMRİ BEKLENİYOR  |  Mesafe: {dist_m:.0f}m"
            else:
                lock_icon = "░░░░ HEDEF ARANYOR ░░░░"
                lock_attr = self._colors["YELLOW"]
                lock_line = f"  ◎ {lock_icon}  |  MOD: {mode_s}  |  Yarıçap: {orbit_r:.0f}m  |  Tur: {orbit_c:.1f}  |  Mesafe: {dist_m:.0f}m"
            
            self._safe_addstr(win, row, 0, lock_line[:ww - 1], lock_attr)

        win.noutrefresh()

    #  PANEL 3: MİSYON LOG
    def _draw_log(self):
        win = self._win_log
        if win is None:
            return
        win.erase()
        wh, ww = win.getmaxyx()

        # Başlık
        hdr = "  ┌── MİSYON LOG " + "─" * max(0, ww - 22) + "┐"
        self._safe_addstr(win, 0, 0, hdr[:ww - 1], self._colors["CYAN"])

        # Log alanı: 1 .. wh-2 (başlık ve alt çizgi hariç)
        log_area = wh - 2
        if log_area < 1:
            win.noutrefresh()
            return

        logs = self._node.get_logs(log_area)

        if not logs:
            self._safe_addstr(win, 1, 0,
                "  │  Henüz log verisi yok — State Machine bağlantısı bekleniyor..."
                [:ww - 1], self._colors["DIM"])
        else:
            for i, line in enumerate(logs):
                if i >= log_area:
                    break
                # Renk kodlama
                if any(k in line for k in ("[ERROR]", "HATA", "FAULT")):
                    attr = self._colors["RED"]
                elif any(k in line for k in ("[WARN]", "UYARI", "ABORT", "RTL")):
                    attr = self._colors["YELLOW"]
                elif any(k in line for k in ("[CMD]", "ENGAGE")):
                    attr = self._colors["CYAN"]
                elif any(k in line for k in ("HEDEF", "TESPİT", "ONAY")):
                    attr = self._colors["GREEN"]
                else:
                    attr = self._colors["MATRIX"]
                self._safe_addstr(win, 1 + i, 0,
                                  f"  │ {line}"[:ww - 1], attr)

        # Alt çizgi
        ftr = "  └" + "─" * max(0, ww - 5) + "┘"
        self._safe_addstr(win, wh - 1, 0, ftr[:ww - 1], self._colors["CYAN"])
        win.noutrefresh()

    #  PANEL 4: KOMUT ÇUBUĞU + GİRDİ TAMPONU
    def _draw_cmd(self):
        win = self._win_cmd
        if win is None:
            return
        win.erase()
        wh, ww = win.getmaxyx()

        # Satır 0: Geçici durum mesajı
        if self._status_msg and time.time() < self._status_expire:
            if any(k in self._status_msg for k in ("HATA", "GEÇERSİZ", "reddedildi")):
                attr = self._colors["RED"]
            elif any(k in self._status_msg for k in ("GÖNDERİLDİ", "TAMAM")):
                attr = self._colors["GREEN"]
            else:
                attr = self._colors["YELLOW"]
            self._safe_addstr(win, 0, 0,
                              f"  » {self._status_msg}"[:ww - 1], attr)

        # Satır 1: Ayırıcı
        self._safe_addstr(win, 1, 0, "═" * (ww - 1), self._colors["GREEN"])

        # Satır 2: Mod'a göre prompt
        prompt = self._build_prompt()
        self._safe_addstr(win, 2, 0, prompt[:ww - 1], self._colors["WHITE"])

        win.noutrefresh()
        curses.doupdate()

    #  PROMPT OLUŞTURucu (Mod'a göre)
    def _build_prompt(self) -> str:
        if self._mode == InputMode.NORMAL:
            state = self._node.get_state()
            if state in ("SEARCH", "WAIT_FOR_CMD"):
                return "  KOMUT » [T] Kalkış  [G] Hedef Gir  [H] Home Set  [E] ⚡ENGAGE⚡  [R] RTL  [A] ABORT  [Q] Çıkış"
            return "  KOMUT » [T] Kalkış  [G] Hedef Gir  [H] Home Set  [E] Engage  [R] RTL  [A] ABORT  [Q] Çıkış"

        elif self._mode == InputMode.GPS_LAT:
            return f"  GPS HEDEF » Enlem (Latitude) girin [-90 ~ +90] (ESC=İptal): {self._input_buf}█"

        elif self._mode == InputMode.GPS_LON:
            return f"  GPS HEDEF » Boylam (Longitude) girin [-180 ~ +180] (ESC=İptal): {self._input_buf}█"

        elif self._mode == InputMode.GPS_CONFIRM:
            return (f"  GPS HEDEF » Lat={self._pending_lat:.6f}, Lon={self._pending_lon:.6f} "
                    f"Gönderilsin mi? [E] Evet  [H] Hayır")

        elif self._mode == InputMode.HOME_LAT:
            return f"  MANUEL HOME » Enlem (Latitude) girin [-90 ~ +90] (ESC=İptal): {self._input_buf}█"

        elif self._mode == InputMode.HOME_LON:
            return f"  MANUEL HOME » Boylam (Longitude) girin [-180 ~ +180] (ESC=İptal): {self._input_buf}█"

        elif self._mode == InputMode.HOME_CONFIRM:
            return (f"  MANUEL HOME » Lat={self._pending_lat:.6f}, Lon={self._pending_lon:.6f} "
                    f"SET EDİLSİN Mİ? [E] Evet  [H] Hayır")

        elif self._mode == InputMode.ENGAGE_CONFIRM:
            return f"  ⚡ TAARRUZ ONAYI » 'EVET' yazıp ENTER basın (ESC=İptal): {self._input_buf}█"

        elif self._mode == InputMode.ABORT_CONFIRM:
            return f"  ⚠ ACİL DURDURMA » 'EVET' yazıp ENTER basın (ESC=İptal): {self._input_buf}█"

        # ── SİHİRBAZ PROMPTLARI ──────────────
        elif self._mode == InputMode.INIT_HOME_LAT:
            return f"  [BAŞLATMA 1/4] » MANUEL HOME (ÜS) ENLEM (Latitude) girin: {self._input_buf}█"

        elif self._mode == InputMode.INIT_HOME_LON:
            return f"  [BAŞLATMA 2/4] » MANUEL HOME (ÜS) BOYLAM (Longitude) girin: {self._input_buf}█"

        elif self._mode == InputMode.INIT_TARGET_LAT:
            return f"  [BAŞLATMA 3/4] » HEDEF (TARGET) ENLEM (Latitude) girin: {self._input_buf}█"

        elif self._mode == InputMode.INIT_TARGET_LON:
            return f"  [BAŞLATMA 4/4] » HEDEF (TARGET) BOYLAM (Longitude) girin: {self._input_buf}█"

        return "  ..."

    #  NON-BLOCKING INPUT İŞLEME
    def _handle_key(self, key: int):
        # ESC her modda iptal
        if key == 27:  # ESC
            if self._mode != InputMode.NORMAL:
                self._mode = InputMode.NORMAL
                self._input_buf = ""
                self._set_status("İşlem iptal edildi.")
            return

        # Mod'a göre dağıt
        if self._mode == InputMode.NORMAL:
            self._key_normal(key)
        elif self._mode in (InputMode.GPS_LAT, InputMode.GPS_LON, InputMode.HOME_LAT, InputMode.HOME_LON,
                             InputMode.INIT_HOME_LAT, InputMode.INIT_HOME_LON, InputMode.INIT_TARGET_LAT, InputMode.INIT_TARGET_LON):
            self._key_numeric_input(key)
        elif self._mode in (InputMode.GPS_CONFIRM, InputMode.HOME_CONFIRM):
            self._key_gps_confirm(key)
        elif self._mode in (InputMode.ENGAGE_CONFIRM, InputMode.ABORT_CONFIRM):
            self._key_text_confirm(key)

    #  NORMAL MOD
    def _key_normal(self, key: int):
        ch = chr(key).upper() if 0 < key < 256 else ""

        if ch == 'Q':
            self._running = False

        elif ch == 'T':
            state = self._node.get_state()
            if state != "IDLE":
                self._set_status(f"Kalkış reddedildi — Uçak IDLE modunda değil ({state}).")
            else:
                self._node.send_command("TAKEOFF")
                self._set_status("TAKEOFF EMRİ GÖNDERİLDİ — Kalkış Başlıyor!")

        elif ch == 'G':
            self._mode = InputMode.GPS_LAT
            self._input_buf = ""
            self._set_status("GPS hedef giriş modu — Önce ENLEM girin.")

        elif ch == 'H':
            self._mode = InputMode.HOME_LAT
            self._input_buf = "" 
            self._set_status("MANUEL HOME giriş modu aktif — Lütfen koordinatları girin.")

        elif ch == 'E':
            state = self._node.get_state()
            if state not in ("SEARCH", "WAIT_FOR_CMD"):
                self._set_status(
                    f"ENGAGE reddedildi — Uçak '{state}' modunda. "
                    f"Sadece SEARCH (hedef kilitli) modunda gönderilebilir!")
                return
            self._mode = InputMode.ENGAGE_CONFIRM
            self._input_buf = ""
            self._set_status("⚡ TAARRUZ ONAYI — 'EVET' yazıp ENTER basın.")

        elif ch == 'R':
            self._node.send_command("RTL")
            self._set_status("RTL KOMUTU GÖNDERİLDİ — Uçak eve dönüyor.")

        elif ch == 'A':
            self._mode = InputMode.ABORT_CONFIRM
            self._input_buf = ""
            self._set_status("⚠ ACİL DURDURMA — Onaylamak için 'EVET' yazın.")

        elif key != -1 and ch:
            self._set_status(
                f"Geçersiz tuş: '{ch}' — Geçerli: G, E, R, A, Q")

    #  NÜMERİK GİRDİ (GPS lat/lon)
    def _key_numeric_input(self, key: int):
        # Backspace
        if key in (curses.KEY_BACKSPACE, 127, 8):
            if self._input_buf:
                self._input_buf = self._input_buf[:-1]
            return

        # Enter — girdiyi onayla
        if key in (10, 13, curses.KEY_ENTER):
            self._commit_numeric()
            return

        # Karakter ekle (rakam, nokta, eksi)
        if 0 < key < 256:
            ch = chr(key)
            if ch in '0123456789.-' and len(self._input_buf) < 20:
                self._input_buf += ch

    def _commit_numeric(self):
        """Girilen sayısal tamponu doğrula ve modlar arasında ilerle."""
        raw = self._input_buf.strip()
        if not raw:
            self._mode = InputMode.NORMAL
            self._input_buf = ""
            self._set_status("GPS giriş iptal edildi (boş değer).")
            return

        try:
            value = float(raw)
        except ValueError:
            self._set_status(f"HATA: '{raw}' geçerli bir sayı değil!")
            self._input_buf = ""
            return

        # ── SİHİRBAZ (WIZARD) AKIŞI ──────────
        if self._mode == InputMode.INIT_HOME_LAT:
            self._pending_lat = value
            self._input_buf = ""
            self._mode = InputMode.INIT_HOME_LON
            self._set_status("2/4 - Home Enlem kaydedildi. Simdi BOYLAM girin.")

        elif self._mode == InputMode.INIT_HOME_LON:
            self._pending_lon = value
            self._input_buf = ""
            self._node.send_home_pos(self._pending_lat, self._pending_lon)
            self._mode = InputMode.INIT_TARGET_LAT
            self._set_status("3/4 - HOME SET EDİLDİ. Simdi HEDEF ENLEM girin.")

        elif self._mode == InputMode.INIT_TARGET_LAT:
            self._pending_lat = value
            self._input_buf = ""
            self._mode = InputMode.INIT_TARGET_LON
            self._set_status("4/4 - Hedef Enlem kaydedildi. Simdi BOYLAM girin.")

        elif self._mode == InputMode.INIT_TARGET_LON:
            self._pending_lon = value
            self._input_buf = ""
            self._node.send_waypoint(self._pending_lat, self._pending_lon)
            self._mode = InputMode.NORMAL
            self._set_status("BAŞARILI: Görev başlatma tamamlandı. Sistem Kalkış (T) için hazır.")

        # ── MANUEL DÜZELTME AKIŞLARI ─────────
        elif self._mode == InputMode.GPS_LAT:
            if not (LAT_MIN <= value <= LAT_MAX):
                self._set_status(
                    f"HATA: Enlem {value:.6f} aralık dışında! ({LAT_MIN}~{LAT_MAX})")
                self._input_buf = ""
                return
            self._pending_lat = value
            self._input_buf = ""
            self._mode = InputMode.GPS_LON
            self._set_status(
                f"Hedef Enlem kaydedildi: {value:.6f}° — Şimdi BOYLAM girin.")

        elif self._mode == InputMode.HOME_LAT:
            if not (LAT_MIN <= value <= LAT_MAX):
                self._set_status(
                    f"HATA: Enlem {value:.6f} aralık dışında! ({LAT_MIN}~{LAT_MAX})")
                self._input_buf = ""
                return
            self._pending_lat = value
            self._input_buf = ""
            self._mode = InputMode.HOME_LON
            self._set_status(
                f"Home Enlem kaydedildi: {value:.6f}° — Şimdi BOYLAM girin.")

        elif self._mode == InputMode.GPS_LON or self._mode == InputMode.HOME_LON:
            if not (LON_MIN <= value <= LON_MAX):
                self._set_status(
                    f"HATA: Boylam {value:.6f} aralık dışında! ({LON_MIN}~{LON_MAX})")
                self._input_buf = ""
                return
            if abs(self._pending_lat) < 0.01 and abs(value) < 0.01:
                self._set_status(
                    "HATA: (0,0) koordinatı reddedildi — muhtemel GPS hatası!")
                self._input_buf = ""
                self._mode = InputMode.NORMAL
                return
            self._pending_lon = value
            self._input_buf = ""
            if self._mode == InputMode.GPS_LON:
                self._mode = InputMode.GPS_CONFIRM
                self._set_status(
                    f"Lat={self._pending_lat:.6f}, Lon={self._pending_lon:.6f} → "
                    f"Onay için [E], İptal için [H]")
            else:
                self._mode = InputMode.HOME_CONFIRM
                self._set_status(
                    f"MANUEL HOME: Lat={self._pending_lat:.6f}, Lon={self._pending_lon:.6f} → "
                    f"Onay için [E], İptal için [H]")

    # ── GPS ONAY (E/H) ───────────────────────────────
    def _key_gps_confirm(self, key: int):
        if 0 < key < 256:
            ch = chr(key).upper()
            if ch in ('E', 'Y'):
                if self._mode == InputMode.GPS_CONFIRM:
                    self._node.send_waypoint(self._pending_lat, self._pending_lon)
                    self._set_status(
                        f"GPS HEDEFİ GÖNDERİLDİ → Lat: {self._pending_lat:.6f}, "
                        f"Lon: {self._pending_lon:.6f}")
                else:
                    self._node.send_home_pos(self._pending_lat, self._pending_lon)
                    self._set_status(
                        f"MANUEL HOME SET EDİLDİ → Lat: {self._pending_lat:.6f}, "
                        f"Lon: {self._pending_lon:.6f}")
                self._mode = InputMode.NORMAL
            elif ch in ('H', 'N'):
                self._set_status("İşlem iptal edildi.")
                self._mode = InputMode.NORMAL

    # ── METİN ONAY (ENGAGE / ABORT — "EVET" yazma) ──
    def _key_text_confirm(self, key: int):
        # Backspace
        if key in (curses.KEY_BACKSPACE, 127, 8):
            if self._input_buf:
                self._input_buf = self._input_buf[:-1]
            return

        # Enter — kontrol et
        if key in (10, 13, curses.KEY_ENTER):
            typed = self._input_buf.strip().upper()
            if typed in ("EVET", "E", "YES", "Y"):
                if self._mode == InputMode.ENGAGE_CONFIRM:
                    self._node.send_command("ENGAGE")
                    self._set_status(
                        "⚡⚡⚡ ENGAGE GÖNDERİLDİ — TAARRUZ BAŞLATILDI ⚡⚡⚡")
                elif self._mode == InputMode.ABORT_CONFIRM:
                    self._node.send_command("ABORT")
                    self._set_status(
                        "!!! ABORT GÖNDERİLDİ — GÖREV İPTAL EDİLDİ !!!")
            else:
                if self._mode == InputMode.ENGAGE_CONFIRM:
                    self._set_status("ENGAGE iptal edildi — Operatör onay vermedi.")
                else:
                    self._set_status("ABORT iptal edildi.")
            self._mode = InputMode.NORMAL
            self._input_buf = ""
            return

        # Harf ekle
        if 0 < key < 256:
            ch = chr(key)
            if ch.isalpha() and len(self._input_buf) < 10:
                self._input_buf += ch

    #  YARDIMCI
    def _set_status(self, msg: str, duration: float = 6.0):
        self._status_msg = msg
        self._status_expire = time.time() + duration

    @staticmethod
    def _fmt_time(sec: float) -> str:
        m, s = divmod(int(sec), 60)
        h, m = divmod(m, 60)
        return f"{h:02d}:{m:02d}:{s:02d}"

    @staticmethod
    def _safe_addstr(win, y: int, x: int, text: str, attr: int = 0):
        """curses sınır taşma hatasını sessizce yakalar."""
        try:
            win.addstr(y, x, text, attr)
        except curses.error:
            pass


#  GİRİŞ NOKTASI
def main(args=None):
    rclpy.init(args=args)
    node = GCSPanelNode()

    # ROS 2 spinning → arka plan daemon thread
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    ui = TacticalUI(node)

    try:
        curses.wrapper(ui.run)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("GCS Panel kapatılıyor...")
        node.destroy_node()
        rclpy.try_shutdown()
        print("\n\033[92m╔════════════════════════════════════════╗\033[0m")
        print("\033[92m║   VTOL GCS Panel güvenle kapatıldı.    ║\033[0m")
        print("\033[92m╚════════════════════════════════════════╝\033[0m")


if __name__ == '__main__':
    main()
