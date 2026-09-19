#!/usr/bin/env python3

import os
import math
import cv2
import numpy as np
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

try:
    from ultralytics import YOLO
    import torch
except ImportError:
    YOLO = None
    torch = None

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        #  PARAMETRELER — camera_params.yaml'dan okunur             
        #  Varsayılan değerler YAML eksikse güvenlik ağı olarak     

        self.declare_parameter('camera_topic', '/camera/image_raw/compressed')
        self.declare_parameter('confidence_threshold', 0.60)
        self.declare_parameter('model_path', '')  # Boş = paket dizinindeki varsayılan best.pt
        
        # ── DİNAMİK İRTİFA-GÖRÜŞ MODELİ PARAMETRELERİ ────────────────
        self.declare_parameter('dynamic_confidence_enabled', True)
        self.declare_parameter('confidence_ref_altitude_m', 70.0)
        self.declare_parameter('confidence_altitude_scale', 0.07)
        self.declare_parameter('confidence_min', 0.40)
        self.declare_parameter('confidence_max', 0.85)
        
        self.camera_topic = self.get_parameter('camera_topic').value
        self.conf_threshold_base = self.get_parameter('confidence_threshold').value
        self.conf_threshold = self.conf_threshold_base  # Aktif eşik (dinamik güncellenir)
        
        # İrtifa telemetrisi (PX4'ten gelecek)
        self._current_altitude_agl = 70.0  # Varsayılan referans irtifa (henüz veri gelmeden)
        
        _model_path_param = self.get_parameter('model_path').value
        if _model_path_param:
            model_path = _model_path_param
        else:
            try:
                package_share_directory = get_package_share_directory('vtol_vision')
                model_path = os.path.join(package_share_directory, 'models', 'best.pt')
            except Exception as e:
                self.get_logger().error(f"Paket dizini bulunamadı: {e}")
                model_path = "best.pt"  
            
        self.get_logger().info(f"YOLO Yapay Zeka Modeli Yükleniyor: {model_path}")
        
        # Modeli sadece node baslatilirken 1 kere hafızaya alıyoruz
        if YOLO is not None:
            try:
                self.yolo_model = YOLO(model_path)
                self.get_logger().info("YOLO motoru başarıyla başlatıldı.")
            except Exception as e:
                self.get_logger().error(f"YOLO modeli yüklenirken hata oluştu: {e}")
                self.yolo_model = None
        else:
            self.get_logger().error("ultralytics kütüphanesi bulunamadı! 'pip install ultralytics' ile kurun.")
            self.yolo_model = None

        self.bridge = CvBridge()
        
        # 2. ROS 2 İletişim Arayüzleri (Pub/Sub Mimarisi)
        # Abonelik (Subscriber): Kameradan gelen ham görüntü verisini okumak için
        stream_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        self.image_sub = self.create_subscription(
            CompressedImage,
            self.camera_topic,
            self.image_callback,
            stream_qos
        )
        
        # İrtifa Aboneliği (Subscriber): PX4'ten anlık irtifa verisi
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self._altitude_cb,
            stream_qos
        )
        
        # Koordinat Yayıncısı (Publisher): Hedefin görüntü üzerindeki merkez piksellerini (X,Y) iletmek için 
        self.coordinate_pub = self.create_publisher(Point, '/vision/target_error', 10)
        
        # Durum Yayıncısı (Publisher): Hedefin o anki durumunu (SEARCHING veya LOCKED) iletmek için
        self.state_pub = self.create_publisher(String, '/vision/target_state', 10)
        
        # Taktik Ekran / Geri Bildirim (HUD) Yayını
        self.debug_pub = self.create_publisher(Image, '/vision/debug_image', 10)

        # ── HUD Telemetri Aboneliği (state_machine JSON yayını) ──
        self._hud_data = {}
        self.create_subscription(String, '/vtol/search_info', self._hud_info_cb, 10)

        self.get_logger().info(f'Vision Node çalışıyor. Kamera dinleniyor: {self.camera_topic}')
        self.get_logger().info(f'Yapay Zeka Modu Aktif (Güvenilirlik Eşiği: {self.conf_threshold_base*100:.0f}%).')
        if self.get_parameter('dynamic_confidence_enabled').value:
            ref_alt = self.get_parameter('confidence_ref_altitude_m').value
            self.get_logger().info(f'DİNAMİK GÖRÜŞ MODELİ AKTİF: Referans irtifa={ref_alt:.0f}m')

    def _altitude_cb(self, msg: VehicleLocalPosition):
        """PX4'ten gelen anlık irtifa verisini günceller."""
        # NED -> AGL: Z negatifken irtifa pozitif
        self._current_altitude_agl = max(1.0, -msg.z)
        
        # Dinamik confidence eşiğini güncelle
        if self.get_parameter('dynamic_confidence_enabled').value:
            self.conf_threshold = self._compute_dynamic_confidence(self._current_altitude_agl)

    def _compute_dynamic_confidence(self, altitude_m):
        """
        DİNAMİK İRTİFA-GÜVEN EŞİĞİ HESAPLAYICI
        ══════════════════════════════════════════
        Logaritmik Model:
          threshold = base - (scale × ln(altitude / ref_altitude))
        
        Mantık:
          - İrtifa referanstan yüksekse → hedef küçülür → threshold düşer (daha toleranslı)
          - İrtifa referanstan alçaksa  → hedef büyür  → threshold artar (daha seçici)
        
        Logaritma kullanmamızın sebebi: Perspektif projeksiyonunda hedef boyutu
        irtifayla lineer değil, ters orantılı azalır. ln() bu azalmayı doğru modeller.
        
        Args:
            altitude_m: Mevcut irtifa (metre AGL, pozitif)
        Returns:
            Kelepçelenmiş dinamik confidence eşiği (0.0-1.0 arası)
        """
        base  = self.conf_threshold_base
        ref   = self.get_parameter('confidence_ref_altitude_m').value
        scale = self.get_parameter('confidence_altitude_scale').value
        c_min = self.get_parameter('confidence_min').value
        c_max = self.get_parameter('confidence_max').value
        
        # Güvenlik: Referans irtifa en az 1m olmalı
        ref = max(1.0, ref)
        altitude_m = max(1.0, altitude_m)
        
        # Logaritmik model: threshold = base - scale × ln(alt / ref)
        # alt > ref → ln > 0 → threshold düşer (yüksek irtifa, toleranslı)
        # alt < ref → ln < 0 → threshold artar (alçak irtifa, seçici)
        dynamic_threshold = base - (scale * math.log(altitude_m / ref))
        
        # Güvenlik kelepçeleme
        return max(c_min, min(c_max, dynamic_threshold))

    def image_callback(self, msg):
        # Sıkıştırılmış JPEG'i (CompressedImage) çözmek için en güvenli yol NumPy ve OpenCV kullanmaktır
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f'Kritik Sensör Hatası! Görüntü çözülemedi: {e}', throttle_duration_sec=2.0)
            return

        if cv_image is None:
            self.get_logger().warn('Bozuk görüntü paketi alındı.', throttle_duration_sec=2.0)
            return

        height, width, _ = cv_image.shape
        img_center_x = width // 2
        img_center_y = height // 2

        target_found = False
        target_point = Point()
        target_point.z = 0.0  

        # 4. Yapay Zeka Çıkarım (Inference) ve Filtreleme
        if self.yolo_model is not None:
            # Modeli çalıştırıp sonuçları al (Ayrıntılı logları kapalı - verbose)
            device = 'cuda:0' if (torch is not None and torch.cuda.is_available()) else 'cpu'
            results = self.yolo_model(cv_image, verbose=False, device=device)
            
            best_conf = 0.0
            best_bbox = None
            
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    conf = float(box.conf[0])
                    if conf >= self.conf_threshold and conf > best_conf:
                        best_conf = conf
                        best_bbox = box.xyxy[0].cpu().numpy()  
                        
            if best_bbox is not None:
                target_found = True
                
                # 5. Koordinat Çıkarımı ve Güdüm Verisi Aktarımı
                # Bounding Box köşe koordinatları (x1, y1, x2, y2)
                x1, y1, x2, y2 = map(int, best_bbox)
                
                # Dört köşenin matematiksel ortalaması ile tam merkez noktası (Center X, Center Y)
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                
                # Merkez koordinatlarını (Ekran merkezine olan hata/error payı olarak) yayıncı formatına ata
                target_point.x = float(center_x - img_center_x)
                target_point.y = float(center_y - img_center_y)
                target_point.z = 1.0  # Hedef Görüldü İşareti
                
                # İşlenen görüntünün üzerine hedefin etrafını saran bir çerçeve (Bounding Box) çiz
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 3)
                # Kilitlenme Metni
                cv2.putText(cv_image, "LOCKED", (x1, max(y1 - 10, 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                # Hedef Merkez Noktası Ekle
                cv2.circle(cv_image, (center_x, center_y), 5, (0, 0, 255), -1)

        # 6. Taktik Ekran (HUD) ve Geri Bildirim Durum Yayını
        state_msg = String()

        if target_found:
            # İşlem gören hedefe merkezden bir nişan çizgisi çiz
            cv2.line(cv_image, (img_center_x, img_center_y), (center_x, center_y), (0, 255, 255), 2)
            
            # Durum Yayıncısı üzerinden sinyal gönder
            state_msg.data = "LOCKED"
            text_color = (0, 0, 255)
        else:
            # Hedef tespit edilemediyse koordinatlar 0 kalsın
            target_point.x = 0.0
            target_point.y = 0.0
            target_point.z = 0.0
            
            # Durum Yayıncısı üzerinden sinyal gönder
            state_msg.data = "SEARCHING"
            text_color = (0, 255, 0)

        # Hesaplanan merkez koordinatını kaslara (servo modülüne) fırlat
        self.coordinate_pub.publish(target_point)
        # Durumu beyne (state_machine) yayınla
        self.state_pub.publish(state_msg)

        # HUD Ekranına Modu (SEARCHING / LOCKED) Yazdır
        cv2.putText(cv_image, f"MODE: {state_msg.data}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2)
        
        # Dinamik Görüş Bilgisi HUD Satırı
        alt_text = f"ALT: {self._current_altitude_agl:.0f}m | CONF: {self.conf_threshold*100:.0f}%"
        cv2.putText(cv_image, alt_text, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Ekranın tam ortasına her zaman statik bir nişangah (Crosshair) çiz (Yeşil)
        cross_size = 20
        cv2.line(cv_image, (img_center_x - cross_size, img_center_y), (img_center_x + cross_size, img_center_y), (0, 255, 0), 2)
        cv2.line(cv_image, (img_center_x, img_center_y - cross_size), (img_center_x, img_center_y + cross_size), (0, 255, 0), 2)
        cv2.circle(cv_image, (img_center_x, img_center_y), 3, (0, 255, 0), -1)

        # ── HUD Overlay (Uçuş Gösterge Paneli) ──
        self._draw_hud(cv_image, img_center_x, img_center_y)

        # Telemetri Yayını (İşlenmiş Görüntü → /vision/debug_image)
        # GUI (cv2.imshow) KULLANILMAZ — callback thread'ini bloklar!
        # Operatör görüntüyü rqt_image_view ile izler.
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.debug_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Debug görüntüsü yayınlanamadı: {e}', throttle_duration_sec=2.0)

    def _hud_info_cb(self, msg):
        """State machine'den gelen JSON telemetri verisini HUD overlay için depolar."""
        try:
            self._hud_data = json.loads(msg.data)
        except (json.JSONDecodeError, Exception):
            pass

    def _draw_hud(self, frame, cx, cy):
        """
        UÇUŞ GÖSTERGE PANELİ (HUD) — cv2 Overlay
        ═══════════════════════════════════════════════════
        Kamera görüntüsü üzerine uçuş aletlerini andıran göstergeler çizer:
          - Sol alt: BAT, HDG, AIR_SPD, CRAB telemetri paneli
          - Merkez: Rüzgar sürüklenme vektörü (Crab Angle ok işareti)

        Veri Kaynağı: /vtol/search_info JSON topic'inden gelir.
        """
        data = self._hud_data
        if not data:
            return

        h, w = frame.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX

        # Renk Paleti
        GREEN  = (0, 255, 0)
        YELLOW = (0, 255, 255)
        RED    = (0, 0, 255)
        GRAY   = (128, 128, 128)
        ORANGE = (0, 165, 255)

        bat_pct  = data.get("battery_percent", 0)
        bat_v    = data.get("battery_volt", 0)
        bat_ok   = data.get("battery_valid", False)
        hdg      = data.get("heading_deg", 0)
        hdg_ok   = data.get("heading_valid", False)
        airspd   = data.get("airspeed_mps", 0)
        air_ok   = data.get("airspeed_valid", False)
        crab     = data.get("crab_angle_deg", 0)
        bingo    = data.get("bingo_battery_pct", 20)

        # ══ SOL ALT KÖŞE: Telemetri Paneli ══
        panel_lines = []

        # BAT
        if bat_ok:
            bat_color = GREEN if bat_pct > 50 else (YELLOW if bat_pct > 30 else RED)
            bat_text = f"BAT  {bat_pct:.0f}% ({bat_v:.1f}V)"
            if bat_pct <= bingo:
                bat_text += " BINGO!"
        else:
            bat_text = "BAT  ---"
            bat_color = GRAY
        panel_lines.append((bat_text, bat_color))

        # HDG
        hdg_text = f"HDG  {hdg:03.0f}" if hdg_ok else "HDG  ---"
        panel_lines.append((hdg_text, GREEN if hdg_ok else GRAY))

        # AIR SPD
        if air_ok:
            panel_lines.append((f"AIR  {airspd:.1f} m/s", GREEN))
        else:
            panel_lines.append((f"GND  {airspd:.1f} m/s", YELLOW))

        # CRAB
        crab_color = RED if abs(crab) > 15 else (YELLOW if abs(crab) > 8 else GREEN)
        panel_lines.append((f"CRAB {crab:+.0f}", crab_color))

        # Yarı saydam arka plan paneli
        line_h = 18
        pad = 6
        panel_h = len(panel_lines) * line_h + 2 * pad
        panel_w = 185
        y0 = h - panel_h - 4

        overlay = frame.copy()
        cv2.rectangle(overlay, (2, y0), (panel_w, h - 2), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, frame)
        cv2.rectangle(frame, (2, y0), (panel_w, h - 2), GREEN, 1)

        for i, (text, color) in enumerate(panel_lines):
            cv2.putText(frame, text,
                        (8, y0 + pad + (i + 1) * line_h - 4),
                        font, 0.45, color, 1, cv2.LINE_AA)

        # ══ MERKEZ: Rüzgar Sürüklenme Vektörü (Crab Angle) ══
        # Nişangahın altında yatay ok ile rüzgarın uçağı
        # ne yöne ittiğini gösterir. Ok uzunluğu açıya orantılıdır.
        if abs(crab) > 2.0:
            crab_px = max(-40, min(40, int(crab * 2.5)))
            vec_color = RED if abs(crab) > 15 else ORANGE
            arrow_y = cy + 30
            cv2.arrowedLine(frame, (cx, arrow_y), (cx + crab_px, arrow_y),
                            vec_color, 2, tipLength=0.4)
            cv2.putText(frame, f"{crab:+.0f}",
                        (cx + crab_px + 5, arrow_y + 4),
                        font, 0.35, vec_color, 1, cv2.LINE_AA)


def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()
    
    try:
        rclpy.spin(vision_node)
    except KeyboardInterrupt:
        vision_node.get_logger().info('Vision Node durduruluyor.')
    finally:
        try:
            vision_node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
