#!/usr/bin/env python3
"""
Camera Stream Node — Uçak Üstü Kamera Yayın Modülü
=====================================================
Görev:
  Uçağın üzerindeki kameradan görüntü yakalar, JPEG formatında sıkıştırır
  ve ROS 2 ağı üzerinden yer istasyonuna CompressedImage olarak yayınlar.

Topic Yayın:
  /camera/image_raw/compressed  (sensor_msgs/CompressedImage)

Topic Dinleme:
  /vtol/current_state  (std_msgs/String) — Uçuş durumu

Durum Algısı (State Awareness):
  Kamera yalnızca aktif görev modlarında (SEARCH, TRACKING, KAMIKAZE)
  çalışır. Diğer modlarda (IDLE, TAKEOFF, CRUISE vb.) uyku modundadır.
  Bu sayede uçuşun başında gereksiz CPU kullanımı ve hata logları önlenir.

Mimari Rol:
  Uçak (stream_node) → WiFi/Telemetri → Yer İstasyonu (vision_node)

QoS:
  BEST_EFFORT — Kablosuz ağda paket kaybında eski kareleri beklemez,
  yeni kareye geçer. Gecikmeyi minimize eder.
"""

import cv2
import rclpy
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class CameraStreamNode(Node):

    _ACTIVE_STATES = frozenset(['SEARCH', 'WAIT_FOR_CMD', 'ENGAGE'])

    def __init__(self):
        super().__init__('camera_stream_node')

        self._current_state = 'IDLE'
        self._camera_awake  = False      

        self.create_subscription(
            String,
            '/vtol/current_state',
            self._state_callback,
            10
        )

        self.get_logger().info(
            '═══════════════════════════════════════════════════\n'
            '  [KAMERA] Uyku Modunda — Aktif mod bekleniyor…\n'
            '  Aktif modlar : SEARCH · WAIT_FOR_CMD · ENGAGE\n'
            '  Dinlenen topic: /vtol/state\n'
            '═══════════════════════════════════════════════════'
        )

        # ╔══════════════════════════════════════════════════╗
        # ║  DİNAMİK PARAMETRELER                           ║
        # ╚══════════════════════════════════════════════════╝
        # Sahada kodu yeniden derlemeden değiştirilebilir
        # 0=USB, GStreamer string, veya "ros:/topic_name" (simülasyon için)
        self.declare_parameter('video_source', '0')           
        self.declare_parameter('image_width',  640)         
        self.declare_parameter('image_height', 480)
        self.declare_parameter('fps',          30)
        self.declare_parameter('jpeg_quality', 60)          

        self._video_source = self.get_parameter('video_source').value
        self._width        = self.get_parameter('image_width').value
        self._height       = self.get_parameter('image_height').value
        self._fps          = self.get_parameter('fps').value
        self._jpeg_quality = self.get_parameter('jpeg_quality').value

           #  Kamera burada AÇILMAZ! Aktif moda geçildiğinde       
        self._cap = None
        self._sim_sub = None
        self._bridge = CvBridge()
        self._latest_sim_frame = None

        # BEST_EFFORT QoS: Kablosuz ağda gecikmeyi minimize eder
        stream_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1     # Sadece son kare — eski kareleri biriktirme
        )

        self._pub = self.create_publisher(
            CompressedImage,
            '/camera/image_raw/compressed',
            stream_qos
        )

        # ╔══════════════════════════════════════════════════╗
        # ║  YAKALAMA DÖNGÜSÜ (Timer)                      ║
        # ╚══════════════════════════════════════════════════╝
        timer_period = 1.0 / max(self._fps, 1)
        self.create_timer(timer_period, self._capture_and_publish)

    # ─────────────────────────────────────────────────────────
    #  DURUM CALLBACK: /vtol/state topic'inden gelen modu kaydeder
    # ─────────────────────────────────────────────────────────
    def _state_callback(self, msg: String):
        """Uçuş durumu her değiştiğinde güncellenir."""
        new_state = msg.data.strip().upper()
        if new_state != self._current_state:
            self._current_state = new_state
            if new_state in self._ACTIVE_STATES:
                self.get_logger().info(
                    f'[KAMERA]  Aktif mod algılandı: {new_state} — Kamera uyanıyor!'
                )
            else:
                self.get_logger().info(
                    f'[KAMERA]  Pasif mod: {new_state} — Kamera uyku modunda.'
                )

 
    def _wake_up_camera(self):
        """Kamerayı ilk kez başlatır (lazy initialization). Bir kez dener, başarısız olsa bile tekrar denemez."""
        if self._camera_awake:
            return

        # tekrar denemeyi engelle
        self._camera_awake = True

        src = self._video_source
        if isinstance(src, str) and src.startswith("ros:"):
            # Simülasyon Girdisi (ROS Topic üzerinden)
            topic_name = src.replace("ros:", "")
            self.get_logger().info(f'[KAMERA] Simülasyon modu! {topic_name} dinleniyor...')
            
            # Otonom Sensör QoS'i 
            sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                depth=10
            )
            self._sim_sub = self.create_subscription(Image, topic_name, self._sim_img_cb, sensor_qos)
        else:
            # Fiziksel veya GStreamer Girdisi
            if isinstance(src, str) and src.isdigit():
                src = int(src)
            self._cap = cv2.VideoCapture(src)

            if not self._cap.isOpened():
                self.get_logger().error(
                    f'[KAMERA] HATA! Kamera kaynağı açılamadı: {self._video_source} '
                    f'— Simülasyonda fiziksel kamera yoksa bu normaldir.'
                )
                self._cap = None   # Temizle — _capture_and_publish throttle ile loglar
                return

            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self._width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
            self._cap.set(cv2.CAP_PROP_FPS,          self._fps)

        self.get_logger().info(
            '═══════════════════════════════════════════════════\n'
            '  [KAMERA] Yayın Başladı!\n'
            '  Kaynak       : %s\n'
            '  Çözünürlük   : %dx%d @ %d fps\n'
            '  JPEG Kalitesi: %%%d\n'
            '  Topic        : /camera/image_raw/compressed\n'
            '═══════════════════════════════════════════════════'
            % (self._video_source, self._width, self._height,
               self._fps, self._jpeg_quality)
        )

    def _sim_img_cb(self, msg):
        """Simülasyondan gelen ham ROS görüntüsünü hafızaya alır."""
        try:
            self._latest_sim_frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"[KAMERA] Image bridge hatası: {str(e)}", throttle_duration_sec=5.0)

   
    #  ANA DÖNGÜ: Yakala → Boyutlandır → Sıkıştır → Yayınla
    def _capture_and_publish(self):
        """
        Kameradan kare okur, JPEG sıkıştırır ve CompressedImage olarak yayınlar.
        Yalnızca aktif görev modlarında çalışır (SEARCH/WAIT_FOR_CMD/ENGAGE).
        """
        # Aktif görev modunda değilse kamerayı açma 
        if self._current_state not in self._ACTIVE_STATES:
            return

        #  Kamera henüz açılmamışsa → lazy init 
        self._wake_up_camera()

        is_sim = (isinstance(self._video_source, str) and self._video_source.startswith("ros:"))

        if not is_sim and (self._cap is None or not self._cap.isOpened()):
            self.get_logger().error(
                '[KAMERA] Kamera bağlantısı yok! Kablo/kaynak kontrol edin.',
                throttle_duration_sec=3.0
            )
            return

        # 1. Kameradan kare oku
        if is_sim:
            frame = self._latest_sim_frame
            if frame is None:
                # Diagnostics: List available topics once in a while
                if not hasattr(self, '_last_diag_time'): self._last_diag_time = 0
                if time.time() - self._last_diag_time > 10.0:
                    self._last_diag_time = time.time()
                    all_topics = self.get_topic_names_and_types()
                    img_topics = [t for t, types in all_topics if 'sensor_msgs/msg/Image' in types]
                    self.get_logger().error(
                        f"[KAMERA DIAG] '{self._video_source.replace('ros:', '')}' bulunamadı!\n"
                        f"Bulunan Görüntü Topicleri: {img_topics}\n"
                        "Eğer liste boşsa Gazebo plugin'i çalışmıyor demektir."
                    )
                return
        else:
            ret, frame = self._cap.read()
            if not ret:
                frame = None

        if frame is None:
            self.get_logger().error(
                "[KAMERA] Kare okunamadı! Kamera kablosu çıkmış olabilir veya simülasyonda görüntü yok.",
                throttle_duration_sec=2.0
            )
            return

        # 2. Parametrelerdeki çözünürlüğe boyutlandır
        if frame.shape[1] != self._width or frame.shape[0] != self._height:
            frame = cv2.resize(frame, (self._width, self._height))

        # 3. JPEG formatında sıkıştır
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality]
        success, encoded = cv2.imencode('.jpg', frame, encode_params)

        if not success:
            self.get_logger().warn(
                "[KAMERA] JPEG encode başarısız!",
                throttle_duration_sec=2.0
            )
            return

        # 4. ROS CompressedImage mesajını oluştur ve yayınla
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        msg.format = "jpeg"
        msg.data = encoded.tobytes()

        self._pub.publish(msg)

    #  TEMİZLİK
    def destroy_node(self):
        """Kamerayı serbest bırakır ve node'u kapatır."""
        if self._cap is not None and self._cap.isOpened():
            self._cap.release()
            self.get_logger().info('[KAMERA] Kamera serbest bırakıldı.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[KAMERA] Yayın durduruldu.')
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
