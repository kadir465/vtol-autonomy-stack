import cv2
import math
from ultralytics import YOLO
import time

# ── AYARLAR ───────────────────────────────────────
MODEL_PATH = "/home/kadir/vtol_ws/src/vtol_vision/models/best.pt"
VIDEO_PATH = "/home/kadir/vtol_ws/test.mp4"  # Test videon
CONFIDENCE_THRESHOLD = 0.50

print(f"[*] Model yükleniyor: {MODEL_PATH}")
model = YOLO(MODEL_PATH)

cap = cv2.VideoCapture(VIDEO_PATH)
if not cap.isOpened():
    print("HATA: Video açılamadı!")
    exit()

print("[*] Merkez Kilitli Video testi başlıyor. Çıkmak için 'Q'")

while cap.isOpened():
    start_time = time.time()
    success, frame = cap.read()
    if not success:
        break

    # Nişangah Merkezi
    height, width, _ = frame.shape
    frame_center_x = width // 2
    frame_center_y = height // 2

    # YOLO Çıkarımı
    results = model.predict(frame, conf=CONFIDENCE_THRESHOLD, verbose=False)
    
    best_target = None
    min_distance = float('inf') # Sonsuzdan başlat

    # 1. Aşama: Ekrandaki tüm hedefleri tara ve MERKEZE EN YAKIN olanı bul
    for r in results:
        boxes = r.boxes
        for box in boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            
            # Bu hedefin merkezi
            target_cx = (x1 + x2) // 2
            target_cy = (y1 + y2) // 2
            
            # Ekrana bu kutuyu soluk renkle çiz (diğer araçları görmek için)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 100, 0), 1)

            # Nişangaha olan uzaklığı hesapla (Pisagor)
            dist = math.sqrt((target_cx - frame_center_x)**2 + (target_cy - frame_center_y)**2)
            
            # Eğer bu araç şu ana kadar bulduğumuz en yakın araçsa, bunu kaydet
            if dist < min_distance:
                min_distance = dist
                best_target = (x1, y1, x2, y2, target_cx, target_cy, conf)

    # 2. Aşama: Sadece EN YAKIN hedefe KİLİTLEN ve Hata Vektörünü çiz
    if best_target is not None:
        x1, y1, x2, y2, target_cx, target_cy, conf = best_target
        
        # PİKSEL HATASI
        err_x = target_cx - frame_center_x
        err_y = target_cy - frame_center_y

        # Seçilen asıl hedefi kalın ve kırmızı çiz
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
        cv2.circle(frame, (target_cx, target_cy), 5, (0, 0, 255), -1)
        
        # O ölümcül Mavi Hata Vektörü
        cv2.line(frame, (frame_center_x, frame_center_y), (target_cx, target_cy), (255, 0, 0), 2)
        
        # Bilgi Ekranı
        text = f"KILIT: ErrX: {err_x}px | ErrY: {err_y}px | Conf: {conf:.2f}"
        cv2.putText(frame, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # Ekranın ortasına nişangah çiz
    cv2.drawMarker(frame, (frame_center_x, frame_center_y), (255, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)

    fps = 1.0 / (time.time() - start_time)
    cv2.putText(frame, f"FPS: {fps:.1f}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

    cv2.imshow("YOLO Merkez Kilit Sistemi", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()