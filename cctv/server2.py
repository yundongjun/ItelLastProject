import cv2
import requests
import numpy as np
from ultralytics import YOLO
import torch
import os
import socket
import threading
import time

# --- Display Scaling ---
try:
    MAP_SCALE = float(os.getenv("MAP_SCALE", "2.0"))
except Exception:
    MAP_SCALE = 2.0

try:
    DETECT_ZOOM = float(os.getenv("DETECT_ZOOM", "2.0"))
except Exception:
    DETECT_ZOOM = 1.0

# --- Digital Zoom ---
def digital_zoom(frame, zoom=1.5):
    if zoom <= 1.0:
        h, w = frame.shape[:2]
        return frame, (0, 0), w, h
    h, w = frame.shape[:2]
    nh, nw = int(h / zoom), int(w / zoom)
    nh = max(1, nh)
    nw = max(1, nw)
    y1 = max((h - nh) // 2, 0)
    x1 = max((w - nw) // 2, 0)
    crop = frame[y1:y1 + nh, x1:x1 + nw]
    resized = cv2.resize(crop, (w, h), interpolation=cv2.INTER_LINEAR)
    return resized, (x1, y1), w, h

# --- Socket Configuration ---
SERVER_HOST = "10.10.16.166"   # TurtleBot (ROS)
SERVER_PORT = 5000
RPI_HOST = "10.10.16.78"       # Raspberry Pi (Buzzer + Temperature)
RPI_PORT = 5000

FLAME_TIMEOUT = 5.0
THERMAL_THRESHOLD = 50.0  # 온도 임계값 (°C)

# --- YOLO Model Loading ---
print("Loading custom model...")
device = 'cuda' if torch.cuda.is_available() else 'cpu'
model = YOLO('best.pt')
model.to(device)
print(f"✅ Model loaded successfully on {device}.")

# --- MJPEG Stream URL ---
MJPEG_URL = f"http://{RPI_HOST}:8080/?action=stream"

# --- Global States ---
flame_detected_by_rpi = False
last_flame_time = 0
current_temp = 0.0


# ============================================================
# 🔹 RPi → Ubuntu 메시지 수신 (TEMP / FLAME_DETECTED)
# ============================================================
def recv_from_rpi(sock: socket.socket):
    global flame_detected_by_rpi, last_flame_time, current_temp
    try:
        while True:
            data = sock.recv(1024)
            if not data:
                print("🔌 RPi disconnected.")
                break

            msg = data.decode().strip()
            if msg.startswith("TEMP:"):
                try:
                    temp_val = float(msg.split(":")[1])
                    current_temp = temp_val
                    print(f"🌡 Current RPi Temp: {temp_val:.2f}°C")
                    if temp_val >= THERMAL_THRESHOLD:
                        flame_detected_by_rpi = True
                        last_flame_time = time.time()
                    else:
                        flame_detected_by_rpi = False
                except ValueError:
                    print(f"⚠️ Invalid TEMP format: {msg}")

            elif "FLAME_DETECTED" in msg:
                flame_detected_by_rpi = True
                last_flame_time = time.time()
                print("🔥 Received FLAME_DETECTED flag from RPi")

            else:
                print(f"📨 RPi says: {msg}")

    except Exception as e:
        print(f"❌ Error recv_from_rpi: {e}")


# ============================================================
# 🔹 YOLO + 온도 감지 + TurtleBot 전송
# ============================================================
def display_stream(turtle_sock: socket.socket, rpi_sock: socket.socket):
    global flame_detected_by_rpi, last_flame_time, current_temp

    homography_path = 'homography_matrix.npy'
    map_path = 'map.pgm'

    if not os.path.exists(homography_path) or not os.path.exists(map_path):
        print("❌ Map or homography missing.")
        return

    homography_matrix = np.load(homography_path)
    map_img = cv2.imread(map_path)

    # 회색 배경 자동 크롭
    crop_x, crop_y = 0, 0
    base_map = map_img
    try:
        gray = cv2.cvtColor(map_img, cv2.COLOR_BGR2GRAY)
        edges = np.concatenate((gray[0, :], gray[-1, :], gray[:, 0], gray[:, -1]))
        bg_val = int(np.bincount(edges.flatten()).argmax()) if edges.size else int(gray[0, 0])
        mask = np.abs(gray - bg_val) > 5
        rows = np.where(mask.any(axis=1))[0]
        cols = np.where(mask.any(axis=0))[0]
        if rows.size and cols.size:
            y1, y2 = int(rows[0]), int(rows[-1]) + 1
            x1, x2 = int(cols[0]), int(cols[-1]) + 1
            crop_x, crop_y = x1, y1
            base_map = map_img[y1:y2, x1:x2].copy()
    except Exception:
        base_map = map_img
    print("🗺️ Map and homography loaded.")

    buzzer_state = 0
    last_trigger_time = 0
    last_coord_sent_time = 0

    print(f"Connecting to stream: {MJPEG_URL}")
    session = requests.Session()
    stream = session.get(MJPEG_URL, stream=True, timeout=10)
    bytes_buffer = bytes()

    while True:
        bytes_buffer += stream.raw.read(4096)
        start = bytes_buffer.find(b'\xff\xd8')
        end = bytes_buffer.find(b'\xff\xd9')
        if start == -1 or end == -1:
            continue

        jpg = bytes_buffer[start:end+2]
        bytes_buffer = bytes_buffer[end+2:]
        frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            continue

        # 디지털 줌 적용 (YOLO 입력용)
        if DETECT_ZOOM > 1.0:
            zoomed_frame, (crop_x1, crop_y1), orig_w, orig_h = digital_zoom(frame, DETECT_ZOOM)
        else:
            zoomed_frame = frame
            crop_x1, crop_y1 = 0, 0
            orig_w, orig_h = frame.shape[1], frame.shape[0]

        annotated_frame = frame.copy()  # ✅ 원본 프레임 기준으로 그림

        # 맵 표시용 확대
        if abs(MAP_SCALE - 1.0) > 1e-6:
            mh, mw = base_map.shape[:2]
            map_display = cv2.resize(base_map, (int(mw * MAP_SCALE), int(mh * MAP_SCALE)),
                                     interpolation=cv2.INTER_NEAREST)
        else:
            map_display = base_map.copy()

        person_coords = None
        fire_coords = None

        # --- YOLO 타일 기반 추론 ---
        h, w = zoomed_frame.shape[:2]
        rows, cols = 2, 2
        stride_ratio = 0.5
        tile_h = max(1, int(np.ceil(h / rows)))
        tile_w = max(1, int(np.ceil(w / cols)))
        stride_h = max(1, int(tile_h * stride_ratio))
        stride_w = max(1, int(tile_w * stride_ratio))

        ys = list(range(0, max(1, h - tile_h) + 1, stride_h))
        xs = list(range(0, max(1, w - tile_w) + 1, stride_w))
        if ys[-1] != max(0, h - tile_h): ys.append(max(0, h - tile_h))
        if xs[-1] != max(0, w - tile_w): xs.append(max(0, w - tile_w))

        for ty1 in ys:
            ty2 = min(h, ty1 + tile_h)
            for tx1 in xs:
                tx2 = min(w, tx1 + tile_w)
                tile_img = zoomed_frame[ty1:ty2, tx1:tx2]
                if tile_img.size == 0:
                    continue

                results = model(tile_img, verbose=False, conf=0.25)
                for r in results:
                    for box in r.boxes:
                        class_name = model.names[int(box.cls[0])]
                        x1t, y1t, x2t, y2t = map(int, box.xyxy[0])
                        x1, y1, x2, y2 = x1t + tx1, y1t + ty1, x2t + tx1, y2t + ty1
                        cx, cy = (x1 + x2)//2, (y1 + y2)//2

                        # 🔧 줌 복원 (원본 프레임 기준 좌표)
                        if DETECT_ZOOM > 1.0:
                            x1d = int(crop_x1 + (x1 / DETECT_ZOOM))
                            y1d = int(crop_y1 + (y1 / DETECT_ZOOM))
                            x2d = int(crop_x1 + (x2 / DETECT_ZOOM))
                            y2d = int(crop_y1 + (y2 / DETECT_ZOOM))
                            cx_orig = crop_x1 + (cx / DETECT_ZOOM)
                            cy_orig = crop_y1 + (cy / DETECT_ZOOM)
                        else:
                            x1d, y1d, x2d, y2d = x1, y1, x2, y2
                            cx_orig, cy_orig = float(cx), float(cy)

                        # --- 원본 프레임에 박스 표시 ---
                        color = (0, 255, 0) if class_name == 'person' else (0, 0, 255)
                        label = f"{class_name}: {float(box.conf[0]):.2f}"
                        cv2.rectangle(annotated_frame, (x1d, y1d), (x2d, y2d), color, 2)
                        cv2.putText(annotated_frame, label, (x1d, y1d - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                        # --- 지도 좌표 변환 ---
                        pts = np.array([[[cx_orig, cy_orig]]], dtype=np.float32)
                        transformed = cv2.perspectiveTransform(pts, homography_matrix)
                        mx, my = int(transformed[0][0][0]), int(transformed[0][0][1])

                        rx, ry = mx - crop_x, my - crop_y
                        if 0 <= rx < base_map.shape[1] and 0 <= ry < base_map.shape[0]:
                            dx, dy = int(rx * MAP_SCALE), int(ry * MAP_SCALE)
                            cv2.circle(map_display, (dx, dy), 10, color, -1)
                            cv2.putText(map_display, class_name, (dx + 10, dy),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                        if class_name == 'person':
                            person_coords = (mx, my)
                        elif class_name == 'fire':
                            fire_coords = (mx, my)

        # 🔥 Flame timeout 처리
        if flame_detected_by_rpi and time.time() - last_flame_time > FLAME_TIMEOUT:
            flame_detected_by_rpi = False

        # ✅ 조건: (사람 + 불 + RPi 온도 감지)
        if person_coords and fire_coords and flame_detected_by_rpi:
            if buzzer_state == 0:
                print(f"🔥 Fire detected — Person+Flame+Temp({current_temp:.1f}°C)")
                buzzer_state = 1
            last_trigger_time = time.time()

            # 좌표 전송 (10분 간격)
            if turtle_sock and (time.time() - last_coord_sent_time > 600):
                px, py = person_coords
                fx, fy = fire_coords
                
                px_adj = max(px - 20, 0)
                py_adj = max(py - 20, 0)
                
                msg = f"{px_adj},{py_adj},{fx},{fy}\n"
                turtle_sock.sendall(msg.encode())
                print(f"📤 Sent coords to TurtleBot: {msg.strip()}")
                last_coord_sent_time = time.time()
        else:
            if buzzer_state == 1 and time.time() - last_trigger_time >= 10:
                print("✅ Fire condition cleared (no buzzer control)")
                buzzer_state = 0

        # --- 온도 표시 오버레이 ---
        cv2.putText(annotated_frame, f"TEMP: {current_temp:.1f}C", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        cv2.imshow("YOLO Detection", annotated_frame)
        cv2.imshow("Map", map_display)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()


# ============================================================
# 🔹 메인
# ============================================================
def main():
    global flame_detected_by_rpi

    # --- Connect to RPi (for temperature feed) ---
    try:
        rpi_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        rpi_sock.connect((RPI_HOST, RPI_PORT))
        print(f"✅ Connected to RPi ({RPI_HOST}:{RPI_PORT})")
    except Exception as e:
        print(f"❌ Failed to connect to RPi: {e}")
        return

    threading.Thread(target=recv_from_rpi, args=(rpi_sock,), daemon=True).start()

    # --- Connect to TurtleBot ---
    turtle_sock = None
    try:
        turtle_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        turtle_sock.connect((SERVER_HOST, SERVER_PORT))
        print(f"✅ Connected to TurtleBot ({SERVER_HOST}:{SERVER_PORT})")
        threading.Thread(target=recv_loop, args=(turtle_sock,), daemon=True).start()
    except Exception as e:
        print(f"⚠️ Could not connect to TurtleBot: {e}")
        turtle_sock = None

    display_stream(turtle_sock, rpi_sock)


def recv_loop(sock: socket.socket):
    """TurtleBot → 메시지 수신"""
    try:
        while True:
            data = sock.recv(1024)
            if not data:
                print("🔌 TurtleBot connection lost.")
                break
            print("📥 From TurtleBot:", data.decode().strip())
    except Exception as e:
        print(f"❌ Error recv_loop: {e}")


if __name__ == "__main__":
    main()
