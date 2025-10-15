import cv2
import requests
import numpy as np
from ultralytics import YOLO
import torch
import os
import socket
import threading
import time

# --- Socket Client Configuration ---
SERVER_HOST = "10.10.16.41"   # TurtleBot (ROS)
SERVER_PORT = 5000
RPI_HOST = "10.10.16.78"      # Raspberry Pi (Buzzer)
RPI_PORT = 5000

FLAME_TIMEOUT = 5.0  # 5초 동안 RPi에서 신호 안 오면 감지 해제
THERMAL_THRESHOLD = 50.0  # 🔥 열 감지 임계값 (°C)

# --- YOLO Model Loading ---
print("Loading custom model...")
device = 'cuda' if torch.cuda.is_available() else 'cpu'
model = YOLO('best.pt')
model.to(device)
print(f"✅ Model loaded successfully on {device}.")

# --- MJPEG Stream URL ---
MJPEG_URL = "http://10.10.16.78:8080/?action=stream"

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
            # TEMP: 메시지 처리
            if msg.startswith("TEMP:"):
                try:
                    temp_val = float(msg.split(":")[1])
                    current_temp = temp_val
                    print(f"🌡 Current RPi Temp: {temp_val:.2f}°C")

                    # 온도 기준으로 화염 판단
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
# 🔹 YOLO + RPi 열 감지 + 버저 제어
# ============================================================
def display_stream(turtle_sock: socket.socket, buzzer_sock: socket.socket):
    global flame_detected_by_rpi, last_flame_time, current_temp

    homography_path = 'homography_matrix.npy'
    map_path = 'map.pgm'

    if not os.path.exists(homography_path) or not os.path.exists(map_path):
        print("❌ Map or homography missing.")
        return

    homography_matrix = np.load(homography_path)
    map_img = cv2.imread(map_path)
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

        annotated_frame = frame.copy()
        map_display = map_img.copy()

        person_coords = None
        fire_coords = None

        results = model(frame, verbose=False, conf=0.3)
        for r in results:
            for box in r.boxes:
                class_name = model.names[int(box.cls[0])]
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy = (x1+x2)//2, (y1+y2)//2
                pts = np.array([[[cx, cy]]], dtype=np.float32)
                transformed = cv2.perspectiveTransform(pts, homography_matrix)
                mx, my = int(transformed[0][0][0]), int(transformed[0][0][1])

                color = (0, 255, 0) if class_name == 'person' else (0, 0, 255)
                label = f"{class_name}: {float(box.conf[0]):.2f}"
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(annotated_frame, label, (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                if 0 <= mx < map_display.shape[1] and 0 <= my < map_display.shape[0]:
                    cv2.circle(map_display, (mx, my), 10, color, -1)
                    cv2.putText(map_display, class_name, (mx+10, my),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                if class_name == 'person':
                    person_coords = (mx, my)
                elif class_name == 'fire':
                    fire_coords = (mx, my)

        # 🔥 Check RPi flame timeout
        if flame_detected_by_rpi and time.time() - last_flame_time > FLAME_TIMEOUT:
            flame_detected_by_rpi = False

        # ✅ AND 조건: (사람 + 불 + RPi 온도 감지)
        if person_coords and fire_coords and flame_detected_by_rpi:
            if buzzer_state == 0:
                buzzer_sock.sendall(b"1\n")
                print(f"📤 Sent buzzer ON — Flame+Person+Temp({current_temp:.1f}°C)")
                buzzer_state = 1
            last_trigger_time = time.time()

            # 좌표 전송 (10분 간격)
            if turtle_sock and (time.time() - last_coord_sent_time > 600):
                msg = f"{person_coords[0]},{person_coords[1]},{fire_coords[0]},{fire_coords[1]}\n"
                turtle_sock.sendall(msg.encode())
                print(f"📤 Sent coords to TurtleBot: {msg.strip()}")
                last_coord_sent_time = time.time()

        else:
            if buzzer_state == 1 and time.time() - last_trigger_time >= 10:
                buzzer_sock.sendall(b"0\n")
                print("📤 Sent buzzer OFF (conditions cleared)")
                buzzer_state = 0

        cv2.imshow("YOLO Detection", annotated_frame)
        cv2.imshow("Map", map_display)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()


# ============================================================
# 🔹 메인 (소켓 연결 초기화)
# ============================================================
def main():
    global flame_detected_by_rpi

    # --- Connect to RPi ---
    try:
        buzzer_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        buzzer_sock.connect((RPI_HOST, RPI_PORT))
        print(f"✅ Connected to RPi ({RPI_HOST}:{RPI_PORT})")
    except Exception as e:
        print(f"❌ Failed to connect to RPi: {e}")
        return

    threading.Thread(target=recv_from_rpi, args=(buzzer_sock,), daemon=True).start()

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

    display_stream(turtle_sock, buzzer_sock)


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
