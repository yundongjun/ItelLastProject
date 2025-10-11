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
RPI_HOST = "10.10.16.78"       # Raspberry Pi (Buzzer)
RPI_PORT = 5000

# --- YOLO Model Loading ---
print("Loading custom model...")
device = 'cuda' if torch.cuda.is_available() else 'cpu'
model = YOLO('runs/detect/person_and_flame_detector/weights/best.pt')
model.to(device)
print(f"Custom model loaded successfully on {device}.")

# --- MJPEG Stream URL ---
MJPEG_URL = "http://10.10.16.78:8080/?action=stream"


def recv_loop(sock: socket.socket):
    """Receives and prints messages from the TurtleBot server."""
    try:
        while True:
            data = sock.recv(1024)
            if not data:
                print("🔌 TurtleBot connection lost.")
                break
            print("📥 From TurtleBot:", data.decode().strip())
    except Exception as e:
        print(f"❌ Error during receive: {e}")


def display_stream(turtle_sock: socket.socket, buzzer_sock: socket.socket):
    """
    YOLO 감지 루프:
      - 사람+불 동시에 감지 → 좌표는 TurtleBot에 한 번 전송
      - RPi 부저는 무조건 10초 이상 ON, 이후 조건 해제되면 OFF
    """
    # Load homography matrix and map
    homography_path = 'homography_matrix.npy'
    map_path = 'map.pgm'

    if not os.path.exists(homography_path):
        print(f"Error: {homography_path} not found. Please run perspective_mapper.py first.")
        return
    if not os.path.exists(map_path):
        print(f"Error: {map_path} not found.")
        return

    homography_matrix = np.load(homography_path)
    map_img = cv2.imread(map_path)
    print("Homography matrix and map loaded successfully.")

    # --- State Flags ---
    buzzer_state = 0         # 0=OFF, 1=ON
    last_trigger_time = 0    # 마지막으로 부저 켠 시간
    alert_sent = False       # 터틀봇에 좌표 보냈는지 여부
    last_coord_sent_time = 0 # 마지막으로 좌표 보낸 시간

    print(f"Connecting to source stream: {MJPEG_URL}")
    try:
        session = requests.Session()
        stream = session.get(MJPEG_URL, stream=True, timeout=10)
        stream.raise_for_status()
        print("Connection to source stream successful.")

        bytes_buffer = bytes()

        while True:
            bytes_buffer += stream.raw.read(4096)
            start = bytes_buffer.find(b'\xff\xd8')
            end = bytes_buffer.find(b'\xff\xd9')

            if start != -1 and end != -1:
                jpg = bytes_buffer[start:end+2]
                bytes_buffer = bytes_buffer[end+2:]

                if not jpg:
                    continue

                frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    continue

                annotated_frame = frame.copy()
                map_display = map_img.copy()

                # --- 초기화 (항상 정의되게) ---
                person_coords = None
                fire_coords = None

                results = model(frame, verbose=False, conf=0.1)

                for r in results:
                    for box in r.boxes:
                        class_name = model.names[int(box.cls[0])]
                        x1, y1, x2, y2 = map(int, box.xyxy[0])

                        center_x = (x1 + x2) // 2
                        center_y = (y1 + y2) // 2
                        pts = np.array([[[center_x, center_y]]], dtype=np.float32)
                        transformed_pts = cv2.perspectiveTransform(pts, homography_matrix)
                        map_x = int(transformed_pts[0][0][0])
                        map_y = int(transformed_pts[0][0][1])

                        if class_name == 'person':
                            person_coords = (map_x, map_y)
                            label = f"Person: {float(box.conf[0]):.2f}"
                            color = (0, 255, 0)  # Green
                        elif class_name == 'fire':
                            fire_coords = (map_x, map_y)
                            label = f"Fire: {float(box.conf[0]):.2f}"
                            color = (0, 0, 255)  # Red
                        else:
                            continue

                        # Draw on local displays
                        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
                        cv2.putText(annotated_frame, label, (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                        if 0 <= map_x < map_display.shape[1] and 0 <= map_y < map_display.shape[0]:
                            cv2.circle(map_display, (map_x, map_y), 10, color, -1)
                            cv2.putText(map_display, class_name, (map_x + 15, map_y + 5),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                # --- Logic: event-based signals ---
                if person_coords and fire_coords:
                    # 부저 ON (최소 10초 울리기)
                    if buzzer_state == 0:
                        buzzer_sock.sendall(b"1\n")
                        print("📤 Sent buzzer ON (triggered)")
                        buzzer_state = 1
                        last_trigger_time = time.time()

                    # TurtleBot 좌표는 10분에 한 번만 전송
                    if turtle_sock and (time.time() - last_coord_sent_time > 600):
                        msg = f"{person_coords[0]},{person_coords[1]},{fire_coords[0]},{fire_coords[1]}\n"
                        turtle_sock.sendall(msg.encode())
                        print(f"📤 Sent coords to TurtleBot: {msg.strip()}")
                        last_coord_sent_time = time.time()  # Update the time

                else:
                    # 부저가 켜져 있다면 최소 10초 유지
                    if buzzer_state == 1:
                        elapsed = time.time() - last_trigger_time
                        if elapsed >= 10:   # 10초 지나면 끔
                            buzzer_sock.sendall(b"0\n")
                            print("📤 Sent buzzer OFF (after 10s)")
                            buzzer_state = 0

                cv2.imshow('Detection', annotated_frame)
                cv2.imshow('Map', map_display)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("'q' pressed, exiting loop.")
                    break

    except requests.exceptions.RequestException as e:
        print(f"Error connecting to the source stream: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        print("Closing stream and windows.")
        cv2.destroyAllWindows()


def main():
    """Connect to Raspberry Pi (buzzer, 필수) and TurtleBot (선택), then start the YOLO stream loop."""

    # --- 라즈베리파이 연결 ---
    try:
        buzzer_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print(f"Connecting to RPi buzzer at {RPI_HOST}:{RPI_PORT}...")
        buzzer_sock.connect((RPI_HOST, RPI_PORT))
        print("✅ Connected to Raspberry Pi buzzer server")
    except Exception as e:
        print(f"❌ Failed to connect to Raspberry Pi: {e}")
        return  # 부저 연결은 필수니까 실패 시 종료

    # --- 터틀봇 연결 (있으면 사용, 없으면 패스) ---
    turtle_sock = None
    try:
        turtle_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print(f"Attempting to connect to TurtleBot at {SERVER_HOST}:{SERVER_PORT}...")
        turtle_sock.connect((SERVER_HOST, SERVER_PORT))
        print("✅ Connected to TurtleBot server")
        threading.Thread(target=recv_loop, args=(turtle_sock,), daemon=True).start()
    except Exception as e:
        print(f"⚠️ Could not connect to TurtleBot: {e}")
        turtle_sock = None

    # --- 스트림 시작 ---
    try:
        display_stream(turtle_sock, buzzer_sock)
    finally:
        if turtle_sock:
            turtle_sock.close()
        if buzzer_sock:
            buzzer_sock.close()
        print("⏹️ Closing client.")


if __name__ == '__main__':
    main()
