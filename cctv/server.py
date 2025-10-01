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
SERVER_HOST = "10.10.16.166"   # 서버 IP (외부라면 IP 바꾸면 됨)
SERVER_PORT = 5000          # 서버 포트 (ROS2 노드 launch 파라미터와 맞추기)

# --- YOLO Model Loading ---
print("Loading custom model...")
device = 'cuda' if torch.cuda.is_available() else 'cpu'
model = YOLO('runs/detect/person_and_flame_detector/weights/best.pt')
model.to(device)
print(f"Custom model loaded successfully on {device}.")

# --- MJPEG Stream URL ---
MJPEG_URL = "http://10.10.16.78:8080/?action=stream"

def recv_loop(sock: socket.socket):
    """서버에서 오는 메시지 수신 및 출력"""
    try:
        while True:
            data = sock.recv(1024)
            if not data:
                print("🔌 서버 연결이 끊어졌습니다.")
                break
            print("📥 From Server:", data.decode().strip())
    except Exception as e:
        print(f"❌ 수신 중 에러 발생: {e}")
        
def display_stream(sock: socket.socket):
    """
    스트림에서 프레임을 가져와 YOLO 처리를 하고, 감지된 객체의 좌표를 서버로 전송합니다.
    """
    # 호모그래피 행렬 및 지도 로드
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

                if not jpg: continue

                frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None: continue

                annotated_frame = frame.copy()
                map_display = map_img.copy()

                results = model(frame, verbose=False, conf=0.1)

                for r in results:
                    for box in r.boxes:
                        class_name = model.names[int(box.cls[0])]

                        if class_name in ['person', 'fire']:
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            conf = float(box.conf[0])

                            if class_name == 'fire':
                                label = f"Fire: {conf:.2f}"
                                color = (0, 0, 255) # Red
                            else: # class_name == 'person'
                                label = f"Person: {conf:.2f}"
                                color = (0, 255, 0) # Green

                            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
                            cv2.putText(annotated_frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

                            center_x = (x1 + x2) // 2
                            center_y = (y1 + y2) // 2

                            pts = np.array([[[center_x, center_y]]], dtype=np.float32)
                            transformed_pts = cv2.perspectiveTransform(pts, homography_matrix)
                            map_x = int(transformed_pts[0][0][0])
                            map_y = int(transformed_pts[0][0][1])
                            
                            # --- 좌표 전송 ---
                            try:
                                msg = f"{map_x},{map_y}\n"
                                sock.sendall(msg.encode())
                                print(f"📤 Sent coordinates: ({map_x}, {map_y}) for {class_name}")
                            except Exception as e:
                                print(f"❌ 좌표 전송 에러: {e}")
                                # 여기서 연결을 끊고 재시도 로직을 추가할 수도 있습니다.
                                
                            if 0 <= map_x < map_display.shape[1] and 0 <= map_y < map_display.shape[0]:
                                cv2.circle(map_display, (map_x, map_y), 10, color, -1)
                                cv2.putText(map_display, class_name, (map_x + 15, map_y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

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
    """소켓을 생성하고 서버에 연결한 후, 스트림 처리를 시작합니다.""" 
    # TCP 소켓 생성
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        print(f"Attempting to connect to server at {SERVER_HOST}:{SERVER_PORT}...")
        sock.connect((SERVER_HOST, SERVER_PORT))
        print(f"✅ 서버({SERVER_HOST}:{SERVER_PORT}) 연결 성공")

        # 서버 수신용 스레드 시작
        threading.Thread(target=recv_loop, args=(sock,), daemon=True).start()

        # 비디오 스트림 처리 및 좌표 전송 시작
        display_stream(sock)

    except ConnectionRefusedError:
        print(f"❌ Connection to server ({SERVER_HOST}:{SERVER_PORT}) failed. Is the server running?")
    except Exception as e:
        print(f"❌ An error occurred during socket connection: {e}")
    finally:
        print("⏹️ Closing client.")
        sock.close()

if __name__ == '__main__':
    main()
