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
SERVER_HOST = "10.10.16.166"
SERVER_PORT = 5000
RPI_HOST = "10.10.16.78"
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
    """Receives and prints messages from the server."""
    try:
        while True:
            data = sock.recv(1024)
            if not data:
                print("🔌 Server connection lost.")
                break
            print("📥 From Server:", data.decode().strip())
    except Exception as e:
        print(f"❌ Error during receive: {e}")

def display_stream(sock: socket.socket):
    """
    Fetches frames, performs YOLO processing, and sends coordinates only when both a person and a fire are detected simultaneously.
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

    # --- Cooldown Configuration ---
    last_sent_time = 0
    COOLDOWN_SECONDS = 120  # 2-minute cooldown

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

                # --- Aggregate detections in the frame ---
                person_coords = None
                fire_coords = None

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
                            color = (0, 255, 0) # Green
                        elif class_name == 'fire':
                            fire_coords = (map_x, map_y)
                            label = f"Fire: {float(box.conf[0]):.2f}"
                            color = (0, 0, 255) # Red
                        else:
                            continue

                        # Draw on local displays
                        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
                        cv2.putText(annotated_frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                        if 0 <= map_x < map_display.shape[1] and 0 <= map_y < map_display.shape[0]:
                            cv2.circle(map_display, (map_x, map_y), 10, color, -1)
                            cv2.putText(map_display, class_name, (map_x + 15, map_y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                # --- Build and send message only if both are detected ---
                if person_coords and fire_coords:
                    current_time = time.time()
                    if current_time - last_sent_time > COOLDOWN_SECONDS:
                        try:
                            msg = f"{person_coords[0]},{person_coords[1]} {fire_coords[0]},{fire_coords[1]}\n"
                            sock.sendall(msg.encode())
                            print(f"📤 Sent message: {msg.strip()}")
                            last_sent_time = current_time
                        except Exception as e:
                            print(f"❌ Message sending error: {e}")

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
    """Creates a socket, connects to the server, and starts the stream processing."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        print(f"Attempting to connect to server at {SERVER_HOST}:{SERVER_PORT}...")
        sock.connect((SERVER_HOST, SERVER_PORT))
        print(f"✅ Server connection successful ({SERVER_HOST}:{SERVER_PORT})")

        threading.Thread(target=recv_loop, args=(sock,), daemon=True).start()
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