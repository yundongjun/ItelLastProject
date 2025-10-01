import cv2
import requests
import numpy as np
from ultralytics import YOLO
import torch
import os

# YOLO 모델 로드
print("Loading custom model...")
device = 'cuda' if torch.cuda.is_available() else 'cpu'
model = YOLO('runs/detect/person_and_flame_detector/weights/best.pt')
model.to(device)
print(f"Custom model loaded successfully on {device}.")

# 원본 MJPEG 스트림 URL
MJPEG_URL = "http://10.10.16.78:8080/?action=stream"

def display_stream():
    """
    스트림에서 프레임을 가져와 YOLO 처리를 한 후, 불꽃을 감지하여 화면에 표시하고, 지도에 위치를 매핑합니다.
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
        lower_flame = np.array([10, 100, 200])
        upper_flame = np.array([40, 255, 255])

        while True:
            # 스트림에서 프레임 읽기
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

                # 객체 감지
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

                            # 원본 프레임에 바운딩 박스 그리기
                            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
                            cv2.putText(annotated_frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

                            # 중심 좌표 계산
                            center_x = (x1 + x2) // 2
                            center_y = (y1 + y2) // 2

                            # 중심 좌표를 지도 좌표로 변환
                            pts = np.array([[[center_x, center_y]]], dtype=np.float32)
                            transformed_pts = cv2.perspectiveTransform(pts, homography_matrix)
                            map_x = int(transformed_pts[0][0][0])
                            map_y = int(transformed_pts[0][0][1])

                            # 지도에 위치 표시
                            if 0 <= map_x < map_display.shape[1] and 0 <= map_y < map_display.shape[0]:
                                cv2.circle(map_display, (map_x, map_y), 10, color, -1)
                                cv2.putText(map_display, class_name, (map_x + 15, map_y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                # 결과 보여주기
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

if __name__ == '__main__':
    display_stream()