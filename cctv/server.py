import cv2
import requests
import numpy as np
from ultralytics import YOLO
import torch

# YOLO 모델 로드
print("Loading YOLOv8 model...")
device = 'cuda' if torch.cuda.is_available() else 'cpu'
model = YOLO('yolov8n.pt')
model.to(device)
print(f"YOLOv8 model loaded successfully on {device}.")

# 원본 MJPEG 스트림 URL
MJPEG_URL = "http://10.10.16.78:8080/?action=stream"

def display_stream():
    """
    스트림에서 프레임을 가져와 YOLO 처리를 한 후, 불꽃을 감지하여 화면에 표시합니다.
    """
    print(f"Connecting to source stream: {MJPEG_URL}")
    try:
        session = requests.Session()
        stream = session.get(MJPEG_URL, stream=True, timeout=10)
        stream.raise_for_status()
        print("Connection to source stream successful.")
        
        bytes_buffer = bytes()
        lower_flame = np.array([10, 100, 200])
        upper_flame = np.array([40, 255, 255])

        for chunk in stream.iter_content(chunk_size=4096):
            bytes_buffer += chunk
            start = bytes_buffer.find(b'\xff\xd8')
            end = bytes_buffer.find(b'\xff\xd9')
            
            if start != -1 and end != -1:
                jpg = bytes_buffer[start:end+2]
                bytes_buffer = bytes_buffer[end+2:]

                if not jpg: continue

                frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None: continue

                annotated_frame = frame.copy()
                hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                # 1. 감지 임계값을 0.1로 낮춰 더 많은 결과를 확인합니다.
                results = model(frame, verbose=False, conf=0.1)

                for r in results:
                    for box in r.boxes:
                        class_name = model.names[int(box.cls[0])]

                        # 2. 'person'과 'candle'만 처리하도록 필터링합니다.
                        if class_name in ['person', 'candle']:
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            conf = float(box.conf[0])

                            if class_name == 'candle':
                                roi_h = int((y2 - y1) * 0.8)
                                roi_y1 = max(0, y1 - roi_h)
                                flame_roi = hsv_frame[roi_y1:y1, x1:x2]

                                if flame_roi.size == 0:
                                    is_lit = False
                                else:
                                    flame_mask = cv2.inRange(flame_roi, lower_flame, upper_flame)
                                    flame_pixel_count = cv2.countNonZero(flame_mask)
                                    is_lit = flame_pixel_count > 20

                                if is_lit:
                                    label = f"Lit Candle: {conf:.2f}"
                                    color = (0, 0, 255) # 빨간색
                                else:
                                    label = f"Candle: {conf:.2f}"
                                    color = (255, 0, 0) # 파란색
                            else: # class_name == 'person'
                                label = f"Person: {conf:.2f}"
                                color = (0, 255, 0) # 초록색

                            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
                            cv2.putText(annotated_frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

                cv2.imshow('Candle and Person Detection', annotated_frame)

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