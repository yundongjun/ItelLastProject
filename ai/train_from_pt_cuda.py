# infer_cam_yolo_fixed.py
# YOLOv8 - webcam inference with fixed best.pt path (CUDA auto)
# 실행: python infer_cam_yolo_fixed.py

import cv2
import torch
import time
from ultralytics import YOLO

# ✅ 하드코딩된 가중치 경로
WEIGHTS_PATH = r"C:\Users\STC\Desktop\test\runs\detect\drone_detector_yolov8s_dataset35\weights\best.pt"

def main():
    # 디바이스 자동 감지
    device = "0" if torch.cuda.is_available() else "cpu"
    print(f"[INFO] Using device: {device} ({'GPU' if device != 'cpu' else 'CPU'})")

    # 모델 로드
    model = YOLO(WEIGHTS_PATH)

    # 카메라 열기 (0번 웹캠)
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    if not cap.isOpened():
        raise RuntimeError("❌ 카메라를 열 수 없습니다.")

    print("[INFO] Camera opened. Press ESC or Q to quit.")
    prev_t = time.time()
    ema_fps = None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] 프레임을 읽지 못했습니다. 종료합니다.")
            break

        # YOLO 추론
        results = model(frame, imgsz=640, conf=0.5, device=device, verbose=False)
        r = results[0]

        # 시각화된 이미지 가져오기
        plotted = r.plot()

        # FPS 계산
        now = time.time()
        fps = 1.0 / (now - prev_t)
        prev_t = now
        ema_fps = fps if ema_fps is None else 0.9 * ema_fps + 0.1 * fps
        cv2.putText(plotted, f"FPS: {ema_fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2, cv2.LINE_AA)

        # 결과 표시
        cv2.imshow("YOLOv8 Detection (GPU)" if device != "cpu" else "YOLOv8 Detection (CPU)", plotted)
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("[INFO] 종료되었습니다.")

if __name__ == "__main__":
    main()
