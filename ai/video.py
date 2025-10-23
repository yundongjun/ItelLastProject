#!/usr/bin/env python3
import cv2
import time
from datetime import datetime

def draw_timestamp(frame):
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    cv2.putText(frame, ts, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (255, 255, 255), 2, cv2.LINE_AA)
    return frame

def main():
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return

    width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps    = cap.get(cv2.CAP_PROP_FPS) or 30

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    filename = f"record_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"
    writer = None
    recording = False
    start_time = None

    print("미리보기 시작 (S: 녹화 시작, Q: 종료)")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = draw_timestamp(frame)
        key = cv2.waitKey(1) & 0xFF

        if recording:
            writer.write(frame)
            cv2.putText(frame, "REC", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

        cv2.imshow("Press 'S' to Start Recording, 'Q' to Quit", frame)

        if key == ord('s') and not recording:
            # 녹화 시작
            filename = f"record_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"
            writer = cv2.VideoWriter(filename, fourcc, fps, (width, height))
            recording = True
            start_time = time.time()
            print(f"[INFO] 녹화 시작: {filename}")

        elif key == ord('q'):
            print("[INFO] 프로그램 종료")
            break

    cap.release()
    if writer:
        writer.release()
    cv2.destroyAllWindows()
    if recording:
        elapsed = time.time() - start_time
        print(f"[INFO] 녹화 종료 ({elapsed:.1f}초) → {filename}")

if __name__ == "__main__":
    main()
