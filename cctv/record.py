import argparse
import os
import sys
import time
from datetime import datetime

import cv2


def parse_args():
    parser = argparse.ArgumentParser(description="웹캠/스트림 영상을 지정 시간 동안 녹화")
    parser.add_argument(
        "--source",
        type=str,
        default=os.environ.get("VIDEO_SOURCE", "http://10.10.16.78:8080/?action=stream"),
        help="입력 소스 (장치 인덱스 또는 URL). 기본: 환경변수 VIDEO_SOURCE 또는 MJPEG URL",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="녹화 시간(초). 기본: 10초",
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="출력 파일 경로(.mp4 권장). 미지정 시 timestamp 기반 자동 생성",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=None,
        help="출력 FPS. 미지정 시 입력에서 추정, 실패 시 20fps",
    )
    parser.add_argument(
        "--fourcc",
        type=str,
        default="mp4v",
        help="비디오 코덱 FourCC. 기본: mp4v (mp4 파일 권장)",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="녹화 중 미리보기 창 표시",
    )
    return parser.parse_args()


def resolve_source(src: str):
    # 숫자 문자열이면 장치 인덱스로 취급
    if src.isdigit():
        return int(src)
    return src


def build_output_path(path: str | None) -> str:
    if path:
        return path
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"record_{ts}.mp4"


def ensure_extension(path: str, default_ext: str = ".mp4") -> str:
    base, ext = os.path.splitext(path)
    if ext:
        return path
    return base + default_ext


def ensure_unique_path(path: str) -> str:
    """이미 존재하면 _1, _2 ... 접미사를 붙여 고유 경로 반환"""
    base, ext = os.path.splitext(path)
    candidate = path
    i = 1
    while os.path.exists(candidate):
        candidate = f"{base}_{i}{ext}"
        i += 1
    return candidate


def main():
    args = parse_args()
    source = resolve_source(args.source)
    out_path = build_output_path(args.output)
    out_path = ensure_extension(out_path, ".mp4")
    unique_out_path = ensure_unique_path(out_path)

    print(f"[INFO] Source: {args.source}")
    if unique_out_path != out_path:
        print(f"[INFO] Output exists. Renamed: {out_path} -> {unique_out_path}")
    print(f"[INFO] Duration: {args.duration:.1f}s  Output: {unique_out_path}")

    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        print("[ERROR] 입력 소스를 열 수 없습니다. URL/장치 인덱스를 확인하세요.")
        sys.exit(1)

    # FPS 추정
    fps = args.fps if args.fps and args.fps > 0 else cap.get(cv2.CAP_PROP_FPS)
    if not fps or fps <= 0:
        fps = 20.0  # MJPEG 스트림 등에서 FPS 정보를 못 줄 수 있음

    # 해상도 결정: 장치가 0 반환 시 첫 프레임 기준으로 설정
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # 첫 프레임 먼저 읽어 사이즈 보정
    ok, frame = cap.read()
    if not ok or frame is None:
        print("[ERROR] 첫 프레임을 읽을 수 없습니다. 소스를 확인하세요.")
        cap.release()
        sys.exit(2)

    if width <= 0 or height <= 0:
        height, width = frame.shape[:2]

    fourcc = cv2.VideoWriter_fourcc(*args.fourcc)
    # mp4 파일 확장자 권장. avi의 경우 'XVID' 등 사용 가능
    writer = cv2.VideoWriter(unique_out_path, fourcc, fps, (width, height))
    if not writer.isOpened():
        print("[ERROR] VideoWriter 초기화 실패. 코덱/경로 권한을 확인하세요.")
        cap.release()
        sys.exit(3)

    start = time.time()
    frames = 0

    # 이미 읽은 첫 프레임도 기록
    writer.write(frame)
    frames += 1
    if args.show:
        cv2.imshow("record", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("[INFO] 사용자 종료(q)")
            writer.release()
            cap.release()
            cv2.destroyAllWindows()
            sys.exit(0)

    # 루프
    while time.time() - start < args.duration:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("[WARN] 프레임을 더 이상 읽을 수 없습니다. 조기 종료합니다.")
            break
        writer.write(frame)
        frames += 1
        if args.show:
            cv2.imshow("record", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("[INFO] 사용자 종료(q)")
                break

    writer.release()
    cap.release()
    if args.show:
        cv2.destroyAllWindows()

    elapsed = time.time() - start
    print(f"[DONE] Saved: {unique_out_path} | Frames: {frames} | Elapsed: {elapsed:.2f}s | ~FPS: {frames/max(elapsed,1e-6):.2f}")


if __name__ == "__main__":
    main()
