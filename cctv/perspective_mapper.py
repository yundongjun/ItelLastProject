
import cv2
import numpy as np
import requests
import argparse
import os

# Global variables
map_points = []            # 맵 좌표(원본 스케일)
stream_points = []         # 스트림 좌표(원본 스케일)
map_img_original = None    # 맵 원본
map_img_display = None     # 맵 표시용(스케일 반영)
warped_frame = None
map_scale = 1.0
stream_scale = 1.0
last_stream_frame = None

def select_points_map(event, x, y, flags, param):
    """Map 창 클릭 → 원본 좌표로 변환해 저장"""
    global map_points, map_img_display, map_scale
    if event == cv2.EVENT_LBUTTONDOWN and len(map_points) < 4:
        # 표시 좌표 → 원본 좌표로 역변환
        ox = int(round(x / max(map_scale, 1e-6)))
        oy = int(round(y / max(map_scale, 1e-6)))
        map_points.append((ox, oy))
        # 표시 이미지에 마커 표시(표시좌표 기준)
        cv2.circle(map_img_display, (x, y), 5, (0, 0, 255), -1)
        cv2.imshow("Map", map_img_display)

def select_points_stream(event, x, y, flags, param):
    """Stream 창 클릭 → 원본 좌표로 변환해 저장"""
    global stream_points, stream_scale
    if event == cv2.EVENT_LBUTTONDOWN and len(stream_points) < 4:
        ox = int(round(x / max(stream_scale, 1e-6)))
        oy = int(round(y / max(stream_scale, 1e-6)))
        stream_points.append((ox, oy))

def parse_args():
    p = argparse.ArgumentParser(description="맵-스트림 대응점 선택으로 호모그래피 계산")
    p.add_argument("--map", dest="map_path", type=str, default="map.pgm", help="맵 이미지 경로 (pgm/png/jpg 등)")
    p.add_argument("--url", dest="mjpeg_url", type=str, default="http://10.10.16.78:8080/?action=stream", help="MJPEG 스트림 URL")
    p.add_argument("--out", dest="out_npy", type=str, default="homography_matrix.npy", help="저장할 호모그래피 .npy 파일 경로")
    p.add_argument("--map-scale", dest="map_scale", type=float, default=2.0, help="Map 창 표시 스케일 배율 (기본 2.0배)")
    p.add_argument("--stream-scale", dest="stream_scale", type=float, default=1.0, help="Stream 창 표시 스케일 배율")
    return p.parse_args()


def main():
    global map_points, stream_points, map_img_original, map_img_display, warped_frame, map_scale, stream_scale, last_stream_frame

    args = parse_args()
    map_scale = max(0.1, float(args.map_scale))
    stream_scale = max(0.1, float(args.stream_scale))

    # Load the map
    map_img_original = cv2.imread(args.map_path)
    if map_img_original is None:
        print(f"Error: {args.map_path} not found or could not be read.")
        return
    # 표시용 이미지 생성
    if abs(map_scale - 1.0) < 1e-6:
        map_img_display = map_img_original.copy()
    else:
        h, w = map_img_original.shape[:2]
        map_img_display = cv2.resize(map_img_original, (int(w * map_scale), int(h * map_scale)), interpolation=cv2.INTER_NEAREST)

    # Setup windows and mouse callbacks
    cv2.namedWindow("Map")
    cv2.setMouseCallback("Map", select_points_map)
    cv2.namedWindow("Stream")
    
    print("Instructions:")
    print("1. 'Map' 창(확대 표시)에서 기준점 4개를 순서대로 클릭")
    print("2. 'Stream' 창(선택적 확대)에서 동일한 대응점 4개를 같은 순서로 클릭")
    print("3. 'w' 키: 호모그래피 계산 및 저장")
    print("4. 'r' 키: 포인트 리셋")
    print("5. 'q' 키: 종료")
    print(f"[INFO] Map: {os.path.abspath(args.map_path)}")
    print(f"[INFO] Stream URL: {args.mjpeg_url}")
    print(f"[INFO] Output NPY: {os.path.abspath(args.out_npy)}")
    print(f"[INFO] Map Scale: x{map_scale:.2f}  | Stream Scale: x{stream_scale:.2f}")

    # MJPEG Stream URL
    MJPEG_URL = args.mjpeg_url
    try:
        session = requests.Session()
        stream = session.get(MJPEG_URL, stream=True, timeout=10)
        stream.raise_for_status()
        bytes_buffer = bytes()

        homography_matrix = None

        while True:
            # Display the map image
            cv2.imshow("Map", map_img_display)

            # Read frame from stream
            chunk = stream.raw.read(4096)
            if not chunk:
                break
            bytes_buffer += chunk
            start = bytes_buffer.find(b'\xff\xd8')
            end = bytes_buffer.find(b'\xff\xd9')

            if start != -1 and end != -1:
                jpg = bytes_buffer[start:end+2]
                bytes_buffer = bytes_buffer[end+2:]
                frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    continue

                # 원본/표시 프레임 준비
                last_stream_frame = frame
                disp = frame
                if abs(stream_scale - 1.0) > 1e-6:
                    h, w = frame.shape[:2]
                    disp = cv2.resize(frame, (int(w * stream_scale), int(h * stream_scale)), interpolation=cv2.INTER_NEAREST)

                # 마우스 콜백(표시 좌표 → 원본 변환)
                cv2.setMouseCallback("Stream", select_points_stream)

                # 선택 포인트 표시(표시좌표로 변환하여 그림)
                disp_copy = disp.copy()
                for px, py in stream_points:
                    dx = int(round(px * stream_scale))
                    dy = int(round(py * stream_scale))
                    cv2.circle(disp_copy, (dx, dy), 5, (0, 255, 0), -1)

                cv2.imshow("Stream", disp_copy)

                if warped_frame is not None:
                    cv2.imshow("Warped Stream", warped_frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('w') and len(map_points) == 4 and len(stream_points) == 4:
                print("Calculating homography matrix...")
                pts_src = np.array(stream_points, dtype=float)
                pts_dst = np.array(map_points, dtype=float)
                homography_matrix, status = cv2.findHomography(pts_src, pts_dst)
                print("Homography matrix calculated.")
                np.save(args.out_npy, homography_matrix)
                print(f"Homography matrix saved to {args.out_npy}")
            
            elif key == ord('r'):
                print("Resetting points.")
                map_points = []
                stream_points = []
                if abs(map_scale - 1.0) < 1e-6:
                    map_img_display = map_img_original.copy()
                else:
                    h, w = map_img_original.shape[:2]
                    map_img_display = cv2.resize(map_img_original, (int(w * map_scale), int(h * map_scale)), interpolation=cv2.INTER_NEAREST)
                warped_frame = None
                homography_matrix = None


            if homography_matrix is not None and last_stream_frame is not None:
                # Warp the current frame to map 원본 크기
                warped = cv2.warpPerspective(last_stream_frame, homography_matrix, (map_img_original.shape[1], map_img_original.shape[0]))
                # 표시용으로 스케일 적용
                if abs(map_scale - 1.0) > 1e-6:
                    h, w = warped.shape[:2]
                    warped = cv2.resize(warped, (int(w * map_scale), int(h * map_scale)), interpolation=cv2.INTER_NEAREST)
                warped_frame = warped


    except requests.exceptions.RequestException as e:
        print(f"Error connecting to the source stream: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
