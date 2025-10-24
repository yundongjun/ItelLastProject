import os
import time
import cv2
import numpy as np
import requests


MJPEG_URL = os.getenv("MJPEG_URL", "http://10.10.16.78:8080/?action=stream")
MAP_PATH = os.getenv("MAP_PATH", "map.pgm")
OUT_PATH = os.getenv("OUT_PATH", "homography_matrix.npy")


def _get_scale(name: str, default: float = 1.0) -> float:
    try:
        v = float(os.getenv(name, str(default)))
        # 제한: 1.0 ~ 10.0 배 사이
        if v < 1.0:
            v = 1.0
        if v > 10.0:
            v = 10.0
        return v
    except Exception:
        return default

CAM_VIEW_SCALE = _get_scale("CAM_VIEW_SCALE", 1.0)
MAP_VIEW_SCALE = _get_scale("MAP_VIEW_SCALE", 1.0)


def grab_one_frame(url: str, timeout: float = 10.0):
    session = requests.Session()
    r = session.get(url, stream=True, timeout=timeout)
    buf = bytes()
    for _ in range(500):  # read a few chunks until we get a full JPEG
        chunk = r.raw.read(4096)
        if not chunk:
            break
        buf += chunk
        s = buf.find(b"\xff\xd8")
        e = buf.find(b"\xff\xd9")
        if s != -1 and e != -1 and e > s:
            jpg = buf[s : e + 2]
            frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                return frame
            buf = buf[e + 2 :]
    raise RuntimeError("Failed to grab a frame from MJPEG stream.")


class PointCollector:
    def __init__(self, window_name: str, image, scale: float = 1.0):
        self.window = window_name
        self.base = image  # 원본 크기 보존
        self.scale = max(1.0, float(scale))
        self.display = self._scaled_copy()
        self.points = []  # 원본 좌표계로 저장
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window, self.on_mouse)

    def _scaled_copy(self):
        if self.scale == 1.0:
            return self.base.copy()
        h, w = self.base.shape[:2]
        sw, sh = int(round(w * self.scale)), int(round(h * self.scale))
        # 확대 시 블럭계단 최소화를 위해 INTER_LINEAR, 선명한 픽셀 유지 원하면 INTER_NEAREST
        return cv2.resize(self.base, (sw, sh), interpolation=cv2.INTER_LINEAR)

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # 클릭 좌표를 원본 좌표로 역변환
            ox = int(round(x / self.scale))
            oy = int(round(y / self.scale))
            h, w = self.base.shape[:2]
            ox = max(0, min(w - 1, ox))
            oy = max(0, min(h - 1, oy))
            self.points.append((ox, oy))

            # 그리기는 표시 스케일 좌표로
            sx = int(round(ox * self.scale))
            sy = int(round(oy * self.scale))
            cv2.circle(self.display, (sx, sy), 6, (0, 255, 255), -1)
            cv2.putText(
                self.display,
                str(len(self.points)),
                (sx + 6, sy - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

    def reset(self):
        self.points.clear()
        self.display = self._scaled_copy()

    def run(self, min_points=4):
        print(f"- {self.window}: 최소 {min_points}개 점을 순서대로 클릭하세요.")
        print("  키도움말: r=리셋, ENTER=확정, ESC=취소")
        while True:
            cv2.imshow(self.window, self.display)
            key = cv2.waitKey(10) & 0xFF
            if key == 27:  # ESC
                return None
            if key == ord("r"):
                self.reset()
            if key == 13 or key == 10:  # ENTER
                if len(self.points) >= min_points:
                    return self.points
                else:
                    print(f"  최소 {min_points}개 필요. 현재 {len(self.points)}개.")


def main():
    print("[1/4] 카메라 프레임 수집 중...")
    try:
        cam_frame = grab_one_frame(MJPEG_URL)
    except Exception as e:
        print(f"카메라 프레임 수집 실패: {e}")
        return

    print("[2/4] 지도 이미지 로드 중...")
    map_img = cv2.imread(MAP_PATH, cv2.IMREAD_UNCHANGED)
    if map_img is None:
        print(f"지도 이미지를 찾을 수 없습니다: {MAP_PATH}")
        return
    if len(map_img.shape) == 2:
        map_img = cv2.cvtColor(map_img, cv2.COLOR_GRAY2BGR)

    # 안내 텍스트
    cam_disp = cam_frame.copy()
    cv2.putText(
        cam_disp,
        "Camera: 기준점 4개 이상 클릭 -> ENTER",
        (20, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    map_disp = map_img.copy()
    cv2.putText(
        map_disp,
        "Map: 동일 순서로 4개 이상 클릭 -> ENTER",
        (20, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )

    cam_pc = PointCollector("Camera", cam_disp, scale=CAM_VIEW_SCALE)
    cam_pts = cam_pc.run(min_points=4)
    if cam_pts is None:
        print("취소됨")
        return

    map_pc = PointCollector("Map", map_disp, scale=MAP_VIEW_SCALE)
    map_pts = map_pc.run(min_points=len(cam_pts))
    if map_pts is None:
        print("취소됨")
        return

    cam_pts_np = np.array(cam_pts, dtype=np.float32)
    map_pts_np = np.array(map_pts, dtype=np.float32)

    print("[3/4] 호모그래피 추정 중(RANSAC)...")
    H, mask = cv2.findHomography(cam_pts_np, map_pts_np, cv2.RANSAC, 3.0)
    if H is None:
        print("호모그래피 계산 실패. 점들을 다시 선택하세요.")
        return

    # 빠른 검증: 카메라 점들을 H로 투영해서 지도에 찍어보기
    verify = map_img.copy()
    cam_pts_h = cv2.convertPointsToHomogeneous(cam_pts_np).reshape(-1, 3).T
    proj = H @ cam_pts_h
    proj /= proj[2:3, :]
    for i in range(proj.shape[1]):
        x, y = int(round(proj[0, i])), int(round(proj[1, i]))
        cv2.circle(verify, (x, y), 6, (0, 0, 255), -1)
        cv2.putText(verify, f"p{i+1}", (x + 7, y - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # 카메라 프레임을 지도 해상도로 워핑해서 반투명 합성
    warped = cv2.warpPerspective(cam_frame, H, (map_img.shape[1], map_img.shape[0]))
    overlay = cv2.addWeighted(map_img, 0.6, warped, 0.4, 0)

    cv2.imshow("Verify Projections (red dots)", verify)
    cv2.imshow("Warp Overlay", overlay)
    print("[검증] 창을 확인하고, s=저장, r=다시 선택, ESC=취소")
    while True:
        key = cv2.waitKey(0) & 0xFF
        if key == ord('s'):
            break
        elif key == ord('r'):
            cv2.destroyAllWindows()
            return main()
        elif key == 27:
            cv2.destroyAllWindows()
            print("취소됨")
            return

    # 백업 저장
    ts = time.strftime("%Y%m%d-%H%M%S")
    backup_path = f"homography_matrix.{ts}.npy"
    np.save(backup_path, H)
    np.save(OUT_PATH, H)
    cv2.destroyAllWindows()
    print(f"[4/4] 저장 완료: {OUT_PATH} (백업: {backup_path})")
    print("server.py가 다음 실행부터 새 호모그래피를 사용합니다.")


if __name__ == "__main__":
    main()
