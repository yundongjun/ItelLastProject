import argparse
import os
import sys
import time
from datetime import datetime

import cv2
from PIL import Image, ImageTk
import tkinter as tk
from tkinter import ttk, filedialog, messagebox


def parse_args():
    parser = argparse.ArgumentParser(description="미리보기 창에서 여러 번 시작/저장 가능한 영상 녹화 GUI")
    parser.add_argument(
        "--source",
        type=str,
        default=os.environ.get("VIDEO_SOURCE", "http://10.10.16.78:8080/?action=stream"),
        help="입력 소스 (장치 인덱스 또는 URL). 기본: 환경변수 VIDEO_SOURCE 또는 MJPEG URL",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=None,
        help="출력 FPS (미지정 시 입력 추정, 실패 시 20)",
    )
    parser.add_argument(
        "--fourcc",
        type=str,
        default="mp4v",
        help="비디오 코덱 FourCC. 기본: mp4v",
    )
    return parser.parse_args()


def resolve_source(src: str):
    if src.isdigit():
        return int(src)
    return src


def ensure_extension(path: str, default_ext: str = ".mp4") -> str:
    base, ext = os.path.splitext(path)
    return path if ext else base + default_ext


def ensure_unique_path(path: str) -> str:
    base, ext = os.path.splitext(path)
    candidate = path
    i = 1
    while os.path.exists(candidate):
        candidate = f"{base}_{i}{ext}"
        i += 1
    return candidate


class RecorderGUI:
    def __init__(self, args):
        self.args = args
        self.source = resolve_source(args.source)
        self.cap = cv2.VideoCapture(self.source)
        if not self.cap.isOpened():
            print("[ERROR] 입력 소스를 열 수 없습니다. URL/장치 인덱스를 확인하세요.")
            sys.exit(1)

        self.fps = args.fps if args.fps and args.fps > 0 else self.cap.get(cv2.CAP_PROP_FPS)
        if not self.fps or self.fps <= 0:
            self.fps = 20.0
        self.fourcc = cv2.VideoWriter_fourcc(*args.fourcc)

        self.recording = False
        self.writer = None
        self.frame_size = None
        self.last_frame = None
        self.frames_written = 0
        self.start_time = None

        self.root = tk.Tk()
        self.root.title("Recorder | 시작/저장 여러 번")
        self.build_ui()

        # 키 바인딩
        self.root.bind("<KeyPress-s>", lambda e: self.on_start())
        self.root.bind("<KeyPress-e>", lambda e: self.on_stop())
        self.root.bind("<KeyPress-q>", lambda e: self.on_quit())
        self.root.protocol("WM_DELETE_WINDOW", self.on_quit)

        # 프레임 업데이트 루프 시작
        self.update_frame()

    def build_ui(self):
        main = ttk.Frame(self.root, padding=8)
        main.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        # 미리보기
        self.preview_label = ttk.Label(main)
        self.preview_label.grid(row=0, column=0, columnspan=4, sticky="nsew", pady=(0, 8))

        # 저장 폴더
        ttk.Label(main, text="저장 폴더:").grid(row=1, column=0, sticky="w")
        self.dir_var = tk.StringVar(value=os.getcwd())
        self.dir_entry = ttk.Entry(main, textvariable=self.dir_var, width=50)
        self.dir_entry.grid(row=1, column=1, columnspan=2, sticky="ew", padx=4)
        ttk.Button(main, text="찾기", command=self.on_browse_dir).grid(row=1, column=3, sticky="e")

        # 기본 파일명
        ttk.Label(main, text="파일명(접두사):").grid(row=2, column=0, sticky="w")
        self.base_var = tk.StringVar(value="clip")
        self.base_entry = ttk.Entry(main, textvariable=self.base_var, width=30)
        self.base_entry.grid(row=2, column=1, sticky="w", padx=4)

        # 컨트롤 버튼
        self.start_btn = ttk.Button(main, text="시작(새 저장) [S]", command=self.on_start)
        self.stop_btn = ttk.Button(main, text="정지 [E]", command=self.on_stop)
        self.quit_btn = ttk.Button(main, text="종료 [Q]", command=self.on_quit)
        self.start_btn.grid(row=3, column=0, pady=6, sticky="ew")
        self.stop_btn.grid(row=3, column=1, pady=6, sticky="ew")
        self.quit_btn.grid(row=3, column=3, pady=6, sticky="ew")

        # 상태
        self.status_var = tk.StringVar(value="대기 중")
        self.status = ttk.Label(main, textvariable=self.status_var)
        self.status.grid(row=4, column=0, columnspan=4, sticky="w", pady=(6, 0))

        # 레이아웃 확장
        for c in range(4):
            main.columnconfigure(c, weight=1)
        main.rowconfigure(0, weight=1)

    def on_browse_dir(self):
        d = filedialog.askdirectory(initialdir=self.dir_var.get() or os.getcwd())
        if d:
            self.dir_var.set(d)

    def build_output_path(self) -> str:
        base = self.base_var.get().strip() or "clip"
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = f"{base}_{ts}.mp4"
        out_dir = self.dir_var.get().strip() or os.getcwd()
        os.makedirs(out_dir, exist_ok=True)
        return ensure_unique_path(os.path.join(out_dir, fname))

    def on_start(self):
        if self.recording:
            messagebox.showinfo("안내", "이미 녹화 중입니다. 먼저 정지하세요.")
            return

        # 프레임 크기 확보
        if self.frame_size is None:
            ok, frame = self.cap.read()
            if not ok or frame is None:
                messagebox.showerror("오류", "첫 프레임을 읽을 수 없습니다.")
                return
            self.last_frame = frame
            h, w = frame.shape[:2]
            self.frame_size = (w, h)

        out_path = self.build_output_path()
        self.writer = cv2.VideoWriter(out_path, self.fourcc, self.fps, self.frame_size)
        if not self.writer.isOpened():
            messagebox.showerror("오류", "VideoWriter 초기화 실패. 코덱/경로를 확인하세요.")
            self.writer = None
            return

        self.recording = True
        self.frames_written = 0
        self.start_time = time.time()
        self.status_var.set(f"녹화 시작: {out_path}")

    def on_stop(self):
        if self.recording and self.writer is not None:
            self.writer.release()
            elapsed = time.time() - (self.start_time or time.time())
            self.status_var.set(
                f"녹화 저장 완료. 프레임: {self.frames_written}, 시간: {elapsed:.2f}s"
            )
        self.recording = False
        self.writer = None
        self.frames_written = 0
        self.start_time = None

    def on_quit(self):
        try:
            self.on_stop()
        finally:
            if self.cap:
                self.cap.release()
            self.root.destroy()

    def update_frame(self):
        ok, frame = self.cap.read()
        if ok and frame is not None:
            self.last_frame = frame
            if self.frame_size is None:
                h, w = frame.shape[:2]
                self.frame_size = (w, h)

            # 미리보기 표시
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(rgb)
            imgtk = ImageTk.PhotoImage(image=img)
            self.preview_label.imgtk = imgtk  # 참조 유지
            self.preview_label.configure(image=imgtk)

            # 녹화 중이면 기록
            if self.recording and self.writer is not None:
                try:
                    self.writer.write(frame)
                    self.frames_written += 1
                    if self.start_time:
                        elapsed = time.time() - self.start_time
                        self.status_var.set(
                            f"녹화 중... 프레임: {self.frames_written}, 시간: {elapsed:.1f}s"
                        )
                except Exception as e:
                    self.status_var.set(f"기록 오류: {e}")
        else:
            # 읽기 실패 시 잠시 대기
            time.sleep(0.01)

        # 다음 프레임 예약 (약 30fps)
        self.root.after(1, self.update_frame)

    def run(self):
        self.root.mainloop()


def main():
    args = parse_args()
    app = RecorderGUI(args)
    app.run()


if __name__ == "__main__":
    main()

