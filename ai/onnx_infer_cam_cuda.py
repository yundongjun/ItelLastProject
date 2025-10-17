#!/usr/bin/env python3
# onnx_infer_cam_cuda.py
import os
import sys
import time
import math
import argparse
from pathlib import Path
from datetime import datetime

import cv2
import numpy as np
import onnxruntime as ort
import platform

# -----------------------------
# Utils
# -----------------------------
def letterbox(image, new_shape=(320, 320), color=(114, 114, 114)):
    """Resize with unchanged aspect ratio using padding."""
    h, w = image.shape[:2]
    r = min(new_shape[0] / h, new_shape[1] / w)
    new_unpad = (int(round(w * r)), int(round(h * r)))
    dw = new_shape[1] - new_unpad[0]
    dh = new_shape[0] - new_unpad[1]
    dw /= 2.0
    dh /= 2.0
    resized = cv2.resize(image, new_unpad, interpolation=cv2.INTER_LINEAR)
    padded = cv2.copyMakeBorder(resized, int(round(dh)), int(round(dh)),
                                int(round(dw)), int(round(dw)),
                                cv2.BORDER_CONSTANT, value=color)
    return padded, r, (dw, dh)

def clip_boxes(x1, y1, x2, y2, w, h):
    x1 = max(0, min(int(round(x1)), w - 1))
    y1 = max(0, min(int(round(y1)), h - 1))
    x2 = max(0, min(int(round(x2)), w - 1))
    y2 = max(0, min(int(round(y2)), h - 1))
    if x2 < x1: x1, x2 = x2, x1
    if y2 < y1: y1, y2 = y2, y1
    return x1, y1, x2, y2

def iou_xyxy(a, b):
    """IoU for [x1,y1,x2,y2] arrays."""
    x1 = max(a[0], b[0])
    y1 = max(a[1], b[1])
    x2 = min(a[2], b[2])
    y2 = min(a[3], b[3])
    iw = max(0.0, x2 - x1)
    ih = max(0.0, y2 - y1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    area_a = max(0.0, (a[2] - a[0])) * max(0.0, (a[3] - a[1]))
    area_b = max(0.0, (b[2] - b[0])) * max(0.0, (b[3] - b[1]))
    union = area_a + area_b - inter
    return inter / max(union, 1e-9)

def nms(dets, iou_thres=0.45):
    """dets: (N,6) [x1,y1,x2,y2,conf,cls]"""
    if dets.size == 0:
        return dets
    idxs = np.argsort(-dets[:, 4])
    dets = dets[idxs]
    keep = []
    while dets.shape[0]:
        cur = dets[0]
        keep.append(cur)
        if dets.shape[0] == 1:
            break
        rest = dets[1:]
        mask = []
        for i, d in enumerate(rest):
            if int(d[5]) != int(cur[5]):
                mask.append(True)
            else:
                mask.append(iou_xyxy(cur[:4], d[:4]) <= iou_thres)
        dets = rest[np.array(mask)]
    return np.array(keep, dtype=np.float32)

def parse_yolov8_outputs(outputs, conf_thres=0.5, iou_thres=0.45, do_nms=True):
    """
    Support:
      - Already NMS'ed: (N,6) or (1,N,6) -> [x1,y1,x2,y2,conf,cls]
      - Raw: (N,85)/(1,N,85)/(85,N) (xywh + obj + cls_scores)
    Returns (M,6) float32
    """
    out = None
    for o in outputs:
        if isinstance(o, np.ndarray):
            out = o
            break
    if out is None:
        return np.empty((0, 6), dtype=np.float32)

    arr = out
    if arr.ndim == 3 and arr.shape[0] == 1:
        arr = arr[0]
    if arr.ndim == 2 and arr.shape[0] in (6, 85) and arr.shape[1] > arr.shape[0]:
        arr = arr.transpose(1, 0)

    if arr.ndim != 2:
        return np.empty((0, 6), dtype=np.float32)

    C = arr.shape[1]
    if C == 6:
        det = arr.astype(np.float32)
        det = det[det[:, 4] >= conf_thres]
        return nms(det, iou_thres) if do_nms else det

    if C > 6:
        xywh = arr[:, 0:4].astype(np.float32)
        obj = arr[:, 4:5].astype(np.float32)
        cls_scores = arr[:, 5:].astype(np.float32)
        cls_id = np.argmax(cls_scores, axis=1).reshape(-1, 1).astype(np.float32)
        cls_conf = cls_scores[np.arange(cls_scores.shape[0]), cls_id[:, 0].astype(int)].reshape(-1, 1)
        conf = obj * cls_conf

        mask = (conf[:, 0] >= conf_thres)
        if not np.any(mask):
            return np.empty((0, 6), dtype=np.float32)

        xywh = xywh[mask]
        conf = conf[mask]
        cls_id = cls_id[mask]

        x, y, w, h = xywh[:, 0], xywh[:, 1], xywh[:, 2], xywh[:, 3]
        x1 = (x - w / 2.0).reshape(-1, 1)
        y1 = (y - h / 2.0).reshape(-1, 1)
        x2 = (x + w / 2.0).reshape(-1, 1)
        y2 = (y + h / 2.0).reshape(-1, 1)

        det = np.concatenate([x1, y1, x2, y2, conf, cls_id], axis=1).astype(np.float32)
        return nms(det, iou_thres) if do_nms else det

    return np.empty((0, 6), dtype=np.float32)

def draw_det(frame, box, label, conf, color=(0, 255, 0)):
    x1, y1, x2, y2 = map(int, box)
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    txt = f"{label} {conf:.2f}"
    cv2.putText(frame, txt, (x1, max(0, y1 - 7)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

def guess_fourcc_from_ext(path: str) -> str:
    ext = os.path.splitext(path)[1].lower()
    if ext in [".mp4", ".m4v", ".mov"]:
        return "mp4v"
    if ext in [".avi"]:
        return "XVID"
    return "mp4v"

# -----------------------------
# Provider / Session
# -----------------------------
PROVIDER_MAP = {
    "cpu": ["CPUExecutionProvider"],
    "dml": ["DmlExecutionProvider", "CPUExecutionProvider"],
    "cuda": ["CUDAExecutionProvider", "CPUExecutionProvider"],
}

def build_session_resilient(model_path: Path, force_provider: str | None = None):
    avail = getattr(ort, "get_available_providers", lambda: ["CPUExecutionProvider"])()
    print("[INFO] ORT path         :", ort.__file__)
    print("[INFO] ORT version      :", getattr(ort, "__version__", "?"))
    print("[INFO] Providers avail  :", avail)

    attempts = []

    # 0) force override
    if force_provider:
        key = force_provider.lower()
        if key in PROVIDER_MAP:
            attempts.append(PROVIDER_MAP[key])
        else:
            print(f"[WARN] Unknown --force-provider '{force_provider}', ignoring.")

    # 1) CPU only first (DLL 충돌 회피용 스모크)
    if "CPUExecutionProvider" in avail:
        if ["CPUExecutionProvider"] not in attempts:
            attempts.append(["CPUExecutionProvider"])

    # 2) DirectML (Windows only)
    if platform.system().lower().startswith("win") and "DmlExecutionProvider" in avail:
        if ["DmlExecutionProvider", "CPUExecutionProvider"] not in attempts:
            attempts.append(["DmlExecutionProvider", "CPUExecutionProvider"])

    # 3) CUDA
    if "CUDAExecutionProvider" in avail:
        if ["CUDAExecutionProvider", "CPUExecutionProvider"] not in attempts:
            attempts.append(["CUDAExecutionProvider", "CPUExecutionProvider"])

    # 4) Last fallback CPU
    if ["CPUExecutionProvider"] not in attempts:
        attempts.append(["CPUExecutionProvider"])

    last_err = None
    for provs in attempts:
        try:
            print(f"[INFO] Trying providers  : {provs}")
            sess = ort.InferenceSession(str(model_path), providers=provs)
            print(f"[INFO] Using providers  : {provs}")
            return sess
        except Exception as e:
            print(f"[WARN] Session create failed with {provs}: {e}")
            last_err = e
            continue

    # If all failed:
    raise RuntimeError(f"Could not create InferenceSession. Last error:\n{last_err}")

# -----------------------------
# Main
# -----------------------------
def main():
    parser = argparse.ArgumentParser(description="ONNX YOLOv8 realtime inference (Windows / Resilient providers)")
    parser.add_argument("--model", type=str, required=False,
                        default=r"C:\Users\STC\Desktop\test\runs\detect\drone_detector_yolov8s_dataset35\weights\best.onnx",
                        help="Path to ONNX model")
    parser.add_argument("--device", type=int, default=0, help="Camera index (default 0)")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--imgsz", type=int, default=320, help="Inference image size (square)")
    parser.add_argument("--conf", type=float, default=0.5, help="Confidence threshold")
    parser.add_argument("--iou", type=float, default=0.45, help="NMS IoU threshold")
    parser.add_argument("--labels", type=str, default="fire,person", help="Comma-separated class names")
    parser.add_argument("--show-fps", action="store_true", help="Show FPS overlay")
    parser.add_argument("--save", type=str, default="", help="Optional. Start saving immediately to this path")
    parser.add_argument("--record", action="store_true", help="Enable recording toggle by 's' key")
    parser.add_argument("--force-provider", type=str, default="", choices=["cpu", "dml", "cuda", ""],
                        help="Force providers order: cpu/dml/cuda (optional)")
    args = parser.parse_args()

    labels = [s.strip() for s in args.labels.split(",") if s.strip()]
    img_size = (args.imgsz, args.imgsz)

    model_path = Path(args.model)
    if not model_path.exists():
        raise FileNotFoundError(f"ONNX model not found: {model_path}")

    # Session (resilient)
    print("[INFO] model            :", model_path)
    session = build_session_resilient(model_path, force_provider=args.force_provider or None)
    input_name = session.get_inputs()[0].name

    # Camera
    cap = cv2.VideoCapture(args.device, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        raise RuntimeError("❌ 카메라를 열 수 없습니다.")

    # Writer (optional)
    writer = None
    recording = False
    out_path = args.save
    if out_path:
        fourcc = cv2.VideoWriter_fourcc(*guess_fourcc_from_ext(out_path))
        writer = cv2.VideoWriter(out_path, fourcc, 30.0, (args.width, args.height))
        if writer.isOpened():
            recording = True
            print(f"[INFO] Recording to {out_path}")
        else:
            print(f"[WARN] Failed to open writer for {out_path}. Recording disabled.")
            writer = None

    win = "ONNX YOLOv8 Detection (q/ESC quit, s record toggle)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, args.width, args.height)

    ema_fps = None
    t_prev = time.time()

    while True:
        ok, frame = cap.read()
        if not ok:
            print("[WARN] Frame read failed")
            break

        h0, w0 = frame.shape[:2]
        img, r, (dw, dh) = letterbox(frame, img_size)
        blob = img[:, :, ::-1].transpose(2, 0, 1).astype(np.float32) / 255.0
        blob = np.expand_dims(blob, axis=0)

        try:
            outputs = session.run(None, {input_name: blob})
        except Exception as e:
            print("[ERROR] session.run 실패:", e)
            break

        det = parse_yolov8_outputs(outputs, conf_thres=args.conf, iou_thres=args.iou, do_nms=True)

        if det.shape[0] == 0:
            cv2.putText(frame, "No detections", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        else:
            for x1, y1, x2, y2, conf, cls in det:
                # reverse letterbox
                x1 = (x1 - dw) / r
                y1 = (y1 - dh) / r
                x2 = (x2 - dw) / r
                y2 = (y2 - dh) / r
                x1, y1, x2, y2 = clip_boxes(x1, y1, x2, y2, w0, h0)

                cid = int(cls)
                label = labels[cid] if 0 <= cid < len(labels) else f"class_{cid}"
                draw_det(frame, (x1, y1, x2, y2), label, float(conf))

        # FPS
        if args.show_fps:
            t_now = time.time()
            fps = 1.0 / max(t_now - t_prev, 1e-6)
            t_prev = t_now
            ema_fps = fps if ema_fps is None else (0.9 * ema_fps + 0.1 * fps)
            cv2.putText(frame, f"FPS: {ema_fps:.1f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)

        # Recording
        if writer and recording:
            writer.write(frame)

        cv2.imshow(win, frame)
        k = cv2.waitKey(1) & 0xFF
        if k in (27, ord('q')):  # ESC/q
            break
        if args.record and k == ord('s'):
            if recording:
                print("[INFO] Stop recording.")
                recording = False
                if writer:
                    writer.release()
                    writer = None
            else:
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                out_path = f"record_{ts}.mp4"
                fourcc = cv2.VideoWriter_fourcc(*guess_fourcc_from_ext(out_path))
                writer = cv2.VideoWriter(out_path, fourcc, 30.0, (w0, h0))
                if writer.isOpened():
                    recording = True
                    print(f"[INFO] Start recording -> {out_path}")
                else:
                    print("[WARN] Failed to start writer.")

    cap.release()
    if writer:
        writer.release()
    cv2.destroyAllWindows()
    print("[INFO] Done.")

if __name__ == "__main__":
    main()
