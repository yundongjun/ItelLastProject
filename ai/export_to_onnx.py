# export_to_onnx.py
from ultralytics import YOLO

pt_path = r"C:\Users\STC\Desktop\test\runs\detect\drone_detector_yolov8s_dataset35\weights\best.pt"
model = YOLO(pt_path)

# 320×320 크기로 고정 export
model.export(format="onnx", opset=12, imgsz=320, simplify=True)

print("✅ 변환 완료! 'best.onnx' 파일이 320x320 입력으로 생성되었습니다.")
