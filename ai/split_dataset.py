import os
import random
import shutil
from pathlib import Path

# 원본 경로
base_dir = Path("train")  # 현재 구조 기준
img_dir = base_dir / "images"
lbl_dir = base_dir / "labels"

# 새 구조
out_base = Path("dataset")
train_img_out = out_base / "train/images"
train_lbl_out = out_base / "train/labels"
val_img_out = out_base / "val/images"
val_lbl_out = out_base / "val/labels"

# 비율
val_ratio = 0.2  # 20%를 val로

# 출력 폴더 생성
for p in [train_img_out, train_lbl_out, val_img_out, val_lbl_out]:
    p.mkdir(parents=True, exist_ok=True)

# 모든 이미지 목록 가져오기
images = sorted([f for f in img_dir.glob("*.*") if f.suffix.lower() in [".jpg", ".png", ".jpeg"]])
random.shuffle(images)

val_count = int(len(images) * val_ratio)
val_images = set(images[:val_count])

print(f"총 {len(images)}장 중 {val_count}장은 val로 이동합니다.")

# 분리 이동
for img_path in images:
    label_path = lbl_dir / (img_path.stem + ".txt")
    if img_path in val_images:
        shutil.copy2(img_path, val_img_out / img_path.name)
        if label_path.exists():
            shutil.copy2(label_path, val_lbl_out / label_path.name)
    else:
        shutil.copy2(img_path, train_img_out / img_path.name)
        if label_path.exists():
            shutil.copy2(label_path, train_lbl_out / label_path.name)

print("✅ 완료: dataset/train, dataset/val 구조로 분리 완료!")
