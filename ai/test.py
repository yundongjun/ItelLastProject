import cv2, os, random
from pathlib import Path

video_path = "KakaoTalk_20251015_135600509.mp4"           # 동영상 경로
out_dir = "frames_fire_rand200"         # 저장 폴더
num_samples = 200                  # 뽑을 이미지 수
seed = 42                          # 재현성 위해 시드 고정(원하면 변경/삭제)

os.makedirs(out_dir, exist_ok=True)
random.seed(seed)

cap = cv2.VideoCapture(video_path)
if not cap.isOpened():
    raise RuntimeError(f"동영상을 열 수 없습니다: {video_path}")

# 총 프레임 수 가져오기(일부 코덱은 -1을 줄 수 있어 보정)
total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
if total <= 0:
    # fallback: 직접 카운트
    total = 0
    while True:
        ret = cap.grab()
        if not ret:
            break
        total += 1
    cap.release()
    cap = cv2.VideoCapture(video_path)

take = min(num_samples, total)
indices = sorted(random.sample(range(total), take))

for i, fidx in enumerate(indices, 1):
    cap.set(cv2.CAP_PROP_POS_FRAMES, fidx)
    ret, frame = cap.read()
    if not ret:
        continue
    out_path = Path(out_dir) / f"frame_{fidx:08d}.jpg"
    cv2.imwrite(str(out_path), frame)
    if i % 50 == 0:
        print(f"{i}/{take} 저장...")

cap.release()
print(f"완료! {take}장 저장 -> {out_dir}")
