import cv2
import numpy as np
import os

# -------------------------------
# 1. 이미지 로드 및 리사이즈
# -------------------------------
image_path = "/home/yb/project2/proj2/pgm_yaml_create/test_map.png"
image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
if image is None:
    print(f"파일 {image_path}를 찾을 수 없습니다.")
    exit()

# 예시: 최대 크기를 1024로 제한 (필요에 따라 조정)
max_size = 1024
h, w = image.shape
scale = min(max_size / h, max_size / w)
new_size = (int(w * scale), int(h * scale))
resized_image = cv2.resize(image, new_size, interpolation=cv2.INTER_AREA)

# -------------------------------
# 2. PGM 파일 저장 (바이너리 옵션 적용)
# -------------------------------
# YAML 파일의 경로와 혼동되지 않도록 정확히 정의
pgm_path = "/home/yb/project2/proj2/pgm_yaml_create/test_map.pgm"

# PGM 파일 저장 부분
if not cv2.imwrite(pgm_path, resized_image, [cv2.IMWRITE_PXM_BINARY, 1]):
    print("PGM 파일 저장 실패")
    exit()
else:
    print(f"PGM 파일 생성 완료: {pgm_path}")


# -------------------------------
# 3. YAML 파일 생성
# -------------------------------
# 아래 YAML 파일의 'resolution' 값은 실제 맵의 해상도(m/px)에 맞게 수정해야 합니다.
resolution = 0.01586  # 픽셀 당 실제 거리(m)

origin_x = -(new_size[0] * resolution) / 2
origin_y = -(new_size[1] * resolution) / 2

yaml_content = f"""image: {os.path.basename(pgm_path)}
mode: trinary
occupied_thresh: 0.65
free_thresh: 0.01586
resolution: {resolution}
origin: [{origin_x}, {origin_y}, 0.0]
negate: 0
"""


yaml_path = "/home/yb/project2/proj2/pgm_yaml_create/test_map.yaml"
with open(yaml_path, "w") as file:
    file.write(yaml_content)

print(f"YAML 파일 생성 완료: {yaml_path}")
