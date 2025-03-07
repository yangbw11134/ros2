import cv2
import numpy as np

# (1) 이미지 경로 설정
image_path = "/home/yb/project2/proj2/pgm_yaml_create/map.pgm"
image = cv2.imread(image_path)

if image is None:
    print("이미지를 불러올 수 없습니다. 경로를 확인하세요.")
    exit()

# (2) 픽셀 -> 미터 변환 계수(예시)
#     실제 맵 크기에 맞춰 조정하세요.
# pixel_to_meter = ㅊ0.01228
pixel_to_meter = 0.01706

# (3) 클릭 정보를 저장할 리스트(선택사항)
clicked_points = []  # (px, py, mx, my)를 저장

# (4) 마우스 콜백 함수
def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        # (x, y): 픽셀 좌표
        # mx, my : 미터로 환산한 좌표
        mx = x * pixel_to_meter
        my = y * pixel_to_meter
        
        # 터미널에 출력
        print(f"[클릭] 픽셀=({x}, {y}),  실제좌표=({mx:.3f}m, {my:.3f}m)")
        
        # 리스트에 저장(선택사항)
        clicked_points.append((x, y, mx, my))

# (5) 윈도우 생성 및 콜백 등록
cv2.namedWindow("Map")
cv2.setMouseCallback("Map", on_mouse)

# (6) 무한 루프: ESC 키(27) 누르면 종료
while True:
    cv2.imshow("Map", image)
    key = cv2.waitKey(1) & 0xFF
    if key == 27:  # ESC
        break

cv2.destroyAllWindows()

# (7) 종료 후, 클릭했던 정보 저장(선택사항)
#     예: 간단히 텍스트 파일로 저장
with open("clicked_points.txt", "w") as f:
    for px, py, mx, my in clicked_points:
        f.write(f"px={px}, py={py}, mx={mx:.3f}m, my={my:.3f}m\n")

print("프로그램을 종료합니다.")
