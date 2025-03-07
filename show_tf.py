import yaml
import cv2
from flask import Flask, jsonify, send_file, render_template_string
import os


app = Flask(__name__)

# YAML 파일 경로 설정
YAML_PATH = "/home/yb/project2/proj2/pgm_yaml_create/test_map.yaml"


def load_map():
    """
    YAML 파일에서 맵 정보를 읽어 PGM 이미지를 PNG로 변환 후 저장하며,
    이미지의 크기를 구해 map_data에 추가합니다.
    """
    global map_data, map_image_path

    if not os.path.exists(YAML_PATH):
        print("❌ YAML 파일을 찾을 수 없습니다.")
        return
    
    with open(YAML_PATH, "r") as file:
        map_config = yaml.safe_load(file)

    # PGM 파일 경로 계산
    pgm_path = os.path.join(os.path.dirname(YAML_PATH), map_config["image"])
    if not os.path.exists(pgm_path):
        print(f"❌ PGM 파일을 찾을 수 없습니다: {pgm_path}")
        return

    print(f"📂 PGM 파일 로드: {pgm_path}")
    pgm_image = cv2.imread(pgm_path, cv2.IMREAD_UNCHANGED)
    height, width = pgm_image.shape[:2]

    # PNG로 변환하여 저장
    map_image_path = "/home/yb/project2/proj2/click_move/static/map.png"
    cv2.imwrite(map_image_path, pgm_image)
    print(f"✅ PNG 맵 생성 완료: {map_image_path}")

    # YAML의 해상도(resolution)와 원점(origin)에 이미지 크기 정보 추가
    map_data = {
        "resolution": map_config["resolution"],
        "origin": map_config["origin"],
        "image": "static/map.png",
        "width": width,
        "height": height
    }

@app.route('/')
def index():
    """
    HTML 페이지를 렌더링합니다.
    페이지 내에서 맵 이미지 클릭 시, 클릭한 픽셀 좌표를 실제 맵 좌표로 변환하여 화면에 출력합니다.
    """
    html_template = '''
    <!DOCTYPE html>
    <html>
    <head>
        <meta charset="utf-8">
        <title>맵 클릭 좌표 표시</title>
        <style>
            img { border: 1px solid black; }
        </style>
    </head>
    <body>
        <h1>맵을 클릭하세요</h1>
        <!-- /get_map_image 엔드포인트를 통해 맵 이미지 표시 -->
        <img id="map" src="/get_map_image" alt="Map">
        <p id="coords">좌표: </p>
        <script>
            // 서버에서 전달된 YAML 기반의 맵 정보 변수 (해상도, 원점, 이미지 높이)
            const resolution = {{ map_data.resolution }};
            const origin = {{ map_data.origin }};
            const imageHeight = {{ map_data.height }};
            const mapImage = document.getElementById('map');

            mapImage.addEventListener('click', function(e) {
                const rect = this.getBoundingClientRect();
                const pixelX = Math.floor(e.clientX - rect.left);
                const pixelY = Math.floor(e.clientY - rect.top);

                // 이미지 좌표를 실제 맵 좌표로 변환
                const mapX = origin[0] + (pixelX * resolution);
                const mapY = origin[1] + ((imageHeight - pixelY) * resolution);

                document.getElementById('coords').textContent = 
                    '좌표: ' + mapX.toFixed(2) + ', ' + mapY.toFixed(2);
            });
        </script>
    </body>
    </html>
    '''
    return render_template_string(html_template, map_data=map_data)

@app.route('/get_map_info', methods=['GET'])
def get_map_info():
    """YAML 파일 기반의 맵 정보를 JSON으로 반환"""
    return jsonify(map_data)

@app.route('/get_map_image', methods=['GET'])
def get_map_image():
    """변환된 PNG 맵 이미지를 반환"""
    return send_file(map_image_path, mimetype='image/png')

if __name__ == "__main__":
    print("📡 Flask 맵 서버 (맵 좌표 변환 기능) 실행 중...")
    load_map()  # 서버 실행 전에 맵 정보 로드
    app.run(host="0.0.0.0", port=5000)
