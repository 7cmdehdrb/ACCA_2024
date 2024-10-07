import requests

import math


def get_static_map(api_key, center, zoom, size, map_type, markers, id):
    url = "https://maps.googleapis.com/maps/api/staticmap"

    # Query parameters for the Static Map API
    params = {
        "center": center,  # 중심 좌표 (위도, 경도)
        "zoom": zoom,  # 확대/축소 레벨
        "size": size,  # 지도 이미지 크기 (예: '600x400')
        "maptype": map_type,  # 지도 유형 ('roadmap', 'satellite', 'hybrid', 'terrain')
        # "markers": markers,  # 마커 위치
        "key": api_key,  # API 키
    }

    # HTTP GET 요청 보내기
    response = requests.get(url, params=params)

    # 응답 확인 및 지도 이미지 저장
    if response.status_code == 200:
        with open(f"static_map{id}.png", "wb") as f:
            f.write(response.content)
        print(f"Static map saved as 'static_map{id}.png'")
    else:
        print("Error:", response.status_code, response.text)


def calculate_lat_lon_bounds(center_lat, center_lon, zoom, img_width, img_height):
    # 지구 둘레 (미터 단위)
    earth_circumference = 40075017  # 적도 기준
    # 타일 한 변의 픽셀 수
    tile_size = 256

    # 줌 레벨에서 한 픽셀당 몇 미터인지 계산
    meters_per_pixel = earth_circumference / (2**zoom * tile_size)

    # 이미지의 경도, 위도 변화량 계산
    lat_per_pixel = meters_per_pixel / 111320  # 위도 변화량 (1도 위도는 약 111.32km)
    lon_per_pixel = meters_per_pixel / (
        math.cos(math.radians(center_lat)) * 111320
    )  # 경도 변화량

    # 이미지의 끝단 좌표 계산
    delta_lat = (img_height / 2) * lat_per_pixel
    delta_lon = (img_width / 2) * lon_per_pixel

    # 이미지의 경계 좌표 계산
    lat_min = center_lat - delta_lat
    lat_max = center_lat + delta_lat
    lon_min = center_lon - delta_lon
    lon_max = center_lon + delta_lon

    return [
        [center_lat - delta_lat, center_lon - delta_lon],
        [center_lat - delta_lat, center_lon + delta_lon],
        [center_lat + delta_lat, center_lon - delta_lon],
        [center_lat - delta_lat, center_lon - delta_lon],
    ]

    return lat_min, lat_max, lon_min, lon_max


# 샘플 사용 예시
api_key = ""
center_lat = 37.4961657
center_lon = 126.9570535
center = "{},{}".format(str(center_lat), str(center_lon))  # 서울 중심 좌표
zoom = 18
img_width = 600
img_height = 600
size = "{}x{}".format(str(img_width), str(img_height))
map_type = "roadmap"
markers = "color:red|label:S|37.5665,126.9780"

get_static_map(api_key, center, zoom, size, map_type, markers, 1)


gps_data = calculate_lat_lon_bounds(center_lat, center_lon, zoom, img_width, img_height)

print(gps_data)
