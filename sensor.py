from shapely.geometry import Polygon, Point, LineString, MultiLineString, GeometryCollection

def sense_environment(current_pos, next_pos, range_sensor, env_free):
    """
    Mô phỏng cảm biến quét từ current_pos đến next_pos với tầm quét range_sensor.
    Trả về vùng mới được phát hiện (intersection với vùng tự do env_free).
    """
    line = LineString([current_pos, next_pos])
    # Buffer đường di chuyển theo bán kính sensor (cap_style=1 cho góc tròn)
    scan_area = line.buffer(range_sensor, cap_style=1)
    # Giới hạn bởi vùng tự do (loại bỏ vùng chướng ngại)
    scan_free = scan_area.intersection(env_free)
    return scan_free