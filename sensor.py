from shapely.geometry import Polygon, Point, LineString, MultiLineString, GeometryCollection

from shapely.geometry import Point, LineString
from shapely.ops import unary_union

def sense_environment(current_pos, next_pos, range_sensor, env_free):
    """
    Mô phỏng cảm biến quét từ current_pos đến next_pos với tầm quét range_sensor.
    Trả về vùng mới được phát hiện (intersection với vùng tự do env_free).
    """
    line = LineString([current_pos, next_pos])

    # Buffer đường di chuyển
    scan_line = line.buffer(range_sensor, cap_style=1)

    # Buffer tại điểm đầu và cuối (đảm bảo quét tĩnh)
    scan_start = Point(current_pos).buffer(range_sensor)
    scan_end = Point(next_pos).buffer(range_sensor)

    # Hợp nhất toàn bộ vùng quét
    full_scan_area = unary_union([scan_line, scan_start, scan_end])

    # Cắt với vùng tự do
    scan_free = full_scan_area.intersection(env_free)
    return scan_free
