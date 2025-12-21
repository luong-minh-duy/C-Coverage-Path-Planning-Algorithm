from shapely.geometry import Polygon, Point, LineString, MultiLineString, GeometryCollection

from shapely.geometry import Point, LineString
from shapely.ops import unary_union

def sense_environment(current_pos, next_pos, range_sensor, env_free):
    """
    Mô phỏng cảm biến quét từ current_pos đến next_pos với tầm quét range_sensor.
    Trả về vùng mới được phát hiện (intersection với vùng tự do env_free).
    """
    scan = Point(next_pos).buffer(range_sensor)
    return scan.intersection(env_free)
