import networkx as nx
from shapely.geometry import Polygon, Point, LineString, MultiLineString, GeometryCollection
from shapely.ops import unary_union
import matplotlib.pyplot as plt

def generate_laps_and_samples(sampling_front, robot_pos, d_s):
    """
    Tạo các đường lap song song (vertical) trong vùng sampling_front và điểm mẫu frontier.
    sampling_front: vùng mới phát hiện (shapely Polygon)
    robot_pos: tọa độ robot (x, y)
    d_s: khoảng cách giữa các lap và giữa các mẫu trên lap
    Trả về: danh sách đường lap và danh sách điểm mẫu frontier (x,y)
    """
    laps = []
    samples = []
    if sampling_front is None or sampling_front.is_empty:
        return laps, samples
    minx, miny, maxx, maxy = sampling_front.bounds
    x0, y0 = robot_pos
    # Đảm bảo lap đầu tiên nằm trong vùng
    if x0 < minx: 
        x0 = minx
    if x0 > maxx:
        x0 = maxx
    # Tạo danh sách các toạ độ x của các lap (x0, x0 ± k*d_s)
    xs = [x0]
    # Lập đi sang phải
    xr = x0
    while True:
        xr += d_s
        if xr <= maxx:
            xs.append(xr)
        else:
            break
    # Lập đi sang trái
    xl = x0
    while True:
        xl -= d_s
        if xl >= minx:
            xs.append(xl)
        else:
            break
    xs = sorted(xs)
    # Với mỗi lap, tính giao của đường thẳng với sampling_front
    for x in xs:
        # Đường thẳng dài đủ để chắn qua sampling_front
        line = LineString([(x, miny - 1), (x, maxy + 1)])
        inter = sampling_front.intersection(line)
        segs = []
        # Xử lý kết quả giao cắt
        if isinstance(inter, LineString):
            if not inter.is_empty:
                segs = [inter]
        elif isinstance(inter, MultiLineString):
            for geom in inter.geoms:
                if isinstance(geom, LineString) and not geom.is_empty:
                    segs.append(geom)
        elif isinstance(inter, GeometryCollection):
            for geom in inter.geoms:
                if isinstance(geom, LineString) and not geom.is_empty:
                    segs.append(geom)
        # Lấy từng đoạn thẳng (segment) trên lap
        for seg in segs:
            if seg.is_empty:
                continue
            y_min, y_max = seg.bounds[1], seg.bounds[3]
            lap_samples = []
            # Nếu lap chứa vị trí robot, lấy robot làm mẫu đầu tiên
            if x == x0 and y_min <= y0 <= y_max:
                lap_samples.append((x0, y0))
                # Mẫu lên trên robot
                yy = y0 + d_s
                while yy <= y_max:
                    lap_samples.append((x, yy))
                    yy += d_s
                # Mẫu xuống dưới robot
                yy = y0 - d_s
                while yy >= y_min:
                    lap_samples.append((x, yy))
                    yy -= d_s
            else:
                # Khởi tạo mẫu từ đáy segment lên
                yy = y_min
                while yy <= y_max:
                    lap_samples.append((x, yy))
                    yy += d_s
            # Chọn những điểm thực sự nằm trên segment (bằng distance)
            for (xx, yy) in lap_samples:
                if Point(xx, yy).distance(seg) < 1e-6:
                    samples.append((xx, yy))
            laps.append(seg)
    return laps, samples