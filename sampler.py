import networkx as nx
from shapely.geometry import Polygon, Point, LineString, MultiLineString, GeometryCollection
from shapely.ops import unary_union
import matplotlib.pyplot as plt

def generate_laps_and_samples(sampling_front, robot_pos, d_s, frontier_eps=1e-2):
    """
    Tạo các lap song song (vertical) trong sampling_front và frontier samples,
    đồng thời gán metadata cần thiết cho pruning C*.

    Trả về:
    - laps: list các LineString (segment của lap)
    - samples: list dict, mỗi phần tử:
        {
          'pos': (x,y),
          'lap_id': int,
          'is_end': bool,
          'near_frontier': bool
        }
    """

    laps = []
    samples = []

    if sampling_front is None or sampling_front.is_empty:
        return laps, samples

    minx, miny, maxx, maxy = sampling_front.bounds
    x0, y0 = robot_pos

    # Clamp x0 vào sampling front
    x0 = max(minx, min(x0, maxx))

    # Danh sách toạ độ x của các lap
    xs = [x0]

    xr = x0
    while (xr := xr + d_s) <= maxx:
        xs.append(xr)

    xl = x0
    while (xl := xl - d_s) >= minx:
        xs.append(xl)

    xs = sorted(xs)

    lap_id = 0  # định danh lap tăng dần

    # Với mỗi lap
    for x in xs:
        line = LineString([(x, miny - 1), (x, maxy + 1)])
        inter = sampling_front.intersection(line)

        segs = []
        if isinstance(inter, LineString):
            segs = [inter] if not inter.is_empty else []
        elif isinstance(inter, MultiLineString):
            segs = [g for g in inter.geoms if isinstance(g, LineString) and not g.is_empty]
        elif isinstance(inter, GeometryCollection):
            segs = [g for g in inter.geoms if isinstance(g, LineString) and not g.is_empty]

        for seg in segs:
            if seg.is_empty:
                continue

            laps.append(seg)

            y_min, y_max = seg.bounds[1], seg.bounds[3]

            lap_points = []

            # Nếu lap đi qua robot
            if abs(x - x0) < 1e-6 and y_min <= y0 <= y_max:
                lap_points.append(y0)
                yy = y0 + d_s
                while yy <= y_max:
                    lap_points.append(yy)
                    yy += d_s
                yy = y0 - d_s
                while yy >= y_min:
                    lap_points.append(yy)
                    yy -= d_s
            else:
                yy = y_min
                while yy <= y_max:
                    lap_points.append(yy)
                    yy += d_s

            lap_points = sorted(lap_points)

            # Xác định end nodes của lap segment
            end_ys = set()
            if lap_points:
                end_ys.add(lap_points[0])
                end_ys.add(lap_points[-1])

            for yy in lap_points:
                p = Point(x, yy)
                if p.distance(seg) > 1e-6:
                    continue

                # near frontier = gần biên sampling front
                near_frontier = (
                    p.distance(sampling_front.boundary) < frontier_eps
                )

                samples.append({
                    'pos': (round(x, 6), round(yy, 6)),
                    'lap_id': lap_id,
                    'is_end': yy in end_ys,
                    'near_frontier': near_frontier
                })

            lap_id += 1

    return laps, samples