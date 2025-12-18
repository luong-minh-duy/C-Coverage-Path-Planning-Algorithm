"""
Implementation of C* (C-star) coverage path planning algorithm for unknown environments
using Rapidly Covering Graphs (Shen et al.).
This code simulates range sensing, sampling, RCG construction, pruning, 
back-and-forth waypoint selection, dead-end retreat, and coverage holes via TSP.
"""

import math
import random
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

def update_discovered(discovered, new_scan):
    """
    Cập nhật vùng đã phát hiện bởi union với vùng vừa quét được new_scan.
    """
    if discovered is None:
        return new_scan
    else:
        return discovered.union(new_scan)

def get_sampling_front(discovered_old, discovered_new):
    """
    Tính vùng sampling front: phần mới được phát hiện (discovered_new - discovered_old).
    """
    if discovered_old is None:
        return discovered_new
    else:
        return discovered_new.difference(discovered_old)

def build_RCG(G, samples, obstacles, d_s):
    """
    Mở rộng Rapidly Covering Graph (RCG) với các node mới từ 'samples'.
    Kết nối các cạnh (edges) giữa nodes trên cùng lap và lap lân cận.
    obstacles: danh sách shapely Polygons.
    """
    # Thêm node mới
    for pt in samples:
        coord = (round(pt[0],6), round(pt[1],6))
        if coord not in G:
            G.add_node(coord, pos=coord, state='open')
    # Kết nối cùng lap: group theo x, sort theo y
    nodes_by_x = {}
    for node in G.nodes:
        x, y = node
        nodes_by_x.setdefault(round(x,6), []).append(node)
    for x_val, nodes in nodes_by_x.items():
        nodes_sorted = sorted(nodes, key=lambda n: n[1])
        for i in range(len(nodes_sorted)-1):
            a = nodes_sorted[i]; b = nodes_sorted[i+1]
            line = LineString([a, b])
            # Kiểm tra va chạm chướng ngại
            collision = False
            for obs in obstacles:
                if line.intersects(obs):
                    collision = True
                    break
            if not collision:
                G.add_edge(a, b)
    # Kết nối lap cạnh: lap trái và phải (x ± d_s)
    for node in list(G.nodes):
        x, y = node
        left_x = round(x - d_s, 6)
        if left_x in nodes_by_x:
            for nb in nodes_by_x[left_x]:
                if abs(y - nb[1]) <= d_s:
                    line = LineString([node, nb])
                    collision = False
                    for obs in obstacles:
                        if line.intersects(obs):
                            collision = True
                            break
                    if not collision:
                        G.add_edge(node, nb)
        right_x = round(x + d_s, 6)
        if right_x in nodes_by_x:
            for nb in nodes_by_x[right_x]:
                if abs(y - nb[1]) <= d_s:
                    line = LineString([node, nb])
                    collision = False
                    for obs in obstacles:
                        if line.intersects(obs):
                            collision = True
                            break
                    if not collision:
                        G.add_edge(node, nb)

def prune_RCG(G):
    """
    Cắt tỉa RCG: loại bỏ các node không quan trọng (degree 2 và cả hai neighbors cùng lap).
    """
    to_remove = []
    to_add_edge = []
    for node in list(G.nodes):
        deg = G.degree(node)
        if deg == 2:
            nbrs = list(G.neighbors(node))
            if len(nbrs) != 2:
                continue
            x0, y0 = node
            x1, y1 = nbrs[0]
            x2, y2 = nbrs[1]
            # Nếu hai neighbors cùng lap (cùng x với node)
            if round(x1,6)==round(x0,6) and round(x2,6)==round(x0,6):
                to_remove.append(node)
                to_add_edge.append((nbrs[0], nbrs[1]))
    # Thêm cạnh liên kết neighbors của node bị cắt
    for (a, b) in to_add_edge:
        if not G.has_edge(a, b) and a in G.nodes and b in G.nodes:
            G.add_edge(a, b)
    # Xóa node
    for node in to_remove:
        if node in G:
            G.remove_node(node)

def select_goal_node(G, current_node, d_s):
    """
    Chọn node kế tiếp theo thứ tự ưu tiên:
    Left lap (L), Up (U), Down (D), Right lap (R).
    Trả về (goal_node, is_retreat_flag).
    Nếu dead-end, chọn retreat node (open node gần nhất).
    Nếu hết open node, trả về (None, False).
    """
    # Lấy các node open
    open_nodes = [n for n,d in G.nodes(data=True) if d['state']=='open']
    if current_node is None:
        # Nếu chưa có node hiện tại, pick bất kỳ open
        if not open_nodes:
            return (None, False)
        return (open_nodes[0], False)
    x0, y0 = current_node
    # Lập dict lap
    nodes_by_x = {}
    for n in open_nodes:
        nodes_by_x.setdefault(round(n[0],6), []).append(n)
    # Lấy node cùng lap hiện tại
    curr_lap_nodes = []
    if round(x0,6) in nodes_by_x:
        curr_lap_nodes = sorted(nodes_by_x[round(x0,6)], key=lambda n: n[1])
    up_node = None; down_node = None
    if curr_lap_nodes:
        try:
            idx = curr_lap_nodes.index(current_node)
        except ValueError:
            # Nếu current đã đóng, lấy neighbors từ graph
            curr_all = sorted([n for n in G.nodes if round(n[0],6)==round(x0,6)], key=lambda n:n[1])
            try:
                idx = curr_all.index(current_node)
                if idx < len(curr_all)-1:
                    up_node = curr_all[idx+1]
                if idx > 0:
                    down_node = curr_all[idx-1]
            except ValueError:
                idx = None
        else:
            if idx is not None:
                if idx < len(curr_lap_nodes)-1:
                    up_node = curr_lap_nodes[idx+1]
                if idx > 0:
                    down_node = curr_lap_nodes[idx-1]
    # Lấy nodes lap trái và phải
    left_x = round(x0 - d_s, 6)
    left_nodes = nodes_by_x.get(left_x, [])
    right_x = round(x0 + d_s, 6)
    right_nodes = nodes_by_x.get(right_x, [])
    # Ưu tiên L
    if left_nodes:
        return (random.choice(left_nodes), False)
    # Tiếp U, D
    if up_node and G.nodes[up_node]['state']=='open':
        return (up_node, False)
    if down_node and G.nodes[down_node]['state']=='open':
        return (down_node, False)
    # Tiếp R
    if right_nodes:
        return (random.choice(right_nodes), False)
    # Dead-end: tìm retreat
    if open_nodes:
        # Loại trừ current_node nếu nó còn mở
        candidates = [n for n in open_nodes if n != current_node]
        if not candidates:
            return (None, False)
        curr_x, curr_y = current_node
        nearest = None; min_dist = float('inf')
        for n in candidates:
            dist = math.hypot(curr_x - n[0], curr_y - n[1])
            if dist < min_dist:
                min_dist = dist; nearest = n
        return (nearest, True)
    # Hết open nodes: hoàn tất
    return (None, False)

def solve_tsp(points, start=None):
    """
    TSP heuristic (nearest neighbor) qua danh sách points [(x,y)].
    Nếu start có thì bắt đầu từ start.
    """
    pts = points.copy()
    if not pts:
        return []
    if start:
        current = start
        if current in pts:
            pts.remove(current)
    else:
        current = pts[0]
        pts.remove(current)
    path = [current]
    while pts:
        nearest = None; dmin = float('inf')
        for p in pts:
            dist = math.hypot(current[0] - p[0], current[1] - p[1])
            if dist < dmin:
                dmin = dist; nearest = p
        path.append(nearest)
        current = nearest
        pts.remove(nearest)
    return path

def C_star_coverage(env_poly, obstacles, start_pos, sensor_range=5.0, cover_radius=1.0, d_s=2.0, max_iters=1000, verbose=False):
    """
    Hàm chính thực hiện C* trên môi trường env_poly với obstacles (list of Polygons).
    start_pos: (x,y) điểm bắt đầu.
    sensor_range: tầm dò (range detector).
    cover_radius: bán kính thiết bị phủ.
    d_s: độ phân giải sampling (khoảng cách lap).
    Trả về: path list (danh sách (x,y)), RCG Graph, và vùng free (env_free).
    """
    # Tính vùng tự do (loại bỏ obstacles)
    env_free = env_poly
    for obs in obstacles:
        env_free = env_free.difference(obs)
    # Khởi tạo discovered bởi sensing tại start
    discovered = update_discovered(None, sense_environment(start_pos, start_pos, sensor_range, env_free))
    discovered_prev = discovered
    # Giai đoạn progressive sampling ban đầu
    sampling_front = get_sampling_front(None, discovered)
    laps, samples = generate_laps_and_samples(sampling_front, start_pos, d_s)
    G = nx.Graph()
    build_RCG(G, samples, obstacles, d_s)
    prune_RCG(G)
    # Đặt node bắt đầu
    current_node = (round(start_pos[0],6), round(start_pos[1],6))
    if current_node not in G:
        G.add_node(current_node, pos=current_node, state='open')
    path = [start_pos]
    iter_count = 1
    # Vòng lặp chính
    while iter_count < max_iters:
        if verbose:
            print(f"Iteration {iter_count}, current node: {current_node}, nodes: {len(G.nodes)}, edges: {len(G.edges)}")
        goal_node, is_retreat = select_goal_node(G, current_node, d_s)
        if goal_node is None:
            if verbose:
                print("No goal found, coverage complete.")
            break
        # Đánh dấu current thành closed
        if current_node in G.nodes:
            G.nodes[current_node]['state'] = 'closed'
        if is_retreat:
            # Tìm đường ngắn (shortest path) đến retreat node
            try:
                route = nx.shortest_path(G, source=current_node, target=goal_node)
            except nx.NetworkXNoPath:
                route = [current_node, goal_node]
            if verbose:
                print(f"Dead-end: retreat to {goal_node} via {route}")
            # Di chuyển theo route
            for n in route[1:]:
                pos_next = (n[0], n[1])
                new_scan = sense_environment(path[-1], pos_next, sensor_range, env_free)
                discovered = update_discovered(discovered, new_scan)
                path.append(pos_next)
            current_node = goal_node
            # Không sampling ở retreat
            continue
        # Di chuyển bình thường đến goal
        pos_next = goal_node
        new_scan = sense_environment(path[-1], pos_next, sensor_range, env_free)
        discovered = update_discovered(discovered, new_scan)
        path.append(pos_next)
        current_node = goal_node
        if current_node in G.nodes:
            G.nodes[current_node]['state'] = 'closed'
        # Progressive sampling trong vùng mới phát hiện
        sampling_front = get_sampling_front(discovered_prev, discovered)
        discovered_prev = discovered
        laps, samples = generate_laps_and_samples(sampling_front, current_node, d_s)
        build_RCG(G, samples, obstacles, d_s)
        prune_RCG(G)
        for pt in samples:
            coord = (round(pt[0],6), round(pt[1],6))
            if coord in G.nodes:
                G.nodes[coord]['state'] = 'open'
        iter_count += 1
    # Xử lý coverage holes (TSP) nếu còn open node
    open_nodes = [n for n,d in G.nodes(data=True) if d['state']=='open']
    if open_nodes:
        if verbose:
            print(f"Open nodes after main coverage: {open_nodes}")
        open_coords = [(n[0], n[1]) for n in open_nodes]
        tsp_path = solve_tsp(open_coords, start=current_node)
        if verbose:
            print(f"TSP hole path: {tsp_path}")
        for next_pt in tsp_path:
            new_scan = sense_environment(path[-1], next_pt, sensor_range, env_free)
            discovered = update_discovered(discovered, new_scan)
            path.append(next_pt)
            coord = (round(next_pt[0],6), round(next_pt[1],6))
            if coord in G.nodes:
                G.nodes[coord]['state'] = 'closed'
    return path, G, env_free

# --- Ví dụ sử dụng và minh họa ---
if __name__ == "__main__":
    # Định nghĩa môi trường và obstacles
    env = Polygon([(0,0),(0,50),(50,50),(50,0)])
    obs1 = Polygon([(10,10),(10,20),(20,20),(20,10)])
    obs2 = Polygon([(30,30),(30,40),(40,40),(40,30)])
    obs3 = Polygon([(5,30),(5,35),(15,35),(15,30)])
    obstacles = [obs1, obs2, obs3]
    start = (5, 5)
    # Chạy thuật toán C*
    path, G, env_free = C_star_coverage(env, obstacles, start_pos=start,
                                       sensor_range=7.0, cover_radius=1.5, d_s=5.0,
                                       verbose=True)
    print("Finished coverage. Path length:", len(path))
    # Vẽ kết quả
    fig, ax = plt.subplots(figsize=(6,6))
    # Vẽ biên giới
    x,y = env.exterior.xy
    ax.plot(x, y, color='black')
    # Vẽ obstacles
    for obs in obstacles:
        ox, oy = obs.exterior.xy
        ax.fill(ox, oy, color='gray', alpha=0.7)
    # Vẽ đường đi coverage (màu xanh dương)
    px = [p[0] for p in path]; py = [p[1] for p in path]
    ax.plot(px, py, '-o', color='blue', markersize=3)
    # Vẽ RCG (nodes đỏ, edges đỏ)
    nx_nodes = list(G.nodes())
    nx_x = [n[0] for n in nx_nodes]
    nx_y = [n[1] for n in nx_nodes]
    ax.scatter(nx_x, nx_y, color='red', s=15)
    for u,v in G.edges():
        ax.plot([u[0], v[0]], [u[1], v[1]], color='red', linewidth=0.5)
    ax.set_title("C* Coverage Path (blue) with RCG (red)")
    ax.set_xlim(-5, 55); ax.set_ylim(-5, 55)
    plt.show()
