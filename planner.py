import math
import random
import json
import networkx as nx
from shapely.geometry import Polygon, Point, LineString, MultiLineString, GeometryCollection
from shapely.ops import unary_union
import matplotlib.pyplot as plt

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