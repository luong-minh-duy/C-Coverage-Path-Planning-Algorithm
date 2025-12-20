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
    Nếu dead-end, chọn retreat node (open node gần nhất có trong G).
    Nếu hết open node, trả về (None, False).
    """
    def safe_node(n):
        return n in G and G.nodes[n].get('state') == 'open'

    # Lấy các node open thực sự có trong G
    open_nodes = [n for n in G.nodes if G.nodes[n].get('state') == 'open']
    if current_node is None:
        if not open_nodes:
            return (None, False)
        return (open_nodes[0], False)

    x0, y0 = current_node
    nodes_by_x = {}
    for n in open_nodes:
        nodes_by_x.setdefault(round(n[0], 6), []).append(n)

    # Lấy node cùng lap
    curr_lap_nodes = []
    if round(x0, 6) in nodes_by_x:
        curr_lap_nodes = sorted(nodes_by_x[round(x0, 6)], key=lambda n: n[1])

    up_node = None
    down_node = None
    if curr_lap_nodes:
        try:
            idx = curr_lap_nodes.index(current_node)
        except ValueError:
            curr_all = sorted([n for n in G.nodes if round(n[0], 6) == round(x0, 6)], key=lambda n: n[1])
            try:
                idx = curr_all.index(current_node)
                if idx < len(curr_all) - 1:
                    up_node = curr_all[idx + 1]
                if idx > 0:
                    down_node = curr_all[idx - 1]
            except ValueError:
                pass
        else:
            if idx < len(curr_lap_nodes) - 1:
                up_node = curr_lap_nodes[idx + 1]
            if idx > 0:
                down_node = curr_lap_nodes[idx - 1]

    # Lap trái và phải
    left_nodes = nodes_by_x.get(round(x0 - d_s, 6), [])
    right_nodes = nodes_by_x.get(round(x0 + d_s, 6), [])

    # Ưu tiên chọn theo L, U, D, R nếu tồn tại trong G
    for group in [left_nodes, [up_node], [down_node], right_nodes]:
        for candidate in group:
            if candidate and safe_node(candidate):
                return (candidate, False)

    # Retreat
    retreat_candidates = [n for n in open_nodes if n != current_node]
    if retreat_candidates:
        nearest = min(retreat_candidates, key=lambda n: math.hypot(n[0] - x0, n[1] - y0))
        if nearest in G:
            return (nearest, True)

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
