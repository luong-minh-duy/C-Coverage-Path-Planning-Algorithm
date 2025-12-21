import math
import random
import json
import networkx as nx
from shapely.geometry import Polygon, Point, LineString, MultiLineString, GeometryCollection
from shapely.ops import unary_union
import matplotlib.pyplot as plt

def select_goal_node(G, current_node, prev_node, d_s):
    """
    Waypoint selection theo C* (đã chống dao động):
    - Ưu tiên: Left lap → Up → Down → Right lap
    - Không quay lại prev_node
    - Retreat: graph-based, ưu tiên khác lap
    """

    def is_open(n):
        return n in G and G.nodes[n].get('state') == 'open'

    # -----------------------------
    # Open nodes (loại prev_node)
    # -----------------------------
    open_nodes = [
        n for n in G.nodes
        if is_open(n) and n != prev_node
    ]

    if not open_nodes:
        return (None, False)

    if current_node is None or current_node not in G:
        return (open_nodes[0], False)

    curr_lap = G.nodes[current_node].get('lap_id')

    # -----------------------------
    # 1. Same lap: Up / Down
    # -----------------------------
    same_lap = [
        n for n in open_nodes
        if G.nodes[n].get('lap_id') == curr_lap
    ]
    same_lap_sorted = sorted(same_lap, key=lambda n: n[1])

    up_node = down_node = None
    if current_node in same_lap_sorted:
        idx = same_lap_sorted.index(current_node)
        if idx + 1 < len(same_lap_sorted):
            up_node = same_lap_sorted[idx + 1]
        if idx - 1 >= 0:
            down_node = same_lap_sorted[idx - 1]

    # -----------------------------
    # 2. Left / Right lap
    # -----------------------------
    left_lap = curr_lap - 1 if curr_lap is not None else None
    right_lap = curr_lap + 1 if curr_lap is not None else None

    left_nodes = [
        n for n in open_nodes
        if G.nodes[n].get('lap_id') == left_lap
    ]
    right_nodes = [
        n for n in open_nodes
        if G.nodes[n].get('lap_id') == right_lap
    ]

    # Thứ tự ưu tiên đúng paper
    priority_groups = [
        left_nodes,
        [up_node],
        [down_node],
        right_nodes
    ]

    for group in priority_groups:
        for n in group:
            if n is not None and is_open(n):
                return (n, False)

    # -----------------------------
    # 3. Retreat (graph-based, khác lap)
    # -----------------------------
    retreat_candidates = [
        n for n in open_nodes if n != current_node
    ]

    # Ưu tiên retreat sang lap khác
    diff_lap = [
        n for n in retreat_candidates
        if G.nodes[n].get('lap_id') != curr_lap
    ]
    if diff_lap:
        retreat_candidates = diff_lap

    if retreat_candidates:
        try:
            nearest = min(
                retreat_candidates,
                key=lambda n: nx.shortest_path_length(G, current_node, n)
            )
            return (nearest, True)
        except nx.NetworkXNoPath:
            pass

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
