import math
import random
import json
import networkx as nx
from shapely.geometry import Polygon, Point, LineString, MultiLineString, GeometryCollection
from shapely.ops import unary_union
import matplotlib.pyplot as plt

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