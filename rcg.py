import math
import random
import json
import networkx as nx
from shapely.geometry import Polygon, Point, LineString, MultiLineString, GeometryCollection
from shapely.ops import unary_union
import matplotlib.pyplot as plt

def build_RCG(G, samples, obstacles, d_s):
    """
    Mở rộng Rapidly Covering Graph (RCG) theo đúng tinh thần C*.
    - samples: list dict từ generate_laps_and_samples()
    """

    # -------------------------------------------------
    # 1. Thêm node mới + metadata (Definition III.8)
    # -------------------------------------------------
    for s in samples:
        coord = s['pos']
        if coord not in G:
            G.add_node(
                coord,
                pos=coord,
                lap_id=s['lap_id'],
                is_end=s['is_end'],
                near_frontier=s['near_frontier'],
                state='open'
            )

    # -------------------------------------------------
    # 2. Group nodes theo lap_id (KHÔNG theo x)
    # -------------------------------------------------
    nodes_by_lap = {}
    for n, data in G.nodes(data=True):
        lap = data.get('lap_id')
        if lap is not None:
            nodes_by_lap.setdefault(lap, []).append(n)

    # -------------------------------------------------
    # 3. Kết nối các node trên cùng lap
    #    (Definition III.9 – same lap)
    # -------------------------------------------------
    for lap_id, nodes in nodes_by_lap.items():
        nodes_sorted = sorted(nodes, key=lambda n: n[1])  # sort theo y
        for i in range(len(nodes_sorted) - 1):
            a = nodes_sorted[i]
            b = nodes_sorted[i + 1]

            if G.has_edge(a, b):
                continue

            line = LineString([a, b])
            if any(line.intersects(obs) for obs in obstacles):
                continue

            G.add_edge(a, b)

    # -------------------------------------------------
    # 4. Kết nối các node trên lap kề nhau
    #    (Definition III.9 – adjacent laps)
    # -------------------------------------------------
    for lap_id, nodes in nodes_by_lap.items():
        for adj_lap in (lap_id - 1, lap_id + 1):
            if adj_lap not in nodes_by_lap:
                continue

            for a in nodes:
                for b in nodes_by_lap[adj_lap]:
                    if abs(a[1] - b[1]) > d_s:
                        continue

                    if G.has_edge(a, b):
                        continue

                    line = LineString([a, b])
                    if any(line.intersects(obs) for obs in obstacles):
                        continue

                    G.add_edge(a, b)

def prune_RCG(G):
    """
    RCG Pruning theo đúng Definition III.8 và III.9 của C*
    - Chỉ loại bỏ non-essential nodes
    - Chỉ giữ essential edges
    """

    # -----------------------------
    # 1. Xác định essential nodes
    # -----------------------------
    for n in G.nodes:
        data = G.nodes[n]

        essential = False

        # (1) Adjacent to unknown area / obstacle
        if data.get('near_frontier', False):
            essential = True

        # (2) End node of a lap
        elif data.get('is_end', False):
            essential = True

        # (3) Connected to end node of adjacent lap
        else:
            for nx in G.neighbors(n):
                nx_data = G.nodes[nx]

                if nx_data.get('is_end', False):
                    # n ∈ N(nx)
                    # check neighbors of nx on n's lap
                    same_lap_neighbors = [
                        k for k in G.neighbors(nx)
                        if G.nodes[k].get('lap_id') == data.get('lap_id')
                    ]

                    # (a) nx has no other neighbor on n's lap
                    if len(same_lap_neighbors) == 1:
                        essential = True
                        break

                    # (b) edge (n, nx) closest to obstacle/frontier
                    else:
                        # heuristic: mark edge closest to frontier
                        # (paper uses geometric proximity)
                        if data.get('near_frontier', False):
                            essential = True
                            break

        G.nodes[n]['essential'] = essential

    # --------------------------------
    # 2. Xác định essential edges
    # --------------------------------
    essential_edges = set()

    for u, v in G.edges:
        u_data = G.nodes[u]
        v_data = G.nodes[v]

        # Both endpoints must be essential
        if not (u_data['essential'] and v_data['essential']):
            continue

        same_lap = (u_data.get('lap_id') == v_data.get('lap_id'))
        adjacent_lap = abs(u_data.get('lap_id', -1) - v_data.get('lap_id', -1)) == 1

        # (1) Same lap
        if same_lap:
            essential_edges.add((u, v))
            continue

        # (2) Adjacent laps
        if adjacent_lap:
            # (a) both end nodes
            if u_data.get('is_end') and v_data.get('is_end'):
                essential_edges.add((u, v))
                continue

            # (b) one end node, one non-end node
            if u_data.get('is_end') ^ v_data.get('is_end'):
                end_node = u if u_data.get('is_end') else v
                other = v if end_node == u else u

                # neighbors of end_node on other's lap
                same_lap_neighbors = [
                    k for k in G.neighbors(end_node)
                    if G.nodes[k].get('lap_id') == G.nodes[other].get('lap_id')
                ]

                # (i) no other neighbor
                if len(same_lap_neighbors) == 1:
                    essential_edges.add((u, v))
                    continue

                # (ii) closest edge to frontier
                if G.nodes[other].get('near_frontier', False):
                    essential_edges.add((u, v))
                    continue

    # --------------------------------
    # 3. Prune non-essential nodes
    # --------------------------------
    nodes_to_remove = [
        n for n in G.nodes
        if not G.nodes[n]['essential']
    ]

    for n in nodes_to_remove:
        if n in G:
            G.remove_node(n)

    # --------------------------------
    # 4. Prune non-essential edges
    # --------------------------------
    edges_to_remove = [
        (u, v) for (u, v) in G.edges
        if (u, v) not in essential_edges and (v, u) not in essential_edges
    ]

    for e in edges_to_remove:
        if G.has_edge(*e):
            G.remove_edge(*e)
