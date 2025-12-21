import math
import random
import json
import networkx as nx 
from shapely.geometry import Polygon, Point, LineString, MultiLineString, GeometryCollection 
from shapely.ops import unary_union 
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time 
from sampler import generate_laps_and_samples
from sensor import sense_environment
from coverage_tracker import update_discovered
from coverage_map import get_sampling_front
from rcg import build_RCG, prune_RCG
from planner import select_goal_node, solve_tsp

def is_valid_edge(p1, p2, obstacles):
    line = LineString([p1, p2])
    for obs in obstacles:
        if line.crosses(obs) or line.within(obs) or line.intersects(obs):
            return False
    return True

def move_and_scan(
    path, target,
    sensor_range, cover_radius,
    env_free,
    discovered, covered,
    step_size=1.0
):
    """
    Di chuyển robot từ path[-1] đến target.
    - discovered: vùng nhận thức (sensor-based)
    - covered: vùng phủ thực sự (coverage footprint)
    """

    current = path[-1]
    line = LineString([current, target])
    num_steps = max(int(line.length // step_size), 1)

    for i in range(1, num_steps + 1):
        x = current[0] + (target[0] - current[0]) * i / num_steps
        y = current[1] + (target[1] - current[1]) * i / num_steps
        intermediate = (x, y)

        # -------- Discovery (nhận thức) --------
        scan = sense_environment(
            path[-1], intermediate, sensor_range, env_free
        )
        discovered = update_discovered(discovered, scan)

        # -------- Coverage (phủ thực sự) --------
        sweep = LineString([path[-1], intermediate]).buffer(cover_radius)
        sweep = sweep.intersection(env_free)
        covered = sweep if covered is None else covered.union(sweep)

        path.append(intermediate)

    return path, discovered, covered


def C_star_coverage(
    env_poly, obstacles, start_pos,
    sensor_range=5.0, cover_radius=1.0,
    d_s=2.0, max_iters=3000, verbose=False
):
    # -------------------------------------------------
    # 0. Free space
    # -------------------------------------------------
    env_free = env_poly
    for obs in obstacles:
        env_free = env_free.difference(obs)

    # -------------------------------------------------
    # 1. Initial discovery & coverage
    # -------------------------------------------------
    discovered = update_discovered(
        None,
        sense_environment(start_pos, start_pos, sensor_range, env_free)
    )
    discovered_prev = discovered

    covered = None   # <<< NEW

    sampling_front = get_sampling_front(None, discovered)
    if sampling_front.is_empty:
        sampling_front = env_free.difference(discovered)

    laps, samples = generate_laps_and_samples(sampling_front, start_pos, d_s)

    # -------------------------------------------------
    # 2. Init RCG
    # -------------------------------------------------
    G = nx.Graph()
    build_RCG(G, samples, obstacles, d_s)
    prune_RCG(G)

    # -------------------------------------------------
    # 3. Init start node
    # -------------------------------------------------
    current_node = (round(start_pos[0], 6), round(start_pos[1], 6))
    if current_node not in G:
        G.add_node(
            current_node,
            pos=current_node,
            lap_id=None,
            is_end=False,
            near_frontier=True,
            essential=True,
            state='open',
            visited=False
        )

    path = [start_pos]
    prev_node = None
    iter_count = 1

    # Lap stall tracking
    lap_stall_count = 0
    STALL_LIMIT = 3

    # =================================================
    # MAIN LOOP
    # =================================================
    while iter_count < max_iters:

        if verbose:
            print(
                f"Iteration {iter_count}, "
                f"current node: {current_node}, "
                f"nodes: {len(G.nodes)}, edges: {len(G.edges)}"
            )

        goal_node, is_retreat = select_goal_node(
            G, current_node, prev_node, d_s
        )

        # -------------------------------------------------
        # Coverage completion (USE COVERED, NOT DISCOVERED)
        # -------------------------------------------------
        if goal_node is None:
            sampling_front = get_sampling_front(discovered_prev, discovered)

            if covered is None:
                uncovered = env_free
            else:
                uncovered = env_free.difference(covered)

            if sampling_front.is_empty and uncovered.area < cover_radius**2:
                if verbose:
                    print("Coverage complete.")
                break
            else:
                open_nodes = [
                    n for n, d in G.nodes(data=True)
                    if d.get('state') == 'open' and n != prev_node
                ]
                if not open_nodes:
                    break
                goal_node = open_nodes[0]
                is_retreat = True

        # -------------------------------------------------
        # Mark current node closed & visited
        # -------------------------------------------------
        if current_node in G:
            G.nodes[current_node]['state'] = 'closed'
            G.nodes[current_node]['visited'] = True

        prev_node_backup = current_node

        # -------------------------------------------------
        # Retreat
        # -------------------------------------------------
        if is_retreat:
            try:
                route = nx.shortest_path(G, current_node, goal_node)
            except nx.NetworkXNoPath:
                route = [current_node, goal_node]

            for n in route[1:]:
                if is_valid_edge(path[-1], n, obstacles):
                    path, discovered, covered = move_and_scan(
                        path, n,
                        sensor_range, cover_radius,
                        env_free,
                        discovered, covered
                    )
                else:
                    if n in G:
                        G.nodes[n]['state'] = 'closed'
                    break

            prev_node = prev_node_backup
            current_node = goal_node
            iter_count += 1
            continue

        # -------------------------------------------------
        # Normal move
        # -------------------------------------------------
        if is_valid_edge(path[-1], goal_node, obstacles):
            path, discovered, covered = move_and_scan(
                path, goal_node,
                sensor_range, cover_radius,
                env_free,
                discovered, covered
            )
            prev_node = prev_node_backup
            current_node = goal_node
            if current_node in G:
                G.nodes[current_node]['state'] = 'closed'
        else:
            if goal_node in G:
                G.nodes[goal_node]['state'] = 'closed'
            iter_count += 1
            continue

        # -------------------------------------------------
        # Progressive sampling
        # -------------------------------------------------
        sampling_front = get_sampling_front(discovered_prev, discovered)
        if sampling_front.is_empty:
            sampling_front = env_free.difference(discovered)

        # Stall detection (still based on discovery)
        if abs(discovered.area - discovered_prev.area) < 1e-6:
            lap_stall_count += 1
        else:
            lap_stall_count = 0

        discovered_prev = discovered

        # Lap saturation handling
        if lap_stall_count >= STALL_LIMIT:
            curr_lap = G.nodes[current_node].get('lap_id')
            if verbose:
                print(f"Lap {curr_lap} saturated, closing it.")

            for n in G.nodes:
                if G.nodes[n].get('lap_id') == curr_lap:
                    G.nodes[n]['state'] = 'closed'

            lap_stall_count = 0
            iter_count += 1
            continue

        laps, samples = generate_laps_and_samples(
            sampling_front, current_node, d_s
        )

        build_RCG(G, samples, obstacles, d_s)
        prune_RCG(G)

        # Only open NEW nodes
        for s in samples:
            coord = s['pos']
            if coord in G and not G.nodes[coord].get('visited', False):
                G.nodes[coord]['state'] = 'open'

        # -------------------------------------------------
        # Cycle guard
        # -------------------------------------------------
        if len(path) >= 6 and path[-1] in path[-6:-2]:
            if verbose:
                print("Cycle detected, terminating.")
            break

        iter_count += 1

    # =================================================
    # Coverage hole handling (TSP, still updates covered)
    # =================================================
    open_nodes = [
        n for n, d in G.nodes(data=True)
        if d.get('state') == 'open'
    ]

    if open_nodes:
        tsp_path = solve_tsp(open_nodes, start=current_node)
        for n in tsp_path:
            if is_valid_edge(path[-1], n, obstacles):
                path, discovered, covered = move_and_scan(
                    path, n,
                    sensor_range, cover_radius,
                    env_free,
                    discovered, covered
                )
                if n in G:
                    G.nodes[n]['state'] = 'closed'

    return path, G, env_free, covered





def load_case_from_json(file_path):
    with open(file_path, 'r') as f:
        data = json.load(f)

    env = Polygon(data['environment'])
    obstacles = [Polygon(coords) for coords in data.get('obstacles', [])]
    start = tuple(data['start'])

    return env, obstacles, start

if __name__ == "__main__":
    env, obstacles, start = load_case_from_json("testcase_1.json")
    path, G, env_free, covered = C_star_coverage(env, obstacles, start_pos=start,
                                       sensor_range=5.0, cover_radius=1.5, d_s=5.0,
                                       verbose=True)
    # ============================
    # Coverage metrics
    # ============================
    if covered is None:
        coverage_ratio = 0.0
    else:
        coverage_ratio = covered.area / env_free.area

    print("Finished coverage. Path length:", len(path))
    print(f"Coverage ratio: {coverage_ratio*100:.2f}%")

    # ============================
    # Visualization
    # ============================
    fig, ax = plt.subplots(figsize=(6,6))

    # Environment boundary
    x, y = env.exterior.xy
    ax.plot(x, y, color='black')

    # Covered region (REAL coverage footprint)
    if covered is not None and not covered.is_empty:
        if covered.geom_type == 'Polygon':
            cx, cy = covered.exterior.xy
            ax.fill(cx, cy, color='cyan', alpha=0.35)
        else:  # MultiPolygon
            for g in covered.geoms:
                cx, cy = g.exterior.xy
                ax.fill(cx, cy, color='cyan', alpha=0.35)

    # Obstacles
    for obs in obstacles:
        ox, oy = obs.exterior.xy
        ax.fill(ox, oy, color='gray', alpha=0.7)

    # RCG nodes & edges
    nx_nodes = list(G.nodes())
    nx_x = [n[0] for n in nx_nodes]
    nx_y = [n[1] for n in nx_nodes]
    ax.scatter(nx_x, nx_y, color='red', s=15)

    for u, v in G.edges():
        ax.plot([u[0], v[0]], [u[1], v[1]], color='red', linewidth=0.5)

    # Coverage path
    for i in range(1, len(path)):
        x_prev, y_prev = path[i - 1]
        x_curr, y_curr = path[i]
        ax.plot([x_prev, x_curr], [y_prev, y_curr],
                color='blue', marker='o', markersize=3)

    plt.title("Final Coverage Path (blue), Covered Area (cyan), RCG (red)")
    ax.set_xlim(-5, 55)
    ax.set_ylim(-5, 55)
    plt.show()

