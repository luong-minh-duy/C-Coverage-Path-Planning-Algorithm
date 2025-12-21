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

def move_and_scan(path, target, sensor_range, env_free, discovered, step_size=1.0):
    current = path[-1]
    line = LineString([current, target])
    num_steps = max(int(line.length // step_size), 1)
    for i in range(1, num_steps + 1):
        x = current[0] + (target[0] - current[0]) * i / num_steps
        y = current[1] + (target[1] - current[1]) * i / num_steps
        intermediate = (x, y)
        scan = sense_environment(path[-1], intermediate, sensor_range, env_free)
        discovered = update_discovered(discovered, scan)
        path.append(intermediate)
    return path, discovered

def C_star_coverage(env_poly, obstacles, start_pos, sensor_range=5.0, cover_radius=1.0, d_s=2.0, max_iters=1000, verbose=False):
    env_free = env_poly
    for obs in obstacles:
        env_free = env_free.difference(obs)

    discovered = update_discovered(None, sense_environment(start_pos, start_pos, sensor_range, env_free))
    discovered_prev = discovered

    sampling_front = get_sampling_front(None, discovered)
    if sampling_front.is_empty:
        sampling_front = env_free.difference(discovered)
    laps, samples = generate_laps_and_samples(sampling_front, start_pos, d_s)

    G = nx.Graph()
    build_RCG(G, samples, obstacles, d_s)
    prune_RCG(G)

    current_node = (round(start_pos[0],6), round(start_pos[1],6))
    if current_node not in G:
        G.add_node(current_node, pos=current_node, state='open')

    path = [start_pos]
    iter_count = 1

    while iter_count < max_iters:
        if verbose:
            print(f"Iteration {iter_count}, current node: {current_node}, nodes: {len(G.nodes)}, edges: {len(G.edges)}")
        goal_node, is_retreat = select_goal_node(G, current_node, d_s)
        if goal_node is None:
            if verbose:
                print("No goal found, coverage complete.")
            break

        if current_node in G.nodes:
            G.nodes[current_node]['state'] = 'closed'

        if is_retreat:
            try:
                route = nx.shortest_path(G, source=current_node, target=goal_node)
            except nx.NetworkXNoPath:
                route = [current_node, goal_node]

            if verbose:
                print(f"Dead-end: retreat to {goal_node} via {route}")

            for n in route[1:]:
                pos_next = (n[0], n[1])
                if is_valid_edge(path[-1], pos_next, obstacles):
                    path, discovered = move_and_scan(path, pos_next, sensor_range, env_free, discovered)
                else:
                    if verbose:
                        print(f"Retreat blocked from {path[-1]} to {pos_next}, marking as closed")
                    if n in G.nodes:
                        G.nodes[n]['state'] = 'closed'
                    break
            current_node = goal_node
            continue

        pos_next = goal_node
        if is_valid_edge(path[-1], pos_next, obstacles):
            path, discovered = move_and_scan(path, pos_next, sensor_range, env_free, discovered)
            current_node = goal_node
            if current_node in G.nodes:
                G.nodes[current_node]['state'] = 'closed'
        else:
            if verbose:
                print(f"Move blocked from {path[-1]} to {pos_next}, marking as closed")
            if goal_node in G.nodes:
                G.nodes[goal_node]['state'] = 'closed'
            continue

        sampling_front = get_sampling_front(discovered_prev, discovered)
        if sampling_front.is_empty:
            if verbose:
                print("Sampling front empty, falling back to unexplored area")
            sampling_front = env_free.difference(discovered)
        discovered_prev = discovered
        laps, samples = generate_laps_and_samples(sampling_front, current_node, d_s)
        build_RCG(G, samples, obstacles, d_s)
        prune_RCG(G)
        for pt in samples:
            coord = (round(pt[0],6), round(pt[1],6))
            if coord in G.nodes:
                G.nodes[coord]['state'] = 'open'
        iter_count += 1

    open_nodes = [n for n,d in G.nodes(data=True) if d['state']=='open']
    if open_nodes:
        if verbose:
            print(f"Open nodes after main coverage: {open_nodes}")
        open_coords = [(n[0], n[1]) for n in open_nodes]
        tsp_path = solve_tsp(open_coords, start=current_node)
        if verbose:
            print(f"TSP hole path: {tsp_path}")
        for next_pt in tsp_path:
            if is_valid_edge(path[-1], next_pt, obstacles):
                path, discovered = move_and_scan(path, next_pt, sensor_range, env_free, discovered)
                coord = (round(next_pt[0],6), round(next_pt[1],6))
                if coord in G.nodes:
                    G.nodes[coord]['state'] = 'closed'
            else:
                if verbose:
                    print(f"TSP move blocked from {path[-1]} to {next_pt}, marking as closed")
                coord = (round(next_pt[0],6), round(next_pt[1],6))
                if coord in G.nodes:
                    G.nodes[coord]['state'] = 'closed'
    return path, G, env_free

def load_case_from_json(file_path):
    with open(file_path, 'r') as f:
        data = json.load(f)

    env = Polygon(data['environment'])
    obstacles = [Polygon(coords) for coords in data.get('obstacles', [])]
    start = tuple(data['start'])

    return env, obstacles, start

if __name__ == "__main__":
    env, obstacles, start = load_case_from_json("testcase_1.json")
    path, G, env_free = C_star_coverage(env, obstacles, start_pos=start,
                                       sensor_range=5.0, cover_radius=1.5, d_s=5.0,
                                       verbose=True)
    print("Finished coverage. Path length:", len(path))
    fig, ax = plt.subplots(figsize=(6,6))
    x, y = env.exterior.xy
    ax.plot(x, y, color='black')
    for obs in obstacles:
        ox, oy = obs.exterior.xy
        ax.fill(ox, oy, color='gray', alpha=0.7)

    nx_nodes = list(G.nodes())
    nx_x = [n[0] for n in nx_nodes]
    nx_y = [n[1] for n in nx_nodes]
    ax.scatter(nx_x, nx_y, color='red', s=15)
    for u, v in G.edges():
        ax.plot([u[0], v[0]], [u[1], v[1]], color='red', linewidth=0.5)

    for i in range(1, len(path)):
        x_prev, y_prev = path[i - 1]
        x_curr, y_curr = path[i]
        ax.plot([x_prev, x_curr], [y_prev, y_curr], color='blue', marker='o', markersize=3)
        coverage_circle = patches.Circle((x_curr, y_curr), radius=1.5, color='cyan', alpha=0.2)
        ax.add_patch(coverage_circle)
        ax.set_title(f"C* Coverage Step {i}/{len(path)}")
        plt.pause(0.05)

    plt.title("Final Coverage Path (blue) with RCG (red)")
    ax.set_xlim(-5, 55)
    ax.set_ylim(-5, 55)
    plt.show()
