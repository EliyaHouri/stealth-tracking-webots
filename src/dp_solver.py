# src/dp_solver.py

import argparse
import json
import pickle
import math

import networkx as nx

from world_generator import generate_obstacles
from world_model import WorldModel
from discretization import GraphBuilder
from dijkstra_planner import compute_cruise_path
from distributions import Distributions
from utils import is_visible, is_detected


def advance_target_states(cruise_path, step_index, p_sus):
    """
    Given cruise_path list of (x,y,theta,sus,i), return list of (prob, next_state).
    """
    x, y, theta, sus, i = cruise_path[step_index]

    # if at final step, remain in place
    if step_index + 1 >= len(cruise_path):
        return [(1.0, cruise_path[step_index])]

    # normal cruising: either move or switch to scan
    if sus == 0:
        nx_, ny_, ntheta, _, _ = cruise_path[step_index + 1]
        normal = (nx_, ny_, ntheta, 0, 0)
        scan   = (x, y, theta, 1, 1)
        return [(1 - p_sus, normal), (p_sus, scan)]

    # scan sequence: three fixed turns then back to normal
    if i == 1:
        return [(1.0, (x, y, theta + math.pi/2, 1, 2))]
    if i == 2:
        return [(1.0, (x, y, theta - math.pi/2, 1, 3))]
    if i == 3:
        return [(1.0, (x, y, theta, 0, 0))]

    # fallback
    return [(1.0, cruise_path[step_index])]


def solve_dp_with_path(G, world, cruise_path, r, gamma, r_d, K, p_sus):
    """
    Finite-horizon DP solver given a cruise_path.
    Returns: policy dict mapping (k, s_o, x_t, y_t, theta_t, sus, i) -> best s_o_next.
    """
    # precompute all successors in the directed graph
    successors = {s: list(G.successors(s)) for s in G.nodes()}

    # backward‑reachable observer states H[k]
    H = [set() for _ in range(K+1)]
    H[K] = set(G.nodes())
    for k in range(K-1, -1, -1):
        H[k] = {s for s in G.nodes() if any(succ in H[k+1] for succ in successors[s])}

    V_next = {}   # V_{k+1}(·)
    policy = {}   # store best action

    # backward induction
    for k in reversed(range(K)):
        V_curr = {}
        for s_o in H[k]:
            for sus in (0, 1):
                for i in ([0] if sus == 0 else [1,2,3]):
                    x_t, y_t, theta_t, _, _ = cruise_path[k]
                    best_val = -math.inf
                    best_action = None

                    for s_o_next in successors[s_o]:
                        # immediate reward
                        vis = is_visible((s_o[0], s_o[1]), s_o[2], (x_t, y_t), world, r, gamma)
                        det = is_detected((x_t, y_t), theta_t, (s_o[0], s_o[1]),
                                          world, r_d, r, gamma)
                        reward = 1 if (vis and not det) else 0

                        # expected value of next step
                        exp_future = 0.0
                        for prob, s_t_next in advance_target_states(cruise_path, k, p_sus):
                            key_next = (k+1, s_o_next, *s_t_next)
                            exp_future += prob * V_next.get(key_next, 0.0)

                        val = reward + exp_future
                        if val > best_val:
                            best_val = val
                            best_action = s_o_next

                    key = (k, s_o, x_t, y_t, theta_t, sus, i)
                    V_curr[key] = best_val
                    policy[key] = best_action

        V_next = V_curr

    return policy


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--seed',   type=int,   required=True)
    parser.add_argument('--num_obs',type=int,   default=1)
    parser.add_argument('--v',      type=float, default=0.1)
    parser.add_argument('--tau',    type=float, default=1.0)
    parser.add_argument('--r',      type=float, default=5.0)
    parser.add_argument('--gamma',  type=float, default=math.pi/4)
    parser.add_argument('--r_d',    type=float, default=1.0)
    parser.add_argument('--p_sus',  type=float, default=0.1)
    parser.add_argument('--K',      type=int,   default=20)
    args = parser.parse_args()

    # 1. Create a 10×10 continuous world with one obstacle
    boundary = [(0,0), (10,0), (10,10), (0,10)]
    obs_list = generate_obstacles(args.seed, args.num_obs, boundary)
    with open('obstacles.json','w') as f:
        json.dump({'boundary': boundary, 'obstacles': obs_list}, f)

    # 2. Build the world model and discretization graph
    world = WorldModel('obstacles.json')
    gb = GraphBuilder(world, args.v, args.tau)
    G  = gb.build()

    # 3. Pick start/goal from existing heading-0 nodes to guarantee connectivity
    nodes_heading0 = [n for n in G.nodes() if n[2] == 0]
    if len(nodes_heading0) < 2:
        raise ValueError("Not enough heading-0 nodes in the graph to pick start and goal.")
    nodes_heading0.sort(key=lambda n: (n[0], n[1]))
    start = nodes_heading0[0]
    goal  = nodes_heading0[-1]
    cruise_nodes = compute_cruise_path(G, start, goal)
    print(f"Planning from {start} to {goal} over {len(cruise_nodes)} steps")

    # 4. Sample the same Bernoulli for each step
    dist = Distributions(args.p_sus)
    cruise_path = [(*node, dist.sample_suspicious(), 0) for node in cruise_nodes]

    # 5. Solve DP and save policy
    policy = solve_dp_with_path(
        G, world, cruise_path,
        args.r, args.gamma, args.r_d,
        args.K, args.p_sus
    )
    with open('policy.pkl','wb') as f:
        pickle.dump(policy, f)
    print("Policy saved to policy.pkl")
    print("Policy size:", len(policy))

    # Optional: save graph and world model for inspection
    with open('graph.pkl', 'wb') as f:
        pickle.dump(G, f)
    with open('world_model.pkl', 'wb') as f:
        pickle.dump(world, f)
    with open('cruise_path.pkl', 'wb') as f:
        pickle.dump(cruise_path, f)

    print("DP solver completed successfully.")
