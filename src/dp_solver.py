import argparse
import json
import pickle
import math
from collections import defaultdict

import numpy as np
import networkx as nx

from world_model import WorldModel
from discretization import GraphBuilder
from dijkstra_planner import compute_cruise_path
from distributions import Distributions
from utils import is_visible, is_detected


def advance_target_states(cruise_path, step_index, p_sus):
    """
    Given the cruise_path list of (x,y,h,sus,i) states, produce next possible target states with probabilities.
    """
    x, y, theta, sus, i = cruise_path[step_index]
    # if at final step, remain
    if step_index + 1 >= len(cruise_path):
        return [(1.0, cruise_path[step_index])]
    # normal mode
    if sus == 0:
        next_node = cruise_path[step_index + 1][:3]  # x,y,theta
        normal = (*next_node, 0, 0)
        scan = (x, y, theta, 1, 1)
        return [(1 - p_sus, normal), (p_sus, scan)]
    # suspicious scan modes
    prev_theta = theta
    if i == 1:
        return [(1.0, (x, y, prev_theta + math.pi/2, 1, 2))]
    if i == 2:
        return [(1.0, (x, y, prev_theta - math.pi/2, 1, 3))]
    if i == 3:
        return [(1.0, (x, y, prev_theta, 0, 0))]
    return [(1.0, cruise_path[step_index])]


def solve_dp_with_path(G, world, cruise_path, r, gamma, r_d, K):
    # Precompute successor map
    successors = {node: list(G.successors(node)) for node in G.nodes()}

    # Backward reachable observer states
    H = [set() for _ in range(K+1)]
    H[K] = set(G.nodes())
    for k in range(K-1, -1, -1):
        H[k] = {s for s in G.nodes() if any(succ in H[k+1] for succ in successors[s])}

    # DP tables
    V_next = {}
    policy = {}

    # Iterate backwards in time
    for k in reversed(range(K)):
        V_curr = {}
        for s_o in H[k]:
            for sus in (0,1):
                for i in ([0] if sus==0 else [1,2,3]):
                    # target state at time k
                    x_t, y_t, theta_t, _, _ = cruise_path[k]
                    best_val = -math.inf
                    best_action = None
                    for s_o_next in successors[s_o]:
                        # compute reward
                        vis = is_visible((s_o[0],s_o[1]), s_o[2], (x_t,y_t), world, r, gamma)
                        det = is_detected((x_t,y_t), theta_t, (s_o[0],s_o[1]), world, r_d, r, gamma)
                        reward = 1 if (vis and not det) else 0
                        exp_future = 0.0
                        # sum over target transitions
                        for prob, s_t_next in advance_target_states(cruise_path, k, dist.p_sus):
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
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--num_obs', type=int, default=5)
    parser.add_argument('--v', type=float, default=0.1)
    parser.add_argument('--tau', type=float, default=1.0)
    parser.add_argument('--r', type=float, default=5.0)
    parser.add_argument('--gamma', type=float, default=math.pi/4)
    parser.add_argument('--r_d', type=float, default=1.0)
    parser.add_argument('--p_sus', type=float, default=0.1)
    parser.add_argument('--K', type=int, default=20)
    args = parser.parse_args()

    # Generate obstacles
    boundary = [[-1,-1],[1,-1],[1,1],[-1,1]]
    obs_list = generate_obstacles(args.seed, args.num_obs, boundary)
    with open('obstacles.json','w') as f:
        json.dump({'boundary': boundary, 'obstacles': obs_list}, f)

    # Build world and graph
    world = WorldModel('obstacles.json')
    gb = GraphBuilder(world, args.v, args.tau)
    G = gb.build()

    # Compute cruise path
    nodes = list(G.nodes())
    start, goal = nodes[0], nodes[-1]
    cruise_nodes = compute_cruise_path(G, start, goal)

    # Sample suspicious flags
    dist = Distributions(args.p_sus)
    cruise_path = []
    for idx, node in enumerate(cruise_nodes):
        sus_flag = dist.sample_suspicious()
        cruise_path.append((*node, sus_flag, 0))

    # Solve DP
    policy = solve_dp_with_path(G, world, cruise_path, args.r, args.gamma, args.r_d, args.K)
    with open('policy.pkl','wb') as f:
        pickle.dump(policy, f)
    print(f"Policy saved to policy.pkl")