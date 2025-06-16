import argparse
import json
import pickle
import math
from collections import defaultdict

import numpy as np
import networkx as nx

from world_model import WorldModel
from discretization import GraphBuilder
from distributions import Distributions
from utils import is_visible, is_detected


def advance_target(state, cruise_path, k, p_sus):
    """
    Given target state (node, sus, scan_i), return list of (prob, 
next_state) tuples.
    state: (x, y, theta, sus, i)
    cruise_path: list of (x,y,theta)
    """
    x, y, theta, sus, i = state
    if sus == 0:
        # normal cruising
        next_cruise = cruise_path[k+1]
        # stay normal
        normal = (*next_cruise, 0, 0)
        # enter suspicious scan
        scan = (x, y, theta, 1, 1)
        return [(1 - p_sus, normal), (p_sus, scan)]
    else:
        # in scan mode: three in-place turns
        prev_heading = cruise_path[k][2]
        if i == 1:
            new_theta = prev_heading + math.pi/2
            return [(1.0, (x, y, new_theta, 1, 2))]
        if i == 2:
            new_theta = prev_heading - math.pi/2
            return [(1.0, (x, y, new_theta, 1, 3))]
        if i == 3:
            new_theta = prev_heading + math.pi
            # after scan return to normal at next time-step
            return [(1.0, (x, y, prev_heading, 0, 0))]
    # should not reach
    return []


def solve_dp(obstacles_file, cruise_file, output_file, v, tau, r, gamma, 
r_d, p_sus, K):
    # load world
    world = WorldModel(obstacles_file)
    # build graph
    gb = GraphBuilder(world, v, tau)
    G = gb.build()
    nodes = list(G.nodes())
    # load cruise path
    cruise_data = json.load(open(cruise_file))
    cruise_path = cruise_data['path']  # list of [x,y,theta]

    # backward pruning sets
    H = [set() for _ in range(K+2)]
    H[K] = set(nodes)
    # successors
    succ = {n: list(G.successors(n)) for n in nodes}

    # DP value tables
    V_next = {}  # mapping (s_o, tgt_state) -> value

    # initialize V_{K+1} = 0
    # iterate backward
    for k in range(K, -1, -1):
        # compute reachable observer nodes
        if k < K:
            H[k] = {s for s,ss in succ.items() if any(succ_node in H[k+1] 
for succ_node in ss)}
        # current DP table
        V_curr = {}
        # for each possible observer state
        for s_o in H[k]:
            for sus in (0, 1):
                for i in (0,1,2,3) if sus else (0,):
                    # determine target state at step k
                    x_t, y_t, theta_t = cruise_path[k]
                    s_t = (x_t, y_t, theta_t, sus, i)
                    # evaluate best action
                    best_val = -np.inf
                    best_action = None
                    for s_o_next in succ[s_o]:
                        # reward at time k
                        rwd = int(is_visible((s_o[0], s_o[1]), s_o[2], 
(x_t, y_t), world, r, gamma)
                                  and not is_detected((x_t, y_t), theta_t,
                                                      (s_o[0], s_o[1]), 
world, r_d, r, gamma))
                        # sum over target transitions
                        exp_future = 0.0
                        for prob, s_t_next in advance_target(s_t, 
cruise_path, k, p_sus):
                            key_next = (s_o_next, *s_t_next)
                            v_next = V_next.get(key_next, 0.0)
                            exp_future += prob * v_next
                        val = rwd + exp_future
                        if val > best_val:
                            best_val = val
                            best_action = s_o_next
                    # store
                    key = (k, s_o, *s_t)
                    V_curr[key] = best_val
                    # policy: map z -> best observer successor
                    policy[key] = best_action
        V_next = V_curr
    # save policy
    with open(output_file, 'wb') as f:
        pickle.dump(policy, f)
    print(f"Policy saved to {output_file}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--obstacles', required=True)
    parser.add_argument('--path', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--v', type=float, default=0.1)
    parser.add_argument('--tau', type=float, default=1.0)
    parser.add_argument('--r', type=float, default=5.0)
    parser.add_argument('--gamma', type=float, default=math.pi/4)
    parser.add_argument('--r_d', type=float, default=1.0)
    parser.add_argument('--p_sus', type=float, default=0.1)
    parser.add_argument('--K', type=int, default=20)
    args = parser.parse_args()

    solve_dp(
        args.obstacles,
        args.path,
        args.output,
        args.v,
        args.tau,
        args.r,
        args.gamma,
        args.r_d,
        args.p_sus,
        args.K
    )
