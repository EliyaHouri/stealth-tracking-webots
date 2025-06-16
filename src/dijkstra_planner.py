import networkx as nx

def compute_cruise_path(G, start, goal):
    # find shortest path on nodes
    node_path = nx.shortest_path(G, source=start, target=goal)
    # node tuples are (x,y,h); ignore headings for path sequence
    # return list of (x,y,heading)
    return node_path
