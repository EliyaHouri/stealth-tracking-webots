import json
import random
from shapely.geometry import Polygon, Point

def generate_obstacles(seed: int, n: int, boundary: list):
    random.seed(seed)
    obstacles = []
    for _ in range(n):
        # generate a small random rectangle inside boundary
        x0, y0 = random.uniform(boundary[0][0], boundary[2][0]), 
random.uniform(boundary[0][1], boundary[2][1])
        w, h = random.uniform(0.1, 0.3), random.uniform(0.1, 0.3)
        rect = [(x0, y0), (x0+w, y0), (x0+w, y0+h), (x0, y0+h)]
        obstacles.append(rect)
    return obstacles

if __name__ == '__main__':
    # Example usage
    boundary = [[-1,-1], [1,-1], [1,1], [-1,1]]
    obstacles = generate_obstacles(seed=42, n=5, boundary=boundary)
    data = {'boundary': boundary, 'obstacles': obstacles}
    with open('obstacles.json','w') as f:
        json.dump(data, f, indent=2)
    print('Generated obstacles.json')
