from controller import Robot
import pickle
import math

class ObserverController(Robot):
    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())
        # load precomputed policy
        with open('policy.pkl', 'rb') as f:
            self.policy = pickle.load(f)

    def run(self):
        left = self.getDevice('left wheel motor')
        right = self.getDevice('right wheel motor')
        left.setPosition(float('inf'))
        right.setPosition(float('inf'))

        while self.step(self.timestep) != -1:
            # get current discrete state z = (s_o, s_t, sus, i)
            z = self.read_state()
            edge = self.policy.get(z)
            # convert edge to wheel velocities
            vl, vr = self.edge_to_wheels(edge)
            left.setVelocity(vl)
            right.setVelocity(vr)

    def read_state(self):
        # placeholder: map robot pose to nearest graph node + flags
        return None

    def edge_to_wheels(self, edge):
        # placeholder: turn/move->velocities
        return 3.0, 3.0

if __name__ == '__main__':
    controller = ObserverController()
    controller.run()
