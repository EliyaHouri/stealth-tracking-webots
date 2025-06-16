from controller import Robot
import numpy as np
import sys

# Parameters (must match DP assumptions)
TAU = 64  # ms, basicTimeStep
P_SUS = 0.1  # suspicious probability

class TargetController(Robot):
    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())
        # Define a simple straight-line path
        self.graph_path = [(-0.5,0,0), (0.0,0,0), (0.5,0,0)]
        self.index = 0
        self.sus = False
        self.scan_stage = 0
        self.prev_heading = 0

    def run(self):
        left = self.getDevice('left wheel motor')
        right = self.getDevice('right wheel motor')
        left.setPosition(float('inf'))
        right.setPosition(float('inf'))

        while self.step(self.timestep) != -1:
            # normal cruising
            if not self.sus:
                # advance along path
                # (omitted: code to move to next graph node)
                if np.random.rand() < P_SUS:
                    self.sus = True
                    self.scan_stage = 1
                    self.prev_heading = self.get_device_heading()
            else:
                # suspicious scan: three in-place turns
                # (omitted: code for in-place turns)
                if self.scan_stage > 3:
                    self.sus = False
                    self.scan_stage = 0
            # simple drive forward
            left.setVelocity(3.0)
            right.setVelocity(3.0)

    def get_device_heading(self):
        # placeholder: read robot heading
        return 0.0

if __name__ == '__main__':
    controller = TargetController()
    controller.run()
