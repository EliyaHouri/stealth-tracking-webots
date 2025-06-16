import numpy as np

class Distributions:
    def __init__(self, p_sus: float):
        self.p_sus = p_sus

    def sample_suspicious(self) -> bool:
        return np.random.rand() < self.p_sus
