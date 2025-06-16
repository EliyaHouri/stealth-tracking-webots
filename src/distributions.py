import numpy as np

class Distributions:
    """Sampling distributions for the target."""
    def __init__(self, p_sus: float):
        self.p_sus = p_sus

    def sample_suspicious(self) -> bool:
        """Bernoulli trial: same distribution every run."""
        return np.random.rand() < self.p_sus