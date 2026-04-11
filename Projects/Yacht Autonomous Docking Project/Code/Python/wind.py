import numpy as np

class WindField:
    def __init__(self, dt,
                 mean_speed=3.0,
                 mean_dir=0.0,
                 gust_sigma=0.5,
                 tau=10.0):
        """
        mean_dir in radians (earth frame, Unity X/Z)
        dt in seconds
        """
        self.dt = dt
        self.tau = tau
        self.gust_sigma = gust_sigma

        self.mean_Vx = mean_speed * np.cos(mean_dir)
        self.mean_Vy = mean_speed * np.sin(mean_dir)

        self.Vx = self.mean_Vx
        self.Vy = self.mean_Vy

    def update(self):
        # Ornstein-Uhlenbeck gust model
        dVx = -(self.Vx - self.mean_Vx)/self.tau * self.dt \
              + self.gust_sigma * np.sqrt(self.dt) * np.random.randn()
        dVy = -(self.Vy - self.mean_Vy)/self.tau * self.dt \
              + self.gust_sigma * np.sqrt(self.dt) * np.random.randn()

        self.Vx += dVx
        self.Vy += dVy

        return self.Vx, self.Vy

    def apply_drift(self, state):
        """
        Adds wind drift in earth frame.
        Unity convention:
        x → +X
        y → +Z
        """
        state["x"] += self.Vx * self.dt
        state["y"] += self.Vy * self.dt
        return state