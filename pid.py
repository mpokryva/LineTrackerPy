class PIDController:
    def __init__(self, tau_p, tau_i, tau_d):
        self.error = 0
        self.prevError = 0
        self.errorSum = 0
        self.tau_p = tau_p
        self.tau_i = tau_i
        self.tau_d = tau_d

    # x and y dist are distances from some reference point.
    def pid(self, xDist, yDist, dt):
        # yDist doesn't get used.
        cte = xDist
        d_cte = (cte - self.prevError) / dt
        self.prevError = cte
        self.errorSum += (cte * dt)
        return - self.tau_p * cte - self.tau_d * d_cte \
                - self.tau_i * self.errorSum
