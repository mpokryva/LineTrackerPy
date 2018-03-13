class PIDController:
    def __init__(self, p_gain, i_gain, d_gain):
        self.error = 0
        self.prevError = 0
        self.errorSum = 0
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain

    # x and y dist are distances from some reference point.
    def pid(self, xDist, yDist, dt):
        # yDist doesn't get used.
        cte = xDist
        d_cte = (cte - self.prevError) / dt
        self.prevError = cte
        self.errorSum += (cte * dt)
        return - self.p_gain * cte - self.d_gain * d_cte \
                - self.i_gain * self.errorSum

    def setGains(self, p_gain, i_gain, d_gain):
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain
