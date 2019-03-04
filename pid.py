class PIDController:
    def __init__(self, p_gain, i_gain, d_gain, s_gain):
        self.error = 0
        self.prev_error = 0
        self.prev_vals = []
        self.error_sum = 0
        self.abs_error_sum = 0
        self.buffer_size = 10
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain
        self.s_gain = s_gain

    # x and y dist are distances from some reference point.
    def pid(self, x_dist, yDist, dt):
        # yDist doesn't get used.
        # cte = cross track error.
        cte = x_dist
        # Calculate derivate.
        d_cte = (cte - self.prev_error) / dt
        self.prev_error = cte
        if (len(self.prev_vals) == self.buffer_size):
            # buffer_size is how many past observations we should consider in I.
            # Get rid of LRU error.
            lru_error = self.prev_vals.pop()
            self.error_sum -= lru_error
            self.abs_error_sum -= abs(lru_error)
        # Calculate new I.
        self.error_sum += cte
        self.abs_error_sum += abs(cte)
        self.prev_vals.append(cte)
        # Apply gains and calculate final correction.
        return - self.p_gain * cte - self.d_gain * d_cte \
                - self.i_gain * self.error_sum \
                - self.s_gain * self.abs_error_sum

    def setGains(self, p_gain, i_gain, d_gain, s_gain):
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain
        self.s_gain = s_gain
