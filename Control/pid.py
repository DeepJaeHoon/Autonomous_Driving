class pidControl:
    def __init__(self):
        self.p_gain = 6
        self.i_gain = 2.2
        self.d_gain = 0
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.1
    
    def pid(self,target_vel, current_vel):
        error = target_vel - current_vel

        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error-self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output