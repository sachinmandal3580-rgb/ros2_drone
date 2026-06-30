"""This file implements a PID and PI controll class"""


class PI:
    def __init__(self, kp, ki, min_out, max_out):
        """
        :param kp: Proportional gain
        :param ki: Integral gain
        :param min_out: Minimum output
        :param max_out: Maximum output
        """
        self.kp = kp
        self.ki = ki
        self.min_out = min_out
        self.max_out = max_out
        self.integral = 0

    def compute(self, error, dt):
        # ==========================================================
        # TODO 1
        #
        # Compute the PI control output for this update step.
        #
        # Requirements:
        # - Accumulate the integral term using the current error
        #   and timestep (dt).
        # - Compute output as the weighted sum of the proportional
        #   and integral terms, using kp and ki.
        # - Clamp the output between min_out and max_out before
        #   returning it.
        #
        # Hint:
        # Use:
        #   • self.integral
        #   • self.kp, self.ki
        #   • self.min_out, self.max_out
        # ==========================================================

        # YOUR CODE HERE


class PID:
    def __init__(self, kp, ki, kd, min_out, max_out):
        """
        :param kp: Proportional gain
        :param ki: Integral gain
        :param kd: Derivative gain
        :param min_out: Minimum output
        :param max_out: Maximum output
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_out = min_out
        self.max_out = max_out
        self.integral = 0
        self.last_error = 0

    def compute(self, error, dt):
        # ==========================================================
        # TODO 2
        #
        # Compute the PID control output for this update step.
        #
        # Requirements:
        # - Accumulate the integral term using the current error
        #   and timestep (dt).
        # - Compute the derivative term from the change in error
        #   over dt, then update self.last_error for next call.
        # - Compute output as the weighted sum of the proportional,
        #   integral, and derivative terms, using kp, ki, and kd.
        # - Clamp the output between min_out and max_out before
        #   returning it.
        #
        # Hint:
        # Use:
        #   • self.integral, self.last_error
        #   • self.kp, self.ki, self.kd
        #   • self.min_out, self.max_out
        # ==========================================================

        # YOUR CODE HERE