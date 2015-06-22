""" @package test.altitude_control
Documentation.

More details.

@author Jeremie Xavier Joseph Bannwarth
"""

import math
import time

class altitude_controller:
    """ Control the height of a quadcopter.

    Compute the throttle sent to the quadcopter based on the height error.
    Use a PID controller:

    \f[
        T_k =
        T_{k-1} + K_c ( e_k - e_{k-1} )
        + \frac{K_c T_s}{T_i} e_k
        + \frac{K_c T_d}{T_s} ( e_k - 2 e_{k-1} + e_{k-2} )
    \f]
    """

    dt = 0.05

    height_setpoint = 0

    past_samples = 2;
    error = 0
    error_past = [0] * past_samples

    output = 0
    output_offset = 500
    output_pid = 0
    output_pid_past = [0] * past_samples

    def __init__(self, dt):
        """ The constructor.
        @param self The object pointer.
        @param dt The controller sampling time (in seconds).
        """
        self.dt = dt

    def set_height_setpoint(self, height_setpoint):
        """ Change the height setpoint used by the controller.
        @param self The object pointer.
        @param height_setpoint The new height setpoint (in cm).
        """
        self.height_setpoint = height_setpoint

    def update_error(self, new_error):
        """ Store previous error and store current error.
        @param self The object pointer.
        @param new_error The difference between the height measured and setpoint.
        """
        self.error_past = [self.error] + self.error_past[1:]
        self.error = new_error

    def update_pid_controller(self):
        """ Apply discrete time PID controller to regulate the height error
        @param self The object pointer.
        """
        output_pid = ( output_past[1] +
                k_c * (error - error_past[1]) +
                k_c * dt * error / t_i +
                k_c * t_d * (error - 2*error_past[1] + error_past[2]) )

        self.output_pid_past = [output_pid] + self.output_pid_past[1:]
        self.output_pid = output_pid

    def update(self, height_measured):
        """ Update error and compute the controller output.
        @param self The object pointer.
        @param height_measured The new height measured (in cm).
        """
        self.update_error(self.height_setpoint - height_measured)
        self.update_pid_controller()

        self.output = self.output_pid + self.output_offset

        # Limit output in the 0-1000 range
        self.output = max(min(1000, self.output), 0)

    def get_output(self):
        """ Return the controller output
        @self The object pointer.
        @return The controller output (in unit).
        """
        return self.output
