""" @package altitude_control
Controllers for altitude regulation of a quadcopter.

This package is developed for use with the motion capture system in the Faculty
of Engineering at the University of Auckland.

@author Jeremie Xavier Joseph Bannwarth
@version 0.1
@date 06/2015
"""

import math
import time

class altitude_controller:
    """ Control the height of a quadcopter.

    Compute the throttle sent to the quadcopter based on the height error.

    The output of the controller at time \f$k\f$, \f$U_k\f$, is obtained by:
    \f[
        U_k = U_\text{feedforward} + \text{PID}_k
    \f]
    where \f$U_\text{feedforward}\f$ is a constant thrust value to counteract
    gravity and \f$\text{PID}_k\f$ is a discrete time PID controller.

    The output \f$U_k\f$ of the controller is a normalised value in the range
    \f$[0,1000]\f$, where 0 represents no thrust and 1000 represents full thrust.
    @warning This controller hasn't been tested.
    """

    # --------------------------------------------------------------------------
    ## Sampling time (in seconds).
    t_s = 0.0225
    ## Target height (in cm).
    height_setpoint = 0.0
    ## Number of past error values to remember.
    past_samples = 2
    ## Current error (in cm) between setpoint and measured heights.
    error = 0.0
    ## Past errors (in cm) between setpoint and measured heights.
    error_past = [0.0] * past_samples
    ## Output of the controller (normalised in range [0,1000]).
    output = 0.0
    ## Feedforward term to counteract gravity.
    output_ff = 500.0
    ## Current output of the PID controller.
    output_pid = 0.0
    ## Past outputs of the controller.
    output_pid_past = [0.0] * past_samples

    # Controller gains
    ## Proportional gain.
    k_p = 1.0
    ## Integral gain.
    k_i = 0.0
    ## derivative gain.
    k_d = 0.0
    # --------------------------------------------------------------------------

    def __init__(self,
                 t_s):
        """ The constructor.
        @param self The object pointer.
        @param t_s The controller sampling time (in seconds).
        """
        self.t_s = t_s

    def set_height_setpoint(self,
                            height_setpoint):
        """ Change the height setpoint used by the controller.
        @param self The object pointer.
        @param height_setpoint The new height setpoint (in cm).
        """
        self.height_setpoint = height_setpoint

    def update_error(self,
                     new_error):
        """ Store previous error and store current error.
        @param self The object pointer.
        @param new_error The difference between the height measured and setpoint.
        """
        self.error_past = [self.error] + self.error_past[1:]
        self.error = new_error

    def update_pid_controller(self):
        """ Apply discrete time PID controller to regulate the height error.

        Output obtained using:
        \f[
            \text{PID}_k =
            \text{PID}_{k-1} + K_p ( e_k - e_{k-1} )
            + K_i T_s e_k
            + \frac{K_d}{T_s} ( e_k - 2 e_{k-1} + e_{k-2} )
        \f]
        where \f$\text{PID}_k\f$ is the output of the controller at time \f$k\f$.
        @param self The object pointer.
        """
        output_pid = ( output_past[1] +
                k_p * (error - error_past[1]) +
                k_i * t_s * error +
                k_d * (error - 2.0*error_past[1] + error_past[2]) / t_s)

        self.output_pid_past = [output_pid] + self.output_pid_past[1:]
        self.output_pid = output_pid

    def update(self,
               height_measured):
        """ Update error and compute the controller output.

        @param self The object pointer.
        @param height_measured The new height measured (in cm).
        """
        self.update_error(self.height_setpoint - height_measured)
        self.update_pid_controller()

        self.output = self.output_pid + self.output_ff

        # Limit output in the [0,1000] range
        self.output = max(min(1000, self.output), 0)

    def get_output(self):
        """ Return the controller output.
        @param self The object pointer.
        @return The controller output (in unit).
        """
        return self.output
