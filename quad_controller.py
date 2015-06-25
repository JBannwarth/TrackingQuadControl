""" @package quad_controller
Run a controllers for arducopter quadcopters runnnig off Vicon position
measurments.

Require ROS for the interfaces between with the quadcopter and the tracking
software.

Credit to Usman Qayyum for providing help with the initial ros interfacing code.

This package is developed for use with the motion capture system in the Faculty
of Engineering at the University of Auckland.

@author Jay Mills
@author Jeremie Xavier Joseph Bannwarth
@version 0.1
@date 06/2015
"""

# ------------------------------------------------------------------------------
# Import commands
import os
import sys
import math
import roslib
# roslib.load_manifest('roscopter')
import rospy
import time
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TransformStamped
from threading import Thread
from altitude_control import altitude_controller
from PyQt4 import QtCore, QtGui
import ControlPannel
from rigid_body import RigidBody
from arduino_serial_interface import ardu_rc_serial_interface

# ------------------------------------------------------------------------------
# UI Setup
try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s
try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

# ------------------------------------------------------------------------------
class control_system(Thread):
    """ Run control algorithm in a separate thread.
    """
    ## Sampling rate of the controller (in ms).
    period = 0.0225

    altitude_ctrl = altitude_controller(period)

    #target position of the quadcopter being controlled, units:cm
    target_x = 0.0
    target_y = 0.0
    target_z = 100.0
    target_yaw = 0.0

    #land positions of quadcopter
    land_x = 0.0
    land_y = 0.0
    land_z = 0.0
    land_yaw = 0.0

    #take off and land commands
    takeoff_command = False
    land_command = False
    stabilize_command = False

    # pwm throttle settings and outputs

    throttle_ppm = 0
    min_throttle_ppm = 670
    mid_throttle_ppm = 1080
    min_throttle_ppm = 1465
    throttle_normalised = 0

    def __init__(self):
        """ Initialise the thread subclass.
        """
        Thread.__init__(self)

        #initialise log file
        self.log_file = open('logfile: '+time.strftime("%c"), 'w')
        self.log_file.write("Test\n")

    def run(self):
        """ Threaded function for this class.
        """
        try:
            print("Waiting")
            self.earliercmdtime = 0
            rospy.sleep(.1)

            # Check if a ROS core is operational
            while not rospy.is_shutdown():
                # Start loop

                # Ensure the required rate of operation is met and update the
                # stored time
                if float(time.time()) - self.period >= self.earliercmdtime:
                    self.earliercmdtime = time.time()

                    # Run the quad update loop iteration
                    quad1.run2()
                    # Call the main control algorithm
                    self.control_loop()

            rospy.spin()
        except rospy.ROSInterruptException:
            pass

    def control_loop(self):
        """ Run main control algorithm.
        """
        # Update the quad position, call update functions below for added rigid bodies
        quad1.update_pos_values()

        # Calculate the control errors, adjust for required errors
        # self.calc_error(quad1.pos_x, quad1.pos_y, quad1.pos_z, quad1.yaw,
        #         self.target_x, self.target_y, self.target_z, self.target_yaw)

        self.altitude_ctrl.set_height_setpoint(target_z)
        self.altitude_ctrl.update(quad1.pos_z)
        throttle_normalised = altitude_ctrl.get_output()

        # checks if the prpogram is allowed to control the quadcopter
        # if quad1.have_control:
        #
        #     if self.takeoff_command:
        #
        #         # resets controller after the quadcopter passes the altitude target
        #         if quad1.pos_z > self.target_z:
        #             self.altitude_control.hover_throttle += self.altitude_control.accel_integral_z
        #             self.altitude_control.reset_controller_z()
        #             self.altitude_control.accel_integral_max_z = 100
        #             self.stabilize_command = True
        #             self.takeoff_command = False
        #
        #         # calls experimental altitude controller, units:cm
        #         self.throttle_control_setting = self.altitude_control.update_z_controller(quad1.pos_z, quad1.old_pos_z,
        #                                                                                   controller.target_z)
        #
        #     elif self.land_command:
        #         if ((quad1.pos_x < self.land_x + 10) and (quad1.pos_x > self.land_x - 10) and
        #                 (quad1.pos_y < self.land_y + 10) and (quad1.pos_y > self.land_y - 10)  and
        #                 (quad1.yaw < self.land_yaw + 5) and (quad1.yaw > self.land_yaw - 5)):
        #             self.target_x = self.land_x
        #             self.target_y = self.land_y
        #             self.target_z = self.land_z
        #             self.target_yaw = self.land_yaw
        #
        #             if quad1.pos_z < self.land_z + 5:
        #                 self.target_x = quad1.pos_x
        #                 self.target_y = quad1.pos_y
        #                 self.target_z = quad1.land_z
        #                 self.target_yaw = quad1.yaw
        #
        #         else:
        #             self.target_x = self.land_x
        #             self.target_y = self.land_y
        #             self.target_z = quad1.pos_z
        #             self.target_yaw = self.land_yaw
        #
        #         # calls experimental altitude controller, units:cm
        #         self.throttle_control_setting = self.altitude_control.update_z_controller(quad1.pos_z, quad1.old_pos_z,
        #                                                                                   controller.target_z)
        #
        #     else:
        #         # calls experimental altitude controller, units:cm
        #         self.throttle_control_setting = self.altitude_control.update_z_controller(quad1.pos_z, quad1.old_pos_z,
        #                                                                                   controller.target_z)

        #convert control numbers to pwm periods
        # throttle_pwm = self.calc_pwm_throttle(self.throttle_control_setting)

        # print time.time(), throttle_pwm, quad1.ch3
        throttle_ppm = map_throttle_ppm()
        # Write position and targets to the log file
        self.log_file.write(
                str(time.time()) + ': Current Position (x, y, z, yaw): '
                + str(round(quad1.pos_x, 0)) + ', '
                + str(round(quad1.pos_y, 0)) + ', '
                + str(round(quad1.pos_y, 0)) + ', '
                + str(round(quad1.yaw, 0)) + "\n               Target Position  (x, y, z, yaw): "
                + str(round(self.target_x, 0)) + ', '
                + str(round(self.target_y, 0)) + ', '
                + str(round(self.target_z, 0)) + ', '
                + str(round(self.target_yaw, 0)) + "\n")

        ui.update()

    def map_throttle_ppm(self):
        """ Map the throttle values from the normalised \f$[0,1000]\f$ range to
        the allowable PPM spacing range.
        """
        if self.throttle_normalised > 500:
            throttle_ppm_range = self.max_throttle_ppm - self.mid_throttle_ppm
            return self.mid_throttle_ppm + ((self.throttle_normalised - 500) * throttle_ppm_range)/500
        else:
            throttle_ppm_range = self.mid_throttle_ppm - self.min_throttle_ppm
            return self.min_throttle_ppm + (self.throttle_normalised * throttle_ppm_range)/500

# ----------------------------------------------------------------------------------------------
class quadcopter(Thread,
                 RigidBody):
    """ Detail the object being tracked and how it is controlled.
    """
    target_system = 0
    target_component = 0

    transmitter_control = True
    have_control = False

    ## Number of channels recognised by the RC transmitter.
    number_channels = 8
    ## RC transmitter channels PPM periods.
    channels = [0] * number_channels
    ## Arduino interface to control RC transmitter via serial.
    ardu_interface = ardu_rc_serial_interface()

    def __init__(self,
                 name= "quad",
                 tracker = "Vicon"):
        """ The constructor.
        @param self The object pointer.
        @param name VRPN name of the body tracked.
        @param tracker Motion capture system used (Vicon or Optitrack).
        """
        Thread.__init__(self)
        assert isinstance(name, object)
        self.name = name
        self.tracker = tracker

        # Subscribe to the VRPN pose stream
        rospy.init_node('roscopter')
        rospy.Rate(100)
        rospy.Subscriber("/" + self.name + "/pose", TransformStamped, self.get_rigid_body)

    def update_channels(self):
        """ Update the channel values to send to the RC transmitter.

        Channel mapping:
        - Channel 0: thrust
        - Channel 1-7: ??
        @param self The object pointer.
        """
        self.channels[0] = controller.throttle_ppm
        for i in xrange(1, len(channels)):
            self.channels[i] = controller.mid_throttle_ppm

    def run(self):
        """ Continuously update the transmitter channel PPM values.
        @param self The object pointer.
        """
        while 1:
            self.update_channels()

    def request_data_stream(self,
                            request,
                            frequency,
                            on_off):
        """ Detail the different types of messages able to be recieved from the
        quadcopter data stream.

        @warning Not implemented.
        @param self The object pointer.
        @param request Something.
        @param frequency Something else.
        @param on_off Something other.
        """
        print "request_data_stream()"

    def disconnect_ppm_channels(self):
        """ Give back control of all channels to the transmitter.

        @warning Not implemented.
        @param self The object pointer.
        """
        self.send_channel_ppm(0, 0, 0, 0)

    def send_channel_ppm(self,
                         roll,
                         pitch,
                         throttle,
                         yaw):
        """ Send PPM to specified control channels (channel 0 to channel 3).

        A setting of 0 gives back control to the transmitter.
        @warning Old implementation.
        @param self The object pointer.
        @param roll Quad roll command (high is roll right).
        @param pitch Quad pitch command (high is pitch up).
        @param throttle Quad throttle command (high is up).
        @param yaw Quad yaw command (high is yaw right).
        """
        # Send off pwm commands
        # self.master.mav.rc_channels_override_send(system, component, roll, pitch, throttle, yaw, 0, 0, 0, 0)
        print roll, " ", pitch, " ", throttle, " ", yaw, " ", " ", 0, " ", 0, " ", 0, " ", 0

# ----------------------------------------------------------------------------------------------
class Ui_ControlPannel(ControlPannel.Ui_ControlPannel):
    """ Interface with GUI control pannel developed in Qt Designer.
    """
    # zero position used to make position requests easier for the user
    zero_x = 0
    zero_y = 0
    zero_z = 0
    zero_yaw = 0

    land_position_set = False
    landed = True

    def update(self):
        """ Update all fields in the UI.
        @param self The object pointer.
        """
        # Tracking boxes
        self.tracking_roll_box.setText(  str(round(quad1.rigid_body_roll * 180 / math.pi, 4)) )
        self.tracking_pitch_box.setText( str(round(quad1.rigid_body_pitch * 180 / math.pi, 4)) )
        self.tracking_yaw_box.setText(   str(round((quad1.rigid_body_yaw-self.zero_yaw) * 180 / math.pi, 4)) )
        self.tracking_x_position_box.setText( str(round(quad1.rigid_body_x-self.zero_x, 4)) )
        self.tracking_y_position_box.setText( str(round(quad1.rigid_body_y-self.zero_y, 4)) )
        self.tracking_z_position_box.setText( str(round(quad1.rigid_body_z-self.zero_z, 4)) )

        # IMU boxes (not used)
        self.imu_roll_box.setText(  str(round(quad1.imu_roll, 4)) )
        self.imu_pitch_box.setText( str(round(quad1.imu_pitch, 4)) )
        self.imu_yaw_box.setText(   str(round(quad1.imu_yaw, 4)) )

        # Channel boxes
        self.ch_box_00.setText( str(round(quad1.channels[0], 4)) )
        self.ch_box_01.setText( str(round(quad1.channels[1], 4)) )
        self.ch_box_02.setText( str(round(quad1.channels[2], 4)) )
        self.ch_box_03.setText( str(round(quad1.channels[3], 4)) )
        self.ch_box_04.setText( str(round(quad1.channels[4], 4)) )
        self.ch_box_05.setText( str(round(quad1.channels[5], 4)) )
        self.ch_box_06.setText( str(round(quad1.channels[6], 4)) )
        self.ch_box_07.setText( str(round(quad1.channels[7], 4)) )

        # Debugging boxes
        self.box_00.setText( str(round(controller.altitude_control.p_effort_z, 4)) )
        self.box_01.setText( str(round(controller.altitude_control.i_effort_z, 4)) )
        self.box_02.setText( str(round(controller.altitude_control.d_effort_z, 4)) )
        self.box_03.setText( str(round(controller.altitude_control.desired_accel_z, 4)) )
        self.box_04.setText( str(round(controller.altitude_control.vel_target_z, 4)) )
        self.box_05.setText( str(round(controller.altitude_control.pos_target_z, 4)) )
        self.box_06.setText( str(round(controller.pitch_direction_error, 4)) )
        self.box_07.setText( str(round(controller.roll_direction_error, 4)) )
        self.box_08.setText( str(round(controller.target_x, 4)) )
        self.box_09.setText( str(round(controller.target_y, 4)) )
        self.box_10.setText( str(round(quad1.pos_x, 4)) )
        self.box_11.setText( str(round(quad1.pos_y, 4)) )

        if (quad1.have_control and not
            controller.takeoff_command and not controller.land_command):
            if (quad1.pos_z < controller.land_z + 5):
                self.take_off_land_button.setText("Take off")
                self.landed = True
            else:
                self.take_off_land_button.setText("Land")
                ui.landed = False


    def setup(self):
        """ Initialise the call-backs from the UI.
        @param self The object pointer.
        """
        QtCore.QObject.connect(self.set_home_position_button,
                               QtCore.SIGNAL(_fromUtf8("clicked()")),
                               self.set_zero_position)
        QtCore.QObject.connect(self.set_home_orientation_button,
                               QtCore.SIGNAL(_fromUtf8("clicked()")),
                               self.set_zero_orientation)
        QtCore.QObject.connect(self.goto_position_button,
                               QtCore.SIGNAL(_fromUtf8("clicked()")),
                               self.goto_position)
        QtCore.QObject.connect(self.control_toggle_button,
                               QtCore.SIGNAL(_fromUtf8("clicked()")),
                               self.control_toggle)
        QtCore.QObject.connect(self.set_land_position_button,
                               QtCore.SIGNAL(_fromUtf8("clicked()")),
                               self.set_land_position)
        QtCore.QObject.connect(self.take_off_land_button,
                               QtCore.SIGNAL(_fromUtf8("clicked()")),
                               self.take_off_land)
        # QtCore.QObject.connect(self.set_PID_coefficients_button,
        #                        QtCore.SIGNAL(_fromUtf8("clicked()")),
        #                        self.set_PID_coefficients)

    def take_off_land(self):
        """ Send take off/landing commands. (?)
        @param self The object pointer.
        """
        if self.land_position_set:
            if quad1.have_control:
                if self.landed:
                    controller.altitude_control.accel_integral_max_z = 600
                    controller.stabilize_command = False
                    controller.land_command = False
                    controller.takeoff_command = True
                    controller.target_x = controller.land_x
                    controller.target_y = controller.land_y
                    controller.target_z = controller.land_z + 100
                    controller.target_yaw = controller.land_yaw
                    self.take_off_land_button.setText("Land")

                    # Inform the user of the updated target position and
                    # orientation via the terminal
                    print "Taking off to 100cm"
                else:
                    controller.altitude_control.accel_integral_max_z = 100
                    controller.stabilize_command = False
                    controller.takeoff_command = False
                    controller.land_command = True
                    self.take_off_land_button.setText("Takeoff")
                    print "Landing"
            else:
                print "Please take over control first"
        else:
            print "Please set the land position"

    def set_land_position(self):
        """ Set the land position before taking off is enabled.
        @param self The object pointer.
        """
        controller.land_x = quad1.pos_x
        controller.land_y = quad1.pos_y
        controller.land_z = quad1.pos_z
        controller.land_yaw = quad1.yaw
        self.land_position_set = True

        print ( "Land position is now x:", controller.land_x, " y:",
            controller.land_y, " z:", controller.land_z )

    def control_toggle(self):
        """ Toggle between computer control and transmitter control of the
        quadcopter.
        @param self The object pointer.
        """
        # Check if the transmitter has allowed computer control
        if not quad1.transmitter_control:

            # Check if the computer already has control
            if quad1.have_control:

                # Give back control and updates the text on the control button
                quad1.have_control = False
                self.control_toggle_button.setText("Take Over Control")
                print "You no longer have control"
                # quad1.disconnect_pwm_channels() # used to enable transmitter control

            else:
                # Change the take off land button to appropriate text
                if quad1.pos_z > (controller.land_z + 10):
                    self.take_off_land_button.setText("Land")
                    self.landed = False
                else:
                    self.take_off_land_button.setText("Take off")
                    self.landed = True

                # Convert the current users throttle output to set the hover
                # throttle
                controller.altitude_control.hover_throttle = controller.reverse_pwm_throttle(quad1.channel[2])
                controller.altitude_control.reset_controller_z()

                # Take over control and update the text on the control button
                quad1.have_control = True
                self.control_toggle_button.setText("Give Back Control")

                # Set target as the current position to stabilise around and
                # inform user via the terminal
                controller.target_x = quad1.pos_x
                controller.target_y = quad1.pos_y
                controller.target_z = quad1.pos_z
                print controller.target_z
                controller.target_yaw = quad1.yaw
                print ( "You now have control, stabilizing about x:",
                     controller.target_x, " y:", controller.target_y, " z:",
                     controller.target_z, " yaw: ", controller.target_yaw )

                # Turns the right controller
                controller.altitude_control.accel_integral_max_z = 100
                controller.takeoff_command = False
                controller.land_command = False
                controller.stabilize_command = True

        # Don't allow control toggle if the transmitter has not enabled computer
        # control
        else:
            # Ensure the computer doesn't have control and inform the user via terminal
            quad1.have_control = False
            print ( "You cannot take control transmitter has control,",
                " please switch channel 5 before trying to take over" )

    def set_zero_position(self):
        """ Set the zero position.
        @param self The object pointer.
        """

        # Define the zero reference position as the quadcopters current position
        self.zero_x = quad1.rigid_body_x
        self.zero_y = quad1.rigid_body_y
        self.zero_z = quad1.rigid_body_z

        # Inform the user of the updated zero position in the tracking reference
        # frame via the terminal
        print ( "Zero position is now x:", self.zero_x, " y:", self.zero_y,
            " z:", self.zero_z )

    def set_zero_orientation(self):
        """ Set the zero orientation.
        @param self The object pointer.
        """

        # Define the zero reference orientation as the quadcopters current
        # orientation
        self.zero_yaw = quad1.rigid_body_yaw

        # Inform the user of the updated zero orientation in the tracking
        # reference frame via the terminal
        print "Zero orientaton is now yaw:", quad1.rigid_body_yaw

    def goto_position(self):
        """ Set the target position.
        """
        # Set the target positions from the users input positions
        controller.target_x = self.zero_x + float(self.goto_x_position_box.text())
        controller.target_y = self.zero_y + float(self.goto_y_position_box.text())
        controller.target_z = self.zero_z + float(self.goto_z_position_box.text())
        controller.target_yaw = self.zero_yaw + float(self.goto_yaw_box.text())

        # Inform the user of the updated target position and orientation via the terminal
        print ( "Going to x:", controller.target_x, " y:", controller.target_y,
            " z:", controller.target_z, " yaw:", controller.target_yaw )

# ----------------------------------------------------------------------------------------------
# Set up the GUI
import sys
app = QtGui.QApplication(sys.argv)
ControlPannel = QtGui.QMainWindow()
ui = Ui_ControlPannel()
ui.setupUi(ControlPannel)
ui.setup()
ControlPannel.show()

# ----------------------------------------------------------------------------------------------
# Main function
if __name__ == "__main__":

    # Initialise the quadcopter
    quad1 = quadcopter("quad", "Vicon")

    # Initialise the control system
    controller = control_system()

    # Set the threaded functions in the classes as daemons, so they close when
    # the parent thread (the GUI) closes
    quad1.daemon = True
    controller.daemon = True

    # Start threads
    # quad1.start()
    controller.start()
    sys.exit(app.exec_())
