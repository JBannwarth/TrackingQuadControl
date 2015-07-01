""" @package rigid_body
Define a class that can recieve ros_vrpn_client pose.

This package is developed for use with the motion capture system in the Faculty
of Engineering at the University of Auckland.

@author Jay Mills
@author Jeremie Xavier Joseph Bannwarth
@version 0.1
@date 06/2015
"""

import tf
import rospy
from geometry_msgs.msg import TransformStamped
import math

class rigid_body(object):
    """ Interface with ros_vrpn_client to retrieve a tracked rigid body's
    pose.
    """
    # --------------------------------------------------------------------------
    # Variables
    ## First time update flag.
    first_time = True
    ## VRPN name of the rigid body tracked.
    name = ""

    ## Current output x position (in cm).
    pos_x = 0
    ## Current output y position (in cm).
    pos_y = 0
    ## Current output z position (in cm).
    pos_z = 0
    ## Current output roll (in rad).
    roll = 0
    ## Current output pitch (in rad).
    pitch = 0
    ## Current output yaw (in rad).
    yaw = 0

    ## Previous output x position (in cm).
    old_pos_x = 0
    ## Previous output y position (in cm).
    old_pos_y = 0
    ## Previous output z position (in cm).
    old_pos_z = 0
    ## Previous output roll (in rad).
    old_roll = 0
    ## Previous output pitch (in rad).
    old_pitch = 0
    ## Previous output yaw (in rad).
    old_yaw = 0

    ## Current rigid body x position (in cm).
    rigid_body_x = 0
    ## Current rigid body y position (in cm).
    rigid_body_y = 0
    ## Current rigid body z position (in cm).
    rigid_body_z = 0
    ## Current rigid body roll (in rad).
    rigid_body_roll = 0
    ## Current rigid body pitch (in rad).
    rigid_body_pitch = 0
    ## Current rigid body yaw (in rad).
    rigid_body_yaw = 0
    # --------------------------------------------------------------------------

    def __init__(self,
                 rigid_body_name,
                 tracker):
        """ The constructor.

        Subscribe to the VRPN stream: continuously retrieve data from the object
        at /rigid_body_name/pose.
        @param self The object pointer.
        @param rigid_body_name VRPN name of the object tracked.
        @param tracker Motion capture system used (Vicon or Optitrack).
        """

        self.name = rigid_body_name
        self.tracker = tracker

        # Initialise the tracking node
        rospy.init_node('roscopter')
        rospy.Rate(100)
        rospy.Subscriber("/" + self.name + "/pose", TransformStamped, self.get_rigid_body)

    def get_rigid_body(self,
                       rigid_body_msg):
        """ Get position and orientation data from ros_vrpn_client.

        The position is received in m and converted to cm.

        The orientation is received as a quarternion and is converted to euler
        angles.

        The object's yaw is limited to the \f$[-\pi,\pi]\f$ range.
        @param self The object pointer.
        @param rigid_body_msg VRPN message containing the object's pose.
        """
        if self.tracker == "Optitrack":
            # Get linear position and convert to cm
            self.rigid_body_x = (rigid_body_msg.transform.translation.x * 100)
            self.rigid_body_y = (rigid_body_msg.transform.translation.y * 100)
            self.rigid_body_z = (rigid_body_msg.transform.translation.z * 100)

            # Get quaternion orientation
            rx = rigid_body_msg.transform.rotation.x
            ry = rigid_body_msg.transform.rotation.y
            rz = rigid_body_msg.transform.rotation.z
            rw = rigid_body_msg.transform.rotation.w

            # Convert from quaternions to euler angles
            (temp_roll, temp_pitch, temp_yaw) = tf.transformations.euler_from_quaternion(
                [rx, ry, rz, rw])

            self.rigid_body_roll = temp_roll * 180 / math.pi
            self.rigid_body_pitch = temp_pitch * 180 / math.pi
            self.rigid_body_yaw = temp_yaw * 180 / math.pi

        elif self.tracker == "Vicon":
            # Get linear position and convert to cm
            self.rigid_body_x = (rigid_body_msg.transform.translation.x * 100)
            self.rigid_body_y = (rigid_body_msg.transform.translation.y * 100)
            self.rigid_body_z = (rigid_body_msg.transform.translation.z * 100)

            # Get quaternion orientation
            rx = rigid_body_msg.transform.rotation.x
            ry = rigid_body_msg.transform.rotation.y
            rz = rigid_body_msg.transform.rotation.z
            rw = rigid_body_msg.transform.rotation.w

            # Convert from quaternions to euler angles
            (temp_roll, temp_pitch, temp_yaw) = tf.transformations.euler_from_quaternion(
                [rx, ry, rz, rw])

            self.rigid_body_yaw = temp_yaw
            self.rigid_body_pitch = temp_pitch
            self.rigid_body_roll = temp_roll

        # Limit yaw input
        if self.rigid_body_yaw > math.pi:
            self.rigid_body_yaw = self.rigid_body_yaw - (math.pi*2)
        if self.rigid_body_yaw < -math.pi:
            self.rigid_body_yaw = self.rigid_body_yaw + (math.pi*2)

    def update_pos_values(self):
        """ Update current and old pose values.
        @param self The object pointer.
        """
        # Update the pose values of the rigid body, and stores previous values
        if self.first_time:
            self.first_time = False
            # Previous values
            self.old_pos_x = self.rigid_body_x
            self.old_pos_y = self.rigid_body_y
            self.old_pos_z = self.rigid_body_z
            self.old_yaw = self.rigid_body_yaw
            self.old_roll = self.rigid_body_roll
            self.old_pitch = self.rigid_body_pitch

            # Current values
            self.pos_x = self.rigid_body_x
            self.pos_y = self.rigid_body_y
            self.pos_z = self.rigid_body_z
            self.yaw = self.rigid_body_yaw
            self.roll = self.rigid_body_roll
            self.pitch = self.rigid_body_pitch
        else:
            # Previous values
            self.old_pos_x = self.pos_x
            self.old_pos_y = self.pos_y
            self.old_pos_z = self.pos_z
            self.old_yaw = self.yaw
            self.old_pitch = self.pitch
            self.old_roll = self.roll

            # Current values
            self.pos_x = self.rigid_body_x
            self.pos_y = self.rigid_body_y
            self.pos_z = self.rigid_body_z
            self.yaw = self.rigid_body_yaw
            self.roll = self.rigid_body_roll
            self.pitch = self.rigid_body_pitch
