#!/usr/bin/env python

"""
    Feed Forward Control that check the status from sensors, estimates the offset,
    Calculates corrected waypoint location and the speed of the vehicle


    FIXME: under development

    File Name: force_listener.py
    Author: Nare Karapetyan
    Data Created: April 15 2019
    Date Last Modified: April 16 2019

"""

import rospy
import tf

from sensor_msgs.msg import NavSatFix
from current_sensor.msg import WaterCurrent
from wind_sensor.msg import Wind
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import WaypointList, GlobalPositionTarget

from feed_forward_control.msg import ForceEffect

from autopilot import Autopilot

class FFControl:
    """
    Parameters
    ----------

    Attributes
    ----------
        Here class member variables should be described

    Example
    -------

    """

    def __init__(self):

        self.force_effect = ForceEffect() # speed, direction, x, y

        rospy.Subscriber("/force_effect", ForceEffect, self.force_effect_callback) # this will have calcualted error
        self.wp_pub = rospy.Publisher("/waypoint", GlobalPositionTarget, queue_size=1) # to keep track of each waypoint that is currently on the execution
        rospy.Subscriber("/jetyak1/mavros/global_position/raw/fix" , NavSatFix, self.gpose_callback) # lat and long

        self.ap = Autopilot()


    def go_to_waypoint(self, waypoint, fwd_speed):
        self.wp_pub.publish(waypoint)

        rate = rospy.Rate(10)

        while (not self.is_wp_reached(waypoint, self.latest_gpose)) and (not rospy.is_shutdown()):
            id_waypoint = self.get_intermediate_waypoint(waypoint) # calculates intermediate waypoints
            id_fwd_speed = self.get_corrected_speed(fwd_speed)
            self.ap.change_global_position(id_waypoint, id_fwd_speed)

            rate.sleep()

    def get_intermediate_waypoint(self, wp):
        """
            Parameters
            ----------
                takes self.force_effect.x self.force_effect.y and wp information

            Returns
            -------
            id_waypoint: [lat, long]
        """
        new_wp = wp
        #FIXME: must be implemented

        #new_wp.latitude
        #new_wp.longitude
        return new_wp


    def gpose_callback(self, msg):
        self.latest_gpose = [msg.latitude, msg.longitude]

        print("The gpose_callback is called")

    def force_effect_callback(self, msg):

        self.force_effect = [msg.x, msg.y]
        print("The force_effect_callback is called")

    def is_wp_reached(self, waypoint, pose):
        #FIXME: should be implemented

        return False
