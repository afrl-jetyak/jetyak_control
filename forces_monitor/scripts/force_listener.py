#!/usr/bin/env python

"""
    Feed Forward Control that check the status from sensors, estimates the offset,
    Calculates corrected waypoint location and the speed of the vehicle

    Listens to forces and calcuates the effect of the force on the waypoint navigation
    also runs the node force_listener


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

class ForceListener:
    """ This is a ForceListener Class for Jetyak that subscribs to the forces
    and publishes the ffect

    Parameters
    ----------

    Attributes
    ----------
        Here class member variables should be described
        latest_current:
        latest_wind:
        latest_hdg:
        latest_lin_vel:
        latest_gpose:

    Example
    -------

    """

    def __init__(self):

        self.latest_current = WaterCurrent()
        self.latest_wind = Wind()

        self.latest_hdg = 0
        self.latest_lin_vel=0

        self.latest_gpose=[0,0] # latitude and longitude

        self.latest_wp = [0,0] # latitude and longitude

        self.force_effect = rospy.Publisher("/force_effect", ForceEffect, queue_size=1) # this will have calcualted error

        rospy.Subscriber("/jetyak1/current_sensor/current_raw", WaterCurrent, self.current_callback)
        rospy.Subscriber("/jetyak1/wind_sensor/wind_raw", Wind, self.wind_callback)

        rospy.Subscriber("/jetyak1/mavros/global_position/compass_hdg", Float64, self.heading_callback)
        #rospy.Subscriber("mavros/global_position/raw/gps_vel", TwistStamped, self.gpsvel_callback)

        rospy.Subscriber("/jetyak1/mavros/global_position/raw/fix" , NavSatFix, self.gpose_callback) # lat and long
        rospy.Subscriber("/waypoint", GlobalPositionTarget, self.wp_callback) # the waypoint information read from mission planner

        rospy.wait_for_message("/force_effect", ForceEffect)


        rospy.spin()



    #FIXME: should not calcualte the force effect everytime a new sensor reading has been received
    # this should be corrected
    def current_callback(self, msg):
        self.latest_current = msg
        # msg.current_star_rear
        # msg.current_port_rear
        # msg.current_star_front
        # msg.current_port_front

        print("The current_callback is called")
        error = self.calculate_force_effect()
        self.force_effect.publish(error)


    def wind_callback(self, msg):
        self.latest_wind = msg
        # msg.windspeedmph
        # msg.winddir

        print("The wind_callback is called")
        error = self.calculate_force_effect()
        self.force_effect.publish(error)

    def heading_callback(self, msg):
        self.latest_hdg = msg.data

        print("The heading_callback is called")
        error = self.calculate_force_effect()
        self.force_effect.publish(error)

    def gpose_callback(self, msg):
        self.latest_gpose = [msg.latitude, msg.longitude]

        print("The gpose_callback is called")
        error = self.calculate_force_effect()
        self.force_effect.publish(error)

    def wp_callback(msg):
        self.latest_wp = [msg.latitude, msg.longitude]

        print("The gpose_callback is called")
        error = self.calculate_force_effect()
        self.force_effect.publish(error)


    def calculate_force_effect(self):
        """
            This function calcualtes the effect of foces and returns that

            Parameters
            ----------
                self.latest_current
                self.latest_wind
                self.latest_hdg
                self.latest_wp  <- check if this is 0

            Returns
            -------
                effect: ForceEffect message that is a pair of speed and direction
        """

        effect = ForceEffect()
        effect.speed = None # FIXME: here goes the calculation of the speed
        effect.direction = None # FIXME: hese goes the calculation of the direction

        effect.x, effect.y = self.convert_to_x_y(effect.speed, effect.direction)

        print("The effect of forces is calculated")
        return effect

    def convert_to_x_y(self, speed, direction):
        """
            will convert the speed and direction

            Parameters
            ----------
                speed
                direction

            Returns
            -------
                x, y

        """

        x = None # here goes the calculation
        y = None # here goes the calculation
        return x, y



if __name__ == "__main__":

    rospy.init_node("force_listener")
    f_listener = ForceListener()

    print("force listener node is running")
