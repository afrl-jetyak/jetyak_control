#!/usr/bin/env python

"""

    April 25 it is reding from the mission file
    and changes jetyaks global posiotin

    File Name: waypoint_navigation.py
    Author: Nare Karapetyan
    Data Created: April 15 2019
    Date Last Modified: June 10 2019

"""


import rospy
import tf

from ffcontrol import FFControl
from mission_reader import get_waypoints


if __name__ == "__main__":
    rospy.init_node("ffcontrol")

    mission_file_name = rospy.get_param('~mission_file_name') # mission file in .txt
    forward_speed = rospy.get_param('~forward_speed')

    ffc = FFControl()

    print("--")
    wp_list = get_waypoints(mission_file_name)


    try:
        for wp in wp_list:
            ffc.go_to_waypoint(wp, forward_speed)
            print("the waypoint has been reached")

        print("The mission has been completed")


    except Exception as e:
        print e
