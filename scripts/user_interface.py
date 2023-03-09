"""
.. module:: user_interface
   :platform: Unix
   :synopsis: Python module for the user Interface
.. moduleauthor:: Suraj Deshini

This node implements an user interface

Service:
 /user_interface
 
"""

import rospy
import time
from rt2_assignment1.srv import Command



def main():
    """
    This function initializes the ROS node and waits for the
    user to
    insert *start* or *stop* to control the robot, by relying
    on the
    `rospy <http://wiki.ros.org/rospy/>`_ module.
    The user message is passed to the service
    ``user_interface``,
    advertised by :mod:`go_to_point`.
    """
    rospy.init_node('user_interface')
    ui_client = rospy.ServiceProxy('/user_interface', Command)
    time.sleep(10)
    rate = rospy.Rate(20)
    print("\n User interface to control the Robot")
    x = int(input("\nPress 1 to start the robot from the current position "))
    while not rospy.is_shutdown():
        if (x == 1):
            ui_client("start")
            x = int(input("\nPress 0 to stop the robot  at the last position"))
        else:
            print("Please wait, the robot is going to stop when the position will be reached")
            ui_client("stop")
            x = int(input("\nPress 1 to start the robot from the current position "))
            
if __name__ == '__main__':
    main()
