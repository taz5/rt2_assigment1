## @package rt2_assignment1
#	\file user_interface.py
#	\brief This file contain the user interface service server
#	\author Suraj Deshini
#	\date 09/03/2023
#	
#	\details
#
#	\Client : <BR>
#		\user_interface
#
#	This node define the command line user interface for the control of the robot


import rospy
import time
from rt2_assignment1.srv import Command



def main():
##   
#    Definition of the command line
#    user interface to set the "start"
#    "stop" varaible by relying on the rospy module.
#
#    The service is passed to the service "user_interface", 
#	advertised by the "module go_to_poit"
#    .
   
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
