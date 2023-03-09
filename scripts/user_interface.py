# Import necessary libraries
import rospy
import time
from rt2_assignment1.srv import Command

# Define the main function
def main():
    # Initialize the ROS node
    rospy.init_node('user_interface')

    # Create a service client for the 'Command' service
    ui_client = rospy.ServiceProxy('/user_interface', Command)

    # Wait for 10 seconds
    time.sleep(10)

    # Set the loop rate to 20 Hz
    rate = rospy.Rate(20)

    # Display a message for the user interface
    print("\n User interface to control the Robot")

    # Prompt the user to enter '1' to start the robot or '0' to stop it
    x = int(input("\nPress 1 to start the robot from the current position "))

    # Run the loop until the program is stopped
    while not rospy.is_shutdown():
        if x == 1:
            # Send the 'start' command to the robot
            ui_client("start")

            # Prompt the user to enter '0' to stop the robot
            x = int(input("\nPress 0 to stop the robot at the last position"))
        else:
            # Display a message indicating that the robot is stopping
            print("Please wait, the robot is going to stop when the position will be reached")

            # Send the 'stop' command to the robot
            ui_client("stop")

            # Prompt the user to enter '1' to start the robot again
            x = int(input("\nPress 1 to start the robot from the current position "))

        # Sleep for a short time to maintain the loop rate
        rate.sleep()

# Run the main function if the script is executed directly
if __name__ == '__main__':
    main()

