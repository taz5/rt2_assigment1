#! /usr/bin/env python


from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations

import rospy
import actionlib
import rt2_assignment1.msg
import math

# robot state variables
position_ = Point()
yaw_ = 0
position_ = 0
state_ = 0
pub_ = None
robot_action_server=None

# parameters for control
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0 
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6


def clbk_odom(msg):
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def change_state(state):
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)


def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def fix_yaw(des_pos):
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        change_state(1)


def go_straight_ahead(des_pos):
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3
        if twist_msg.linear.x > ub_d:
           twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        pub_.publish(twist_msg)
    else: 
        change_state(2)
    if math.fabs(err_yaw) > yaw_precision_:
      change_state(0)

def fix_final_yaw(des_yaw):
    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        change_state(3)
        
def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)

def go_to_point(goal):
    # Declare global variables.
    global robot_action_server
    global state_

    # Create a Point object and set its values.
    desired_position = Point(x=goal.x, y=goal.y)

    # Set the desired yaw angle.
    des_yaw = goal.theta

    # Change the state to "0".
    change_state(0)

    # Create instances of the RcFeedback and RcResult message types.
    feedback = rt2_assignment1.msg.RcFeedback()
    result = rt2_assignment1.msg.RcResult()

    # Enter an infinite loop.
    while True:
        # Check if a preempt request has been received.
        if robot_action_server.is_preempt_requested():
            # Print a message and call the done() function.
            print("Preempted the goal")
            done()
            # Update the state to "3".
            state_ = 3

        # Check the value of the state variable and call the corresponding function.
        if state_ == 0:
            fix_yaw(desired_position)
        elif state_ == 1:
            go_straight_ahead(desired_position)
        elif state_ == 2:
            fix_final_yaw(des_yaw)
        elif state_ == 3:
            # Call the done() function and break out of the loop.
            done()
            break

    # Set the result and mark the goal as succeeded.
    robot_action_server.set_succeeded(result)

    # Return True to indicate success.
    return True

                
        


def main():
    # Declare global variables.
    global pub_
    global robot_action_server

    # Initialize the ROS node.
    rospy.init_node('go_to_point')

    # Create a publisher to send Twist messages.
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # Create a subscriber to receive Odometry messages.
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

    # Create an action server to handle requests to go to a point.
    robot_action_server = actionlib.SimpleActionServer('/go_to_point', rt2_assignment1.msg.GoToPointAction, execute_cb=go_to_point, auto_start=False)
    robot_action_server.start()

    # Enter a loop to keep the node running.
    rospy.spin()


if __name__ == '__main__':
    main()
