#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

joy_msg = None

def callback(data):
    global joy_msg
    joy_msg = data

def Main(): 
    global joy_msg
    threshold = 0.2
    linear_speed = 2.0
    angular_speed = 3.0

    # Writing a Publisher and Subscriber with a Custom Message(Python)
    # http://wiki.ros.org/ROS/Tutorials/CustomMessagePublisherSubscriber(python)
    rospy.init_node('xbox_teleop')
    rospy.Subscriber('joy', Joy, callback)

    # TurtleSim commands
    # http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 60)

    rate = rospy.Rate(60) # 60 hz
    while not rospy.is_shutdown():

        # Wait until joy_msg is correctly initializied.
        if joy_msg is not None:

            # Is node shutdown requested?
            #if joy_msg.buttons[6] == 1:
            #    #http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown
            #    rospy.signal_shutdown('User requested shutdown')

            # Filter to small movement
            if( abs(joy_msg.axes[0]) > threshold or
                abs(joy_msg.axes[1]) > threshold or
                abs(joy_msg.axes[3]) > threshold ):

                twist_msg = Twist()
                twist_msg.linear.x = joy_msg.axes[1] * linear_speed
                twist_msg.linear.y = joy_msg.axes[0] * linear_speed
                twist_msg.angular.z = joy_msg.axes[3] * angular_speed
                pub.publish(twist_msg)
            else:
                # Stop movement
                pub.publish(Twist())

        rate.sleep()


if __name__ == '__main__':
    Main()
