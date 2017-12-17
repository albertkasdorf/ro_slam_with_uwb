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

    threshold = rospy.get_param('~threshold', 0.2)
    linear_speed = rospy.get_param('~linear_speed', 2.0)
    angular_speed = rospy.get_param('~angular_speed', 3.0)

    rospy.Subscriber('joy', Joy, callback)

    # TurtleSim commands
    # http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=60)

    rate = rospy.Rate(60) # 60 hz
    while not rospy.is_shutdown():

        # Wait until joy_msg is correctly initializied.
        if joy_msg is not None:

            # Is node shutdown requested?
            #if joy_msg.buttons[6] == 1:
            #    #http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown
            #    rospy.signal_shutdown('User requested shutdown')

            # axes[0]: Left Top Stick - Left (-) to right (+)
            # axes[1]: Left Top Stick - Top (-) to bottom (+)
            # axes[3]: Right Bottom Stick - Left (-) to right (+)
            # axes[6]: Left Bottom Stick - Top (-) to bottom (+)
            # axes[7]: Left Bottom Stick - Left (-) to right (+)
            should_publish = (
                abs(joy_msg.axes[0]) > threshold or
                abs(joy_msg.axes[1]) > threshold or
                abs(joy_msg.axes[3]) > threshold or
                abs(joy_msg.axes[6]) > threshold or
                abs(joy_msg.axes[7]) > threshold
            )

            # Filter to small movement
            if should_publish:

                twist_msg = Twist()

                if abs(joy_msg.axes[1]) > threshold:
                    twist_msg.linear.x = joy_msg.axes[1] * linear_speed
                else:
                    twist_msg.linear.x = joy_msg.axes[7] * linear_speed

                if abs(joy_msg.axes[0]) > threshold:
                    twist_msg.linear.y = joy_msg.axes[0] * linear_speed
                else:
                    twist_msg.linear.y = joy_msg.axes[6] * linear_speed

                twist_msg.angular.z = joy_msg.axes[3] * angular_speed

                pub.publish(twist_msg)
            else:
                # Stop movement
                pub.publish(Twist())

        rate.sleep()


if __name__ == '__main__':
    Main()
