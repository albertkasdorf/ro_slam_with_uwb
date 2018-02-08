#!/usr/bin/env python



import rospy


from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


max_linear_vel = None
min_linear_vel = None
max_angular_vel = None
min_angular_vel = None
threshold_linear = None
threshold_angular = None
pub_cmd_vel = None


def linear_mapping(x, in_min, in_max, out_min, out_max):
    # function for linear mapping between two ranges
    # https://machinelearning1.wordpress.com/2014/07/13/linear-vector-mapping-scaling-matlab/
    a = in_min
    b = in_max
    c = out_min
    d = out_max
    y = ((c + d) + (d - c) * ((2 * x - (a + b)) / (b - a))) / 2
    return y

def joy_callback(msg):
    global pub_cmd_vel
    global max_linear_vel, min_linear_vel
    global max_angular_vel, min_angular_vel
    global threshold_linear, threshold_angular

    # axes[0]: Left Top Stick - Left (+) to right (-)
    # axes[1]: Left Top Stick - Top (+) to bottom (-)
    # axes[3]: Right Bottom Stick - Left (+) to right (-)
    # axes[6]: Left Bottom Stick - Left (+) to right (-)
    # axes[7]: Left Bottom Stick - Top (+) to bottom (-)

    twist_msg = Twist()

    # Bewegung ueber die X-Achse
    if msg.axes[1] >= threshold_linear:
        twist_msg.linear.x = linear_mapping( msg.axes[1], threshold_linear, 1.0, min_linear_vel, max_linear_vel )

    if msg.axes[1] <= -threshold_linear:
        twist_msg.linear.x = linear_mapping( msg.axes[1], -1.0, -threshold_linear, -max_linear_vel, -min_linear_vel )

    if msg.axes[7] >= threshold_linear:
        twist_msg.linear.x = linear_mapping( msg.axes[7], threshold_linear, 1.0, min_linear_vel, max_linear_vel )

    if msg.axes[7] <= -threshold_linear:
        twist_msg.linear.x = linear_mapping( msg.axes[7], -1.0, -threshold_linear, -max_linear_vel, -min_linear_vel )

    # Bewegung ueber die Y-Achse (pos geht nach links)
    if msg.axes[0] >= threshold_linear:
        twist_msg.linear.y = linear_mapping( msg.axes[0], threshold_linear, 1.0, min_linear_vel, max_linear_vel )

    if msg.axes[0] <= -threshold_linear:
        twist_msg.linear.y = linear_mapping( msg.axes[0], -1.0, -threshold_linear, -max_linear_vel, -min_linear_vel )

    # Gegen den Uhrzeigersinn
    if msg.axes[3] >= threshold_angular:    
        twist_msg.angular.z = linear_mapping( msg.axes[3], threshold_angular, 1.0, min_angular_vel, max_angular_vel )

    # Mit dem Uhrzeigersinn
    if msg.axes[3] <= -threshold_angular:
        twist_msg.angular.z = linear_mapping( msg.axes[3], -1.0, -threshold_angular, -max_angular_vel, -min_angular_vel )

    pub_cmd_vel.publish(twist_msg)
    return


rospy.init_node('xbox_teleop2_node')

max_linear_vel = rospy.get_param('~max_linear_vel', 0.2)
min_linear_vel = rospy.get_param('~min_linear_vel', 0.05)

max_angular_vel = rospy.get_param('~max_angular_vel', 1.0)
min_angular_vel = rospy.get_param('~min_angular_vel', 0.1)

threshold_linear = rospy.get_param('~threshold_linear', 0.2)
threshold_angular = rospy.get_param('~threshold_angular', 0.2)

pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=60)

rospy.loginfo('Subscribing to topic: joy')
rospy.Subscriber('joy', Joy, joy_callback)

rospy.loginfo('Waiting for new messages.')
rospy.spin()

rospy.loginfo('Time to rest, zzzZZzzzZZZ.')
