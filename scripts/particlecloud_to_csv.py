#!/usr/bin/env python

import sys
import os
import csv
import rospy

from geometry_msgs.msg import *

csv_writer = None
csv_file = None
csv_file_name = ''


def particlecloud_pose_callback(msg):
    particlecloud_callback(msg, ord('P'))
    return


def particlecloud_beacons_callback(msg):
    particlecloud_callback(msg, ord('B'))
    return


def particlecloud_callback(msg, msg_id):
    global csv_writer

    msg_time = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)

    for pose in msg.poses:
        csv_writer.writerow([
            msg_time.to_time(), msg.header.stamp.secs, msg.header.stamp.nsecs,
            msg_id,
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w])
    return


# TODO: Use ROS Parameter Server!!!
if len(sys.argv) >= 2:
    csv_file_name = sys.argv[1]
else:
    print "%s [filename_without_extension]"%sys.argv[0]
    sys.exit(1)

csv_file_name = os.path.splitext(csv_file_name)[0]
csv_file_name = csv_file_name + '.csv'

rospy.init_node('particlecloud_to_csv_node')

rospy.loginfo('Subscribing to topic: particlecloud')
rospy.Subscriber(
    'particlecloud', PoseArray, particlecloud_pose_callback)

rospy.loginfo('Subscribing to topic: particlecloud_beacons')
rospy.Subscriber(
    'particlecloud_beacons', PoseArray, particlecloud_beacons_callback)

rospy.loginfo('Creating a new file: %s', csv_file_name)
with open(csv_file_name, "w") as csv_file:
    csv_writer = csv.writer(csv_file, delimiter=';', lineterminator='\n')
    csv_writer.writerow(
        ["time", "sec", "nsec", "id", "x", "y", "z", "qx", "qy", "qz", "qw"])

    rospy.loginfo('Waiting for new messages.')
    rospy.spin()

rospy.loginfo('Time to rest, zzzZZzzzZZZ.')