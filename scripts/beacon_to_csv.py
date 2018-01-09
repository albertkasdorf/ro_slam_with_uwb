#!/usr/bin/env python

import sys
import os
import csv
import rospy

from mrpt_msgs.msg import *


csv_writer = None
csv_file = None
csv_file_name = 'filename'
beacon_topic_name = '/beacon_raw'
csv_lines_written = 1
callback_works = False


def ObservationRangeBeaconCallback(msg):
    global csv_writer, csv_lines_written, callback_works

    if not callback_works:
        callback_works=True
        rospy.loginfo('Subscription to topic works!')

    msg_time = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)

    for srbo in msg.sensed_data:
        csv_writer.writerow([
            msg_time.to_time(),
            msg.header.stamp.secs,
            msg.header.stamp.nsecs,
            srbo.id,
            srbo.range])
        csv_lines_written += 1
    return

# TODO: Use ROS Parameter Server!!!
if len(sys.argv) == 2:
    csv_file_name = sys.argv[1]
elif len(sys.argv) == 3:
    csv_file_name = sys.argv[1]
    beacon_topic_name = sys.argv[2]
else:
    print "%s [filename_without_extension [beacon_topic_name=/beacon_raw]]"%sys.argv[0]
    sys.exit(1)

csv_file_name = os.path.splitext(csv_file_name)[0]
csv_file_name = csv_file_name + '.csv'

rospy.init_node('beacon_to_csv_node')

rospy.loginfo('Subscribing to topic: %s', beacon_topic_name)
rospy.Subscriber(
    beacon_topic_name, ObservationRangeBeacon, ObservationRangeBeaconCallback)

rospy.loginfo('Creating a new file: %s', csv_file_name)
with open(csv_file_name, "w") as csv_file:
    csv_writer = csv.writer(csv_file, delimiter=';', lineterminator='\n')
    csv_writer.writerow(["time", "sec", "nsec", "id", "range"])

    rospy.loginfo('Waiting for new messages.')
    rospy.spin()


rospy.loginfo('%d lines witten to file: %s', csv_lines_written, csv_file_name)
rospy.loginfo('Time to rest, zzzZZzzzZZZ.')