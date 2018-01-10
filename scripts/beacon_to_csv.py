#!/usr/bin/env python

import sys
import os
import csv
import rospy

from mrpt_msgs.msg import *

csv_writer = None
csv_file = None
csv_file_name = ''
beacon_topic_name = ''
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

rospy.init_node('beacon_to_csv_node')

csv_file_name = rospy.get_param('~csv_file_name', '')
if not csv_file_name:
    rospy.logfatal('Invalid parameter: csv_file_name')
    sys.exit(1)

csv_file_name = os.path.splitext(csv_file_name)[0]
csv_file_name = csv_file_name + '.csv'

beacon_topic_name = rospy.get_param('~beacon_topic_name', 'beacon_raw')
if not beacon_topic_name:
    rospy.logfatal('Invalid parameter: beacon_topic_name')
    sys.exit(1)

rospy.loginfo('Subscribing to topic: %s', beacon_topic_name)
rospy.Subscriber(
    beacon_topic_name, ObservationRangeBeacon, ObservationRangeBeaconCallback)

rospy.loginfo('Creating a new file: %s', csv_file_name)
with open(csv_file_name, "w") as csv_file:
    csv_writer = csv.writer(csv_file, delimiter=';', lineterminator='\n')
    csv_writer.writerow(["time", "sec", "nsec", "id", "range"])

    rospy.loginfo('Waiting for new messages.')
    rospy.spin()

rospy.loginfo('%d lines written to file: %s', csv_lines_written, csv_file_name)
rospy.loginfo('Time to rest, zzzZZzzzZZZ.')
