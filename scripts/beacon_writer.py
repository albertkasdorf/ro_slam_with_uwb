#!/usr/bin/env python

import sys
import os
import csv
import rospy

from ro_slam_with_uwb.msg import Beacon


csv_file_counter = { }
csv_writer = None
csv_file_max = 0

def beacon_callback(msg):
    global csv_writer, csv_file_max, csv_file_counter

    if msg.type != Beacon.TYPE_NEW_RANGE:
        return

    if msg.anchor_address not in csv_file_counter:
        csv_file_counter[msg.anchor_address] = 0

    if csv_file_counter[msg.anchor_address] >= csv_file_max:
        return

    msg_time = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)

    csv_writer.writerow([
        msg_time.to_time(), msg.header.stamp.secs, msg.header.stamp.nsecs,
        msg.type,
        msg.tag_address, msg.anchor_address,
        msg.range,
        msg.receive_power, msg.first_path_power, msg.receive_quality,
        msg.temperature, msg.voltage,
        msg.tag_antenna_delay
    ])

    csv_file_counter[msg.anchor_address] = csv_file_counter[msg.anchor_address] + 1
    return


def timer_callback(event):
    global csv_file_counter, csv_file_max
    rospy.loginfo('Range counter: %s', csv_file_counter)

    do_shutdown = True
    for key in csv_file_counter:
        if csv_file_counter[key] < csv_file_max:
            do_shutdown = False
            break

    if do_shutdown:
        rospy.signal_shutdown('Range data collected.')

    return


rospy.init_node('beacon_writer_node')

csv_file_name = rospy.get_param('~csv_file_name', '')
if not csv_file_name:
    rospy.logfatal('Invalid parameter: csv_file_name')
    sys.exit(1)

csv_file_name = os.path.splitext(csv_file_name)[0]
csv_file_name = csv_file_name + '.csv'

csv_file_mode = rospy.get_param('~csv_file_mode', 'a')
if not (csv_file_mode is 'a' or csv_file_mode is 'w'):
    rospy.logfatal('Invalid parameter: csv_file_mode')
    sys.exit(1)

csv_file_max = rospy.get_param('~csv_file_max', 10)
if csv_file_max <= 0:
    rospy.logfatal('Invalid parameter: csv_file_max')
    sys.exit(1)

beacon_topic_name = rospy.get_param('~beacon_topic_name', 'beacon_raw')
if not beacon_topic_name:
    rospy.logfatal('Invalid parameter: beacon_topic_name')
    sys.exit(1)

rospy.loginfo('Subscribing to topic: %s', beacon_topic_name)
rospy.Subscriber(beacon_topic_name, Beacon, beacon_callback)

rospy.Timer(rospy.Duration(1), timer_callback)

csv_file_write_header = not os.path.isfile(csv_file_name)

rospy.loginfo('Opening/Creating a old/new file: %s', csv_file_name)
with open(csv_file_name, csv_file_mode) as csv_file:
    csv_writer = csv.writer(csv_file, delimiter=';', lineterminator='\n')

    if csv_file_write_header:
        csv_writer.writerow([
            "time", "sec", "nsec",
            "tag_address", "anchor_address",
            "range",
            "receive_power", "first_path_power", "receive_quality",
            "temperature", "voltage",
            "tag_antenna_delay"
        ])

    rospy.loginfo('Waiting for new messages.')
    rospy.spin()

rospy.loginfo('Time to rest, zzzZZzzzZZZ.')
