#!/usr/bin/env python
#------------------------------------------------------------------------------
# Erstellen von csv-Dateien mit dem folgenden Inhalt:
#   - Beacon Originaldaten (dieser Zeitstempel wird fuer alle anderen Daten verwendet)
#   - Pose des Roboters
#   - Pose Schaetzung des Roboters
#   - Positionsschaetzung der Beacons
#   - Positionsschaetzung des Beacons (MC/SOG)
#
# CLI:
#   rosrun ro_slam_with_uwb ro_slam_data_to_csv.py _source_file_name:="/home/albert/Downloads/test.bag"
#------------------------------------------------------------------------------

import sys
import os
import csv
import rospy

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from mrpt_msgs.msg import BeaconObservationResult
from ro_slam_with_uwb.msg import Beacon


file_beacon = None
file_bor = None
file_bord = None
file_pt = None
file_pe = None

csv_beacon = None
csv_bor = None                      # beacon_observation_result
csv_bord = None                     # beacon_observation_result_data
csv_pt = None                       # csv_pose_trajectory
csv_pe = None                       # csv_pose_estimation

counter = [0, 0, 0, 0, 0]           # [beacon, bor, bord, pt, pe]


def beacon_callback(msg):
    global csv_beacon, counter

    if msg.type != Beacon.TYPE_NEW_RANGE:
        return

    msg_time = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)

    csv_beacon.writerow([
        msg_time.to_time(), msg.header.stamp.secs, msg.header.stamp.nsecs,
        msg.tag_address, msg.anchor_address,
        msg.range,
        msg.receive_power, msg.first_path_power, msg.receive_quality,
        msg.temperature, msg.voltage,
        msg.tag_antenna_delay
    ])
    counter[0] = counter[0] + 1
    return

def beacon_observation_result_callback(msg):
    global csv_bor, csv_bord, counter

    msg_time = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)

    csv_bor.writerow([
        msg_time.to_time(), msg.header.stamp.secs, msg.header.stamp.nsecs,
        msg.beacon_id, msg.pdf_type,
        msg.mean[0], msg.mean[1],
        msg.cov[0], msg.cov[1], msg.cov[2], msg.cov[3]
    ])
    counter[1] = counter[1] + 1

    N = len(msg.mc)
    STEP = 3

    for i in range(0, N, STEP):
        csv_bord.writerow([
            msg_time.to_time(), msg.header.stamp.secs, msg.header.stamp.nsecs,
            msg.beacon_id,
            msg.mc[i + 0], msg.mc[i + 1], msg.mc[i + 2],
            0, 0, 0, 0
        ])
        counter[2] = counter[2] + 1

    N = len(msg.sog)
    STEP = 7

    for i in range(0, N, STEP):
        csv_bord.writerow([
            msg_time.to_time(), msg.header.stamp.secs, msg.header.stamp.nsecs,
            msg.beacon_id,
            msg.sog[i + 0], msg.sog[i + 1], msg.sog[i + 6],
            msg.sog[i + 2], msg.sog[i + 3], msg.sog[i + 4], msg.sog[i + 5]
        ])
        counter[2] = counter[2] + 1
    return

def pose_trajectory_callback(msg):
    global csv_pt, counter

    msg_time = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)

    csv_pt.writerow([
        msg_time.to_time(), msg.header.stamp.secs, msg.header.stamp.nsecs,
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
        msg.pose.orientation.x, msg.pose.orientation.y,
        msg.pose.orientation.z, msg.pose.orientation.w
    ])
    counter[3] = counter[3] + 1
    return

def pose_estimation_callback(msg):
    global csv_pe

    msg_time = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)

    for pose in msg.poses:
        csv_pe.writerow([
            msg_time.to_time(), msg.header.stamp.secs, msg.header.stamp.nsecs,
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w
        ])
        counter[4] = counter[4] + 1
    return

def timer_callback(event):
    global counter
    rospy.loginfo(
        'Beacon: %s - Bor: %s - Bord: %s - Pt: %s - Pe: %s',
        counter[0], counter[1], counter[2], counter[3], counter[4])
    return


rospy.init_node('ro_slam_data_to_csv')

source_file_name = rospy.get_param('~source_file_name', '')
if not source_file_name:
    rospy.logfatal('Invalid parameter: source_file_name')
    sys.exit(1)

source_file_name = os.path.splitext(source_file_name)[0]

topic_beacon = rospy.get_param('~beacon_topic_name', 'beacon_raw')
topic_bor = rospy.get_param(
    '~beacon_observation_result_topic_name', 'beacon_observation_result')
topic_pt = rospy.get_param('~pose_trajectory_topic_name', 'pose_trajectory')
topic_pe = rospy.get_param('~pose_estimation_topic_name', 'pose_estimation')

rospy.loginfo('Subscribing to topic: %s', topic_beacon)
rospy.Subscriber(topic_beacon, Beacon, beacon_callback)

rospy.loginfo('Subscribing to topic: %s', topic_bor)
rospy.Subscriber(
    topic_bor, BeaconObservationResult, beacon_observation_result_callback)

rospy.loginfo('Subscribing to topic: %s', topic_pt)
rospy.Subscriber(topic_pt, PoseStamped, pose_trajectory_callback)

rospy.loginfo('Subscribing to topic: %s', topic_pe)
rospy.Subscriber(topic_pe, PoseArray, pose_estimation_callback)

rospy.Timer(rospy.Duration(1), timer_callback)

csv_file_name = source_file_name + '_beacon.csv'
rospy.loginfo('Creating a new file: %s', csv_file_name)
file_beacon = open(csv_file_name, 'w')
csv_beacon = csv.writer(file_beacon, delimiter=';', lineterminator='\n')
csv_beacon.writerow([
    'time', 'sec', 'nsec',
    'tag_address', 'anchor_address',
    'range',
    'receive_power', 'first_path_power', 'receive_quality',
    'temperature', 'voltage', 'tag_antenna_delay'
])

csv_file_name = source_file_name + '_bor.csv'
rospy.loginfo('Creating a new file: %s', csv_file_name)
file_bor = open(csv_file_name, 'w')
csv_bor = csv.writer(file_bor, delimiter=';', lineterminator='\n')
csv_bor.writerow([
    'time', 'sec', 'nsec',
    'beacon_id', 'pdf_type',
    'mean_x', 'mean_y', 'cov_00', 'cov_01', 'cov_10', 'cov_11'
])

csv_file_name = source_file_name + '_bord.csv'
rospy.loginfo('Creating a new file: %s', csv_file_name)
file_bord = open(csv_file_name, 'w')
csv_bord = csv.writer(file_bord, delimiter=';', lineterminator='\n')
csv_bord.writerow([
    'time', 'sec', 'nsec',
    'beacon_id', 'x', 'y', 'log_w', 'cov_00', 'cov_01', 'cov_10', 'cov_11'
])

csv_file_name = source_file_name + '_pt.csv'
rospy.loginfo('Creating a new file: %s', csv_file_name)
file_pt = open(csv_file_name, 'w')
csv_pt = csv.writer(file_pt, delimiter=';', lineterminator='\n')
csv_pt.writerow([
    'time', 'sec', 'nsec', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw' ])

csv_file_name = source_file_name + '_pe.csv'
rospy.loginfo('Creating a new file: %s', csv_file_name)
file_pe = open(csv_file_name, 'w')
csv_pe = csv.writer(file_pe, delimiter=';', lineterminator='\n')
csv_pe.writerow([
    'time', 'sec', 'nsec', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw' ])

rospy.loginfo('Waiting for new messages.')
rospy.spin()

file_beacon.close()
file_bor.close()
file_bord.close()
file_pt.close()
file_pe.close()

rospy.loginfo('Time to rest, zzzZZzzzZZZ.')