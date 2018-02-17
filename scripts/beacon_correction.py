#!/usr/bin/env python
#------------------------------------------------------------------------------
# Korrektur der Beacon-Entfernung um einen systematischen Fehler.
#
# CLI:
#   rosrun ro_slam_with_uwb beacon_correction.py \
#       _beacon_in_topic_name:="beacon" \
#       _beacon_out_topic_name:="beacon_corrected"
#------------------------------------------------------------------------------


import sys
import rospy


from mrpt_msgs.msg import ObservationRangeBeacon
from mrpt_msgs.msg import SingleRangeBeaconObservation


#correction = {177:-0.32, 178:-0.16, 179:-0.23, 180:0.05}
#correction = {177:-0.30, 178:-0.30, 179:-0.30, 180:-0.30}
correction = {177:0, 178:0, 179:0, 180:0}
sigma = 0.02
pub_beacon_corrected = None

def beacon_callback(msg):
    global correction, sigma, pub_beacon_corrected

    id = msg.sensed_data[0].id
    range = msg.sensed_data[0].range

    msg.sensed_data[0].range = range + correction[id]
    msg.sensor_std_range = sigma
    pub_beacon_corrected.publish(msg)
    return


rospy.init_node('beacon_correction_node')

topic_beacon_in = rospy.get_param('~beacon_in_topic_name', 'beacon')
topic_beacon_out = rospy.get_param('~beacon_out_topic_name', 'beacon_corrected')

pub_beacon_corrected = rospy.Publisher(
    topic_beacon_out, ObservationRangeBeacon, queue_size=10)

rospy.loginfo('Subscribing to topic: %s', topic_beacon_in)
rospy.Subscriber(topic_beacon_in, ObservationRangeBeacon, beacon_callback)

rospy.loginfo('Waiting for new messages.')
rospy.spin()

rospy.loginfo('Time to rest, zzzZZzzzZZZ.')