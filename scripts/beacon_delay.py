#!/usr/bin/env python
#------------------------------------------------------------------------------
# Verzoegern der Beacon weitergabe.
#
# CLI:
#   rosrun ro_slam_with_uwb beacon_delay.py \
#       _beacon_in_topic_name:="beacon" \
#       _beacon_out_topic_name:="beacon_corrected"
#------------------------------------------------------------------------------


import sys
import rospy


from mrpt_msgs.msg import ObservationRangeBeacon
from mrpt_msgs.msg import SingleRangeBeaconObservation


delay = {177:5, 178:10, 179:15, 180:20}
first_occurrence = {}
pub_beacon_delay = None

def beacon_callback(msg):
    global first_occurrence, pub_beacon_delay

    id = msg.sensed_data[0].id

    if id not in first_occurrence:
        first_occurrence[id] = msg.header.stamp
        return

    stamp_diff = msg.header.stamp - first_occurrence[id]

    if stamp_diff > rospy.Duration(delay[id]):
        pub_beacon_delay.publish(msg)

    return


rospy.init_node('beacon_delay_node')

topic_beacon_in = rospy.get_param('~beacon_in_topic_name', 'beacon')
topic_beacon_out = rospy.get_param('~beacon_out_topic_name', 'beacon_corrected')

pub_beacon_delay = rospy.Publisher(
    topic_beacon_out, ObservationRangeBeacon, queue_size=10)

rospy.loginfo('Subscribing to topic: %s', topic_beacon_in)
rospy.Subscriber(topic_beacon_in, ObservationRangeBeacon, beacon_callback)

rospy.loginfo('Waiting for new messages.')
rospy.spin()

rospy.loginfo('Time to rest, zzzZZzzzZZZ.')