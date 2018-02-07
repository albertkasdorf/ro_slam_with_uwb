#!/usr/bin/env python
#------------------------------------------------------------------------------
# Anzeigen der aktuellen Beacon Messungen
#
# CLI:
#   rosrun ro_slam_with_uwb beacon_observer.py _source_file_name:="/home/albert/Downloads/test.bag"
#------------------------------------------------------------------------------


import sys
import rospy


from mrpt_msgs.msg import ObservationRangeBeacon
from mrpt_msgs.msg import SingleRangeBeaconObservation


counter = { }


def beacon_callback(msg):
    global counter

    id = msg.sensed_data[0].id
    range = msg.sensed_data[0].range

    if id not in counter:
        counter[id] = (range, 1)
    else:
        counter[id] = (range, counter[id][1] + 1)

    log_msg = ''
    for key in counter:
        log_msg = log_msg + '[%3d-%.2fm-%4d] ' % (key, counter[key][0], counter[key][1])

    rospy.loginfo(log_msg)
    return


rospy.init_node('beacon_observer_node')

topic_beacon = rospy.get_param('~beacon_topic_name', 'beacon')

rospy.loginfo('Subscribing to topic: %s', topic_beacon)
rospy.Subscriber(topic_beacon, ObservationRangeBeacon, beacon_callback)

rospy.loginfo('Waiting for new messages.')
rospy.spin()

rospy.loginfo('Time to rest, zzzZZzzzZZZ.')