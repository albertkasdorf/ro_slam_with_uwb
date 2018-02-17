#!/usr/bin/env python
#------------------------------------------------------------------------------
#
# Zusammenfuehren der einzelnen Beacon Nachrichten zu einer.
#
#------------------------------------------------------------------------------


import rospy


from mrpt_msgs.msg import ObservationRangeBeacon
from mrpt_msgs.msg import SingleRangeBeaconObservation


pub = None
beacon_collection = {}
beacon_count = 0


def beacon_callback(msg):
    global pub, beacon_collection, beacon_count

    id = msg.sensed_data[0].id
    range = msg.sensed_data[0].range
    beacon_collection[id] = range

    if len(beacon_collection) >= beacon_count:
        orb = ObservationRangeBeacon()

        orb.header.frame_id = msg.header.frame_id
        orb.header.stamp = rospy.Time.now()

        orb.sensor_pose_on_robot.position.x = msg.sensor_pose_on_robot.position.x
        orb.sensor_pose_on_robot.position.y = msg.sensor_pose_on_robot.position.y
        orb.sensor_pose_on_robot.position.z = msg.sensor_pose_on_robot.position.z
        
        orb.sensor_pose_on_robot.orientation.x = msg.sensor_pose_on_robot.orientation.x
        orb.sensor_pose_on_robot.orientation.y = msg.sensor_pose_on_robot.orientation.y
        orb.sensor_pose_on_robot.orientation.z = msg.sensor_pose_on_robot.orientation.z
        orb.sensor_pose_on_robot.orientation.w = msg.sensor_pose_on_robot.orientation.w
        
        orb.min_sensor_distance = msg.min_sensor_distance
        orb.max_sensor_distance = msg.max_sensor_distance
        
        orb.sensor_std_range = msg.sensor_std_range

        i = 0

        for key in beacon_collection:
            orb.sensed_data.append(SingleRangeBeaconObservation())
            orb.sensed_data[i].id = key
            orb.sensed_data[i].range = beacon_collection[key]
            i = i + 1
        
        beacon_collection = {}
        pub.publish(orb)

    return


rospy.init_node('beacon_reunition_node')

in_topic = rospy.get_param('~in_topic_name', 'beacon')
out_topic = rospy.get_param('~out_topic_name', 'beacon_reunited')
beacon_count = rospy.get_param('~beacon_count', 4)

pub = rospy.Publisher(
    out_topic, ObservationRangeBeacon, queue_size=10)

rospy.loginfo('Subscribing to topic: %s', in_topic)
rospy.Subscriber(in_topic, ObservationRangeBeacon, beacon_callback)

rospy.loginfo('Waiting for new messages.')
rospy.spin()

rospy.loginfo('Time to rest, zzzZZzzzZZZ.')