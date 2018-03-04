#!/usr/bin/env python
#------------------------------------------------------------------------------
#
# Simuliert die existens von virtuellen Beacons um den ro-slam zu testen.
#
#------------------------------------------------------------------------------


import math
import random
import rospy
import tf


from tf import TransformListener
from geometry_msgs.msg import PoseStamped
from mrpt_msgs.msg import ObservationRangeBeacon
from mrpt_msgs.msg import SingleRangeBeaconObservation

sigma = 0.2

def publish_fake_beacon(pub, frame_id, beacon_id, range):
    orb = ObservationRangeBeacon()

    orb.header.frame_id = frame_id
    orb.header.stamp = rospy.Time.now()
    
    orb.sensor_pose_on_robot.position.x = 0
    orb.sensor_pose_on_robot.position.y = 0
    orb.sensor_pose_on_robot.position.z = 0
    orb.sensor_pose_on_robot.orientation.x = 0
    orb.sensor_pose_on_robot.orientation.y = 0
    orb.sensor_pose_on_robot.orientation.z = 0
    orb.sensor_pose_on_robot.orientation.w = 1
    orb.min_sensor_distance = 0.0
    orb.max_sensor_distance = 120.0
    orb.sensor_std_range = 0.02
    orb.sensed_data.append(SingleRangeBeaconObservation())
        
    orb.sensed_data[0].range = range
    orb.sensed_data[0].id = beacon_id

    pub.publish(orb)
    return

def publish_fake_beacon2(pub, frame_id, range1, range2, range3, range4):
    global sigma
    orb = ObservationRangeBeacon()

    orb.header.frame_id = frame_id
    orb.header.stamp = rospy.Time.now()
    
    orb.sensor_pose_on_robot.position.x = 0.04
    orb.sensor_pose_on_robot.position.y = 0
    orb.sensor_pose_on_robot.position.z = 0.26
    orb.sensor_pose_on_robot.orientation.x = 0
    orb.sensor_pose_on_robot.orientation.y = 0
    orb.sensor_pose_on_robot.orientation.z = 0
    orb.sensor_pose_on_robot.orientation.w = 1
    orb.min_sensor_distance = 0.0
    orb.max_sensor_distance = 120.0
    #orb.sensor_std_range = 0.02
    orb.sensor_std_range = sigma
    orb.sensed_data.append(SingleRangeBeaconObservation())
    orb.sensed_data.append(SingleRangeBeaconObservation())
    orb.sensed_data.append(SingleRangeBeaconObservation())
    orb.sensed_data.append(SingleRangeBeaconObservation())
        
    orb.sensed_data[0].range = range1
    orb.sensed_data[0].id = 1

    orb.sensed_data[1].range = range2
    orb.sensed_data[1].id = 2

    orb.sensed_data[2].range = range3
    orb.sensed_data[2].id = 3

    orb.sensed_data[3].range = range4
    orb.sensed_data[3].id = 4

    pub.publish(orb)
    return

def generate_range(position, x, y):
    global sigma

    random_x = random.uniform(x - sigma, x + sigma)
    random_y = random.uniform(y - sigma, y + sigma)

    range = math.sqrt((position.x - random_x) ** 2 + (position.y - random_y) ** 2)
    return range


rospy.init_node('beacon_faker_node')

topic_name = rospy.get_param('~topic_name', 'beacon_fake')
global_frame_id = rospy.get_param('~global_frame_id', 'map')
base_frame_id = rospy.get_param('~base_frame_id', 'base_link')
sensor_frame_id = rospy.get_param('~sensor_frame_id', 'uwb_reciever_link')
#sensor_frame_id = rospy.get_param('~sensor_frame_id', 'base_link')

rospy.loginfo('topic_name: %s', topic_name)
rospy.loginfo('global_frame_id: %s', global_frame_id)
rospy.loginfo('base_frame_id: %s', base_frame_id)

rospy.loginfo('Publishing to topic: %s', topic_name)
pub = rospy.Publisher(topic_name, ObservationRangeBeacon, queue_size=10)

tfl = TransformListener()

rate = rospy.Rate(2)   # 10 Hz
while not rospy.is_shutdown():

    try:
        # (position, quaternion) = tfl.lookupTransform(
        #     base_frame_id,
        #     global_frame_id,
        #     rospy.Time())

        bl = PoseStamped()
        bl.header.frame_id = base_frame_id
        bl.header.stamp = rospy.Time()
        bl.pose.orientation.w = 1.0

        bl_map = tfl.transformPose(global_frame_id, bl)
        #rospy.loginfo('%.2f %.2f' % (bl_map.pose.position.x, bl_map.pose.position.y))

        #range1 = generate_range(bl_map.pose.position, 1.0, 0.0)
        #range2 = generate_range(bl_map.pose.position, 4.0, 0.0)
        #range3 = generate_range(bl_map.pose.position, 4.0, 2.0)
        #range4 = generate_range(bl_map.pose.position, 1.0, 2.0)

        range1 = generate_range(bl_map.pose.position, 1.98,  0.88)
        range2 = generate_range(bl_map.pose.position, 1.98,  1.20)
        range3 = generate_range(bl_map.pose.position, 4.00, -0.12)
        range4 = generate_range(bl_map.pose.position, 5.04,  1.88)

        publish_fake_beacon2(pub, sensor_frame_id, range1, range2, range3, range4)
        rospy.loginfo('%.2f %.2f - %.2f %.2f %.2f %.2f' % (bl_map.pose.position.x, bl_map.pose.position.y, range1, range2, range3, range4))

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass
 
    rate.sleep()


rospy.loginfo('Time to rest, zzzZZzzzZZZ.')
