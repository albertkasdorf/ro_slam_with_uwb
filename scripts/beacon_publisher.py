#!/usr/bin/env python

import json
import rospy
import serial

from mrpt_msgs.msg import ObservationRangeBeacon
from mrpt_msgs.msg import SingleRangeBeaconObservation
from ro_slam_with_uwb.msg import Beacon

from dynamic_reconfigure.server import Server
from ro_slam_with_uwb.cfg import BeaconPublisherConfig

ser = None

def config_callback(config, level):
    global ser

    rospy.loginfo("""Reconfiugre Request: {antenna_delay}""".format(**config))
    if ser is not None:
        adb = str(config.antenna_delay).encode()
        ser.write(adb)
        rospy.loginfo("Antenna delay is transfered.")
    else:
        rospy.loginfo("Serial is not open.")

    return config

def main():
    global ser

    rospy.init_node('beacon_publisher_node')

    pub = rospy.Publisher('beacon', ObservationRangeBeacon, queue_size=10)
    pub_raw = rospy.Publisher('beacon_raw', Beacon, queue_size=10)

    antenna_delay = rospy.get_param('~antenna_delay', 16450)
    sensor_frame_id = rospy.get_param('~sensor_frame_id', 'uwb_reciever_link')
    serial_port = rospy.get_param('~serial_port', '/dev/CP2104_Friend')

    srv = Server(BeaconPublisherConfig, config_callback)
    ser = serial.Serial(serial_port, 115200, timeout=2)

    # Set inital antenna delay
    while True:
        line = ser.readline()
        line = line.decode("utf-8")
        #print line
        if line.startswith("### TAG ###"):
            ser.write(str(antenna_delay).encode())
            rospy.loginfo("Antenna delay is transfered.")
            break

    orb = ObservationRangeBeacon()
    orb.header.frame_id = sensor_frame_id
    orb.sensor_pose_on_robot.position.x = 0.05
    orb.sensor_pose_on_robot.position.y = 0
    orb.sensor_pose_on_robot.position.z = 0.2
    orb.sensor_pose_on_robot.orientation.x = 0
    orb.sensor_pose_on_robot.orientation.y = 0
    orb.sensor_pose_on_robot.orientation.z = 0
    orb.sensor_pose_on_robot.orientation.w = 1
    orb.min_sensor_distance = 0.0
    orb.max_sensor_distance = 120.0
    orb.sensor_std_range = 0.1
    orb.sensed_data.append(SingleRangeBeaconObservation())

    beacon = Beacon()
    beacon.header.frame_id = sensor_frame_id

    rospy.loginfo("Starting main loop...")
    while not rospy.is_shutdown():
        line = ser.readline()
        try:
            #print(line)
            obj = json.loads(line)
        except ValueError:
            continue

        orb.header.stamp = rospy.Time.now()
        orb.sensed_data[0].range = obj['r']
        orb.sensed_data[0].id = obj['aa']
        pub.publish(orb)

        beacon.header.stamp = orb.header.stamp
        beacon.type = obj['type']
        beacon.tag_address = obj['ta']
        beacon.anchor_address = obj['aa']
        beacon.range = obj['r']
        beacon.receive_power = obj['rxp']
        beacon.first_path_power = obj['fpp']
        beacon.receive_quality = obj['q']
        beacon.voltage = obj['v']
        beacon.temperature = obj['t']
        beacon.tag_antenna_delay = obj['ad']
        pub_raw.publish(beacon)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
