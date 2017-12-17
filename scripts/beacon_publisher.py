#!/usr/bin/env python

import json
import rospy
import serial

from mrpt_msgs.msg import ObservationRangeBeacon
from mrpt_msgs.msg import SingleRangeBeaconObservation
from scipy import interpolate

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

    pub = rospy.Publisher('/beacon', ObservationRangeBeacon, queue_size=10)
    pub_raw = rospy.Publisher('/beacon_raw', ObservationRangeBeacon, queue_size=10)
    rospy.init_node('beacon_publisher')
    antenna_delay = rospy.get_param('~antenna_delay', 16450)

    srv = Server(BeaconPublisherConfig, config_callback)
    ser = serial.Serial('/dev/CP2104_Friend', 115200, timeout=2)

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
    orb.header.frame_id = 'uwb_reciever_link'
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

    rospy.loginfo("Starting main loop...")
    while not rospy.is_shutdown():
        line = ser.readline()
        try:
            #print(line)
            obj = json.loads(line)
        except ValueError:
            continue

        orb.header.stamp = rospy.Time.now()
        orb.header.seq = orb.header.seq + 1
        orb.sensed_data[0].range = obj["r"]
        orb.sensed_data[0].id = obj["aa"]
        pub_raw.publish(orb)

        orb.sensed_data[0].range = correction(obj["r"])
        pub.publish(orb)

def correction(value):
    x = [0.504,0.603,0.696,0.807,0.928,1.084,1.193,1.305,1.418,1.496,1.596,1.710,1.820,1.900,2.019,2.133,2.233,2.323,2.434,2.523,2.640,2.739,2.856,2.971,3.066,3.166,3.249,3.351,3.451,3.568,3.665,3.740,3.875,3.910,4.108,4.232,4.261,4.355,4.300,4.490,4.554]
    #y = [0.05,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0,1.10,1.20,1.30,1.40,1.50,1.60,1.70,1.80,1.90,2.00,2.10,2.20,2.30,2.40,2.50,2.60,2.70,2.80,2.90,3.00,3.10,3.20,3.30,3.40,3.50,3.60,3.70,3.80,3.90,4.00]
    y = [0.454,0.503,0.496,0.507,0.528,0.584,0.593,0.605,0.618,0.596,0.596,0.610,0.620,0.600,0.619,0.633,0.633,0.623,0.634,0.623,0.640,0.639,0.656,0.671,0.666,0.666,0.649,0.651,0.651,0.668,0.665,0.640,0.675,0.610,0.708,0.732,0.661,0.655,0.500,0.590,0.554]
    f = interpolate.interp1d(x, y, kind='linear')

    if value >= x[0] and value <= x[-1]:
        return max(value - f(value), 0)
    else:
        return max(value - 0.615, 0)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
