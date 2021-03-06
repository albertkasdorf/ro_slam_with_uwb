#!/usr/bin/env python
#------------------------------------------------------------------------------
# Bestimmen der Trajektorie ueber den Service-Aufruf von GetRobotTrajectory
# und speichern jeder Pose in einer CSV-Datei.
#
# Confused by python file mode "w+"
# https://stackoverflow.com/questions/16208206/confused-by-python-file-mode-w
#
# CLI:
#   ./trajectory_to_csv.py file_name.csv
#------------------------------------------------------------------------------

import sys
import csv
import rospy

from nav_msgs.msg import Path
from hector_nav_msgs.srv import GetRobotTrajectory

csv_file_name = "file_name.csv"

if len(sys.argv) == 2:
    csv_file_name = sys.argv[1]
else:
    print "%s [csv_file_name]"%sys.argv[0]
    sys.exit(1)

rospy.wait_for_service('trajectory')
get_robot_trajectory = rospy.ServiceProxy('trajectory', GetRobotTrajectory)
response = get_robot_trajectory()

with open(csv_file_name, "w") as csv_file:
    csv_writer = csv.writer(csv_file, delimiter=';', lineterminator='\n')
    csv_writer.writerow(["time","sec","nsec","x", "y", "z", "qx", "qy", "qz", "qw"])

    for pose in response.trajectory.poses:
        t = rospy.Time(
            pose.header.stamp.secs, pose.header.stamp.nsecs)

        csv_writer.writerow([
            t.to_time(),
            pose.header.stamp.secs,
            pose.header.stamp.nsecs,
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w])
