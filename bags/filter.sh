#!/bin/sh
# Remove tf_static and tf with odom to base_link
# $1 <in-bag>
# $2 <out-bag>
rosbag filter $1 $2 '(topic != "/tf" and topic != "/tf_static") or topic == "/tf" and m.transforms[0].header.frame_id != "odom" and m.transforms[0].child_frame_id != "base_link"'

# A backup of each bag file (with the extension .orig.bag) is made before the bag is compressed. Use LZ4 to compress data.
rosbag compress --lz4 $2