#!/usr/bin/env python3
import rosbag
import rospy
import sys

def fix_bag(input_bag, output_bag):
    print(f"Processing {input_bag} -> {output_bag}...")
    
    with rosbag.Bag(output_bag, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(input_bag).read_messages():
            if topic == "/lslidar_point_cloud" or topic == "/left_camera/image":
                # The LiDAR/Camera header stamp is wrong (boot time). 
                # We replace it with the recording time 't' (Unix time).
                # Note: 't' is a rospy.Time object.
                msg.header.stamp = t
                outbag.write(topic, msg, t)
            else:
                outbag.write(topic, msg, t)
    
    print("Done! New bag saved as " + output_bag)

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 fix_timestamp.py input.bag output.bag")
    else:
        fix_bag(sys.argv[1], sys.argv[2])
