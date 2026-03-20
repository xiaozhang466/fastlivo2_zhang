
import rosbag
import sys

def check_timestamps(bag_path):
    bag = rosbag.Bag(bag_path)
    
    last_cam_stamp = None
    last_lidar_stamp = None
    
    cam_stamps = []
    lidar_stamps = []
    
    print(f"Reading {bag_path}...")
    
    for topic, msg, t in bag.read_messages(topics=['/left_camera/image', '/lslidar_point_cloud']):
        if topic == '/left_camera/image':
            cam_stamps.append(msg.header.stamp)
        elif topic == '/lslidar_point_cloud':
            lidar_stamps.append(msg.header.stamp)
            
    bag.close()
    
    print(f"\nTotal Camera Frames: {len(cam_stamps)}")
    print(f"Total LiDAR Frames: {len(lidar_stamps)}")
    
    print("\n--- Last 5 Camera Timestamps ---")
    for i in range(max(0, len(cam_stamps)-5), len(cam_stamps)):
        print(f"Frame {i}: {cam_stamps[i].to_sec():.6f}")
        
    print("\n--- Last 5 LiDAR Timestamps ---")
    for i in range(max(0, len(lidar_stamps)-5), len(lidar_stamps)):
        print(f"Frame {i}: {lidar_stamps[i].to_sec():.6f}")

    if cam_stamps and lidar_stamps:
        diff = cam_stamps[-1].to_sec() - lidar_stamps[-1].to_sec()
        print(f"\nDifference (Last Cam - Last LiDAR): {diff:.6f} sec")
        
        # Check for duplicates in camera
        if len(cam_stamps) >= 2:
            if cam_stamps[-1] == cam_stamps[-2]:
                 print("\n[WARNING] The last two camera frames have IDENTICAL timestamps!")
            else:
                 print(f"\nCamera last frame interval: {cam_stamps[-1].to_sec() - cam_stamps[-2].to_sec():.6f} sec")

if __name__ == "__main__":
    check_timestamps("/home/ros/fastlivo2_ws/data/livo_dataset3.bag")
