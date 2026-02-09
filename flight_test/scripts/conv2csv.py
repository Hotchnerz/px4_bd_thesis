import rosbag
import csv

def convert_mocap_bag_to_csv(bag_file):
    bag = rosbag.Bag(bag_file)
    
    # Open CSV files for writing
    ground_pose_file = open('ground_pose.csv', 'w')
    pose_file = open('pose_stamped.csv', 'w')
    
    ground_writer = csv.writer(ground_pose_file)
    pose_writer = csv.writer(pose_file)
    
    # Write headers
    ground_writer.writerow(['timestamp', 'x', 'y', 'theta'])
    pose_writer.writerow(['timestamp', 'header_stamp', 'frame_id', 'position_x', 'position_y', 'position_z', 
                         'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w'])
    
    ground_count = 0
    pose_count = 0
    
    for topic, msg, t in bag.read_messages():
        timestamp = t.to_sec()
        
        if topic == '/mocap_node/Robot_1/ground_pose':
            ground_writer.writerow([timestamp, msg.x, msg.y, msg.theta])
            ground_count += 1
            
        elif topic == '/mocap_node/Robot_1/pose':
            pose_writer.writerow([timestamp, msg.header.stamp.to_sec(), msg.header.frame_id,
                                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                                msg.pose.orientation.x, msg.pose.orientation.y, 
                                msg.pose.orientation.z, msg.pose.orientation.w])
            pose_count += 1
    
    # Close files
    ground_pose_file.close()
    pose_file.close()
    
    # Print results using old string formatting
    print("Saved {} ground pose messages to ground_pose.csv".format(ground_count))
    print("Saved {} pose messages to pose_stamped.csv".format(pose_count))
    
    bag.close()

# Usage
convert_mocap_bag_to_csv('/home/raddev/mocap_bag/2025-09-12-15-21-04.bag')