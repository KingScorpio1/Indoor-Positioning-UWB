import rosbag
import pandas as pd
import math
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_matrix, translation_matrix, concatenate_matrices

# --- Configuration ---
bag_file = 'lidar_pose_data.bag'
csv_file = 'lidar_2d_poses_revised.csv'
map_frame = 'map'
odom_frame = 'odom'
base_frame = 'base_footprint'

data = []

def get_transform_matrix(transform_stamped):
    """Converts a TransformStamped message into a 4x4 numpy matrix."""
    trans = transform_stamped.transform.translation
    rot = transform_stamped.transform.rotation
    
    # Create translation matrix
    t_mat = translation_matrix([trans.x, trans.y, trans.z])
    
    # Create rotation matrix from quaternion
    r_mat = quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
    
    # Multiply them to get the full transformation matrix
    return concatenate_matrices(t_mat, r_mat)

print(f"Opening rosbag: {bag_file}")

try:
    with rosbag.Bag(bag_file, 'r') as bag:
        print("Processing TF messages...")
        
        # Store latest transforms
        latest_map_to_odom = None
        latest_odom_to_base = None

        for topic, msg, t in bag.read_messages(topics=['/tf']):
            for transform in msg.transforms:
                
                # Update our latest known transforms
                if transform.header.frame_id == map_frame and transform.child_frame_id == odom_frame:
                    latest_map_to_odom = transform
                elif transform.header.frame_id == odom_frame and transform.child_frame_id == base_frame:
                    latest_odom_to_base = transform

                # We only calculate a position if we have BOTH parts of the chain
                if latest_map_to_odom and latest_odom_to_base:
                    
                    # 1. Convert ROS messages to Matrices
                    mat_map_odom = get_transform_matrix(latest_map_to_odom)
                    mat_odom_base = get_transform_matrix(latest_odom_to_base)
                    
                    # 2. Multiply matrices to get Map -> Base_Footprint
                    # Result = (Map->Odom) * (Odom->Base)
                    mat_map_base = np.dot(mat_map_odom, mat_odom_base)
                    
                    # 3. Extract X, Y from the resulting matrix
                    # The translation is in the last column (indices 0,3; 1,3; 2,3)
                    final_x = mat_map_base[0, 3]
                    final_y = mat_map_base[1, 3]
                    
                    # 4. Extract Yaw from the resulting matrix
                    # We convert the rotation part of the matrix back to Euler angles
                    (roll, pitch, yaw) = euler_from_quaternion([
                        0, 0, 0, 1 # Dummy quaternion, we extract from matrix directly below
                    ], axes='sxyz')
                    
                    # Standard way to get Euler from Matrix in ROS tf
                    from tf.transformations import euler_from_matrix
                    (roll, pitch, yaw) = euler_from_matrix(mat_map_base)

                    # 5. Append Data
                    # We use the timestamp of the base_footprint update because that is the 'movement'
                    data.append({
                        'timestamp': latest_odom_to_base.header.stamp.to_sec(),
                        'x': final_x,   # This is now the CORRECTED Map coordinate
                        'y': final_y,
                        'yaw': yaw
                    })

    print(f"Finished. Collected {len(data)} corrected pose entries.")
    
    # Save to CSV
    if data:
        df = pd.DataFrame(data)
        # Drop duplicates timestamps to clean up the data (TF publishes at high rates)
        df = df.drop_duplicates(subset=['timestamp'])
        df.to_csv(csv_file, index=False)
        print(f"Successfully saved to {csv_file}")
    else:
        print("No valid transform chain found.")

except Exception as e:
    print(f"Error: {e}")
