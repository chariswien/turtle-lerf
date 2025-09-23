#!/usr/bin/env python3

import csv
import numpy as np
import sys
from datetime import datetime

def quat_to_R(x, y, z, w):

    norm = np.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    
    R = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ])
    return R

def R_to_quat(R):
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    
    return np.array([x, y, z, w])

def create_T(translation, quaternion=None):
   
    T = np.eye(4)
    T[0:3, 3] = translation
    if quaternion is not None:
        T[0:3, 0:3] = quat_to_R(*quaternion)
    return T

def invert_T(T):
    T_inv = np.eye(4)
    R = T[0:3, 0:3]
    t = T[0:3, 3]
    T_inv[0:3, 0:3] = R.T # transponiert = inverse
    T_inv[0:3, 3] = -R.T @ t
    return T_inv

def transform_pose(baseLinkPosePosition, baseLinkPoseOrientation, T_baselink_to_camera):
    # Erstelle Transformation von map zu base_link (aktuelle Roboterpose)
    T_map_to_baselink = create_T(
        baseLinkPosePosition,
        baseLinkPoseOrientation
    )
    
    # tf: map -> camenra
    T_map_to_camera = T_map_to_baselink @ T_baselink_to_camera
    
    camera_position = T_map_to_camera[0:3, 3]
    camera_orientation = R_to_quat(T_map_to_camera[0:3, 0:3])
    
    return camera_position, camera_orientation

def main():
    if len(sys.argv) != 2:
        print("Expected: python3 transform_pose_csv.py input_pose_data.csv")
        sys.exit(1)
    
    input_file = sys.argv[1]
        
    # tf-kette base_link -> oakd_rgb_camera_optical_frame
    
    T_base_to_shell = create_T([0, 0, 0.0942])
    T_shell_to_bracket = create_T([-0.118, 0, 0.05257])
    T_bracket_to_oakd = create_T([0.0584, 0, 0.09676])
    T_oakd_to_rgb = create_T([0, 0, 0])
    T_rgb_to_optical = create_T([0, 0, 0])
    
    # alle tf's
    T_base_to_optical = T_base_to_shell @ T_shell_to_bracket @ T_bracket_to_oakd @ T_oakd_to_rgb @ T_rgb_to_optical
    
    print(f"Reading poses from: {input_file}")
    
    # output filename
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_file = f'camera_pose_{timestamp}.csv'
    
    with open(input_file, 'r') as infile, open(output_file, 'w', newline='') as outfile:
        reader = csv.DictReader(infile)
        writer = csv.writer(outfile)
        
        # header row
        writer.writerow([
            'timestamp', 'frame_id',
            'pos_x', 'pos_y', 'pos_z',
            'or_x', 'or_y', 'or_z', 'or_w'
        ])
        
        count = 0
        for row in reader:
            # get pose from input csv
            timestamp = row['timestamp']
            position = [float(row['pos_x']), float(row['pos_y']), float(row['pos_z'])]
            orientation = [float(row['or_x']), float(row['or_y']), float(row['or_z']), float(row['or_w'])]
            
            # Ttf
            new_position, new_orientation = transform_pose(position, orientation, T_base_to_optical)
            
            # write tf pose
            writer.writerow([
                timestamp, 'oakd_rgb_camera_optical_frame',
                new_position[0], new_position[1], new_position[2],
                new_orientation[0], new_orientation[1], new_orientation[2], new_orientation[3]
            ])
            
            count += 1
            if count % 100 == 0:
                print(f"Processed {count} poses...")
        
    print(f"\nTransformation complete!")
    print(f"  Total poses processed: {count}")
    print(f"  Output saved to: {output_file}")
    
if __name__ == '__main__':
    main()