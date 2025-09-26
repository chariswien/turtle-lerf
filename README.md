# LERF-ROS2 Integration for Turtlebot4

This repository contains the implementation for capturing RGB data from a Turtlebot4, processing it for LERF (Language Embedded Radiance Fields) training, and visualizing semantically-filtered point clouds in ROS2.

## Prerequisites

### 1. Install Requirements
```bash
pip install -r requirements.txt
```

### 2. Start Turtlebot4
Ensure the Turtlebot4 is running with the minimum required topics:
- `/oakd/rgb/image_raw`
- `/oakd/rgb/camera_info`
- `/odom`
- `/tf`
- `/tf_static`
- `/scan`

## Data Capture

### Running on Live Data or ROS2 Bag

1. **Start the data capture subscribers:**
```bash
cd turtle-lerf/implementation/data_postprocessing/pp_ros/
./start_subs.sh
```
This script launches three parallel processes:
- `sub_pose.py`: Captures robot pose data to `baselink_pose.csv`
- `sub_images.py`: Saves images to `images/` directory with timestamps
- `sub_cameraInfo.py`: Captures camera intrinsics to `intrinsics.json`

Press `CTRL+C` to stop all processes when data collection is complete.

2. **Transform poses to camera frame:**
```bash
python3 tf_to_cpose.py baselink_pose.csv
```
This will output a timestamped file: `camera_pose_YYYYMMDD_HHMMSS.csv`

## Data Preparation

3. **Manual Image Curation**
Review the captured images in the `images/` directory and:
- Remove duplicate images
- Remove blurred or poor quality images
- Ensure good coverage of the scene

4. **Create LERF-compatible dataset:**

The kept images will be matched to the corresponding camera poses via timestamps. Therefore, `create_json.py` looks up the nearest neighbor, where the parameter `--max_dt` sets the threshold for matching. Images with timestamps not matchable to a camera pose will be ignored.

```bash
python3 create_json.py \
    --images_dir ./images \
    --timestamps_csv ./images/timestamps.csv \
    --poses_csv ./camera_pose_[timestamp].csv \
    --intrinsics ./intrinsics.json \
    --out ./transforms.json \
    --max_dt 0.15 
```

**Parameters:**
- `--images_dir`: Directory containing the captured images
- `--timestamps_csv`: CSV file with image filenames and timestamps
- `--poses_csv`: Camera poses CSV file (from tf_to_cpose.py)
- `--intrinsics`: Camera intrinsics JSON file
- `--out`: Output path for transforms.json
- `--max_dt`: Maximum time difference (seconds) to match poses to images (default: 0.15)

## LERF Training

5. **Setup LERF Environment**
Follow the instructions at: https://github.com/SimonSchwaiger/nerfstudio-ros-ws

6. **Transfer Files to Container**
Once inside the container, transfer:
- `LERF.ipynb`
- `images/` directory
- `transforms.json`

7. **Run LERF Training**
Open `LERF.ipynb` and update the paths:
```python
data_path = "/path/to/your/input"  # Directory containing transforms.json and images/
output_dir = "/path/to/your/output"
```

Execute the notebook cells to:
- Train the LERF model (100,000 iterations)
- Export point clouds
- Evaluate metrics
- Perform semantic queries and export filtered point clouds

## Visualization in ROS2

8. **Setup Visualization Environment**
On your target machine:

```bash
# Open RViz2 with the provided configuration
rviz2 -d ~/implementation/pcd_visualization.rviz
```

9. **Publish Point Cloud and Map**
```bash
# Publish the LERF point cloud and occupancy grid
python3 lerf_publisher.py pointcloud.ply map.yaml map.pgm

# In another terminal, publish goal poses based on point cloud center
python3 goal_pose.py
```

The `lerf_publisher.py` publishes:
- Point cloud on `/pointcloud` topic
- Occupancy grid on `/map` topic

The `goal_pose.py` computes the center of the `/pointcloud` and publishes it as a goal pose on `/goal_pose`.

## Output Structure

After processing, you'll have:
```
project/
├── images/                    # Captured RGB images
├── timestamps.csv             # Image timestamps
├── baselink_pose.csv         # Robot poses
├── camera_pose_*.csv         # Camera frame poses
├── intrinsics.json           # Camera parameters
├── transforms.json           # LERF-ready dataset
└── output/
    ├── exports/
    │   ├── point_cloud.ply  # Dense reconstruction
    │   └── metrics.json      # Evaluation metrics
    └── semantics/
        └── *_filtered.ply    # Semantically filtered point clouds
```

## Notes

- The system uses the OAK-D camera's RGB stream for LERF training
- Coordinate transformations handle the robot's base_link to camera optical frame conversion

## Troubleshooting

- Ensure all ROS2 topics are publishing before starting data capture
- Check that timestamp synchronization tolerance (`max_dt`) is appropriate for your system
- For better LERF results, ensure good image coverage with minimal blur
- Adjust the semantic query threshold in the notebook based on your specific use case. You may need to try out different scales and threshold

## Citation

If you use this work in your research, please cite:
```bibtex
@inproceedings{lerf2023,
  title={LERF: Language Embedded Radiance Fields},
  author={Kerr, Justin and Kim, Chung Min and Goldberg, Ken and Kanazawa, Angjoo and Tancik, Matthew},
  booktitle={International Conference on Computer Vision (ICCV)},
  year={2023}
}
```