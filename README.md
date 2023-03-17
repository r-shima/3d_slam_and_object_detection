# 3D SLAM and Object Detection for Jackal Navigation
The goal of this project is to equip the Clearpath Jackal UGV with perception, localization, and mapping capabilities for navigation. For more details, please go to my [portfolio post](https://r-shima.github.io/jackal_3d_slam.html).
## Package
[jackal_3d_slam](https://github.com/r-shima/3d_slam_and_object_detection/tree/main/jackal_3d_slam): This is a ROS2 package that implements 3D SLAM using RTAB-Map, autonomous navigation using Nav2, and real-time object detection using YOLOv7 on the Clearpath Jackal UGV. It also implements point cloud processing.
## Hardware
* Clearpath Jackal UGV
* Intel RealSense D435i
* Velodyne LiDAR VLP-16
## Software
* ROS2 Humble
* RTAB-Map
* Nav2
* YOLOv7
* Point Cloud Library (PCL)
## Dependencies
Install with `sudo apt install`:
* ros-humble-rtabmap
* ros-humble-rtabmap-ros
* ros-humble-navigation2
* ros-humble-nav2-bringup
* ros-humble-perception-pcl

Please refer to the next section for the dependencies related to the Jackal.
## Setting Up the Jackal on ROS2 Humble
The additional part of this project was bringing the Jackal up to a running state on ROS2 Humble. The instructions are provided in a separate repository, which is available [here](https://github.com/r-shima/jackal_ros2_humble). Additional dependencies related to the Jackal are listed there.
## Quickstart
1. After setting up the Jackal, clone this repository in the src directory of your workspace on the Jackal's computer. In your workspace, build the package by running `colcon build`.
2. From your computer, SSH into the Jackal's computer (instructions are provided in the repository mentioned before). Go to your workspace and source it by running `source install/setup.bash`.
3. Run `ros2 launch jackal_3d_slam jackal_transform.launch.py use_filtered:=true` to launch RTAB-Map with filtered point cloud data. Otherwise, run `ros2 launch jackal_3d_slam jackal_transform.launch.py use_unfiltered:=true`.
4. Open a new terminal on your computer and SSH into the Jackal's computer again. Go to your workspace, source it, and run `ros2 launch jackal_3d_slam start_3d_slam.launch.xml filter:=true` to filter the point cloud. Otherwise, run `ros2 launch jackal_3d_slam start_3d_slam.launch.xml`.
5. Open a new terminal on your computer, go to your workspace, source it, and run `ros2 launch nav2_bringup rviz_launch.py`. You should now be able to send goals to the Jackal on RViz.
## Demo

[Alt-Text](https://user-images.githubusercontent.com/113070827/225349395-0dd1a5d8-b876-47b1-9f97-bb60d8b0e50d.mp4)

The corresponding video of the Jackal navigating in the real world can be found [here](https://www.youtube.com/watch?v=7uk2VOqHJxs).