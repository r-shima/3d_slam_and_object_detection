# jackal_3d_slam
This package equips the Clearpath Jackal UGV with perception, localization, and mapping capabilities for navigation. Specifically, it utilizes RTAB-Map for 3D SLAM, Nav2 for autonomous navigation, and YOLOv7 for real-time object detection. In addition, it allows you to perform point cloud processing.
## Usage
Before using this package, you must set up the Jackal on ROS2 Humble. Follow the instructions [here](https://github.com/r-shima/jackal_ros2_humble) to get the Jackal up and running.
## Executables
* `filter`: This runs the `point_cloud_processing` node, which removes and downsamples noisy point cloud data using PassThrough, RadiusOutlierRemoval, and VoxelGrid filters. The node subscribes to `/velodyne_points` and publishes sensor_msgs/msg/PointCloud2 messages to `/filtered_velodyne_points`.
* `object_detection.py`: This runs YOLOv7 to perform object detection. It is based on the code in a separate repository [here](https://github.com/r-shima/YOLOv7_ROS2/tree/rshima).
## Launch Files
* `ros2 launch jackal_3d_slam filtered_velodyne.launch.py` allows RTAB-Map to use the filtered point cloud to generate an occupancy grid map. The arguments are the following:
    * `use_sim_time`: Uses simulation clock. Default is false.
    * `deskewing`: Enables LiDAR deskewing. Default is true.

* `ros2 launch jackal_3d_slam jackal_nav.launch.py` loads the Nav2 parameters that are modified to work with the Jackal
* `ros2 launch jackal_3d_slam jackal_transform.launch.py` publishes a static transform between `base_link` and `velodyne`, and runs RTAB-Map. The arguments are the following:
    * `publish_static_tf`: Publishes a static transform between `base_link` and `velodyne`. Default is true.
    * `use_unfiltered`: Launches RTAB-Map with unfiltered point cloud. Default is false.
    * `use_filtered`: Launches RTAB-Map with filtered point cloud. Default is false.

* `ros2 launch jackal_3d_slam object_detection.launch.xml` runs YOLOv7 object detection on a RealSense camera
* `ros2 launch jackal_3d_slam start_3d_slam.launch.xml` starts the Jackal, Velodyne nodes, and Nav2. It runs the `point_cloud_processing` node depending on the value of the argument below.
    * `filter`: Determines whether or not to filter the point cloud. Default is false.

* `ros2 launch jackal_3d_slam velodyne.launch.py` allows RTAB-Map to use the unfiltered point cloud to generate an occupancy grid map. The arguments are the following:
    * `use_sim_time`: Uses simulation clock. Default is false.
    * `deskewing`: Enables LiDAR deskewing. Default is true.

## Parameters
The following parameters in `config/filter_params.yaml` can be used to change the filter settings:
* `x_filter_min`: the minimum value for the PassThrough filter limit in the x direction
* `x_filter_max`: the maximum value for the PassThrough filter limit in the x direction
* `z_filter_min`: the minimum value for the PassThrough filter limit in the z direction
* `z_filter_max`: the maximum value for the PassThrough filter limit in the z direction
* `search_radius`: the sphere radius used for finding the k-nearest neighbors for the RadiusOutlierRemoval filter
* `num_neighbors`: the minimum number of neighbors that a point needs to have for the RadiusOutlierRemoval filter
* `voxel_leaf_size`: the leaf size for the VoxelGrid filter