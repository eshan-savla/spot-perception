# RTAB-Map

RTAB-Map (Real-Time Appearance-Based Mapping) is an RGB-D Graph SLAM (Simultaneous Localization and Mapping) method for map recording that utilizes a global Bayesian loop closure detector. This detector employs a bag-of-words approach to assess the likelihood of whether a new image corresponds to a previously visited location or a new one. When a loop closure hypothesis is confirmed, a new constraint is introduced into the map's graph, and a graph optimizer works to minimize errors in the map. A memory management strategy is implemented to restrict the number of locations used for loop closure detection and graph optimization, ensuring that real-time performance is maintained even in large-scale environments.

The documentation and a few tutorials for RTAB-Map can be found on the following page: [RTAB-Map ROS Wiki](http://wiki.ros.org/rtabmap_ros/noetic_and_newer)

![map](https://github.com/user-attachments/assets/2822ff93-fce2-4c2d-a73a-e3b6e898cb43)

## Overview of architecture

Using RTAB-Map within this package enables the capability to map environments using the depth cameras integrated into Spot mini. The decision to use RTAB-Map was made because nav2 Navigation Stack does not support depth cameras as sensor inputs and RTAB-Map allows for 3D mapping. This 3D mapping capability holds greater potential for navigating Spot mini in multi-story buildings. RTAB-Map supports integrating various sensors and odometry sources for mapping. Additionally, RTAB-Map provides tools to optimize recorded maps to enhance navigation and can also operate as a navigation instance providing the optimized map to the nav2 Navigation Stack.

![rtab](https://github.com/user-attachments/assets/be548011-e740-4124-b5db-584e9ca098a2)

## Installation and Configuration of RTAB-Map

RTAB-Map can be installed as an apt package with the following command:
```bash
sudo apt install ros-$ROS_DISTRO-rtabmap-ros
```

Following the installation, the package should be configured for use with Spot mini in `entrypoint.sh`. The following lines are responsible for starting the RTAB-Map package. Only the following arguments should be passed. Additional arguments may compromise the smooth operation with Spot mini.

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
    rgb_topic:=/hkaspot/camera/back/image \
    depth_topic:=/hkaspot/depth_registered/back/image \
    camera_info_topic:=/hkaspot/camera/back/camera_info \
    database_path:=/home/robot/spot-perception/map/rtabmap_copy.db \
    odom_frame_id:=hkaspot/odom \
    odom_topic:=/hkaspot/odometry \
    visual_odometry:=false \
    frame_id:=hkaspot/body \
    subscribe_depth:=false \
    subscribe_rgbd:=false \
    rgbd_sync:=true \
    approx_sync:=true \
    queue_size:=1000 \
    qos:=1 \
    rviz:=true \
    rtabmap_viz:=true \
    localization:=true
```

Below, the entries of the entrypoint are further explained.

### Camera Information

```bash
rgb_topic:=/hkaspot/camera/back/image \
```

Here, the corresponding camera topic is passed to RTAB-Map. Spot mini has five depth cameras positioned at various locations. At the time of this project, RTAB-Map supports recording with only one camera signal, so not all five cameras can be used. Due to the fact that the raw data from the two front cameras are rotated by 90 degrees, the rear camera is chosen here. Since the robot's odometry is also used for recording, this orientation does not affect its navigation.

```bash
depth_topic:=/hkaspot/depth_registered/back/image \ 
```

This topic is used to pass depth data to RTAB-Map. Depth data and RGB data are synchronized with the following line.

```bash
rgbd_sync:=true \
```

The following line should be used to pass camera information to RTAB-Map in order to correctly create point clouds:

```bash
camera_info_topic:=/hkaspot/camera/back/camera_info \
```

### Orientation and Odometry

For mapping, odometry is necessary to continuously track the robot's position and orientation. This can be achieved in two ways. Firstly, RTAB-Map can determine odometry directly from the provided image data. Secondly, if available, the robot's odometry information can be directly fed into RTAB-Map.

The determination of odometry via the camera images is only recommended as long as the environment has a lot of visual structure and features. If this is not the case, RTAB-Map will not be able to continuously calculate the odometry (also shown in the visualisation) and the odometry data of the robot should be used for this. As these are available via the Ros driver for Spot mini, this has been done here. The topic for the odometry is set in the following line:

```bash
odom_topic:=/hkaspot/odometry \
```

The frame in which the odometry data is given should also be set:

```bash
odom_frame_id:=hkaspot/odom \ 
```

In addition, the calculation of visual odometries from the camera data must be explicitly deactivated:

```bash
visual_odometry:=false \ 
```

The frame ID in which the map is recorded must also be configured:

```bash
frame_id:=hkaspot/body \
```

### Visualization via RTAB-Tool and RViz

To record the map, you can start your own visualisation provided by RTAB-Map as well as RViz:

```bash
rviz:=true
```

```bash
rviz:=false
```

The image below shows the RTAB-Map visualisation tool. In the right half, the map created in 3D and the path of Spot mini can be viewed. The left half provides information on loop closure detection and odometry. 

![Screenshot from 2024-07-04 14-17-16](https://github.com/user-attachments/assets/2182d1c3-c06a-4099-b14d-c80a8f16ea6a)

Loop Closure Detection checks whether locations that have already passed can be recognised. If this is the case, the odometry data is matched, which can lead to a better-resolved map in terms of position and orientation. 

The odometry window should never appear dark red if the odometry data is fed in directly. If this is the case, the visualisation indicates that the odometry could not be calculated from the image data at this time, which consequently suggests that the odometry data of the robot is not being used, which can lead to some inaccuracies depending on the environment.


Parallel to the visualisation of RTAB-Map itself, the map can also be viewed in RViz parallel to the recording.

![Screenshot from 2024-07-04 14-16-29](https://github.com/user-attachments/assets/e7123188-61c9-4858-a218-deacc4f9e5d1)

RViz is automatically started correctly configured. The three-dimensional map can also be seen as a point cloud. The paths already travelled are also shown here. Below the point cloud on one level, you can also see the two-dimensional map, which you can then navigate through using nav2 Navigation Stack.


### Saving of Map (Database)

The storage path for the database should be configured with this entry both for recording the map and for navigation:

```bash
database_path:=/home/robot/spot-perception/map/rtabmap_copy.db \
```

As the application runs in Docker containers, this file path should also be mounted externally on the PC!


### Mapping or Navigation mode

As already mentioned in Overview, RTAB-Map should also be started for navigation in a map. To record a map itself, the following launch parameter should be set to FALSE:

```bash
localization:=false
```

For navigation with nav2 Navigation Stack, RTAB-Map provides the map and the localization of the Spot mini. The parameter should then be set to TRUE:

```bash
localization:=true
```

All other launch parameters should also be set as shown in the extract above.


## Recording of Map

To record a map, the container should be started with the following configuration in ‘entrypoint.sh’:

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
    rgb_topic:=/hkaspot/camera/back/image \
    depth_topic:=/hkaspot/depth_registered/back/image \
    camera_info_topic:=/hkaspot/camera/back/camera_info \
    database_path:=/home/robot/spot-perception/map/rtabmap_copy.db \
    odom_frame_id:=hkaspot/odom \
    odom_topic:=/hkaspot/odometry \
    visual_odometry:=false \
    frame_id:=hkaspot/body \
    subscribe_depth:=false \
    subscribe_rgbd:=false \
    rgbd_sync:=true \
    approx_sync:=true \
    queue_size:=1000 \
    qos:=1 \
    rviz:=true \
    rtabmap_viz:=true \
    localization:=false
```

As you can see, localization has been deactivated so that RTAB-Map starts in mapping mode. RViz and the RTAB-Map visualisation should now start. Recording is then already active. 

To record, Spot mini should be controlled manually using the Boston Dynamics remote control. The map should now be three-dimensional in both visualisations. 

Due to the rather low resolution of the Spot internal cameras of 640x480, the robot should not be moved too quickly. It can also be useful to rotate the robot at various points. This allows more image data to be taken into account when creating the map. Care should also be taken to ensure that the robot moves smoothly and experiences few vibrations, as these can affect the accuracy of the odometry data and thus cause errors in the localization on the map. 

The database for the map is constantly updated during recording. Nevertheless, it is advisable to save the map again at the end using the RTAB map visualisation.


## Navigation

As already mentioned, path planning and navigation is handled by nav2 Navigation Stack. However, an instance of RTAB-Map must be started so that the map is made available to nav2 via a topic. To do this, the entry in ‘entrypoint.sh’ must be configured as follows:

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
    rgb_topic:=/hkaspot/camera/back/image \
    depth_topic:=/hkaspot/depth_registered/back/image \
    camera_info_topic:=/hkaspot/camera/back/camera_info \
    database_path:=/home/robot/spot-perception/map/rtabmap_copy.db \
    odom_frame_id:=hkaspot/odom \
    odom_topic:=/hkaspot/odometry \
    visual_odometry:=false \
    frame_id:=hkaspot/body \
    subscribe_depth:=false \
    subscribe_rgbd:=false \
    rgbd_sync:=true \
    approx_sync:=true \
    queue_size:=1000 \
    qos:=1 \
    rviz:=true \
    rtabmap_viz:=true \
    localization:=true
```

This starts RTAB-Map in localization mode. As the navigation of Spot mini takes place via a separate computer in the same network anyway, no visualization needs to be started directly on the Jetson. For this reason, RViz and RTABMap_Viz are both set to FALSE. The configuration of Localization to TRUE is crucial. Only then will RTAB-Map be started in localization mode and no map will be recorded. 


## Subsequent optimization of the map

RTAB-Map already offers another tool, rtabmap-databaseViewer, for subsequent optimisation of the map after recording. 

The application can be started with the following command. The path belonging to the map must be specified.

```bash
rtabmap-databaseViewer /home/robot/spot-perception/map/rtabmap_copy.db
```

The Database Viewer opens, as shown in the following illustration:

![Screenshot from 2024-07-17 13-53-12](https://github.com/user-attachments/assets/b28f845d-8d45-497f-937f-48c06360a59c)

This tool can be used to view and customise all the images contained in the map. Via Edit > Detect more loop closures, the entire data set is looked through again and optically identical points are found that can close the nodes in the map. 

### Optimization of 2D map
Since only the 2D map will be used for navigation with the nav2 Navigation Stack in a later step, this map can also be adjusted and edited here. To do this, the map can be opened in a new window via Edit -> Optimize 2D map, as shown below.

![Screenshot from 2024-07-17 13-54-36](https://github.com/user-attachments/assets/7e6a9537-12e9-4fb0-a327-7fbb5b16a85f)

Here, you can right-click to choose whether to add an obstacle or delete an obstacle. Then, by pressing the left mouse button, you can modify or redraw the map. This step can be particularly helpful when dealing with low-resolution cameras. Often, the 2D map contains individual points that are stored as obstacles, even though there is actually none. These points can then be retouched using this tool, allowing the robot to pass through the corresponding area.

### Save the optimized map
![Screenshot from 2024-07-17 13-55-35](https://github.com/user-attachments/assets/d70423cb-1a0f-4efe-bf38-f58fb4fab3e2)

As shown above, the optimized map should then be exported and saved via File -> Export, so that the changes take effect.


##### Additional settings for local costmaps
The RTAB-Map package also provides functionalities to detect obstacles by segmenting input point clouds to differentiate between the ground and obstacles. These are required to achieve full functionality within local costmaps detailed in the [section]() for Nav2.

The relevant commands to start the nodes can be found in the entrypoint file under docker/rtab-map/entrypoint.sh. The following highlights relevant information about these commands:

The command below converts depth images to point clouds. The topic depth/image is remapped to the appropriate topic for depth images from Spot. Camera intrinsics also need to be provided to convert depth values to point clouds

```bash
$ ros2 run rtabmap_util point_cloud_xyz --ros-args -r depth/image:=/hkaspot/depth_registered/frontleft/image -r depth/camera_info:=/hkaspot/depth_registered/frontleft/camera_info -p voxel_size:=0.05 -p decimation:=4 -p max_depth:=10.0 -p approx_sync:=false 
```

The command below is relevant for obstacle detection and requires two parameters, which are listed in the command below. They can remain unchanged, unless the robot name has been defined differently in the Spot ROS-2 Driver.

```bash
$ ros2 run rtabmap_util obstacles_detection --ros-args -p frame_id:=hkaspot/body -p map_frame_id:=map 
```