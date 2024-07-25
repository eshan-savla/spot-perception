### Software-Architecture 
This section details the software architecture for the spot-navigation package. The following image depicts the overarching interfacing between the individual software components: 
To add an image in a markdown file, you can use the following syntax:

![Software Architecture](images/spot_architektur.drawio.svg)

#### Spot ROS-2
To achieve motion control of Spot mini via ROS the [ROS-2 driver](https://github.com/bdaiinstitute/spot_ros2) can be utilized. Within this package, all the necessary services, topics and actions to teleoperate or navigate with Spot Mini can be found. It also offers functionalities to interface with the different payloads offered for Spot Mini. It is important to note however, that SpotCAM has certain known issues which makes using it within the Spot ROS-2 Driver difficult. More information on how to set up this package can be found under the dedicated [Spot ROS-2 Section](#ros-2-driver-spot)

Despite being still under development, this package also offers access to Spot's internal mapping and navigation tool GraphNav. This allows for ROS independent mapping and multi-storey navigation. To incorporate this functionality into ROS, a [fork](https://github.com/Banane01/spot_ros2) of the original Spot ROS-2 driver was created


#### RTAB-Map
The [RTAB-Map ROS-2 Package](https://github.com/introlab/rtabmap_ros/tree/ros2) provides SLAM functionalities in 3D to be used in cohort with a robot navigation stack. In addition to creating a 2D planar costmap of a location, it registers its surroundings in 3D, creating a point cloud of the environment and adjusting it along with the global costmap. This package is utilized to provide fully ROS native SLAM functionality.

In addition to creating the global costmap, the RTAB-Map package also offers the functionality to visually detect obstacles in the form of point clouds, which can be utilized to create a local costmap around Spot. 


#### Nav-2 Navigation Stack
The [Nav-2](https://docs.nav2.org/) provides ROS native planar navigation and path planning functionalities for mobile robots. This package can be used in tandem with Rviz to provide an easy and intuitive way to navigate Spot Mini within a mapped environment. 



### Docker
Each of the packages listed above have been packaged in docker containers to manage dependencies and provide ease of use, ensuring system specific setups and configurations do not interfere with the functions of the packages. For now, the docker containers are only provided for the arm64-v8 processor architecture, as an NVIDIA Jetson Orin Developer Kit was utilized for local edge computing. By connecting user devices to the same network as the Jetson and using the same ROS-Domain ID, it is possible to interface with these packages and access their functionalities remotely. 

#### Spot Driver Docker
The docker image for the spot driver is based off a ROS image for arm64-v8 and installs all the necessary ROS dependencies before moving on to clone and install the forked version of the spot ROS-2 Driver as mentioned in this [section](#spot-ros-2). 

#### RTAB-Map + Nav2 Docker
This docker image packages both the ROS wrapper for RTAB-Map and the Nav2 package in one. As this is based off of the official docker image for RTAB-Map, it can be installed for an amd64 or arm64 architecture by using cross compilation provided within docker's build functionality. 

#### Building and starting the containers
For ease of use, both images have been packaged within a docker-compose file, ensuring they can be built with the correct prerequisites and also started with the right flags to ensure full functionality. 

Building:
Execute the following command from within the root of the repository.
```bash
bash build_docker.sh # If building natively on arm64
```

```bash
bash build_docker.sh --cross-compile # Cross compiling for arm64 on other host architectures
```
For details on the structure, refer to the Dockerfiles.
