### ROS-2 Driver Spot
This section details all the necessary configuration steps for setting up the ROS-2 Driver for Spot Mini to work correctly and interface with other ROS packages. 

#### Installing the driver:
Alternatively to running the driver on an edge computer locally on Spot Mini, the driver can be installed on seperate host device and run, as long as the device is physically connected to Spot via the Ethernet port or Spot's direct wifi connection. Details on the exact installation process and installation from source can be found on the GitHub page for the driver.

In a gist, the driver can be installed as follows, assuming ROS-2 is already installed on the machine.

```bash
$ git clone https://github.com/bdaiinstitute/spot_ros2.git

cd spot_ros2
git submodule init
git submodule update
```
To install all dependencies for this driver, the bundled script can be used. Depending on which processor architecture is being used (amd64 or arm64), the appropriate flag needs to be provided as follows:

```bash
cd <path to spot_ros2>
./install_spot_ros2.sh
or
./install_spot_ros2.sh --arm64
```

Next, the ROS-2 workspace needs to be built:
```bash
cd <ros2 ws>
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-ignore proto2ros_tests
source install/local_setup.bash
```

###### Starting the driver after installation:
The general command used to start the driver can be used as follows:
```bash
ros2 launch spot_driver spot_driver.launch.py [config_file:=<path/to/config.yaml>] [spot_name:=<Robot Name>] [publish_point_clouds:=<True|False>] [launch_rviz:=<True|False>] [uncompress_images:=<True|False>] [publish_compressed_images:=<True|False>]
```

Details on configuring the driver can be found in the next [section](#configuring-the-driver)

#### Configuring the driver:
To configure the Spot ROS-2 Driver with the correct parameters a YAML file is utilized. Highlighted below are certain crucial parameters which need to be set to ensure correct functioning of the driver:

```yaml
username: # Required to access Spot-SDK
password: 
hostname: #IP of payload port or spot wifi
```

The username and password are provided with Spot and need to be input into the yaml file

The hostname is necessary to allow communication with Spot`s internal computer and is also the channel through which outputs such as point clouds and images will be streamed.

```yaml
preferred_odom_frame: hkaspot/odom # example. If spot-name provided, it needs to be included like this
```

The preferred odometery frame has to be provided, which will also be used in the mapping and navigation section. For additional information on the choice of odometery available for spot, refer to: https://dev.bostondynamics.com/docs/concepts/geometry_and_frames.html?highlight=frame#frames-in-the-spot-robot-world

Details on additional parameters can be found under the official documentation for the driver under the respective GitHub page.

The driver is started directly within the docker container when it is booted. For more information on the exact start command, refer to the respective entrypoint file under docker/spot-ros2. 


#### Visualizing the ouputs of Spot's depth cameras
It is possible to visualize the outputs of Spot's depth cameras using Rviz. Start Rviz on the Jetson or on the local machine with a ROS-2 Installation connected to the same network and having the same domain id.

```bash
export $ROS_DOMAIN_ID=33
echo $ROS_DOMAIN_ID # Ensure Domain ID is set
rviz2 # Start Rviz
```

Once started, select the config file `spot.rviz`under configs using file>open. This should load all the necessary topics to visualize the outputs from the depth camera.