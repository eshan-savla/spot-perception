# spot-perception

This repository extends the [ROS2 Package for Spot](https://github.com/bdaiinstitute/spot_ros2) by integrating ROS based SLAM and navigation functionalities.

**Important:** As the `humble` branch is outdated and `main` is a rolling release branch, this repository uses a fixed commit to guarantee functionality. This means that newer features of of the Spot ROS2 package might not be available out of the box. However, updating to a newer version of the package is easily possible as long as the config file is updated aptly.

The SLAM package is computationally intensive and requires sufficient processing power to function smoothly. During the develepement of this package, a [NVIDIA Jetson Orin 64gb](https://developer.nvidia.com/embedded/learn/jetson-agx-orin-devkit-user-guide/index.html) was used for edge computing locally on Spot.

### Quick Start:

This assumes you have configured your hardware correctly with Spot. Refer to the [Hardware Section](#hardware-and-interfacing) for more information on configuring and interfacing your hardware directly with spot.

1. Configure the `spot_ros_config.yaml` under configs with your spot username, password and Payload port IP under hostname:

```yaml
username: "user"
password: "password"
hostname: "10.0.0.3"
```

2. Select a name for spot and add it to the entrypoint file under `docker/spot-ros2/entrypoint.sh`. The file looks like this:

```bash
#!/bin/bash
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/robot/spot_ros2_ws/install/setup.bash
export $SPOT_NAME="myspot" # --> HERE
ros2 launch spot_driver spot_driver.launch.py config_file:=./src/spot_ros2/configs/spot_ros_config.yaml spot_name:=$SPOT_NAME publish_point_clouds:=True

```

3. select your preferred odom frame in `spot_ros_config.yaml`under configs using your **SPOT_NAME**:

```yaml
preferred_odom_frame: "myspot/odom" # pass either odom/vision. This frame will become the parent of body in tf2 tree and will be used in odometry topic. https://dev.bostondynamics.com/docs/concepts/geometry_and_frames.html?highlight=frame#frames-in-the-spot-robot-world for more info.
```

4. Pull the latest docker images:
   **Note**: This requires an account at [dockerhub](www.dockerhub.com) and logging on locally on your machine

```bash
docker pull eshansavla0512/ros2-spot-arm64:rework
docker pull eshansavla0512/ros2-rtabmap-arm64:latest
```

5. Alternatively, you can build the containers locally on your amd64 machine or on arm64 machine with at least 16gb of RAM

```bash
bash ./build_docker.sh # for build on native arm64 architecture
bash ./build_docker.sh --cross-compile # for emulating arm64 builds on amd64 architecture
```

**Note**: Depending on your internet connection, this can take between 30-60 mins for a fresh build.

6. Running the containers
   To run the containers, just run:

```bash
bash start_docker.sh
```

This will launch the docker compose command which takes care of starting both containers - the spot ros2 driver and slam + nav2 container with the appropriate parameters.

7. Start Navigating with Nav2 on your local machine:
   **Note:** This requires you to locally install ros2 humble and nav2 on your own machine. If you have them installed in a container, run the same command inside your container.

```bash
source /opt/ros/humble/setup.bash
ros2 launch nav2_bringup rviz.launch.py
```

Your screen should look something like this:

%TODO: Add screenshot of rviz with map

**Hint -** You can blend spots model in rviz by selecting the right topic under RobotModel>%TODO finish rest

8. Using the _Nav2 Goal_ option you can click on your goal point and drag your mouse while holding down the left button to provide your goal orientation for spot.

%TODO: Add screenshot with arrow and planned trajectory

### Hardware and Interfacing

##### Parts list

In order to copy the subsequent description of the hardware setup, additional parts are required. The following list contains all the parts needed, where some of them are models which need to be 3D-printed.

• [NVIDIA Jetson Orin 64gb](https://developer.nvidia.com/embedded/learn/jetson-agx-orin-devkit-user-guide/index.html)  
• [General Expansion Payload (GXP) module](https://support.bostondynamics.com/s/article/Spot-General-Expansion-Payload-GXP)  
• „jetson-platte.prt“ (3D printed) [(click here to download the 3D model)](models/jetson-platte.stl)  
• „jetson-deckel.prt“ (3D printed) [(click here to download the 3D model)](models/jetson-deckel.stl)  
• 4x threaded inserts M6  
• 4x cylinder head screw M6x80  
• 4x cylinder head screw M5x10  
• 4x washer M6  
• 4x washer M5  
• 6x [T-slot nut](https://de.misumi-ec.com/vona2/detail/110302247820/?HissuCode=HNTR5-5)  
• [15-pin D-SUB male connector](https://www.conrad.de/de/p/tru-components-t1904c098-d-sub-stiftleisten-set-polzahl-15-schrauben-1-st-2108938.html?hk=SEM&WT.mc_id=google_pla&gad_source=1&gclid=CjwKCAjwuJ2xBhA3EiwAMVjkVI6hCHULTX5bZtmSpPjy9u6TFpEu-Qj3K-N-bZPuuvF46AN-jl-E2hoCSQwQAvD_BwE&refresh=true)  
• [power cable](https://www.reichelt.de/adapterkabel-2000-mm-dc-stecker-auf-freies-ende-dc-aks-7521-p150125.html?PROVID=2788&gad_source=1&gclid=EAIaIQobChMIvvSshPrIhQMVigcGAB0KjAioEAQYASABEgK4bfD_BwE)  
• Ethernet cable

##### Mount for Jetson:

For operating the SLAM package, the [NVIDIA Jetson Orin 64gb](https://developer.nvidia.com/embedded/learn/jetson-agx-orin-devkit-user-guide/index.html) must first be permanently mounted on the robot. Spot has two mounting rails on its back for this purpose, on which such payloads can be mounted using t-nuts. However, the Jetson computer does not have any mounting holes. For this reason, a bracket is designed that fixes the computer from above and below by clamping and can be produced using a 3D printer (see picture below). Threaded inserts are melted into the plastic in the lower part in order to securely clamp Jetson with four screws in the corners. The recess in the upper part is positioned above the fan location to ensure that it operates seamlessly. The unit is mounted on Spot using a further four screws in the base plate.

<img src="images\Jetson_bracket.png" alt="Example Image" width="80%">

##### Power supply and data communication interface:

Jetson must be supplied with power and connected correctly to the spot core to ensure communication between the systems. To connect such external devices, Boston Dynamics offers the [General Expansion Payload (GXP) module](https://support.bostondynamics.com/s/article/Spot-General-Expansion-Payload-GXP). This is also screwed onto the mounting rails and connected to the front of the two DB25 sockets. A regulated power supply can then be tapped from the GXP module using an [15-pin D-SUB male connector](https://www.conrad.de/de/p/tru-components-t1904c098-d-sub-stiftleisten-set-polzahl-15-schrauben-1-st-2108938.html?hk=SEM&WT.mc_id=google_pla&gad_source=1&gclid=CjwKCAjwuJ2xBhA3EiwAMVjkVI6hCHULTX5bZtmSpPjy9u6TFpEu-Qj3K-N-bZPuuvF46AN-jl-E2hoCSQwQAvD_BwE&refresh=true). The table below shows which pins to use for connection. We chose pins 13 (+24V) and 1 (GND) for a supply voltage of 24 V (max. 150 W).

<img src="images\GXP_module_pins.png" alt="Example Image" width="50%">

The data connection is made via an Ethernet cable, which is simply connected to the GXP bridge. The figure beneath shows the connected and mounted Jetson on Spot.

<img src="images\mounted_payload.png" alt="Example Image" width="50%">

##### Add payload in web interface for collision avoiding:

When adding payloads on top of spot, the internal collision avoiding system does not automatically take this into account while path planning. For example Spot can walk under tables with nothing attached, but when the Jetson is mounted on its back, he has to crouch to not collide. Therefor these additional obstructions have to get added to the kinematic structure to make the algorithm consider them. In order to do this, the following steps are required.

###### Step 1: Connect to Spots WIFI

On Spots belly you can find Spots "Default Network information" (See picture below). Connect your computer to spots WIFI SSID, in our this case it is "spot-BD-31580012". You can find the WIFI password on the sticker as well.

<img src="images\add payload 1.png" alt="Example Image" width="80%">

###### Step 2: login to the spot configuration page

On the sticker from "step 1" you can also find the login credentials for the spot configuration page.

- open your browser and search for "192.168.80.3"
- a page saying "Sign in to TurboDoggy" should open. Sign in witth the "Admin" credentials from the sticker.

<img src="images\add payload 2.png" alt="Example Image" width="80%">

###### Step 3:

<img src="images\add payload 3.png" alt="Example Image" width="80%">

###### Step 4:

<img src="images\add payload 4.png" alt="Example Image" width="80%">

###### Step 5:

<img src="images\add payload 5.png" alt="Example Image" width="80%">

%TODO: Paar Sätze über Verkablung + Konfiguration über Webinterface
