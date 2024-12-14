<div id="top"></div>

# Cruise Control

<h3>ECE/MAE148 Final Project</h3>
<p>
Team 5 Fall 2024
</p>

![image](https://github.com/UCSD-ECEMAE-148/fall-2024-final-project-team-5/blob/main/Media/robocar1.jpg)
</div>

<!-- TEAM MEMBERS -->
## Team Members

<div align="center">
    <p align = "center">Darren, Colby, and Guy</p>
</div>

<h4>Team Member Major and Class </h4>
<ul>
  <li>Darren - Computer Engineering - Class of 2024</li>
  <li>Colby - Mechanical Engineering, Controls and Robotics - Class of 2025</li>
  <li>Guy - Mechanical Engineering, Controls and Robotics - Class of 2025</li>
</ul>

<!-- Final Project -->
## Final Project
Our project aimed to develop an autonomous RC car capable of following a lead car. The system adjusts speed dynamically, slowing down as the lead car approaches, accelerating as it moves further away, and mirroring its path to maintain seamless alignment.

<!-- Original Goals -->
### Original Goals
Originally, we envisioned an autonomous RC car that could follow a lead car seamlessly, mimicking real-world vehicle behaviors. Our goalpost task was “follow the lead car while adjusting speed and direction dynamically.” This required programming the RC car to detect the lead car, adjust its speed to slow down when the lead car was closer, accelerate when it moved further away, and navigate turns by following the same path. Our ultimate aim was to demonstrate how autonomous systems can emulate safe and efficient driving behaviors.

<!-- End Results -->
### Goals We Met
We successfully developed an autonomous system capable of identifying and following a lead car. Using the OAK-D camera, we were able to detect the lead car and calculate its angle (left or right) relative to our autonomous car. This information was then fed into a LIDAR system to determine the precise distance between the two vehicles. By combining these inputs, our car adjusted its speed dynamically—slowing down as the lead car approached, speeding up as it moved further away, and turning to match the lead car's path. One test we conducted involved positioning the lead car at varying angles and distances. Our car consistently calculated the correct angle and distance, allowing it to smoothly adjust its trajectory and speed. This demonstrated a reliable ability to follow the lead car even during sharp turns or abrupt changes in speed. We feel this integration of camera-based object detection and LIDAR distance measurement was a significant achievement. The collaboration between these systems enabled our car to replicate human-like driving behaviors in a controlled environment. Overall, our project successfully met its goal of creating a car that could autonomously follow a lead vehicle with precision and adaptability.

### Future Goals
#### Future Goal 1
We aim to achieve a system where, as the speeds of both the lead car and the autonomous following car increase, the distance between them automatically adjusts to ensure a safe gap that provides sufficient space for braking. This behavior will enhance the safety and responsiveness of the autonomous following system, allowing it to adapt dynamically to varying driving conditions and ensure collision avoidance even at higher speeds.

#### Future Goal 2
We aim to integrate an emergency braking system utilizing our LiDAR. This system will automatically bring the car to a stop if an object comes too close, prioritizing safety for the vehicle and its surroundings.

## Final Project Documentation

### Software
#### Embedded Systems
To run the system, we used a Jetson Nano with an Oakd depth camera, an ld06 lidar sensor.

#### ROS2
For commands, we made a 2 ROS2 Packages called camera_pkg and cruise_control that work with the UCSD Robocar framework. 

The camera_pkg runs the roboflow model and uses the x-pixel offset to find the angle to the target. This is published to the /camera_data topic

The cruise_control package sends Twist commands to the /cmd_vel topic to control the robot. It is subscribed to the /camera_data and /scan topics, and uses a PD controller to publish linear and angular velocities to the vesc topic. 


### How to Run
Use the UCSD Robocar Docker image (devel version) and add the camera_pkg and cruise_control folders yourself into the /home/projects/ros2_ws/src directory. The roboflow folder can be used to test the roboflow model on its own. Python3 is required.

Step 1: Clone depthai repositories for use of OAK-D camera and roboflow in the docker /home/projects directory.

```
    git clone https://github.com/luxonis/depthai.git && \
    git clone https://github.com/luxonis/depthai-python.git && \
    cd depthai && \
    curl -fL https://docs.luxonis.com/install_dependencies.sh | bash && \
    python3 install_requirements.py && \
    cd ../depthai-python/examples && \
    python3 install_requirements.py
    echo "export OPENBLAS_CORETYPE=ARMV8" >> ~/.bashrc
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | tee /etc/udev/rules.d/80-movidius.rules
```
Step 2: Implement the new packages for use with all_nodes.launch.py

In ```gedit src/ucsd_robocar_hub2/ucsd_robocar_nav2_pkg/config/node_pkg_locations_ucsd.yaml``` add 

``` 
camera_pkg: ["camera_pkg", "camera.launch.py"]
cruise_control: ["cruise_control", "cruise_control.launch.py"]
```
In ```gedit src/ucsd_robocar_hub2/ucsd_robocar_nav2_pkg/config/node_config.yaml``` add 

```
camera_pkg: 1
cruise_control: 1
```
In ```gedit src/ucsd_robocar_hub2/ucsd_robocar_nav2_pkg/config/car_config.yaml```
Set vesc_without_odom and the lidar that you are using to 1.

Step 3: In the docker, first source ros2. 

```source_ros2```

```build_ros2```

```ros2 launch ucsd_robocar_nav2_pkg all_nodes.launch.py```


<!-- Authors -->
## Authors
Darren, Colby, and Guy

![image](https://github.com/UCSD-ECEMAE-148/fall-2024-final-project-team-5/blob/main/Media/groupphoto148.jpg)

<!-- Badges -->
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
@@ -176,12 +176,10 @@
## Acknowledgments
*Thank you to my teammates, Professor Jack Silberman, and our incredible TAs Alexander and Winston for an amazing Fall 2024 class! Thank you Alexander for the amazing readme template.*

<!-- CONTACT -->
## Contact

* Darren | dwng@ucsd.edu
* Colby | chettinger@ucsd.edu 
* Guy | gvwalter@ucsd.edu
