<div id="top"></div>

<h1 align="center">PathFinder</h1>
<h4 align="center">Follows and Adapts</h4>
<!-- PROJECT LOGO -->:
<br />
<div align="center">
  <a href="https://jacobsschool.ucsd.edu/">
    <img src="/media/UCSDLogo_JSOE_BlueGold.png" alt="Logo" width="400" height="100">
  </a>
<h3>ECE/MAE148 Final Project</h3>
<p>
Team 5 Fall 2024
</p>

![image](https://github.com/JL2200/mae148_group8/blob/main/media/IMG_4898.JPG)
</div>


<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#team-members">Team Members</a></li>
    <li><a href="#final-project">Final Project</a></li>
      <ul>
        <li><a href="#original-goals">Original Goals</a></li>
          <ul>
            <li><a href="#goals-we-met">Goals We Met</a></li>
            <li><a href="#our-hopes-and-dreams">Our Hopes and Dreams</a></li>
              <ul>
                <li><a href="#stretch-goal-1">Stretch Goal 1</a></li>
                <li><a href="#stretch-goal-2">Stretch Goal 2</a></li>
              </ul>
          </ul>
        <li><a href="#final-project-documentation">Final Project Documentation</a></li>
      </ul>
    <li><a href="#robot-design">Robot Design </a></li>
      <ul>
        <li><a href="#cad-parts">CAD Parts</a></li>
          <ul>
            <li><a href="#final-assembly">Final Assembly</a></li>
            <li><a href="#custom-designed-parts">Custom Designed Parts</a></li>
            <li><a href="#open-source-parts">Open Source Parts</a></li>
          </ul>
        <li><a href="#electronic-hardware">Electronic Hardware</a></li>
        <li><a href="#software">Software</a></li>
          <ul>
            <li><a href="#embedded-systems">Embedded Systems</a></li>
            <li><a href="#ros2">ROS2</a></li>
            <li><a href="#donkeycar-ai">DonkeyCar AI</a></li>
          </ul>
      </ul>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
    <li><a href="#authors">Authors</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>



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
We want to have chatgpt's path following trigger the manage.py drive command automatically so that chatgpt can navigate fully autonomous. We also want to have chatgpt only use one model instead of two seperated models. Finally, we want to turn the lidar data into a SLAM map and feed chatgpt an image map of its surroundings to generate better maps.

#### Future Goal 2
We want automatic lidar stopping to be implemented for safety. Since chatgpt does not control the robot in real time, we need a way for the robot to stop if it is about to hit an object or person.

## Final Project Documentation

<!-- Early Quarter -->
### Robot Design CAD
<img src="/media/full%20car%20cad.png" width="400" height="300" />

#### Open Source Parts
| Part | CAD Model | Source |
|------|--------|-----------|
| Jetson Nano Case | <img src="/media/jetson%20nano%20case.png" /> | [Thingiverse](https://www.thingiverse.com/thing:3518410) |

### Software
#### Chatgpt
We connected Chatgpt to the robocar by using the Openai API, utilizing two seperate models. The first model was GPT4 with Vision, which processes commands from the user and images. Then this model creates an action plan for what the car can do. Since the image based models don't have function calling to trigger the drive commands, we used a second GPT4-Turbo model to read the vision models plan and turn those into functions. Chatgpt had acces to two functions, a drive command to control steering, speed, and motion timeout. Then it had control over a path function, which let chatgpt generate a csv path of x and y coordinates + a throttle for the car to follow. Chatgpt had access to the cameras, lidar data, gps data, and user prompts. We picked and chose which data to give Chatgpt based on the use case.

#### Embedded Systems
To run the system, we used a Jetson Nano with an Oakd depth camera, an ld06 lidar sensor, and a point one Fusion Engine gps. For motion we used a VESC Driver within the Donkey Car framework. https://www.donkeycar.com/

#### ROS2
For commands, we made a ROS2 Node called ChatgptDriveSubpub that works with the UCSD Robocar framework. Most of the files we created are in the basics2 package of the ros2_ws (ros2 workspace). We altered the nav2 config files to add the chatgpt node to start up automatically, but never finished this. So, if you follow the steps to get Ros2 running from the UCSD robocar framework, you are mostly complete.

In our project files, we had to add the fusion engine driver for gps manually, so the nodes for fusion gps are prone to error. One will need 4 to 5 terminals to run this system. One for starting up gps, one for launching all_nodes, one for launching the chatgpt node, one for sending chatgpt messeges over a chat topic, and finally one to use donkeycar's manage.py drive command to drive the car in a desired path. 

#### DonkeyCar AI
For path following, we used the DonkeyCar AI framework and tuned our own PID values. With the donkey car framework, we connected through gps and used PID following of chatgpt generated waypoints for the car. Some example paths are in the donkey_paths folder. Often, chatgpt's paths were innacurate or straight lines, so you may have to be descriptive in your prompts to chatgpt. We found that the system worked best when chatgpt had a good reference understanding of its area size, how many path points to use, and that you want it to use funciton calling.

### How to Run
Use the UCSD Robocar Docker images and add the projects folder yourself. Python3 is required, install any reposotries not there.

Step 1: In the first terminal, start the fusion client. First source ros2. Note, in the build_ros2 file, we exluded the build for the fusion client as each build takes an extra 50 seconds with it.

```source_ros2```

```build_ros2```

```ros2 run fusion-engine-driver fusion_engine_ros_driver --ros-args -p connection_type:=tty -p tty_port:=/dev/ttyUSB1```


Step 2: In a second terminal, build and source ros2 as usual from the ucsd robocar process. Then launch all nodes.

```source_ros2```

```ros2 launch ucsd_robocar_nav2_pkg all_nodes.launch.py```


Step 3: Adjust the chatgpt_drive_node.py in the basics package with your OPENAI API KEY. Launch the chatgpt node. Source ros2 in a third terminal and launch this command.

```source_ros2```

```ros2 launch ucsd_robocar_basics2_pkg chatgpt_drive_launch.launch.py```


Step 4: You are ready to talk to chatgpt. Make sure the terminals output no errors and chatgpt has said "finished init". Then, you publish to a /chat_input topic to communicate with the chatgpt node.

```source_ros2```

```ros2 topic pub -1 /chat_input std_msgs/msg/String "data: 'YOUR MESSEGE HERE'"```


The messege should show up in the chatgpt terminal and if it does not, it did not work. First the vision model will respond with an action plan. Responses can take up to 20 seconds. Then a Function tool caller will work. Sometimes the tool caller gets stuck and you must restart the chatgpt node. Its recommended to restart all nodes, as another error can be no image input being recieved from the all_nodes.

Step 5: If you desire a path to be made, then chatgpt will have outputed the path to the donkey_paths folder in the basics2 package. You can take this csv, and use its path in the donkey car manage.py config files for donkey car to drive on this desired path. Make sure your path starts at 0,0 or to zero the car and drive to the path start. The donkey command to run in a 5th terminal is

```manage.py drive ```

For more information, see the donkey car framework on running with your own car. When manage.py runs, use x to zero the car, b to load a path, and double tap start for the car to follow the path.

Here is a link which contains many photos and videos of our robot doing different chatgpt powered actions:
https://photos.google.com/u/0/share/AF1QipNbnRdItg1uGULNdo4saggAoKI3nbOa-YogXLWNfc9HYORkTFDVzfL07XVTVJIqRg?key=WGtpSk1jdEJXZjJ6cHpObENFQU9MSzItVG1jOXN3 

<!-- Authors -->
## Authors
Jason, Jesse, Maahir, Alexander

![image](https://github.com/UCSD-ECEMAE-148/winter-2024-team-2aka8/blob/main/media/148groupphoto.jpg)

<!-- Badges -->
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
@@ -176,12 +176,10 @@
## Acknowledgments
*Thank you to my teammates, Professor Jack Silberman, and our incredible TA Arjun Naageshwaran for an amazing Winter 2024 class! Thank you Kiersten for the amazing readme template.*

![image](https://github.com/UCSD-ECEMAE-148/winter-2024-team-2aka8/blob/main/media/148groupphoto.jpg)
<!-- CONTACT -->
## Contact

* Jason | yul202@ucsd.edu
* Jesse | jerupe@ucsd.edu 
* Maahir | masgharali@ucsd.edu
* Alexander | ahaken@ucsd.edu
