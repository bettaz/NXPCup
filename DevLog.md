
# NXP Cup 

**Brief summarized development log(diary):**  [Markdown syntax](https://www.markdownguide.org/basic-syntax/#code)
## Semester 2
### March 11st:
1. We are trying to apply homography to the detected vector, so that the inclination of the vector will be calibrated
1. We copy everything to test the second robot car
2. There are some ideas for improving the performance of line vector detection:
> 1. modify and flash a customized version of firmware for Pixy2 camera
> 2. wait for the white background and find the best configuration in the camera software
> 3. communicate via RhaspberryPi the raw images to train a new robust model only used for line detection

### March 4th:
We successfully implemented the first edition of our control algorithm by translating the code from Gazebo simulation to the firmware.
> Now the car is running on the track, but sometimes it is not stable
> 
Now the problem is to deal with the line detection. We ask to buy a white plastic so that we can place it on the ground to increase the contrast for the pixy camera to better detect the feature.

We would like to use a CNN model to detect the line from the raw image but pixy camera only allows video transmitting via usb connection(So far there is no way to send the camera image via I2C connection). We are suggested to use Rhaspberry Pi to check the camera data.

### February 25th:
1. Fixed the car assembly problems and the wiring.(configure the car correctly) Now it is easier to set up a new vehicle. Just install everything, download our firmware, make and upload it to the board, configure the right parameters in QGroundControl. The race program will start automatically and the car will be armed by pressing and holding the safety switch button on the GPS, after which the control algorithm will be executed and the car will run.
2. Checked the code from previous year students

> The next step is to configure the pixy camera and get the correct features in real time. Then we can start working on the control algorithms.
> Note: There are some problems due to the version of PX4 firmware. The firmware version we use is 1.11.3 and some changes should be properly done.
> We are working to fix the problem of git clone:[]
### February 22nd:
The latest firmware is updated and modified, we expect it to work but there are still some issues. 

### February 21st:
Now the Gazebo simulation is working, and we can change the settings of parameters in the code to see the performance
> The next step is to move the code from simulation to the flight control board
> 
Successfully testing the NXPCup race module(included in the PX4 firmware), now the car is running and we have to develop our own control algorithm
> The next step is to access the Pixy camera API and find the solution to extarct the proper line detection results and regulate the car's movement

### February 11st:
Continue working for the three branches

### Feberary 7th:
We are currently working on three branches: 

1. Main algorithm development

2. RaspberryPi remote control implementation, which means we might be able to send commands via WIFI to the console running on the raspberry pi and then indirectly control the flight board. 
> This is achieved by Alessio, with installing the Manjero OS and the QGroundControl on it. Now the raspberryPi can function as a onboard computer which: 
> 1. could be controlled remotely to send instructions(commands) to the FMU, 
> 2. execute computationally expensive tasks offline,
> 3. could be used in a low input current(1 Ample), which can be powered up by a portable power bank with light weight.  

3. Gazebo simulation, to move the code from the ROS simulation to the firmwave.
> This is somehow done with the help of our tutor Mr.Rodriguez. Now the Gazebo simulation is working.

### Feberary 4th:
Due to the fact that we missed three weeks of the project, out tutor added the plan back for us. We are having a more intensive schedule now. The new rules and schdule is now published officially. We have one month to develop our code and the qualification will be held in March.
(The NXP CUP gitbook has a lot new useful updates, and we are benefiting a lot from it.) Now we understand the all framework much better.
1. PWM motor driving test completed
2. Assembly of the kit double checked
3. Rhaspberry setup - able to connect via WIFI


## Semester 1
### October 8th:
The very first meeting of the industrial project group. We get introduced with the [NXP project](https://nxp.gitbook.io/nxp-cup/), make acquaintance with out tutor and know the colleages from the same lab room.
We also receive some materials like codes and reports from the last year group. With that we are able to get some useful infomation and make a general plan for this project.

### October 15th:
1. Work on different compiling environment(Download softwares...) 
2. We divide the group and try to understand the framework of the project.  

### October 22nd:
1. Complete the team registration. 
2. Check the latest rules of the NXP racing competition. 
3. Still work on the [build environment](https://nxp.gitbook.io/hovergames/developerguide/tools/mcuxpresso). 
4. Accomplish installing and running the [Gazebo](http://gazebosim.org/tutorials?tut=quick_start) simulation and QGroundControl. 

> ### November 16th:
> **Idea of PID control method:** 
> The car is constantly running forward with a set velocity, then controls the steering in order to keep the car in track. 
> Once the car is running successfully, then 1)increase the velocity 2)minimize the overshoot and maximize the respond speed
> 1. extract a line from the camera image
> 2. calculate the two distances between the car and both sides
> 2. compare the d, use the error as the input for controller
> 2. adjust the PID parameters 

### November 19th:
1. Got access to [NXP Course](https://courses.iealearning.org/courses/course-v1:IEA+NXP+2021/course/). 
2. Worked on settings up [NXP Gazebo](https://nxp.gitbook.io/nxp-cup/gazebo/milestone-1-intro-to-nxp-gazebo-and-ros/installation-of-nxp-gazebo-1) environment

We had an online meeting with the other teams instructors and got access to the online course. This helped us build up the knowledge framework and provided indexes of many materials.

### Novenmber 26th:
We successfully set uo the Gazebo simulator. But the repository from NXP CUP summer camp didn't work as our expectation

### December 3rd:
Accessed the Pixy2 camera from the computer, we could see what the camera would see.
Managed to put a hello world example project on the car, and run it from the computer.

### December 10th:
We managed to run NXP Gazebo Summer Camp simulation. (apriltags repos are still wrong, they should be removed from `gen_params_milestone4.json`, and aimline follow should be changed to python, as explained [here](https://nxp.gitbook.io/nxp-cup/gazebo/milestone-1-intro-to-nxp-gazebo-and-ros/change-between-c++-and-python))

## Semester 2
### February 21th:
Jamie's simulation now works because he changed gen_params.json `ros2_node_2`, specifically `linear_velocity`.<br>
David's simulation now works on the virtual machine too, after adding `import cv2` to the `ros2ws/src/nxp_cup_vision/nxp_cup_vision/nxp_track_vision.py`<br>
Alessio fixed the connections of the motor's and the steering, now  we are able to turn the motors and control the Pixy using our code.<br>
Yuyang updated the local PX4 repository, we're debugging why the simulation is not working for him.<br>
