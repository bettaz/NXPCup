
# NXP Cup 

**Brief summarized development log(diary):**  [Markdown syntax](https://www.markdownguide.org/basic-syntax/#code)
## Semester 2
### Feberary 7th:
We are currently working on three branches: a)Main algorithm development b)RaspberryPi remote control implementation, which means we might be able to send commands via WIFI to the console running on the raspberry pi and then indirectly control the flight board c)Gazebo simulation, to move the code from the ROS simulation to the firmwave.
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


