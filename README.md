# DynamixelSDK XSeries Arm Examples

## Overview
Welcome to the *dynamixelsdk_xsarm_examples* repository! This repo contains example code written in both C++ and Python which can be used to control any Interbotix XSeries Arm. To accomplish this, the DynamixelSDK [C++](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/api_reference/cpp/cpp_porthandler/#cpp-porthandler) and [Python](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/api_reference/python/python_porthandler/#python-porthandler) APIs were used. Additionally, the C++ code was loosely based on the [read_write.cpp](https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/master/c%2B%2B/example/protocol2.0/read_write/read_write.cpp) and [sync_read_write.cpp](https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/master/c%2B%2B/example/protocol2.0/sync_read_write/sync_read_write.cpp) examples provided by Robotis in the [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK) repository. Similarly, the Python code was loosely based on the [read_write.py](https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/master/python/tests/protocol2_0/read_write.py) and [sync_read_write.py](https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/master/python/tests/protocol2_0/sync_read_write.py) examples as well.

## Compatible Products

The examples are based on the Protocol 2.0 communication protocol, so any arm made with Protocol 2.0 compatible Dynamixel servos can be used. Note that while the examples are specific to the WidowX 200 arm, it can be easily modified to work with any of the XSeries arms shown below.

- [PincherX 100 Robot Arm](https://www.trossenrobotics.com/pincherx-100-robot-arm.aspx)
- [PincherX 150 Robot Arm](https://www.trossenrobotics.com/pincherx-150-robot-arm.aspx)
- [ReactorX 150 Robot Arm](https://www.trossenrobotics.com/reactorx-150-robot-arm.aspx)
- [ReactorX 200 Robot Arm](https://www.trossenrobotics.com/reactorx-200-robot-arm.aspx)
- [WidowX 200 Robot Arm](https://www.trossenrobotics.com/widowx-200-robot-arm.aspx)
- [WidowX 250 Robot Arm](https://www.trossenrobotics.com/widowx-250-robot-arm.aspx)
- [WidowX 250 Robot Arm 6DOF](https://www.trossenrobotics.com/widowx-250-robot-arm-6dof.aspx)
- [ViperX 250 Robot Arm](https://www.trossenrobotics.com/viperx-250-robot-arm.aspx)
- [ViperX 300 Robot Arm](https://www.trossenrobotics.com/viperx-300-robot-arm.aspx)
- [ViperX 300 Robot Arm 6DOF](https://www.trossenrobotics.com/viperx-300-robot-arm-6dof.aspx)
- [PincherX 100 Mobile Robot Arm](https://www.trossenrobotics.com/pincherx-100-mobile-robot-arm.aspx)
- [WidowX 200 Mobile Robot Arm](https://www.trossenrobotics.com/widowx-200-robot-arm-mobile-base.aspx)
- [WidowX 250 Mobile Robot Arm 6DOF](https://www.trossenrobotics.com/widowx-250-mobile-robot-arm-6dof.aspx)

## Requirements
Below is a list of the hardware you will need to get started.
- Computer (any operating system compatible with the DynamixelSDK)
- One of the X-Series Robot Arm Kits mentioned above

## Hardware Setup
There is not much required to get the robot ready to work as most of the setup is done for you. Just make sure to do the following steps:
1. Remove the robot from its packaging and place on a sturdy tabletop surface near an electrical outlet. To prevent the robot from potentially toppling during operation, secure it to a flat surface (via clamping or using the holes on the base's perimeter). At your own risk, you could instead place a small heavy bean-bag on top of the acrylic plate by the base of the robot. Finally, make sure that there are no obstacles within the workspace of the arm.
2. Plug the 12V power cable into an outlet and insert the barrel plug into the barrel jack on the X-series power hub (located under the see-through acrylic on the base of the robot). You should briefly see the LEDs on the Dynamixel motors flash red.
3. Plug in the micro-usb cable into the U2D2 (located under the see-through acrylic on the robot's base) and your computer.

## Software Setup
To get all the code setup, refer to Robotis's documentation [here](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/download/#repository) to download the various APIs, and the instructions [here](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/library_setup/c_windows/#c-windows) to install them (click the appropriate library on the left-hand side). Then download a zip file with the code or clone this repository wherever you'd like on your computer by typing...
```
$ git clone https://github.com/Interbotix/dynamixelsdk_xsarm_examples.git
```
If you're going the 'cpp' route (at least in Linux), navigate to the directory that contains the Makefile and type `make`. An executable file should pop up called 'interbotix_arm'. To run it, just type `./interbotix_arm`.

## Structure
Both the 'Cpp' and 'Python' examples in this repo do the exact same thing so that you can get a feel for which language you prefer to use. Also, the code in this repo is pretty 'lightweight' in that all code is kept to just a single file. This is to make it easier for you to see how to *implement* the DynamixelSDK without jumping all over the place to view Python modules or Cpp header files. In that sense, the code shown here is meant to act as a starting point for your own development.

The structure for both files is pretty straightforward and contains two parts. The first part is the largest and just contains many helper functions that can be used to work with the motors. This includes functions that allow you to read/write from/to a single motor or many motors sequentially. It also includes functions to allow you to read/write to multiple motors simultaneously (using the 'sync read/write' feature of the Dynamixel SDK). Note that the main benefit of communicating simultaneously with the motors is that it takes much less time than communicating sequentially, and all data has the same 'time stamp'. For example, if you command the arm motors to move to your desired goal positions simultaneously (and specify each one to take 1.5 seconds), all motors will start and stop at the same time. However, if you were to command the arm motors to move to your desired goal positions sequentially in the same manner, the motors will each start at different times and finish their motions at different times.

Another helper function defined in the first part of both files is called 'CalibrateDualJoint'. This function can be used to calibrate a joint that is composed from two motors (like the shoulder joint in our WidowX 200 arm, or the elbow joint in our WidowX 250 arm). Essentially, the issue that can arise is that the encoder on each motor can show slightly different positions despite having their horns physically aligned during robot assembly. As a result, if a goal position is commanded to that joint, one motor might reach the goal position and stop moving. However, the other motor's encoder might show that the goal position hasn't been reached yet and exert more effort to reach it. As a result, the first motor will exert more effort to keep holding its position. This leads to a vicious cycle that can ultimately lead to one or both motors overloading and torquing off. To prevent that from happening, the 'calibrateDualJoint' function sets the Homing Offset register of the second motor in a dual joint setup so that the second motor's encoder shows the same value as the first motor's encoder.

The second part of both files is the 'main' function where you write code to interact with the robot. It consists of defining vectors/lists of the IDs that make up the arm as well as initial values that should be sent to those motors' registers. It also entails the *writing* of those initial values to the motors. This portion of the code should be modified depending on which robot arm is being used. Following that 'initial setup part', the code shows how to move the arm, open/close the gripper, and get data like current temperatures and PWMs or currents.

As an FYI, the code does not contain any IK Solvers or higher level APIs. It is only meant to showcase how to implement the DynamixelSDK API so that you don't have to try figuring it out on your own.

## Troubleshooting
For help using the examples, feel free to submit an Issue. Just please include your operating system, any applicable error messages, and the commands you ran as well. You can also contact us directly at trsupport@trossenrobotics.com, but we recommend submitting an Issue so that other people who may be facing the same difficulty can benefit. Note that this repository is actively maintained and any open Issues will be addressed as soon as possible.

## Contributing
Feel free to send PRs to add features (like demos in other languages) or bug fixes. All PRs should follow the same type of organizational structure and documentation as shown in this repo.

## Contributors
- [Solomon Wiznitzer](https://github.com/swiz23) - **Software Engineer**
