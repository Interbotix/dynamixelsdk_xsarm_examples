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
To get all the code setup, refer to Robotis's documentation [here](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/download/#repository) to download the various APIs, and the instructions [here](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/library_setup/c_windows/#c-windows) to install them (click the appropriate library on the left-hand side).
