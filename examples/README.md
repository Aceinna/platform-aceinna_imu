# Open Aceinna Navigation Platform

![Aceinna Logo](https://github.com/Aceinna/openimu/blob/master/aceinna.png)

## Description
OpenIMU is a highly-efficient software stack for the development of navigation systems.

  - powerful IDE, platformio vscode
  - effiecient source code control, github
  - affordable debugging tool, st-link

## Table of Contents

  1. [Hardware requirements](#hardwarerequirements)
  2. [Software requirements](#softwarerequirements)
  3. [Installation](#installation)
  4. [Editing](#editing)
  5. [Build](#build)
  6. [Debug](#debug)
  7. [Firmware Upgrade](#firmware-upgrade)
  8. [Data Record](#data-record)
  9. [OpenIMU framework](#openimu-framework)
  10. [OpenIMU data acquisition and processing](#openimu-data-acquisition-and-processing)
  11. [Contributions](#contributions)
  12. [License](#license)
  
### Hardware requirements

 - Windows 10 or 7 PC
 - Ubuntu version 14.0+
 - Mac book
 - Aceinna's Dev Kit
 - ST-Link V2

### Software requirements

 - Windows, Mac or Linux OS
 - VSCode source code editor (open source) 
 - ST-link driver
 - Aceinna navigation studio extension for VSCode
 - Aceinna's utility based upon python

### Installation
 
  - Install VSCode source code editor
  ```sh
       https://code.visualstudio.com
  ```
  - Install Aceinna Navigation Studio extension for VSCode
  ```sh
        Click extension icon at left bar and type aceinna, then click installation button.
  ```
  - Install ST-link V2 driver (Windows only)
  ```sh
        http://www.st.com/en/development-tools/st-link-v2.html
  ```
  - Download zip image of this repository into folder Documents\platformio, unzip it and rename directory "openimu_master" to "openimu"

### Editing

  - In VSCode open source tree clicking on Explorex icon (top left)
  - Choose required file(s) and apply desired changes
  - Save file (CTRL-S of File->Save) 

### Build

 - Click Home icon on the bottom of pio IDE
 - Choose "Open Project"
 - Navigate to the folder \Documents\platformio\openimu\OpenIMU300ZA
 - Click button "Open OpenIMU300ZA"
 - Click Build (checkmark) icon on the bottom of pio IDE

### Debug
 - Create a pio's account
```sh
      pio account register
      pio account login
```
 - Press "F5" or start debug session or  choose "Debug->Start Debugging" in menu
 - Use Debug tool panel on top of the screen Debug menu or keyboard shortcuts 
   (F5, F10, F11) to control debug functions
 - Set up breakpoints anywhere in the code double clicking space left of required line number

### Firmware upgrade

- Click "Upload" button on the bottom of Aceinna Navigation Studio to upload firmware onto unit.

 
### Data Record

 - Install Aceinna's utility 
```sh
   git clone https://github.com/Aceinna/python-imu380
```
 - Install python pip, 
     pyserial  
     tornado  
     azure-storage-blob
```sh
   Windows PC
   https://packaging.python.org/tutorials/installing-packages/
```
```sh
   Ubuntu
   $ sudo apt-get install python-pip python-dev build-essential 
   $ sudo pip install --upgrade pip 
   $ sudo pip install pyserial
```
   - Connect IMU Dev board to PC
   - cd ...python-imu380/
   - run "python demo.py"



### OpenIMU framework

Aceinna OpenIMU platform framework gives users unique opportunity to create their own navigation solutions.
In IMU configuration init provides user with compensated 9 DOF sensors data (Acceleration, Rotation and Magnetic Field).

OpenIMU framework is based on FreeRtos operating system.

Framework contains next main features:

1. Capability of using provided IMU hardware with built-in FW in "out-of-the-box" fashion (unit outputs scaled sensors 
   data periodically with desired rate).
2. Capability of using provided IMU FW examples in "out-of-the-box" fashion (unit outputs scaled sensors 
   data periodically with desired rate).
3. Placeholder for adding custom user filters to adjust unit sensors noise characteristics to cirtain environments.
   - Please refer to the information in files UserFilter.c and UserFilters.h
4. Placeholder for adding user/defined algorithm (EKF or like) to further improve unit performance and/or convert unit into AHRS solution.
   - Please refer to the information in files UserAlgorithm.c and UserAlgorithm.h
5. Placeholder for combining all user defined processing in one place.
   - Please refer to the information in files dataProcessingAndPresentation.c
6. Examples of implementation low pass filters as well as EKF (extended Kalman filter) algorithm.
   - Please refer to the information in files lowpass_filter.c, lowpass_filter.h
7. Examples of implementation low pass filters as well as EKF (extended Kalman filter) algorithm.
   - Please refer to files in folders lib\algorithm and src\utilities
8. User messaging engine - easily extendable framework for adding user-defined input control messages as well output data messages to existing engine
   - Please refer to the information in files UserMessaging.c and UserMessaging.h and files in the folder userProtocol.
9. User configuration engine - easily extendable framework for creating user-defined configuration structure and mechanizm to save/retrieve 
   this structure to/from nonvolatile memory.
      - Please refer to the information in files UserConfiguration.c and UserConfiguration.h
10. Capability of reusing existing sensors data structures as well as introducing custom structures
      - Please refer to the information in files UserData.c and UserData.h
11. Capability of reusing or disabling built-in low-pass filters 
      - Please refer to the information in files UserConfiguration.c and UserConfiguration.h

### OpenIMU data acquisition and processing

In OpenIMU framework data from sensors acquired and processed in cyclical fashion with the rate of 800 Hz.
After applying low-pass filtering (if desired) and calibration sensors data provided to the user for further processing
in cyclical fashion with the rates 200Hz (default) or 100 Hz. Data processing rates provided to the user algorithm as a reference.
After application of user-defined algorithm and/or filters resulting data sent out in a packets with user-defined rate. Framework has 
built-in messages with predefined format and also user can easily create their own messages and use provided transport layer to sent 
them out. Please refer to the file dataProcessingAndPresentation.c for more information



### Contribution

Please refer to each project's style guidelines and guidelines for submitting patches and additions. In general, we follow the "fork-and-pull" Git workflow.

 - Fork the repo on GitHub
 - Clone the project to your own machine
 - Commit changes to your own branch
 - Push your work back up to your fork
 - Submit a Pull request so that we can review your changes

### License

ANS open source projects is licensed under the Apache 2.0 license.

ANS does not require you to assign the copyright of your contributions, you retain the copyright. ANS does require that you make your contributions available under the Apache license in order to be included in the main repo.

If appropriate, include the Apache 2.0 license summary at the top of each file along with the copyright info. If you are adding a new file that you wrote, include your name in the copyright notice in the license summary at the top of the file.

Copyright (C) 2018 Aceinna Navigation System Open Source Project

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.


   