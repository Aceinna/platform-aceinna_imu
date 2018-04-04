# Open Aceinna Navigation Platform

![Aceinna Logo](https://github.com/Aceinna/platform-aceinna_imu/blob/develop/aceinna.png)

## Description
Open-ANS is a highly-efficient software stack for the development of navigation systems.

  - powerful IDE, platformio vscode
  - effiecient source code control, github
  - affordable debugging tool, st-link

## Table of Contents

  1. [Hardware requirements](#hardwarerequirements)
  2. [Software requirements](#softwarerequirements)
  3. [Installation](#installation)
  4. [Build](#build)
  5. [Debug](#debug)
  6. [Firmware upgrade](#firmwareupgrade)
  7. [Contributions](#contributions)
  8. [License](#license)
  
### Hardware requirements

 - Windows 10 or 7 PC
 - Ubuntu version 14.0+
 - Mac book
 - Aceinna's Dev Kit
 - ST-Link V2

### Software requirements

 - ST-link Windows driver
 - Platformio IDE for vsCode
 - Aceinna's utility based upon python

### Installation
 
  - Download ST-link V2 windows driver 
  ```sh
        http://www.st.com/en/development-tools/st-link-v2.html
  ```
  - Install platformio IDE
  ```sh
        http://platformio.org/platformio-ide
  ```
   - Install Git on Windows PC
  ```sh
        https://www.git-scm.com
  ```
   - Install Git on Ubuntu
  ```sh
        sudo apt-get install git
  ```
   - Install Aceinna extensions
  ```sh
        Click extension icon at left bar and type aceinna, then click installation button.
  ```
   - Clone workspace from GitHub
```sh
        git clone https://github.com/Aceinna/platform-aceinna_imu
  ```
   
### Build

 - Click Home icon on the bottom of pio IDE
 - Choose "Open Project"
 - Or import project by choosing "Project Examples"
 - Choose working space platform-aceinna_imu/examples/base
 - Click Open button
 - Click Build icon on the bottom of pio IDE

### Debug
 - Create a pio's account
```sh
      pio account register
      pio account login
```
 - Press "F5" or choose "debug" in menu
 - Click Source manager at left bar
 - Choose any of the files you would modify
 - Click build icon at the botton line
 
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


   