# Open Aceinna Navigation Platform

![Aceinna Logo](https://raw.githubusercontent.com/Aceinna/platform-aceinna_imu/develop/misc/aceinna_logo.png)

## Description
Open-ANS is a highly-efficient software stack for the development of navigation systems:

- Powerful cross-platform and open-source IDE
- Effiecient source code control
- Affordable debugging tool (ST-Link)

## Table of Contents

- [Hardware requirements](#hardware-requirements)
- [Software requirements](#software-requirements)
- [Installation](#installation)
- [Build firmware](#build-firmware)
- [Firmware upgrade](#firmware-upgrade)
- [Debug](#debug)
- [Data Record](#data-record)
- [Contributions](#contributions)
- [License](#license)

### Hardware requirements
- Aceinna's Dev Kit
- ST-Link V2

### Software requirements
- Windows, Mac or Linux OS
- VSCode source code editor (open source)
- Aceinna Navigation Studio (extension for VSCode)
- ST-Link driver
- Aceinna's utility based upon Python

### Installation
- Install [VSCode](https://code.visualstudio.com/) source code editor
- Install [Aceinna Navigation Studio](https://marketplace.visualstudio.com/items?itemName=platformio.aceinna-ide) extension for VSCode
- Install [ST-link V2 driver](http://www.st.com/en/development-tools/st-link-v2.html) (Windows only)

### Build firmware
- Open VSCode source code editor
- Click ["Home"](http://docs.platformio.org/en/latest/ide/vscode.html?utm_source=github&utm_medium=aceinna#platformio-toolbar) button on the bottom of Aceinna Navigation Studio
- Choose "Open Project" (if you have already Aceinna IMU project) or import pre-configured project from  "Custom IMU examples"
- Click ["Build"](http://docs.platformio.org/en/latest/ide/vscode.html?utm_source=github&utm_medium=aceinna#platformio-toolbar) button on the bottom of Aceinna Navigation Studio

### Firmware upgrade
- Click ["Upload"](http://docs.platformio.org/en/latest/ide/vscode.html?utm_source=github&utm_medium=aceinna#platformio-toolbar) button on the bottom of Aceinna Navigation Studio.

### Debug
- Create a PlatformIO Account using ["Home"](http://docs.platformio.org/en/latest/ide/vscode.html?utm_source=github&utm_medium=aceinna#platformio-toolbar) button and "PIO Home > Account" menu
- Press "F5" or choose "Debug" in VSCode menu
- Open "Debug" view at left bar.

### Data Record

- Install [Git](https://www.git-scm.com)
- Install [Python 2.7 Interpreter](http://docs.platformio.org/en/latest/faq.html?utm_source=github&utm_medium=aceinna#install-python-interpreter)
- Open system terminal and install Aceinna's utilities
  ```sh
  git clone https://github.com/Aceinna/python-imu380
  ```
- Install Python dependencies
  ```sh
  pip install pyserial tornado azure-storage-blob
  ```
- Connect IMU Dev board to PC
- Change current directory to `cd python-imu380`
- Run `python demo.py`

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


