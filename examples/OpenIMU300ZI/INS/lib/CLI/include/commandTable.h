/** ***************************************************************************
 * @file   commandTable.h table of commands, calbacks, and help strings
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *  Table of commands available to be sent from the commandLine.c shell
 ******************************************************************************/
/*******************************************************************************
Copyright 2018 ACEINNA, INC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*******************************************************************************/


//#include "commandLine.h"
#include "commands.h"

#define COMMAND_TABLE_END {"",0,0,""}

//Command table
// {char *name, tShellCallback callback, uint32_t callbackData, const *help }
const tCommand gCommands[] =
{
  {"ver",      &CmdVersion,                0, "Display firmware version"},
  {"raccel",   &CmdReadAccelerometer,      0, "Read accelerometer"},
  {"rgyro",    &CmdReadGyro,               0, "Read Gyro"},
  {"rmag",     &CmdReadMagnetometer,       0, "Read magnetometer"},
//  {"rGPS",     &CmdGpsRead, 0,  "Read current GPS value" },
  COMMAND_TABLE_END  //MUST BE LAST!!!
};

