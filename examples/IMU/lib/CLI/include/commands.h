/******************************************************************************
 * @file commands.h
*******************************************************************************/
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

#ifndef COMMANDS_H
#define _COMMANDS_H

void CmdVersion(uint32_t data);
void CmdUsartBaudRate( uint32_t data );
void CmdReadAccelerometer(uint32_t data);
void CmdReadAccelTemp(uint32_t data);
void CmdReadGyro(uint32_t data);
void CmdReadGyroTemp(uint32_t data);
void CmdGpsRead(uint32_t data);
void CmdReadMagnetometer(uint32_t data);


#endif /* COMMANDS_H */