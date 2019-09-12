/** ***************************************************************************
 * @file commandLine.h DEBUG parser functions
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
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

#ifndef COMMAND_LINE_H
#define COMMAND_LINE_H

#include <stdint.h>


typedef void(*tShellCallback)(uint32_t);

/// command token, callback and help table
typedef struct {
    char const     *name;
    tShellCallback callback;
    uint32_t       callbackData;
    char const     *help;
} tCommand;

extern const tCommand gCommands[];

void CmdPrintPrompt();
void CmdLineLookup(tCommand const *cmd_table);
int  CmdLineGetArgString(uint8_t **s);
int  CmdLineGetArgInt(int32_t *i);
int  CmdLineGetArgUInt(uint32_t *i);
int  CmdLineGetArgFloat(float *f);
void CmdLineExec(char const *cmdline, const tCommand *cmd_table);
void TaskCommandLine(void const *argument);


#endif //COMMAND_LINE_H