/** ***************************************************************************
 * @file commandLine.h DEBUG parser functions
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
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


#endif //COMMAND_LINE_H