/** ***************************************************************************
 * @file   commandLine.c
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * Simple interactive serial console shell - line input
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

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "commandLine.h"
#include "debug.h"
#include "debug_usart.h"
#include "utilities.h"
#include "osapi.h"
#include "osresources.h"

static void _ExecLine(tCommand const *cmd_table);
static void _Dispatch(char const *token, tCommand const *table);
static void _CmdHelp(tCommand const *table);

static char gCmdLine[80];
static char *gCmdLineIndex;
#define PARSE_SEPARATOR ' '

/** ***************************************************************************
 * @name    CmdPrintPrompt()
 * @brief send prompt characters out serial to the debug serial console
 *
 * @param [in] N/A
 * @param [out] "#> " out serial to console
 * @retval N/A
 ******************************************************************************/
void CmdPrintPrompt()
{
	static uint16_t lineno;
    DebugPrintInt("", lineno);
    lineno++;
    DebugPrintString("> ");
}

/** ***************************************************************************
 * @name    CmdLineLookup()
 * @brief at startup call send prompt characters out serial to console
 * the debug serial
 *
 * @param [in] cmd_table - table entry to parse
 * @param [in] "#> " out serial to console
 * @retval N/A
 ******************************************************************************/
void CmdLineLookup(tCommand const *cmd_table)
{
    static uint32_t index = 0;
    if (DebugSerialReadLine((uint8_t*) gCmdLine, &index, 80)) {
        strrep(gCmdLine, '\r', 0);
        strrep(gCmdLine, '\n', 0);
        if (gCmdLine[0]) {//don't process empty string
            //Ignore lines that start with # character
            if (gCmdLine[0] != '#') {
                _ExecLine(cmd_table);
            }
        }
        CmdPrintPrompt();
	} // else wait until prompt is ready
}

/** ***************************************************************************
 * @name    _ExecLine() LOCAL
 * @brief extract tokens from strings with ' ' as the delimiter
 *
 * @param [in] cmd_table - table entry to parse
 * @param [in] "#> " out serial to console
 * @retval N/A
 ******************************************************************************/
static void _ExecLine(tCommand const *cmd_table) {
	char *token = strtok_r1(gCmdLine, PARSE_SEPARATOR, &gCmdLineIndex);
	_Dispatch(token, cmd_table);
}

void CmdLinexec(char const     *cmdline,
                tCommand const *cmd_table) {
    char *dst;
	if (cmdline && *cmdline) {
		dst = gCmdLine;
		while ((*dst++ = *cmdline++)){;}
        _ExecLine(cmd_table);
	}
}

/** ***************************************************************************
 * @name    CmdLineGetArgString()
 * @brief get string from command line arguments
 *
 * @param [out] s - pointer to a string
 * @retval N/A
 ******************************************************************************/
int CmdLineGetArgString(uint8_t **s) {
    char *token = strtok_r1(0, PARSE_SEPARATOR, &gCmdLineIndex);
    if (token) {
        if (s) {
            *s = (uint8_t*)token;
        }
        return 1;
    } else {
        return 0;
    }
}

int CmdLineGetArgInt(int32_t *i) {
    char *token = strtok_r1(0, PARSE_SEPARATOR, &gCmdLineIndex);
    if (token) {
        if (i) {
            *i = atoi(token);
        }
        return 1;
    } else {
        return 0;
    }
}

/** ***************************************************************************
 * @name    CmdLineGetArgUInt()
 * @brief get unsigned integer from command line arguments
 *
 * @param [out] t - ingteger to return
 * @retval N/A
 ******************************************************************************/
int CmdLineGetArgUInt(uint32_t *i) {
    char *token = strtok_r1(0, PARSE_SEPARATOR, &gCmdLineIndex);
    if (token) {
        if (i) {
            *i = atoi(token);
        }
        return 1;
    } else {
        return 0;
    }
}

int CmdLineGetArgFloat(float *f) {
    char *token = strtok_r1(0, PARSE_SEPARATOR, &gCmdLineIndex);
    if (token) {
        if (f) {
            *f = atof(token);
        }
        return 1;
    } else {
        return 0;
    }
}


/** ***************************************************************************
 * @name    _Dispatch()
 * @brief Traverses a command table and dispatches the appropriate callback
 *
 * @param [in] token - command text from the console
 * @param [in] table - command table
 * @retval N/A
 ******************************************************************************/
static void _Dispatch(char const     *token,
                      tCommand const *table)
{
	tCommand *cmd = (tCommand*) table;
	if (0 == strcmpi(token, "help")) {
        _CmdHelp(table);
	} else {
        while (cmd->callback != 0) {
            if (0 == strcmpi(token, cmd->name)) {
                (*(cmd->callback))(cmd->callbackData);
                return;
            } else {
                cmd++;
            }
        }
        //If we get here, the command wasn't found
        DebugPrintString("Unknown command (");
        DebugPrintString(token);
        DebugPrintString(") Type 'help' for list.\r\n");
    }
}

/** ***************************************************************************
 * @name    _CmdHelp() LOCAL
 * @brief traverses command table and prints the help strings
 *
 * @param [in] token - command text from the console
 * @param [in] table - command table
 * @retval N/A
 ******************************************************************************/
static void _CmdHelp(tCommand const *table) {
    char command[30];

	tCommand *cmd = (tCommand*) table;

	while (cmd->callback != 0) {
		DebugPrintString(" ");
        sprintf(command, "%-10s", cmd->name);
//		DebugPrintString(cmd->name);
		DebugPrintString(command);
//		DebugPrintString("    ");
		DebugPrintString(cmd->help);
        DebugPrintEndline();
        DebugPrintEndline();
		cmd++;
        while (!IsDebugSerialIdle())
        {/*spin*/;}
	}
}

/** ****************************************************************************
 * @name TaskCommandLine() LOCAL task callback function
 * @brief handles debug serial console command line messaging
 *        When the USART interrupt handler indicates a RX on the USART line,
 *        toggle the LED and parse the received message using the commandTable
 *
 * @param [in] gCommands - command line command and arguments
 * @param [out] "#> " out serial to console
 * @retval N/A
 ******************************************************************************/
void TaskCommandLine(void const *argument)
{
    CmdPrintPrompt(); // out to serial console

    while (1) {
        /// context switch and wait for the semaphore, BINSEM_SERIAL_RX, to
        /// change state: occurs in the USART interrupt handler (DEBUG_USART_IRQ)
        osSemaphoreWait( cliSem, OS_INFINITE_TIMEOUT );
        CmdLineLookup( gCommands ); // commandLine.c
    }
}
