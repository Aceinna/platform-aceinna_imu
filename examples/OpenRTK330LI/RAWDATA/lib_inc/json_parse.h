/*******************************************************************************
* File Name          : json_parse.h
* Author             : Daich
* Revision           : 1.0
* Date               : 29/09/2019
* Description        : json_parse.h
*
* HISTORY***********************************************************************
* 29/09/2019  |                                             | Daich
* 06/01/2020  |                                             | Daich
* Description: add send_rtk_json_message function to send openrtk board message
*******************************************************************************/
#ifndef _JSON_PARSE_H_
#define _JSON_PARSE_H_
#include "cJSON.h"

#define json_true 1
#define json_false -1

int json_create();
int change_item(char *text,char* json_to_write,char* key,char* value);
void create_json_object(cJSON** json);
void send_rtk_json_message(cJSON* root);
char* get_rtk_json_item_value(cJSON *json,char* key);
void send_rtk_json_to_esp32();
#endif

