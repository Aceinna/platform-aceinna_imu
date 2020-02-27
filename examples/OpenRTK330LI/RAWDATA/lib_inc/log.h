/*******************************************************************************
* File Name          : log.h
* Author             : Daich
* Revision           : 1.0
* Date               : 19/09/2019
* Description        : log head file
*
* HISTORY***********************************************************************
* 19/09/2019  |                                             | Daich
*
*******************************************************************************/
#ifndef _LOG_H_
#define _LOG_H_
//#pragma once



#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BYTES_PER_LINE 16

#ifdef DEBUG
        #define DbgPrintf(format,args...) \
        fprintf(stderr, format, ##args)
#else
        #define DbgPrintf(format,args...)
#endif

extern const char* log_type[7] ;

typedef enum {
    ATS_LOG_NONE,       /*!< No log output */
    ATS_LOG_ERROR,      /*!< Critical errors, software module can not recover on its own */
    ATS_LOG_WARN,       /*!< Error conditions from which recovery measures have been taken */
    ATS_LOG_INFO,       /*!< Information messages which describe normal flow of events */
    ATS_LOG_MES,
    ATS_LOG_DEBUG,      /*!< Extra information which is not necessary for normal use (values, pointers, sizes, etc). */
    ATS_LOG_VERBOSE     /*!< Bigger chunks of debugging information, or frequent messages which can potentially flood the output. */
} ats_log_level_t;
#define LOG_LOCAL_LEVEL ATS_LOG_DEBUG

#define CONFIG_LOG_COLORS
#ifdef CONFIG_LOG_COLORS
#define LOG_COLOR_BLACK   "30"
#define LOG_COLOR_RED     "31"
#define LOG_COLOR_GREEN   "32"
#define LOG_COLOR_BROWN   "33"
#define LOG_COLOR_BLUE    "34"
#define LOG_COLOR_PURPLE  "35"
#define LOG_COLOR_CYAN    "36"
#define LOG_COLOR_WHITE   "37"
#define LOG_COLOR(COLOR)  "\033[0;" COLOR "m"   //高亮
//#define LOG_COLOR(COLOR)  "\033[5;" COLOR "m"   //闪烁 可用分号叠加

#define LOG_BOLD(COLOR)   "\033[1;" COLOR "m"
#define LOG_DEFAULT_COLOR   "\033[0m"
#define LOG_COLOR_E       LOG_COLOR(LOG_COLOR_RED)//LOG_COLOR(LOG_COLOR_RED)
#define LOG_COLOR_W       LOG_COLOR(LOG_COLOR_BROWN)
#define LOG_COLOR_I       LOG_COLOR(LOG_COLOR_GREEN)
#define LOG_COLOR_M       LOG_COLOR(LOG_COLOR_CYAN)
#define LOG_COLOR_D       LOG_COLOR(LOG_COLOR_WHITE)
#define LOG_COLOR_V
#else //CONFIG_LOG_COLORS
#define LOG_COLOR_E
#define LOG_COLOR_W
#define LOG_COLOR_I
#define LOG_COLOR_D
#define LOG_COLOR_V
#define LOG_DEFAULT_COLOR
#endif //CONFIG_LOG_COLORS

uint32_t ats_log_timestamp(void);
void ats_log_write(ats_log_level_t level, const char* tag, const char* format, ...) __attribute__ ((format (printf, 3, 4)));

#ifdef PUT_TIME
#define LOG_FORMAT(letter, format)  LOG_COLOR_ ## letter #letter " (%d) %s: " format LOG_DEFAULT_COLOR "\r\n"

#define ATS_LOGE( tag, format, ... )  if (LOG_LOCAL_LEVEL >= ATS_LOG_ERROR)   { ats_log_write(ATS_LOG_ERROR,   tag, LOG_FORMAT(E, format), ats_log_timestamp(), tag, ##__VA_ARGS__); }
#define ATS_LOGI( tag, format, ... )  ATS_LOG_LEVEL_LOCAL(ATS_LOG_INFO,    tag, format, ##__VA_ARGS__)
#define ATS_LOGV( tag, format, ... )  if (LOG_LOCAL_LEVEL >= ATS_LOG_VERBOSE) { ats_log_write(ATS_LOG_VERBOSE, tag, LOG_FORMAT(V, format), ats_log_timestamp(), tag, ##__VA_ARGS__); }

#define ATS_LOGD( tag, format, ... )  if (LOG_LOCAL_LEVEL >= ATS_LOG_DEBUG)   { ats_log_write(ATS_LOG_DEBUG,   tag, LOG_FORMAT(D, format), ats_log_timestamp(), tag, ##__VA_ARGS__); }

#define ATS_LOGW( tag, format, ... )  if (LOG_LOCAL_LEVEL >= ATS_LOG_WARN)    { ats_log_write(ATS_LOG_WARN,    tag, LOG_FORMAT(W, format), ats_log_timestamp(), tag, ##__VA_ARGS__); }
#else
#define LOG_FORMAT(letter, format)  LOG_COLOR_ ## letter #letter " %s: " format LOG_DEFAULT_COLOR "\r\n"
//#define LOG_FORMAT_RTK(letter, format)  log_type[letter], "%s %s: " format LOG_DEFAULT_COLOR "\r\n"  


#define LOG_FORMAT_RTK(letter, format)  letter  "%s %s: " format LOG_DEFAULT_COLOR "\r\n"       //加上tag和mes字符串
#define LOG_MES_TYPE(prio) log_type[prio]

//#define ATS_LOG(tag, prio, format, ... )  if (LOG_LOCAL_LEVEL >= ATS_LOG_ERROR)   { ats_log_write(ATS_LOG_ERROR, tag ,LOG_FORMAT_RTK(prio, format), LOG_MES_TYPE(prio),tag, ##__VA_ARGS__); }
//#define ATS_LOG(tag, prio, format, ... )  if (LOG_LOCAL_LEVEL >= ATS_LOG_ERROR)   { ats_log_write(ATS_LOG_ERROR, tag ,LOG_FORMAT_RTK(LOG_COLOR_W, format), LOG_MES_TYPE(prio),tag, ##__VA_ARGS__); }
#define ATS_LOG(tag, prio, format, ... )  if (LOG_LOCAL_LEVEL >= ATS_LOG_ERROR)   { RTK_LOG(prio, tag ,format, ##__VA_ARGS__); }

#define ATS_LOGE( tag, format, ... )  if (LOG_LOCAL_LEVEL >= ATS_LOG_ERROR)   { ats_log_write(ATS_LOG_ERROR,   tag, LOG_FORMAT(E, format), tag, ##__VA_ARGS__); }
#define ATS_LOGV( tag, format, ... )  if (LOG_LOCAL_LEVEL >= ATS_LOG_VERBOSE) { ats_log_write(ATS_LOG_VERBOSE, tag, LOG_FORMAT(V, format), tag, ##__VA_ARGS__); }

#define ATS_LOGD( tag, format, ... )  if (LOG_LOCAL_LEVEL >= ATS_LOG_DEBUG)   { ats_log_write(ATS_LOG_DEBUG,   tag, LOG_FORMAT(D, format), tag, ##__VA_ARGS__); }

#define ATS_LOGW( tag, format, ... )  if (LOG_LOCAL_LEVEL >= ATS_LOG_WARN)    { ats_log_write(ATS_LOG_WARN,    tag, LOG_FORMAT(W, format), tag, ##__VA_ARGS__); }
#endif






void ats_log_level_set(ats_log_level_t log_level_set);
void RTK_LOG(ats_log_level_t level,const char *tag,const char *format, ...);
void RTK_LOG_BUFF(ats_log_level_t log_level,const char *tag, const void *buffer, uint16_t buff_len);
void RTK_LOG_HEXDUMP(ats_log_level_t log_level,const char *tag, const void *buffer, uint16_t buff_len);

#ifdef __cplusplus
}
#endif

#endif