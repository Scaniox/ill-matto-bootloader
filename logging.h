/*******************************************************************************
 *         File: include\logging.h
 *       Author: Alexander Mills (am9g22)
 *         Date: 2024-11-18 12:04:40
 *  Description: simple little logging thing to make debug nicer
 ******************************************************************************/

#ifndef INCLUDE_LOGGING_H_
#define INCLUDE_LOGGING_H_

#include <stdio.h>
#include <stdarg.h>

#ifndef MODULE_NAME
#error logging.h - no module defined (add #define MODULE_NAME "[name_here] before including this file)"
#else

#if(LOGGING_ENABLE == 1)
#define log_print(...)  printf("%-10s : ", MODULE_NAME); printf(__VA_ARGS__); printf("\n"); 
#else
#define log_print(...)
#endif


#endif

#endif // INCLUDE_LOGGING_H_