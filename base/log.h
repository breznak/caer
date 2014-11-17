/*
 * log.h
 *
 *  Created on: Dec 30, 2013
 *      Author: llongi
 */

#ifndef LOG_H_
#define LOG_H_

#include "main.h"
#include "module.h"
#include <stdarg.h>

// Debug severity levels
#define LOG_EMERGENCY (0)
#define LOG_ALERT     (1)
#define LOG_CRITICAL  (2)
#define LOG_ERROR     (3)
#define LOG_WARNING   (4)
#define LOG_NOTICE    (5)
#define LOG_INFO      (6)
#define LOG_DEBUG     (7)

void caerLogInit(void);
void caerLogDisableConsole(void);
void caerLog(uint8_t logLevel, caerModuleData modData, const char *format, ...) __attribute__ ((format (printf, 3, 4)));
const char *caerLogStrerror(int errnum);

#endif /* LOG_H_ */
