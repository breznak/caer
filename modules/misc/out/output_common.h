#ifndef OUTPUT_COMMON_H_
#define OUTPUT_COMMON_H_

#include "main.h"
#include "base/module.h"

bool caerOutputCommonInit(caerModuleData moduleData, int fd);
void caerOutputCommonExit(caerModuleData moduleData);
void caerOutputCommonRun(caerModuleData moduleData, size_t argsNumber, va_list args);

#endif /* OUTPUT_COMMON_H_ */
