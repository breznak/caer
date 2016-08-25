#ifndef INPUT_COMMON_H_
#define INPUT_COMMON_H_

#include "base/module.h"
#include "modules/misc/inout_common.h"

extern size_t CAER_INPUT_COMMON_STATE_STRUCT_SIZE;

bool caerInputCommonInit(caerModuleData moduleData, int readFd, bool isNetworkStream,
bool isNetworkMessageBased);
void caerInputCommonExit(caerModuleData moduleData);
void caerInputCommonRun(caerModuleData moduleData, size_t argsNumber, va_list args);

#endif /* INPUT_COMMON_H_ */
