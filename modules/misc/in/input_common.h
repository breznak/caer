#ifndef INPUT_COMMON_H_
#define INPUT_COMMON_H_

#include "main.h"
#include "base/module.h"

#include <stdatomic.h>
#include <unistd.h>

#define AEDAT3_NETWORK_HEADER_LENGTH 20
#define AEDAT3_NETWORK_MAGIC_NUMBER 0x1D378BC90B9A6658
#define AEDAT3_NETWORK_VERSION 0x01
#define AEDAT3_FILE_VERSION "3.1"

extern size_t CAER_INPUT_COMMON_STATE_STRUCT_SIZE;

bool caerInputCommonInit(caerModuleData moduleData, int readFd, bool isNetworkStream,
bool isNetworkMessageBased);
void caerInputCommonExit(caerModuleData moduleData);
void caerInputCommonRun(caerModuleData moduleData, size_t argsNumber, va_list args);

#endif /* INPUT_COMMON_H_ */
