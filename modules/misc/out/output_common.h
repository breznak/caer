#ifndef OUTPUT_COMMON_H_
#define OUTPUT_COMMON_H_

#include "main.h"
#include "base/module.h"

#include <stdatomic.h>
#include <unistd.h>

extern size_t CAER_OUTPUT_COMMON_STATE_STRUCT_SIZE;

struct output_common_fds {
	int serverFd;
	size_t fdsSize;
	int fds[];
};

typedef struct output_common_fds *outputCommonFDs;

outputCommonFDs caerOutputCommonAllocateFdArray(size_t size);
bool caerOutputCommonInit(caerModuleData moduleData, outputCommonFDs fds);
void caerOutputCommonExit(caerModuleData moduleData);
void caerOutputCommonRun(caerModuleData moduleData, size_t argsNumber, va_list args);

#endif /* OUTPUT_COMMON_H_ */
