#ifndef OUTPUT_COMMON_H_
#define OUTPUT_COMMON_H_

#include "main.h"
#include "base/module.h"

struct output_common_fds {
	size_t fdsSize;
	int fds[];
};

typedef struct output_common_fds *outputCommonFDs;

bool caerOutputCommonInit(caerModuleData moduleData, outputCommonFDs fds);
void caerOutputCommonExit(caerModuleData moduleData);
void caerOutputCommonRun(caerModuleData moduleData, size_t argsNumber, va_list args);

#endif /* OUTPUT_COMMON_H_ */
