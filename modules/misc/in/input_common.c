#include "input_common.h"

size_t CAER_INPUT_COMMON_STATE_STRUCT_SIZE = 0;

bool caerInputCommonInit(caerModuleData moduleData, int readFd, bool isNetworkStream,
bool isNetworkMessageBased) {
	return (true);
}

void caerInputCommonExit(caerModuleData moduleData) {

}

void caerInputCommonRun(caerModuleData moduleData, size_t argsNumber, va_list args) {

}
