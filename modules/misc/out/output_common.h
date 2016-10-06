#ifndef OUTPUT_COMMON_H_
#define OUTPUT_COMMON_H_

#include "base/module.h"
#include "modules/misc/inout_common.h"

extern size_t CAER_OUTPUT_COMMON_STATE_STRUCT_SIZE;

struct output_common_netio {
	bool isTCP;
	bool isUDP;
	bool isPipe;
	bool isServer;
	uv_stream_t *server;
	size_t clientsSize;
	uv_stream_t *clients[];
};

typedef struct output_common_netio *outputCommonNetIO;

bool caerOutputCommonInit(caerModuleData moduleData, int fileDescriptor, outputCommonNetIO streams);
void caerOutputCommonExit(caerModuleData moduleData);
void caerOutputCommonRun(caerModuleData moduleData, size_t argsNumber, va_list args);
void caerOutputCommonReset(caerModuleData moduleData, uint16_t resetCallSourceID);

#endif /* OUTPUT_COMMON_H_ */
