#ifndef OUTPUT_COMMON_H_
#define OUTPUT_COMMON_H_

#include "base/module.h"
#include "modules/misc/inout_common.h"

#define MAX_OUTPUT_RINGBUFFER_GET 10
#define MAX_OUTPUT_UDP_SIZE (1472 - AEDAT3_NETWORK_HEADER_LENGTH)

extern size_t CAER_OUTPUT_COMMON_STATE_STRUCT_SIZE;

struct output_common_netio {
	/// Keep the full network header around, so we can easily update and write it.
	struct aedat3_network_header networkHeader;
	bool isTCP;
	bool isUDP;
	bool isPipe;
	bool isServer;
	void *address;
	uv_loop_t loop;
	uv_async_t shutdown;
	uv_idle_t ringBufferGet;
	uv_stream_t *server;
	size_t activeClients;
	size_t clientsSize;
	uv_stream_t *clients[];
};

typedef struct output_common_netio *outputCommonNetIO;

bool caerOutputCommonInit(caerModuleData moduleData, int fileDescriptor, outputCommonNetIO streams);
void caerOutputCommonExit(caerModuleData moduleData);
void caerOutputCommonRun(caerModuleData moduleData, size_t argsNumber, va_list args);
void caerOutputCommonReset(caerModuleData moduleData, uint16_t resetCallSourceID);
void caerOutputCommonOnServerConnection(uv_stream_t *server, int status);
void caerOutputCommonOnClientConnection(uv_connect_t *connectionRequest, int status);

#endif /* OUTPUT_COMMON_H_ */
