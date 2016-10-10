#include "net_tcp.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "output_common.h"

static bool caerOutputNetTCPInit(caerModuleData moduleData);

static struct caer_module_functions caerOutputNetTCPFunctions = { .moduleInit = &caerOutputNetTCPInit, .moduleRun =
	&caerOutputCommonRun, .moduleConfig = NULL, .moduleExit = &caerOutputCommonExit, .moduleReset =
	&caerOutputCommonReset };

void caerOutputNetTCP(uint16_t moduleID, size_t outputTypesNumber, ...) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "NetTCPOutput", CAER_MODULE_OUTPUT);
	if (moduleData == NULL) {
		return;
	}

	va_list args;
	va_start(args, outputTypesNumber);
	caerModuleSMv(&caerOutputNetTCPFunctions, moduleData, CAER_OUTPUT_COMMON_STATE_STRUCT_SIZE, outputTypesNumber,
		args);
	va_end(args);
}

static bool caerOutputNetTCPInit(caerModuleData moduleData) {
	// First, always create all needed setting nodes, set their default values
	// and add their listeners.
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "ipAddress", "127.0.0.1");
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "portNumber", 8888);

	int retVal;

	// Generate address.
	struct sockaddr_in serverAddress;

	char *ipAddress = sshsNodeGetString(moduleData->moduleNode, "ipAddress");
	retVal = uv_ip4_addr(ipAddress, sshsNodeGetInt(moduleData->moduleNode, "portNumber"), &serverAddress);
	UV_RET_CHECK(retVal, moduleData->moduleSubSystemString, "uv_ip4_addr", free(ipAddress));
	free(ipAddress);

	// Allocate memory.
	size_t numClients = 1;
	outputCommonNetIO streams = malloc(sizeof(*streams) + (numClients * sizeof(uv_stream_t *)));
	if (streams == NULL) {
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for streams structure.");
		return (false);
	}

	// Initialize common info.
	streams->isServer = false;
	streams->isTCP = true;
	streams->isUDP = false;
	streams->isPipe = false;
	streams->clientsSize = numClients;
	streams->server = NULL;

	// Remember address.
	streams->address = malloc(sizeof(struct sockaddr_in));
	memcpy(streams->address, &serverAddress, sizeof(struct sockaddr_in));

	// Initialize loop and network handles.
	uv_loop_init(&streams->loop);

	streams->clients[0] = malloc(sizeof(uv_tcp_t));

	uv_tcp_init(&streams->loop, (uv_tcp_t *) streams->clients[0]);
	streams->clients[0]->data = streams;

	uv_connect_t *connectRequest = malloc(sizeof(uv_connect_t));
	uv_tcp_connect(connectRequest, (uv_tcp_t *) streams->clients[0], streams->address,
		&caerOutputCommonOnClientConnection);

	// Start.
	if (!caerOutputCommonInit(moduleData, -1, streams)) {
		free(streams);

		return (false);
	}

	return (true);
}
