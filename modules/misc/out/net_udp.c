#include "net_udp.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "output_common.h"

static bool caerOutputNetUDPInit(caerModuleData moduleData);

static struct caer_module_functions caerOutputNetUDPFunctions = { .moduleInit = &caerOutputNetUDPInit, .moduleRun =
	&caerOutputCommonRun, .moduleConfig = NULL, .moduleExit = &caerOutputCommonExit, .moduleReset =
	&caerOutputCommonReset };

void caerOutputNetUDP(uint16_t moduleID, size_t outputTypesNumber, ...) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "NetUDPOutput", CAER_MODULE_OUTPUT);
	if (moduleData == NULL) {
		return;
	}

	va_list args;
	va_start(args, outputTypesNumber);
	caerModuleSMv(&caerOutputNetUDPFunctions, moduleData, CAER_OUTPUT_COMMON_STATE_STRUCT_SIZE, outputTypesNumber,
		args);
	va_end(args);
}

static bool caerOutputNetUDPInit(caerModuleData moduleData) {
	// First, always create all needed setting nodes, set their default values
	// and add their listeners.
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "ipAddress", "127.0.0.1");
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "portNumber", 7777);

	int retVal;

	// Generate address.
	struct sockaddr_in serverAddress;

	char *ipAddress = sshsNodeGetString(moduleData->moduleNode, "ipAddress");
	retVal = uv_ip4_addr(ipAddress, sshsNodeGetInt(moduleData->moduleNode, "portNumber"), &serverAddress);
	UV_RET_CHECK(retVal, moduleData->moduleSubSystemString, "uv_ip4_addr", free(ipAddress); return (false));
	free(ipAddress);

	// Allocate memory.
	size_t numClients = 1;
	outputCommonNetIO streams = malloc(sizeof(*streams) + (numClients * sizeof(uv_stream_t *)));
	if (streams == NULL) {
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for streams structure.");
		return (false);
	}

	streams->address = malloc(sizeof(struct sockaddr_in));
	if (streams->address == NULL) {
		free(streams);

		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for network address.");
		return (false);
	}

	uv_udp_t *udp = malloc(sizeof(uv_udp_t));
	if (udp == NULL) {
		free(streams->address);
		free(streams);

		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for network structure.");
		return (false);
	}

	// Initialize common info.
	streams->isTCP = false;
	streams->isUDP = true;
	streams->isPipe = false;
	streams->activeClients = 0;
	streams->clientsSize = numClients;
	streams->clients[0] = NULL;
	streams->server = NULL;

	// Remember address.
	memcpy(streams->address, &serverAddress, sizeof(struct sockaddr_in));

	udp->data = streams;

	// Assign here instead of caerOutputCommonOnClientConnection(), since that doesn't
	// exist for UDP connections in libuv.
	streams->clients[0] = (uv_stream_t *) udp;
	streams->activeClients = 1;

	// Initialize loop and network handles.
	retVal = uv_loop_init(&streams->loop);
	UV_RET_CHECK(retVal, moduleData->moduleSubSystemString, "uv_loop_init",
		free(udp); free(streams->address); free(streams));

	retVal = uv_udp_init(&streams->loop, udp);
	UV_RET_CHECK(retVal, moduleData->moduleSubSystemString, "uv_udp_init",
		uv_loop_close(&streams->loop); free(udp); free(streams->address); free(streams));

	// Start.
	if (!caerOutputCommonInit(moduleData, -1, streams)) {
		libuvCloseLoopHandles(&streams->loop);
		uv_loop_close(&streams->loop);
		free(streams->address);
		free(streams);

		return (false);
	}

	return (true);
}
