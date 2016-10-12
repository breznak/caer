#include "net_tcp_server.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "output_common.h"

static bool caerOutputNetTCPServerInit(caerModuleData moduleData);

static struct caer_module_functions caerOutputNetTCPServerFunctions = { .moduleInit = &caerOutputNetTCPServerInit,
	.moduleRun = &caerOutputCommonRun, .moduleConfig = NULL, .moduleExit = &caerOutputCommonExit, .moduleReset =
		&caerOutputCommonReset };

void caerOutputNetTCPServer(uint16_t moduleID, size_t outputTypesNumber, ...) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "NetTCPServerOutput", CAER_MODULE_OUTPUT);
	if (moduleData == NULL) {
		return;
	}

	va_list args;
	va_start(args, outputTypesNumber);
	caerModuleSMv(&caerOutputNetTCPServerFunctions, moduleData, CAER_OUTPUT_COMMON_STATE_STRUCT_SIZE, outputTypesNumber,
		args);
	va_end(args);
}

static bool caerOutputNetTCPServerInit(caerModuleData moduleData) {
	// First, always create all needed setting nodes, set their default values
	// and add their listeners.
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "ipAddress", "127.0.0.1");
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "portNumber", 7777);
	sshsNodePutShortIfAbsent(moduleData->moduleNode, "backlogSize", 5);
	sshsNodePutShortIfAbsent(moduleData->moduleNode, "concurrentConnections", 10);

	int retVal;

	// Generate address.
	struct sockaddr_in serverAddress;

	char *ipAddress = sshsNodeGetString(moduleData->moduleNode, "ipAddress");
	retVal = uv_ip4_addr(ipAddress, sshsNodeGetInt(moduleData->moduleNode, "portNumber"), &serverAddress);
	UV_RET_CHECK(retVal, moduleData->moduleSubSystemString, "uv_ip4_addr", free(ipAddress); return (false));
	free(ipAddress);

	// Allocate memory.
	size_t numClients = (size_t) sshsNodeGetShort(moduleData->moduleNode, "concurrentConnections");
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

	streams->server = malloc(sizeof(uv_tcp_t));
	if (streams->server == NULL) {
		free(streams->address);
		free(streams);

		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for network server.");
		return (false);
	}

	// Initialize common info.
	streams->isTCP = true;
	streams->isUDP = false;
	streams->isPipe = false;
	streams->activeClients = 0;
	streams->clientsSize = numClients;
	for (size_t i = 0; i < streams->clientsSize; i++) {
		streams->clients[i] = NULL;
	}

	// Remember address.
	memcpy(streams->address, &serverAddress, sizeof(struct sockaddr_in));

	streams->server->data = streams;

	// Initialize loop and network handles.
	retVal = uv_loop_init(&streams->loop);
	UV_RET_CHECK(retVal, moduleData->moduleSubSystemString, "uv_loop_init",
		free(streams->server); free(streams->address); free(streams));

	retVal = uv_tcp_init(&streams->loop, (uv_tcp_t *) streams->server);
	UV_RET_CHECK(retVal, moduleData->moduleSubSystemString, "uv_tcp_init",
		uv_loop_close(&streams->loop); free(streams->server); free(streams->address); free(streams));

	retVal = uv_tcp_bind((uv_tcp_t *) streams->server, streams->address, 0);
	UV_RET_CHECK(retVal, moduleData->moduleSubSystemString, "uv_tcp_bind",
		libuvCloseLoopHandles(&streams->loop); uv_loop_close(&streams->loop); free(streams->address); free(streams));

	retVal = uv_listen(streams->server, sshsNodeGetShort(moduleData->moduleNode, "backlogSize"),
		&caerOutputCommonOnServerConnection);
	UV_RET_CHECK(retVal, moduleData->moduleSubSystemString, "uv_listen",
		libuvCloseLoopHandles(&streams->loop); uv_loop_close(&streams->loop); free(streams->address); free(streams));

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
