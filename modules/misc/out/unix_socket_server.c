#include "unix_socket_server.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "output_common.h"

static bool caerOutputUnixSocketServerInit(caerModuleData moduleData);

static struct caer_module_functions caerOutputUnixSocketServerFunctions = { .moduleInit =
	&caerOutputUnixSocketServerInit, .moduleRun = &caerOutputCommonRun, .moduleConfig = NULL, .moduleExit =
	&caerOutputCommonExit, .moduleReset = &caerOutputCommonReset };

void caerOutputUnixSocketServer(uint16_t moduleID, size_t outputTypesNumber, ...) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "UnixSocketServerOutput", CAER_MODULE_OUTPUT);
	if (moduleData == NULL) {
		return;
	}

	va_list args;
	va_start(args, outputTypesNumber);
	caerModuleSMv(&caerOutputUnixSocketServerFunctions, moduleData, CAER_OUTPUT_COMMON_STATE_STRUCT_SIZE,
		outputTypesNumber, args);
	va_end(args);
}

static bool caerOutputUnixSocketServerInit(caerModuleData moduleData) {
	// First, always create all needed setting nodes, set their default values
	// and add their listeners.
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "socketPath", "/tmp/caer.sock");
	sshsNodePutShortIfAbsent(moduleData->moduleNode, "backlogSize", 5);
	sshsNodePutShortIfAbsent(moduleData->moduleNode, "concurrentConnections", 10);

	// Allocate memory.
	size_t numClients = (size_t) sshsNodeGetShort(moduleData->moduleNode, "concurrentConnections");
	outputCommonNetIO streams = malloc(sizeof(*streams) + (numClients * sizeof(uv_stream_t *)));
	if (streams == NULL) {
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for streams structure.");
		return (false);
	}

	streams->server = malloc(sizeof(uv_pipe_t));
	if (streams->server == NULL) {
		free(streams);

		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for network server.");
		return (false);
	}

	// Initialize common info.
	streams->isTCP = false;
	streams->isUDP = false;
	streams->isPipe = true;
	streams->activeClients = 0;
	streams->clientsSize = numClients;
	for (size_t i = 0; i < streams->clientsSize; i++) {
		streams->clients[i] = NULL;
	}

	// Remember address.
	streams->address = sshsNodeGetString(moduleData->moduleNode, "socketPath");

	streams->server->data = streams;

	// Initialize loop and network handles.
	int retVal = uv_loop_init(&streams->loop);
	UV_RET_CHECK(retVal, moduleData->moduleSubSystemString, "uv_loop_init",
		free(streams->server); free(streams->address); free(streams));

	retVal = uv_pipe_init(&streams->loop, (uv_pipe_t *) streams->server, false);
	UV_RET_CHECK(retVal, moduleData->moduleSubSystemString, "uv_pipe_init",
		uv_loop_close(&streams->loop); free(streams->server); free(streams->address); free(streams));

	retVal = uv_pipe_bind((uv_pipe_t *) streams->server, streams->address);
	UV_RET_CHECK(retVal, moduleData->moduleSubSystemString, "uv_pipe_bind",
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
