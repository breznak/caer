#include "unix_socket.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "output_common.h"

static bool caerOutputUnixSocketInit(caerModuleData moduleData);

static struct caer_module_functions caerOutputUnixSocketFunctions = { .moduleInit = &caerOutputUnixSocketInit,
	.moduleRun = &caerOutputCommonRun, .moduleConfig = NULL, .moduleExit = &caerOutputCommonExit, .moduleReset =
		&caerOutputCommonReset };

void caerOutputUnixSocket(uint16_t moduleID, size_t outputTypesNumber, ...) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "UnixSocketOutput", CAER_MODULE_OUTPUT);
	if (moduleData == NULL) {
		return;
	}

	va_list args;
	va_start(args, outputTypesNumber);
	caerModuleSMv(&caerOutputUnixSocketFunctions, moduleData, CAER_OUTPUT_COMMON_STATE_STRUCT_SIZE, outputTypesNumber,
		args);
	va_end(args);
}

static bool caerOutputUnixSocketInit(caerModuleData moduleData) {
	// First, always create all needed setting nodes, set their default values
	// and add their listeners.
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "socketPath", "/tmp/caer.sock");

	// Allocate memory.
	size_t numClients = 1;
	outputCommonNetIO streams = malloc(sizeof(*streams) + (numClients * sizeof(uv_stream_t *)));
	if (streams == NULL) {
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for streams structure.");
		return (false);
	}

	uv_pipe_t *pipe = malloc(sizeof(uv_pipe_t));
	if (pipe == NULL) {
		free(streams);

		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for network structure.");
		return (false);
	}

	uv_connect_t *connectRequest = malloc(sizeof(uv_connect_t));
	if (connectRequest == NULL) {
		free(pipe);
		free(streams);

		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for network connection.");
		return (false);
	}

	// Initialize common info.
	streams->isTCP = false;
	streams->isUDP = false;
	streams->isPipe = true;
	streams->activeClients = 0;
	streams->clientsSize = numClients;
	streams->clients[0] = NULL;
	streams->server = NULL;

	// Remember address.
	streams->address = sshsNodeGetString(moduleData->moduleNode, "socketPath");

	pipe->data = streams;

	// Initialize loop and network handles.
	int retVal = uv_loop_init(&streams->loop);
	UV_RET_CHECK(retVal, moduleData->moduleSubSystemString, "uv_loop_init",
		free(connectRequest); free(pipe); free(streams->address); free(streams));

	retVal = uv_pipe_init(&streams->loop, pipe, false);
	UV_RET_CHECK(retVal, moduleData->moduleSubSystemString, "uv_pipe_init",
		uv_loop_close(&streams->loop); free(connectRequest); free(pipe); free(streams->address); free(streams));

	uv_pipe_connect(connectRequest, pipe, streams->address, &caerOutputCommonOnClientConnection);
	// No return value to check here.

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
