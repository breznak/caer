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

	// Initialize common info.
	streams->isServer = false;
	streams->isTCP = false;
	streams->isUDP = false;
	streams->isPipe = true;
	streams->clientsSize = numClients;
	streams->clients[0] = NULL;
	streams->server = NULL;

	// Remember address.
	streams->address = sshsNodeGetString(moduleData->moduleNode, "socketPath");

	// Initialize loop and network handles.
	uv_loop_init(&streams->loop);

	uv_pipe_t *pipe = malloc(sizeof(uv_pipe_t));

	uv_pipe_init(&streams->loop, pipe, false);
	pipe->data = streams;

	uv_connect_t *connectRequest = malloc(sizeof(uv_connect_t));
	uv_pipe_connect(connectRequest, pipe, streams->address, &caerOutputCommonOnClientConnection);

	// Start.
	if (!caerOutputCommonInit(moduleData, -1, streams)) {
		free(streams);

		return (false);
	}

	return (true);
}
