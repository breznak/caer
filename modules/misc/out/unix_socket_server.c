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

	// Initialize common info.
	streams->isServer = true;
	streams->isTCP = false;
	streams->isUDP = false;
	streams->isPipe = true;
	streams->clientsSize = numClients;
	for (size_t i = 0; i < streams->clientsSize; i++) {
		streams->clients[i] = NULL;
	}

	// Remember address.
	streams->address = sshsNodeGetString(moduleData->moduleNode, "socketPath");

	// Initialize loop and network handles.
	uv_loop_init(&streams->loop);

	streams->server = malloc(sizeof(uv_pipe_t));

	uv_pipe_init(&streams->loop, (uv_pipe_t *) streams->server, false);
	streams->server->data = streams;

	uv_pipe_bind((uv_pipe_t *) streams->server, streams->address);

	uv_listen(streams->server, sshsNodeGetShort(moduleData->moduleNode, "backlogSize"),
		&caerOutputCommonOnServerConnection);

	// Start.
	if (!caerOutputCommonInit(moduleData, -1, streams)) {
		free(streams);

		return (false);
	}

	return (true);
}
