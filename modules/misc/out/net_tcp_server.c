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
	struct sockaddr_in configServerAddress;

	char *ipAddress = sshsNodeGetString(moduleData->moduleNode, "ipAddress");
	retVal = uv_ip4_addr(ipAddress, sshsNodeGetInt(moduleData->moduleNode, "portNumber"), &configServerAddress);
	UV_RET_CHECK(retVal, moduleData->moduleSubSystemString, "uv_ip4_addr", free(ipAddress));
	free(ipAddress);

	size_t numClients = (size_t) sshsNodeGetShort(moduleData->moduleNode, "concurrentConnections");
	outputCommonNetIO streams = malloc(sizeof(*streams) + (numClients * sizeof(uv_stream_t *)));
	if (streams == NULL) {
		// TODO: error.
	}

	streams->clientsSize = numClients;
	streams->isServer = true;
	streams->isMessageBased = false;

	if (!caerOutputCommonInit(moduleData, -1, streams)) {
		free(streams);

		return (false);
	}

	caerLog(CAER_LOG_INFO, moduleData->moduleSubSystemString, "TCP server socket connected to %s:%" PRIu16 ".",
		inet_ntop(AF_INET, &tcpServer.sin_addr, (char[INET_ADDRSTRLEN] ) { 0x00 }, INET_ADDRSTRLEN),
		ntohs(tcpServer.sin_port));

	return (true);
}
