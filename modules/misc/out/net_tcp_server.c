#include "net_tcp_server.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "output_common.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include "ext/nets.h"

static bool caerOutputNetTCPServerInit(caerModuleData moduleData);

static struct caer_module_functions caerOutputNetTCPServerFunctions = { .moduleInit = &caerOutputNetTCPServerInit,
	.moduleRun = &caerOutputCommonRun, .moduleConfig = NULL, .moduleExit = &caerOutputCommonExit };

void caerOutputNetTCPServer(uint16_t moduleID, size_t outputTypesNumber, ...) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "NetTCPServerOutput");
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

	// Open a TCP server socket for others to connect to.
	int serverSockFd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (serverSockFd < 0) {
		caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString, "Could not create TCP server socket. Error: %d.",
		errno);
		return (false);
	}

	// Make socket address reusable right away.
	socketReuseAddr(serverSockFd, true);

	// Set server socket, on which accept() is called, to non-blocking mode.
	if (!socketBlockingMode(serverSockFd, false)) {
		close(serverSockFd);

		caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString,
			"Could not set TCP server socket to non-blocking mode.");
		return (false);
	}

	struct sockaddr_in tcpServer;
	memset(&tcpServer, 0, sizeof(struct sockaddr_in));

	tcpServer.sin_family = AF_INET;
	tcpServer.sin_port = htons(U16T(sshsNodeGetInt(moduleData->moduleNode, "portNumber")));

	char *ipAddress = sshsNodeGetString(moduleData->moduleNode, "ipAddress");
	if (inet_pton(AF_INET, ipAddress, &tcpServer.sin_addr) == 0) {
		close(serverSockFd);

		caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString,
			"No valid server IP address found. '%s' is invalid!", ipAddress);

		free(ipAddress);
		return (false);
	}
	free(ipAddress);

	// Bind socket to above address.
	if (bind(serverSockFd, (struct sockaddr *) &tcpServer, sizeof(struct sockaddr_in)) < 0) {
		close(serverSockFd);

		caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString, "Could not bind TCP server socket. Error: %d.",
		errno);
		return (false);
	}

	// Listen to new connections on the socket.
	if (listen(serverSockFd, sshsNodeGetShort(moduleData->moduleNode, "backlogSize")) < 0) {
		close(serverSockFd);

		caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString,
			"Could not listen on TCP server socket. Error: %d.", errno);
		return (false);
	}

	outputCommonFDs fileDescriptors = caerOutputCommonAllocateFdArray(
		(size_t) sshsNodeGetShort(moduleData->moduleNode, "concurrentConnections"));
	if (fileDescriptors == NULL) {
		close(serverSockFd);

		caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString,
			"Unable to allocate memory for file descriptors.");
		return (false);
	}

	fileDescriptors->serverFd = serverSockFd;

	if (!caerOutputCommonInit(moduleData, fileDescriptors, true, false)) {
		close(serverSockFd);
		free(fileDescriptors);

		return (false);
	}

	caerLog(CAER_LOG_INFO, moduleData->moduleSubSystemString, "TCP server socket connected to %s:%" PRIu16 ".",
		inet_ntop(AF_INET, &tcpServer.sin_addr, (char[INET_ADDRSTRLEN] ) { 0x00 }, INET_ADDRSTRLEN),
		ntohs(tcpServer.sin_port));

	return (true);
}
