#include "net_tcp.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "output_common.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

static bool caerOutputNetTCPInit(caerModuleData moduleData);

static struct caer_module_functions caerOutputNetTCPFunctions = { .moduleInit = &caerOutputNetTCPInit, .moduleRun =
	&caerOutputCommonRun, .moduleConfig = NULL, .moduleExit = &caerOutputCommonExit };

void caerOutputNetTCP(uint16_t moduleID, size_t outputTypesNumber, ...) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "NetTCPOutput", OUTPUT);
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

	// Open a TCP socket to the remote client, to which we'll send data packets.
	int sockFd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (sockFd < 0) {
		caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString, "Could not create TCP socket. Error: %d.", errno);
		return (false);
	}

	struct sockaddr_in tcpClient;
	memset(&tcpClient, 0, sizeof(struct sockaddr_in));

	tcpClient.sin_family = AF_INET;
	tcpClient.sin_port = htons(U16T(sshsNodeGetInt(moduleData->moduleNode, "portNumber")));

	char *ipAddress = sshsNodeGetString(moduleData->moduleNode, "ipAddress");
	if (inet_pton(AF_INET, ipAddress, &tcpClient.sin_addr) == 0) {
		close(sockFd);

		caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString, "No valid IP address found. '%s' is invalid!",
			ipAddress);

		free(ipAddress);
		return (false);
	}
	free(ipAddress);

	if (connect(sockFd, (struct sockaddr *) &tcpClient, sizeof(struct sockaddr_in)) != 0) {
		close(sockFd);

		caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString,
			"Could not connect to remote TCP server %s:%" PRIu16 ". Error: %d.",
			inet_ntop(AF_INET, &tcpClient.sin_addr, (char[INET_ADDRSTRLEN] ) { 0x00 }, INET_ADDRSTRLEN),
			ntohs(tcpClient.sin_port), errno);
		return (false);
	}

	outputCommonFDs fileDescriptors = caerOutputCommonAllocateFdArray(1);
	if (fileDescriptors == NULL) {
		close(sockFd);

		caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString,
			"Unable to allocate memory for file descriptors.");
		return (false);
	}

	fileDescriptors->fds[0] = sockFd;

	if (!caerOutputCommonInit(moduleData, fileDescriptors, true, false)) {
		close(sockFd);
		free(fileDescriptors);

		return (false);
	}

	caerLog(CAER_LOG_INFO, moduleData->moduleSubSystemString, "TCP socket connected to %s:%" PRIu16 ".",
		inet_ntop(AF_INET, &tcpClient.sin_addr, (char[INET_ADDRSTRLEN] ) { 0x00 }, INET_ADDRSTRLEN),
		ntohs(tcpClient.sin_port));

	return (true);
}
