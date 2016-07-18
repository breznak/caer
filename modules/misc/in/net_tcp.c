#include "net_tcp.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "input_common.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

static bool caerInputNetTCPInit(caerModuleData moduleData);

static struct caer_module_functions caerInputNetTCPFunctions = { .moduleInit = &caerInputNetTCPInit, .moduleRun =
	&caerInputCommonRun, .moduleConfig = NULL, .moduleExit = &caerInputCommonExit };

caerEventPacketContainer caerInputNetTCP(uint16_t moduleID) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "NetTCPInput", INPUT);
	if (moduleData == NULL) {
		return (NULL);
	}

	caerEventPacketContainer result = NULL;

	caerModuleSM(&caerInputNetTCPFunctions, moduleData, CAER_INPUT_COMMON_STATE_STRUCT_SIZE, 1, &result);

	return (result);
}

static bool caerInputNetTCPInit(caerModuleData moduleData) {
	// First, always create all needed setting nodes, set their default values
	// and add their listeners.
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "ipAddress", "127.0.0.1");
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "portNumber", 7777);

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

	if (!caerInputCommonInit(moduleData, sockFd, true, false)) {
		close(sockFd);
		return (false);
	}

	caerLog(CAER_LOG_INFO, moduleData->moduleSubSystemString, "TCP socket connected to %s:%" PRIu16 ".",
		inet_ntop(AF_INET, &tcpClient.sin_addr, (char[INET_ADDRSTRLEN] ) { 0x00 }, INET_ADDRSTRLEN),
		ntohs(tcpClient.sin_port));

	return (true);
}
