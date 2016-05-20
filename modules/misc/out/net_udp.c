#include "net_udp.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "output_common.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

static bool caerOutputNetUDPInit(caerModuleData moduleData);

static struct caer_module_functions caerOutputNetUDPFunctions = { .moduleInit = &caerOutputNetUDPInit, .moduleRun =
	&caerOutputCommonRun, .moduleConfig = NULL, .moduleExit = &caerOutputCommonExit };

void caerOutputNetUDP(uint16_t moduleID, size_t outputTypesNumber, ...) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "NetUDPOutput");
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
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "portNumber", 8888);

	// Open a UDP socket to the remote client, to which we'll send data packets.
	int sockFd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sockFd < 0) {
		caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString, "Could not create UDP socket. Error: %d.", errno);
		return (false);
	}

	struct sockaddr_in udpClient;
	memset(&udpClient, 0, sizeof(struct sockaddr_in));

	udpClient.sin_family = AF_INET;
	udpClient.sin_port = htons(U16T(sshsNodeGetInt(moduleData->moduleNode, "portNumber")));

	char *ipAddress = sshsNodeGetString(moduleData->moduleNode, "ipAddress");
	inet_aton(ipAddress, &udpClient.sin_addr); // htonl() is implicit here.
	free(ipAddress);

	if (connect(sockFd, (struct sockaddr *) &udpClient, sizeof(struct sockaddr_in)) != 0) {
		close(sockFd);

		caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString,
			"Could not connect to remote UDP client %s:%" PRIu16 ". Error: %d.", inet_ntoa(udpClient.sin_addr),
			ntohs(udpClient.sin_port), errno);
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

	if (!caerOutputCommonInit(moduleData, fileDescriptors, true, true)) {
		close(sockFd);
		free(fileDescriptors);

		return (false);
	}

	caerLog(CAER_LOG_INFO, moduleData->moduleSubSystemString, "UDP socket connected to %s:%" PRIu16 ".",
		inet_ntoa(udpClient.sin_addr), ntohs(udpClient.sin_port));

	return (true);
}
