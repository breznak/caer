#include "unixs.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "output_common.h"
#include <sys/socket.h>
#include <sys/un.h>

static bool caerOutputUnixSInit(caerModuleData moduleData);

static struct caer_module_functions caerOutputUnixSFunctions = { .moduleInit = &caerOutputUnixSInit, .moduleRun =
	&caerOutputCommonRun, .moduleConfig = NULL, .moduleExit = &caerOutputCommonExit };

void caerOutputUnixS(uint16_t moduleID, size_t outputTypesNumber, ...) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "UnixSocketOutput");

	va_list args;
	va_start(args, outputTypesNumber);
	caerModuleSMv(&caerOutputUnixSFunctions, moduleData, CAER_OUTPUT_COMMON_STATE_STRUCT_SIZE, outputTypesNumber, args);
	va_end(args);
}

static bool caerOutputUnixSInit(caerModuleData moduleData) {
	// First, always create all needed setting nodes, set their default values
	// and add their listeners.
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "socketPath", "/tmp/caer.sock");

	// Open a Unix local socket on a known path, to be accessed by other processes.
	int sockFd = socket(AF_UNIX, SOCK_DGRAM, 0);
	if (sockFd < 0) {
		caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString, "Could not create local Unix socket. Error: %d.",
		errno);
		return (false);
	}

	struct sockaddr_un unixSocketAddr;
	memset(&unixSocketAddr, 0, sizeof(struct sockaddr_un));

	unixSocketAddr.sun_family = AF_UNIX;

	char *socketPath = sshsNodeGetString(moduleData->moduleNode, "socketPath");
	strncpy(unixSocketAddr.sun_path, socketPath, sizeof(unixSocketAddr.sun_path) - 1);
	free(socketPath);

	// Connect socket to above address.
	if (connect(sockFd, (struct sockaddr *) &unixSocketAddr, sizeof(struct sockaddr_un)) < 0) {
		close(sockFd);

		caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString,
			"Could not connect to local Unix socket. Error: %d.", errno);
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

	caerLog(CAER_LOG_INFO, moduleData->moduleSubSystemString, "Local Unix socket ready at '%s'.",
		unixSocketAddr.sun_path);

	return (true);
}
