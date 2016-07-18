#include "unix_socket_server.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "output_common.h"
#include <sys/socket.h>
#include <sys/un.h>

static bool caerOutputUnixSocketServerInit(caerModuleData moduleData);
void caerOutputUnixSocketServerExit(caerModuleData moduleData);

static struct caer_module_functions caerOutputUnixSocketServerFunctions = { .moduleInit =
	&caerOutputUnixSocketServerInit, .moduleRun = &caerOutputCommonRun, .moduleConfig = NULL, .moduleExit =
	&caerOutputUnixSocketServerExit };

void caerOutputUnixSocketServer(uint16_t moduleID, size_t outputTypesNumber, ...) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "UnixSocketServerOutput", OUTPUT);
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

	// Open a Unix local socket on a known path, to be accessed by other processes (server-like mode).
	int serverSockFd = socket(AF_UNIX, SOCK_STREAM, 0);
	if (serverSockFd < 0) {
		caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString, "Could not create local Unix socket. Error: %d.",
		errno);
		return (false);
	}

	struct sockaddr_un unixSocketAddr;
	memset(&unixSocketAddr, 0, sizeof(struct sockaddr_un));

	unixSocketAddr.sun_family = AF_UNIX;

	char *socketPath = sshsNodeGetString(moduleData->moduleNode, "socketPath");
	strncpy(unixSocketAddr.sun_path, socketPath, sizeof(unixSocketAddr.sun_path) - 1);
	unixSocketAddr.sun_path[sizeof(unixSocketAddr.sun_path) - 1] = '\0'; // Ensure NUL terminated string.
	free(socketPath);

	// Bind socket to above address.
	if (bind(serverSockFd, (struct sockaddr *) &unixSocketAddr, sizeof(struct sockaddr_un)) < 0) {
		close(serverSockFd);

		caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString, "Could not bind local Unix socket. Error: %d.",
		errno);
		return (false);
	}

	// Listen to new connections on the socket.
	if (listen(serverSockFd, sshsNodeGetShort(moduleData->moduleNode, "backlogSize")) < 0) {
		close(serverSockFd);

		caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString,
			"Could not listen on local Unix socket. Error: %d.", errno);
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

	caerLog(CAER_LOG_INFO, moduleData->moduleSubSystemString, "Local Unix socket ready at '%s'.",
		unixSocketAddr.sun_path);

	return (true);
}

void caerOutputUnixSocketServerExit(caerModuleData moduleData) {
	// Get socket path before its closed by commonExit().
	int serverFd = caerOutputCommonGetServerFd(moduleData->moduleState);

	socklen_t unixSocketAddrLength = sizeof(struct sockaddr_un);
	struct sockaddr_un unixSocketAddr;
	memset(&unixSocketAddr, 0, unixSocketAddrLength);

	getsockname(serverFd, (struct sockaddr *) &unixSocketAddr, &unixSocketAddrLength);

	// Do common cleanup, close all file descriptors, server included.
	caerOutputCommonExit(moduleData);

	// Remove socket file after use. All else has already been cleaned up.
	unlink(unixSocketAddr.sun_path);
}
