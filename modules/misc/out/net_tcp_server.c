#include "net_tcp_server.h"
#include "base/mainloop.h"
#include "base/module.h"
#include <poll.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include "ext/nets.h"

struct netTCP_state {
	int serverDescriptor;
	size_t clientDescriptorsLength;
	struct pollfd *clientDescriptors;
	bool validOnly;
	struct iovec *sgioMemory;
};

typedef struct netTCP_state *netTCPState;

static bool caerOutputNetTCPServerInit(caerModuleData moduleData);
static void caerOutputNetTCPServerRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerOutputNetTCPServerConfig(caerModuleData moduleData);
static void caerOutputNetTCPServerExit(caerModuleData moduleData);

static struct caer_module_functions caerOutputNetTCPServerFunctions = { .moduleInit = &caerOutputNetTCPServerInit,
	.moduleRun = &caerOutputNetTCPServerRun, .moduleConfig = &caerOutputNetTCPServerConfig, .moduleExit =
		&caerOutputNetTCPServerExit };

void caerOutputNetTCPServer(uint16_t moduleID, size_t outputTypesNumber, ...) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "NetTCPServerOutput");

	va_list args;
	va_start(args, outputTypesNumber);
	caerModuleSMv(&caerOutputNetTCPServerFunctions, moduleData, sizeof(struct netTCP_state), outputTypesNumber, args);
	va_end(args);
}

static void caerOutputNetTCPServerConnectionHandler(netTCPState state);
static void caerOutputNetTCPServerConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);

static bool caerOutputNetTCPServerInit(caerModuleData moduleData) {
	netTCPState state = moduleData->moduleState;

	// First, always create all needed setting nodes, set their default values
	// and add their listeners.
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "ipAddress", "127.0.0.1");
	sshsNodePutShortIfAbsent(moduleData->moduleNode, "portNumber", 7777);
	sshsNodePutShortIfAbsent(moduleData->moduleNode, "backlogSize", 5);
	sshsNodePutShortIfAbsent(moduleData->moduleNode, "concurrentConnections", 5);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "validEventsOnly", false);

	// Install default listener to signal configuration updates asynchronously.
	sshsNodeAddAttrListener(moduleData->moduleNode, moduleData, &caerOutputNetTCPServerConfigListener);

	// Open a TCP server socket for others to connect to.
	state->serverDescriptor = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (state->serverDescriptor < 0) {
		caerLog(LOG_CRITICAL, "Could not create TCP server socket. Error: %s (%d).", caerLogStrerror(errno), errno);
		return (false);
	}

	// Make socket address reusable right away.
	socketReuseAddr(state->serverDescriptor, true);

	// Set server socket, on which accept() is called, to non-blocking mode.
	if (!socketBlockingMode(state->serverDescriptor, false)) {
		caerLog(LOG_CRITICAL, "Could not set TCP server socket to non-blocking mode.");
		close(state->serverDescriptor);
		return (false);
	}

	struct sockaddr_in tcpServer;
	memset(&tcpServer, 0, sizeof(struct sockaddr_in));

	tcpServer.sin_family = AF_INET;
	tcpServer.sin_port = htons(sshsNodeGetShort(moduleData->moduleNode, "portNumber"));
	char *ipAddress = sshsNodeGetString(moduleData->moduleNode, "ipAddress");
	inet_aton(ipAddress, &tcpServer.sin_addr); // htonl() is implicit here.
	free(ipAddress);

	// Bind socket to above address.
	if (bind(state->serverDescriptor, (struct sockaddr *) &tcpServer, sizeof(struct sockaddr_in)) < 0) {
		caerLog(LOG_CRITICAL, "Could not bind TCP server socket. Error: %s (%d).", caerLogStrerror(errno), errno);
		close(state->serverDescriptor);
		return (false);
	}

	// Listen to new connections on the socket.
	if (listen(state->serverDescriptor, sshsNodeGetShort(moduleData->moduleNode, "backlogSize")) < 0) {
		caerLog(LOG_CRITICAL, "Could not listen on TCP server socket. Error: %s (%d).", caerLogStrerror(errno),
		errno);
		close(state->serverDescriptor);
		return (false);
	}

	// Prepare memory to hold connected clients fds.
	state->clientDescriptorsLength = sshsNodeGetShort(moduleData->moduleNode, "concurrentConnections");
	state->clientDescriptors = malloc(state->clientDescriptorsLength * sizeof(*(state->clientDescriptors)));
	if (state->clientDescriptors == NULL) {
		caerLog(LOG_CRITICAL, "Could not allocate memory for TCP client descriptors. Error: %s (%d).",
			caerLogStrerror(errno), errno);
		close(state->serverDescriptor);
		return (false);
	}

	// Initialize connected clients array to empty.
	// We only care about checking inbound data to detect close() from the client.
	for (size_t c = 0; c < state->clientDescriptorsLength; c++) {
		state->clientDescriptors[c].fd = -1;
		state->clientDescriptors[c].events = POLLIN;
	}

	// Set valid events flag, and allocate memory for scatter/gather IO for it.
	state->validOnly = sshsNodeGetBool(moduleData->moduleNode, "validEventsOnly");

	if (state->validOnly) {
		state->sgioMemory = calloc(IOVEC_SIZE, sizeof(struct iovec));
		if (state->sgioMemory == NULL) {
			caerLog(LOG_ALERT, "Impossible to allocate memory for scatter/gather IO, using memory copy method.");
		}
		else {
			caerLog(LOG_INFO, "Using scatter/gather IO for outputting valid events only.");
		}
	}
	else {
		state->sgioMemory = NULL;
	}

	caerLog(LOG_INFO, "TCP server socket connected to %s:%" PRIu16 ".", inet_ntoa(tcpServer.sin_addr),
		ntohs(tcpServer.sin_port));

	return (true);
}

static void caerOutputNetTCPServerConnectionHandler(netTCPState state) {
	// First let's check if anybody closed their connection, to free up space
	// for eventual new connections.
	int pollResult = poll(state->clientDescriptors, state->clientDescriptorsLength, 0);
	if (pollResult < 0) {
		// Poll failure. Log and then continue.
		caerLog(LOG_ERROR, "TCP server poll() failed. Error: %s (%d).", caerLogStrerror(errno), errno);
	}

	// Handle clients that have inbound data (close() calls).
	if (pollResult > 0) {
		for (size_t c = 0; c < state->clientDescriptorsLength; c++) {
			if (state->clientDescriptors[c].fd >= 0 && (state->clientDescriptors[c].revents & POLLIN) != 0) {
				// Check if this one wants to close(), which should be the only
				// inbound action to ever happen.
				uint8_t buffer[1];
				ssize_t recvResult = recv(state->clientDescriptors[c].fd, buffer, 1, 0);

				if (recvResult <= 0) {
					// Recv failure or closed connection.
					close(state->clientDescriptors[c].fd);
					caerLog(LOG_DEBUG, "Disconnected TCP client on recv (fd %d).", state->clientDescriptors[c].fd);
					state->clientDescriptors[c].fd = -1;
				}
				else {
					// Incoming data: what?
					caerLog(LOG_ERROR, "Incoming data from client on TCP server. Clients should never send data!");
				}
			}
		}
	}

	// So now that the connections array is cleaned up, let's see if any new
	// connections are waiting on the listening socket to be accepted.
	int acceptResult = accept(state->serverDescriptor, NULL, NULL);
	if (acceptResult < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
		// Accept failure (but not would-block error). Log and then continue.
		caerLog(LOG_ERROR, "TCP server accept() failed. Error: %s (%d).", caerLogStrerror(errno), errno);
	}

	// New connection present!
	if (acceptResult >= 0) {
		// Put it in the list of watched fds if possible, or close.
		bool putInFDList = false;

		for (size_t c = 0; c < state->clientDescriptorsLength; c++) {
			if (state->clientDescriptors[c].fd == -1) {
				// Empty place in watch list, add this one.
				state->clientDescriptors[c].fd = acceptResult;
				putInFDList = true;
				break;
			}
		}

		// No space for new connection, just close it (client will exit).
		if (!putInFDList) {
			close(acceptResult);
			caerLog(LOG_DEBUG, "Rejected TCP client (fd %d), queue full.", acceptResult);
		}
		else {
			caerLog(LOG_DEBUG, "Accepted new TCP connection from client (fd %d).", acceptResult);
		}
	}
}

static void caerOutputNetTCPServerRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	netTCPState state = moduleData->moduleState;

	// Update the current connections on which to output data.
	caerOutputNetTCPServerConnectionHandler(state);

	// For each output argument, write it to the TCP socket.
	// Each type has a header first thing, that gives us the length, so we can
	// cast it to that and use this information to correctly interpret it.
	for (size_t i = 0; i < argsNumber; i++) {
		caerEventPacketHeader packetHeader = va_arg(args, caerEventPacketHeader);

		// Only work if there is any content.
		if (packetHeader != NULL) {
			if ((state->validOnly && caerEventPacketHeaderGetEventValid(packetHeader) > 0)
				|| (!state->validOnly && caerEventPacketHeaderGetEventNumber(packetHeader) > 0)) {
				// Send to each connected client.
				for (size_t c = 0; c < state->clientDescriptorsLength; c++) {
					if (state->clientDescriptors[c].fd >= 0) {
						caerOutputCommonSend(packetHeader, state->clientDescriptors[c].fd, state->validOnly,
							state->sgioMemory);
					}
				}
			}
		}
	}
}

static void caerOutputNetTCPServerConfig(caerModuleData moduleData) {
	netTCPState state = moduleData->moduleState;

	// Get the current value to examine by atomic exchange, since we don't
	// want there to be any possible store between a load/store pair.
	uintptr_t configUpdate = atomic_ops_uint_swap(&moduleData->configUpdate, 0, ATOMIC_OPS_FENCE_NONE);

	if (configUpdate & (0x01 << 0)) {
		// validOnly flag changed.
		bool validOnlyFlag = sshsNodeGetBool(moduleData->moduleNode, "validEventsOnly");

		// Only react if the actual state differs from the wanted one.
		if (state->validOnly != validOnlyFlag) {
			// If we want it, turn it on.
			if (validOnlyFlag) {
				state->validOnly = true;

				state->sgioMemory = calloc(IOVEC_SIZE, sizeof(struct iovec));
				if (state->sgioMemory == NULL) {
					caerLog(LOG_ALERT,
						"Impossible to allocate memory for scatter/gather IO, using memory copy method.");
				}
				else {
					caerLog(LOG_INFO, "Using scatter/gather IO for outputting valid events only.");
				}
			}
			else {
				// Else disable it.
				state->validOnly = false;

				free(state->sgioMemory);
				state->sgioMemory = NULL;
			}
		}
	}

	if (configUpdate & (0x01 << 1)) {
		// TCP server address related changes.
		// Changes to the backlogSize parameter are only ever considered when
		// fully opening a new socket, which happens only on changes to either
		// the server IP address or its port.

		// Open a TCP socket on which to listen for connections.
		int newServerDescriptor = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (newServerDescriptor < 0) {
			caerLog(LOG_CRITICAL, "Could not create TCP socket. Error: %s (%d).", caerLogStrerror(errno), errno);
			goto configUpdate_2;
		}

		// Make socket address reusable right away.
		socketReuseAddr(newServerDescriptor, true);

		// Set server socket, on which accept() is called, to non-blocking mode.
		if (!socketBlockingMode(newServerDescriptor, false)) {
			caerLog(LOG_CRITICAL, "Could not set TCP server socket to non-blocking mode.");
			close(newServerDescriptor);
			goto configUpdate_2;
		}

		struct sockaddr_in tcpServer;
		memset(&tcpServer, 0, sizeof(struct sockaddr_in));

		tcpServer.sin_family = AF_INET;
		tcpServer.sin_port = htons(sshsNodeGetShort(moduleData->moduleNode, "portNumber"));
		char *ipAddress = sshsNodeGetString(moduleData->moduleNode, "ipAddress");
		inet_aton(ipAddress, &tcpServer.sin_addr); // htonl() is implicit here.
		free(ipAddress);

		// Bind socket to above address.
		if (bind(newServerDescriptor, (struct sockaddr *) &tcpServer, sizeof(struct sockaddr_in)) < 0) {
			caerLog(LOG_CRITICAL, "Could not bind TCP server socket. Error: %s (%d).", caerLogStrerror(errno), errno);
			close(newServerDescriptor);
			goto configUpdate_2;
		}

		// Listen to new connections on the socket.
		if (listen(newServerDescriptor, sshsNodeGetShort(moduleData->moduleNode, "backlogSize")) < 0) {
			caerLog(LOG_CRITICAL, "Could not listen on TCP server socket. Error: %s (%d).", caerLogStrerror(errno),
			errno);
			close(newServerDescriptor);
			goto configUpdate_2;
		}

		// New fd ready and connected, close old and set new.
		close(state->serverDescriptor);
		state->serverDescriptor = newServerDescriptor;
	}

	configUpdate_2: if (configUpdate & (0x01 << 2)) {
		// Number of allowed connections just changed.
		size_t newConnectionsLimit = sshsNodeGetShort(moduleData->moduleNode, "concurrentConnections");

		// Prepare memory to hold connected clients fds.
		struct pollfd *newConnectionsArray = malloc(newConnectionsLimit * sizeof(*newConnectionsArray));
		if (newConnectionsArray == NULL) {
			caerLog(LOG_CRITICAL, "Could not allocate memory for TCP client descriptors. Error: %s (%d).",
				caerLogStrerror(errno), errno);
			return;
		}

		// Initialize connected clients array to empty.
		// We only care about checking inbound data to detect close() from the client.
		for (size_t c = 0; c < newConnectionsLimit; c++) {
			newConnectionsArray[c].fd = -1;
			newConnectionsArray[c].events = POLLIN;
		}

		// Copy over as many already established connections as possible, so as
		// to not interrupt them. Once the limit is reached, close() any others.
		for (size_t c = 0, i = 0; c < state->clientDescriptorsLength; c++) {
			if (state->clientDescriptors[c].fd >= 0) {
				if (i < newConnectionsLimit) {
					newConnectionsArray[i++].fd = state->clientDescriptors[c].fd;
				}
				else {
					// Close any descriptors that have no free slot anymore.
					close(state->clientDescriptors[c].fd);
				}

				state->clientDescriptors[c].fd = -1;
			}
		}

		// And now exchange the two connection arrays.
		free(state->clientDescriptors);
		state->clientDescriptors = newConnectionsArray;
		state->clientDescriptorsLength = newConnectionsLimit;
	}
}

static void caerOutputNetTCPServerExit(caerModuleData moduleData) {
	netTCPState state = moduleData->moduleState;

	// Close all open connections to clients.
	for (size_t c = 0; c < state->clientDescriptorsLength; c++) {
		if (state->clientDescriptors[c].fd >= 0) {
			close(state->clientDescriptors[c].fd);
			state->clientDescriptors[c].fd = -1;
		}
	}

	// Free memory associated with the client descriptors.
	free(state->clientDescriptors);
	state->clientDescriptors = NULL;
	state->clientDescriptorsLength = 0;

	// Close open TCP server socket.
	close(state->serverDescriptor);

	// Make sure to free scatter/gather IO memory.
	free(state->sgioMemory);
	state->sgioMemory = NULL;
}

static void caerOutputNetTCPServerConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);
	UNUSED_ARGUMENT(changeValue);

	caerModuleData data = userData;

	// Distinguish changes to the validOnly flag or to the TCP server, by setting
	// configUpdate appropriately like a bit-field.
	if (event == ATTRIBUTE_MODIFIED) {
		if (changeType == BOOL && strcmp(changeKey, "validEventsOnly") == 0) {
			atomic_ops_uint_or(&data->configUpdate, (0x01 << 0), ATOMIC_OPS_FENCE_NONE);
		}

		if ((changeType == STRING && strcmp(changeKey, "ipAddress") == 0)
			|| (changeType == SHORT && strcmp(changeKey, "portNumber") == 0)) {
			atomic_ops_uint_or(&data->configUpdate, (0x01 << 1), ATOMIC_OPS_FENCE_NONE);
		}

		if (changeType == SHORT && strcmp(changeKey, "concurrentConnections") == 0) {
			atomic_ops_uint_or(&data->configUpdate, (0x01 << 2), ATOMIC_OPS_FENCE_NONE);
		}
	}
}
