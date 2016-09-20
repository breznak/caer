#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include "ext/nets.h"
#include "modules/misc/inout_common.h"

#include <libcaer/events/common.h>

#include <signal.h>
#include <stdatomic.h>

static atomic_bool globalShutdown = ATOMIC_VAR_INIT(false);

static void globalShutdownSignalHandler(int signal) {
	// Simply set the running flag to false on SIGTERM and SIGINT (CTRL+C) for global shutdown.
	if (signal == SIGTERM || signal == SIGINT) {
		atomic_store(&globalShutdown, true);
	}
}

int main(int argc, char *argv[]) {
	// Install signal handler for global shutdown.
	struct sigaction shutdownAction;

	shutdownAction.sa_handler = &globalShutdownSignalHandler;
	shutdownAction.sa_flags = 0;
	sigemptyset(&shutdownAction.sa_mask);
	sigaddset(&shutdownAction.sa_mask, SIGTERM);
	sigaddset(&shutdownAction.sa_mask, SIGINT);

	if (sigaction(SIGTERM, &shutdownAction, NULL) == -1) {
		caerLog(CAER_LOG_CRITICAL, "ShutdownAction", "Failed to set signal handler for SIGTERM. Error: %d.", errno);
		return (EXIT_FAILURE);
	}

	if (sigaction(SIGINT, &shutdownAction, NULL) == -1) {
		caerLog(CAER_LOG_CRITICAL, "ShutdownAction", "Failed to set signal handler for SIGINT. Error: %d.", errno);
		return (EXIT_FAILURE);
	}

	// First of all, parse the local path we need to connect to.
	// That is the only parameter permitted at the moment.
	// If none passed, attempt to connect to default local path.
	const char *localSocket = "/tmp/caer.sock";

	if (argc != 1 && argc != 2) {
		fprintf(stderr, "Incorrect argument number. Either pass none for default local socket"
			"path of /tmp/caer.sock, or pass the absolute path to the socket.\n");
		return (EXIT_FAILURE);
	}

	// If explicitly passed, parse arguments.
	if (argc == 2) {
		localSocket = argv[1];
	}

	// Create listening local Unix socket.
	int listenUnixSocket = socket(AF_UNIX, SOCK_STREAM, 0);
	if (listenUnixSocket < 0) {
		fprintf(stderr, "Failed to create local Unix socket.\n");
		return (EXIT_FAILURE);
	}

	struct sockaddr_un unixSocketAddr;
	memset(&unixSocketAddr, 0, sizeof(struct sockaddr_un));

	unixSocketAddr.sun_family = AF_UNIX;
	strncpy(unixSocketAddr.sun_path, localSocket, sizeof(unixSocketAddr.sun_path) - 1);
	unixSocketAddr.sun_path[sizeof(unixSocketAddr.sun_path) - 1] = '\0'; // Ensure NUL terminated string.

	if (connect(listenUnixSocket, (struct sockaddr *) &unixSocketAddr, sizeof(struct sockaddr_un)) < 0) {
		close(listenUnixSocket);

		fprintf(stderr, "Failed to connect to local Unix socket.\n");
		return (EXIT_FAILURE);
	}

	// 1M data buffer should be enough for the Unix Socket event packets. Frames are very big!
	size_t dataBufferLength = 1024 * 1024;
	uint8_t *dataBuffer = malloc(dataBufferLength);
	if (dataBuffer == NULL) {
		close(listenUnixSocket);

		fprintf(stderr, "Failed to allocate memory for data buffer.\n");
		return (EXIT_FAILURE);
	}

	// Get network header (20 bytes).
	if (!recvUntilDone(listenUnixSocket, dataBuffer, AEDAT3_NETWORK_HEADER_LENGTH)) {
		free(dataBuffer);
		close(listenUnixSocket);

		fprintf(stderr, "Error in network header recv() call: %d\n", errno);
		return (EXIT_FAILURE);
	}

	// Decode network header.
	struct aedat3_network_header networkHeader = caerParseNetworkHeader(dataBuffer);

	printf("Magic number: %" PRIi64 "\n", networkHeader.magicNumber);
	printf("Sequence number: %" PRIi64 "\n", networkHeader.sequenceNumber);
	printf("Version number: %" PRIi8 "\n", networkHeader.versionNumber);
	printf("Format number: %" PRIi8 "\n", networkHeader.formatNumber);
	printf("Source ID: %" PRIi16 "\n", networkHeader.sourceID);

	while (!atomic_load_explicit(&globalShutdown, memory_order_relaxed)) {
		// Get packet header, to calculate packet size.
		if (!recvUntilDone(listenUnixSocket, dataBuffer, CAER_EVENT_PACKET_HEADER_SIZE)) {
			free(dataBuffer);
			close(listenUnixSocket);

			fprintf(stderr, "Error in header recv() call: %d\n", errno);
			return (EXIT_FAILURE);
		}

		// Decode successfully received data.
		caerEventPacketHeader header = (caerEventPacketHeader) dataBuffer;

		int16_t eventType = caerEventPacketHeaderGetEventType(header);
		int16_t eventSource = caerEventPacketHeaderGetEventSource(header);
		int32_t eventSize = caerEventPacketHeaderGetEventSize(header);
		int32_t eventTSOffset = caerEventPacketHeaderGetEventTSOffset(header);
		int32_t eventTSOverflow = caerEventPacketHeaderGetEventTSOverflow(header);
		int32_t eventCapacity = caerEventPacketHeaderGetEventCapacity(header);
		int32_t eventNumber = caerEventPacketHeaderGetEventNumber(header);
		int32_t eventValid = caerEventPacketHeaderGetEventValid(header);

		printf(
			"type = %" PRIi16 ", source = %" PRIi16 ", size = %" PRIi32 ", tsOffset = %" PRIi32 ", tsOverflow = %" PRIi32 ", capacity = %" PRIi32 ", number = %" PRIi32 ", valid = %" PRIi32 ".\n",
			eventType, eventSource, eventSize, eventTSOffset, eventTSOverflow, eventCapacity, eventNumber, eventValid);

		// Get rest of event packet, the part with the events themselves.
		if (!recvUntilDone(listenUnixSocket, dataBuffer + CAER_EVENT_PACKET_HEADER_SIZE,
			(size_t) (eventCapacity * eventSize))) {
			free(dataBuffer);
			close(listenUnixSocket);

			fprintf(stderr, "Error in data recv() call: %d\n", errno);
			return (EXIT_FAILURE);
		}

		if (eventValid > 0) {
			void *firstEvent = caerGenericEventGetEvent(header, 0);
			void *lastEvent = caerGenericEventGetEvent(header, eventValid - 1);

			int32_t firstTS = caerGenericEventGetTimestamp(firstEvent, header);
			int32_t lastTS = caerGenericEventGetTimestamp(lastEvent, header);

			int32_t tsDifference = lastTS - firstTS;

			printf("Time difference in packet: %" PRIi32 " (first = %" PRIi32 ", last = %" PRIi32 ").\n", tsDifference,
				firstTS, lastTS);
		}

		printf("\n\n");
	}

	// Close connection.
	close(listenUnixSocket);

	free(dataBuffer);

	return (EXIT_SUCCESS);
}
