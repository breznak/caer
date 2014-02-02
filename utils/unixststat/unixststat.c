#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>

#define PRINTF_LOG 1

#include "events/common.h"

int main(int argc, char *argv[]) {
	// First of all, parse the local path we need to listen on.
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
	int listenUnixSocket = socket(AF_UNIX, SOCK_DGRAM, 0);
	if (listenUnixSocket < 0) {
		fprintf(stderr, "Failed to create local Unix socket.\n");
		return (EXIT_FAILURE);
	}

	struct sockaddr_un unixSocketAddr;
	memset(&unixSocketAddr, 0, sizeof(struct sockaddr_un));

	unixSocketAddr.sun_family = AF_UNIX;
	strncpy(unixSocketAddr.sun_path, localSocket, sizeof(unixSocketAddr.sun_path) - 1);

	if (bind(listenUnixSocket, (struct sockaddr *) &unixSocketAddr, sizeof(struct sockaddr_un)) < 0) {
		fprintf(stderr, "Failed to listen on local Unix socket.\n");
		return (EXIT_FAILURE);
	}

	// 64K data buffer should be enough for the event packets..
	size_t dataBufferLength = 1024 * 64;
	uint8_t *dataBuffer = malloc(dataBufferLength);

	while (true) {
		ssize_t result = recv(listenUnixSocket, dataBuffer, dataBufferLength, 0);
		if (result <= 0) {
			fprintf(stderr, "Error in recv() call: %d\n", errno);
			break;
		}

		printf("Result of recv() call: %zd\n", result);

		// Decode successfully received data.
		caerEventPacketHeader header = (caerEventPacketHeader) dataBuffer;

		uint16_t eventType = caerEventPacketHeaderGetEventType(header);
		uint16_t eventSource = caerEventPacketHeaderGetEventSource(header);
		uint32_t eventSize = caerEventPacketHeaderGetEventSize(header);
		uint32_t eventTSOffset = caerEventPacketHeaderGetEventTSOffset(header);
		uint32_t eventCapacity = caerEventPacketHeaderGetEventCapacity(header);
		uint32_t eventNumber = caerEventPacketHeaderGetEventNumber(header);
		uint32_t eventValid = caerEventPacketHeaderGetEventValid(header);

		printf(
			"type = %" PRIu16 ", source = %" PRIu16 ", size = %" PRIu32 ", tsOffset = %" PRIu32 ", capacity = %" PRIu32 ", number = %" PRIu32 ", valid = %" PRIu32 ".\n",
			eventType, eventSource, eventSize, eventTSOffset, eventCapacity, eventNumber, eventValid);

		if (eventValid > 0) {
			void *firstEvent = caerGenericEventGetEvent(header, 0);
			void *lastEvent = caerGenericEventGetEvent(header, eventValid - 1);

			uint32_t firstTS = caerGenericEventGetTimestamp(firstEvent, header);
			uint32_t lastTS = caerGenericEventGetTimestamp(lastEvent, header);

			uint32_t tsDifference = lastTS - firstTS;

			printf("Time difference in packet: %" PRIu32 " (first = %" PRIu32 ", last = %" PRIu32 ").\n", tsDifference,
				firstTS, lastTS);
		}

		printf("\n\n");
	}

	// Close connection.
	close(listenUnixSocket);

	// Remove socket file.
	unlink(localSocket);

	free(dataBuffer);

	return (EXIT_SUCCESS);
}
