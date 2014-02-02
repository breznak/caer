/*
 * udpststat.c
 *
 *  Created on: Jan 14, 2014
 *      Author: llongi
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#define PRINTF_LOG 1

#include "events/common.h"

int main(int argc, char *argv[]) {
	// First of all, parse the IP:Port we need to listen on.
	// Those are for now also the only two parameters permitted.
	// If none passed, attempt to connect to default UDP IP:Port.
	const char *ipAddress = "127.0.0.1";
	uint16_t portNumber = 8888;

	if (argc != 1 && argc != 3) {
		fprintf(stderr, "Incorrect argument number. Either pass none for default IP:Port"
			"combination of 127.0.0.1:8888, or pass the IP followed by the Port.\n");
		return (EXIT_FAILURE);
	}

	// If explicitly passed, parse arguments.
	if (argc == 3) {
		ipAddress = argv[1];
		sscanf(argv[2], "%" SCNu16, &portNumber);
	}

	// Create listening socket for UDP data.
	int listenUDPSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (listenUDPSocket < 0) {
		fprintf(stderr, "Failed to create UDP socket.\n");
		return (EXIT_FAILURE);
	}

	struct sockaddr_in listenUDPAddress;
	memset(&listenUDPAddress, 0, sizeof(struct sockaddr_in));

	listenUDPAddress.sin_family = AF_INET;
	listenUDPAddress.sin_port = htons(portNumber);
	inet_aton(ipAddress, &listenUDPAddress.sin_addr); // htonl() is implicit here.

	if (bind(listenUDPSocket, (struct sockaddr *) &listenUDPAddress, sizeof(struct sockaddr_in)) < 0) {
		fprintf(stderr, "Failed to listen on UDP socket.\n");
		return (EXIT_FAILURE);
	}

	// 64K data buffer should be enough for the UDP packets.
	size_t dataBufferLength = 1024 * 64;
	uint8_t *dataBuffer = malloc(dataBufferLength);

	while (true) {
		ssize_t result = recv(listenUDPSocket, dataBuffer, dataBufferLength, 0);
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
	close(listenUDPSocket);

	free(dataBuffer);

	return (EXIT_SUCCESS);
}
