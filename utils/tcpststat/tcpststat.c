#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include "ext/libuv.h"
#include "ext/nets.h"
#include "modules/misc/inout_common.h"

#include <libcaer/events/common.h>
#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>

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
#if defined(_WIN32)
	if (signal(SIGTERM, &globalShutdownSignalHandler) == SIG_ERR) {
		caerLog(CAER_LOG_CRITICAL, "ShutdownAction", "Failed to set signal handler for SIGTERM. Error: %d.", errno);
		return (EXIT_FAILURE);
	}

	if (signal(SIGINT, &globalShutdownSignalHandler) == SIG_ERR) {
		caerLog(CAER_LOG_CRITICAL, "ShutdownAction", "Failed to set signal handler for SIGINT. Error: %d.", errno);
		return (EXIT_FAILURE);
	}
#else
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
#endif

	// First of all, parse the IP:Port we need to listen on.
	// Those are for now also the only two parameters permitted.
	// If none passed, attempt to connect to default TCP IP:Port.
	const char *ipAddress = "127.0.0.1";
	uint16_t portNumber = 7777;

	if (argc != 1 && argc != 3) {
		fprintf(stderr, "Incorrect argument number. Either pass none for default IP:Port"
			"combination of 127.0.0.1:7777, or pass the IP followed by the Port.\n");
		return (EXIT_FAILURE);
	}

	// If explicitly passed, parse arguments.
	if (argc == 3) {
		ipAddress = argv[1];
		sscanf(argv[2], "%" SCNu16, &portNumber);
	}

	struct sockaddr_in listenTCPAddress;

	int retVal = uv_ip4_addr(ipAddress, portNumber, &listenTCPAddress);
	UV_RET_CHECK_STDERR(retVal, "uv_ip4_addr", return (EXIT_FAILURE));

	// Create listening socket for TCP data.
	uv_os_sock_t listenTCPSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (listenTCPSocket < 0) {
		fprintf(stderr, "Failed to create TCP socket.\n");
		return (EXIT_FAILURE);
	}

	if (connect(listenTCPSocket, (struct sockaddr *) &listenTCPAddress, sizeof(struct sockaddr_in)) < 0) {
		close(listenTCPSocket);

		fprintf(stderr, "Failed to connect to remote TCP data server.\n");
		return (EXIT_FAILURE);
	}

	// 1M data buffer should be enough for the TCP event packets. Frames are very big!
	size_t dataBufferLength = 1024 * 1024;
	uint8_t *dataBuffer = malloc(dataBufferLength);
	if (dataBuffer == NULL) {
		close(listenTCPSocket);

		fprintf(stderr, "Failed to allocate memory for data buffer.\n");
		return (EXIT_FAILURE);
	}

	// Get network header (20 bytes).
	if (!recvUntilDone(listenTCPSocket, dataBuffer, AEDAT3_NETWORK_HEADER_LENGTH)) {
		free(dataBuffer);
		close(listenTCPSocket);

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
		if (!recvUntilDone(listenTCPSocket, dataBuffer, CAER_EVENT_PACKET_HEADER_SIZE)) {
			free(dataBuffer);
			close(listenTCPSocket);

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
		if (!recvUntilDone(listenTCPSocket, dataBuffer + CAER_EVENT_PACKET_HEADER_SIZE,
			(size_t) (eventCapacity * eventSize))) {
			free(dataBuffer);
			close(listenTCPSocket);

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

			// Additional example for Polarity and Frame events.
			if (eventType == POLARITY_EVENT) {
				caerPolarityEventPacket polarityPacket = (caerPolarityEventPacket) header;

				// Only get first event as example.
				caerPolarityEvent polarityEvent = caerPolarityEventPacketGetEvent(polarityPacket, 0);

				uint16_t xAddr = caerPolarityEventGetX(polarityEvent);
				uint16_t yAddr = caerPolarityEventGetY(polarityEvent);
				bool polarity = caerPolarityEventGetPolarity(polarityEvent);

				printf("First polarity event data - X Address: %" PRIu16 ", Y Address %" PRIu16 ", Polarity: %d.\n",
					xAddr, yAddr, polarity);
			}
			else if (eventType == FRAME_EVENT) {
				caerFrameEventPacket framePacket = (caerFrameEventPacket) header;

				// Only get first event as example.
				caerFrameEvent frameEvent = caerFrameEventPacketGetEvent(framePacket, 0);

				int32_t frameSizeX = caerFrameEventGetLengthX(frameEvent);
				int32_t frameSizeY = caerFrameEventGetLengthY(frameEvent);
				uint16_t *framePixels = caerFrameEventGetPixelArrayUnsafe(frameEvent);

				printf(
					"First frame event data - X Size: %" PRIi32 ", Y Size: %" PRIi32 ", Pixel 0 Value: %" PRIu16 ".\n",
					frameSizeX, frameSizeY, framePixels[0]);
			}
		}

		printf("\n\n");
	}

	// Close connection.
	close(listenTCPSocket);

	free(dataBuffer);

	return (EXIT_SUCCESS);
}
