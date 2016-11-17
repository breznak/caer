#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include "ext/libuv.h"
#include "ext/uthash/utarray.h"
#include "ext/uthash/utlist.h"
#include "modules/misc/inout_common.h"

#include <libcaer/events/common.h>

#include <signal.h>
#include <stdatomic.h>

struct udp_packet {
	struct udp_packet *next;
	caerEventPacketHeader content;
	int64_t startSequenceNumber;
	int64_t endSequenceNumber;
	size_t numUDPPackets;
	bool udpPacketsReceived[];
};

typedef struct udp_packet *udpPacket;

struct udp_message {
	struct udp_message *next;
	int64_t sequenceNumber;
	size_t messageLength;
	uint8_t message[];
};

typedef struct udp_message *udpMessage;

static void analyzeUDPMessage(int64_t highestParsedSequenceNumber, udpPacket *incompleteUDPPackets,
	udpMessage *unassignedUDPMessages, int64_t sequenceNumber, uint8_t *data, size_t dataLength);
static void printPacketInfo(caerEventPacketHeader header);

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

	struct sockaddr_in listenUDPAddress;

	int retVal = uv_ip4_addr(ipAddress, portNumber, &listenUDPAddress);
	UV_RET_CHECK_STDERR(retVal, "uv_ip4_addr", return (EXIT_FAILURE));

	// Create listening socket for UDP data.
	uv_os_sock_t listenUDPSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (listenUDPSocket < 0) {
		fprintf(stderr, "Failed to create UDP socket.\n");
		return (EXIT_FAILURE);
	}

	if (bind(listenUDPSocket, (struct sockaddr *) &listenUDPAddress, sizeof(struct sockaddr_in)) < 0) {
		close(listenUDPSocket);

		fprintf(stderr, "Failed to listen on UDP socket.\n");
		return (EXIT_FAILURE);
	}

	// Use cAER maximum UDP message size.
	size_t dataBufferLength = MAX_OUTPUT_UDP_SIZE;
	uint8_t *dataBuffer = malloc(dataBufferLength);
	if (dataBuffer == NULL) {
		close(listenUDPSocket);

		fprintf(stderr, "Failed to allocate memory for data buffer.\n");
		return (EXIT_FAILURE);
	}

	udpPacket incompleteUDPPackets = NULL;

	udpMessage unassignedUDPMessages = NULL;

	int64_t highestParsedSequenceNumber = -1;

	while (!atomic_load_explicit(&globalShutdown, memory_order_relaxed)) {
		ssize_t result = recv(listenUDPSocket, dataBuffer, dataBufferLength, 0);
		if (result <= 0) {
			free(dataBuffer);
			close(listenUDPSocket);

			fprintf(stderr, "Error in recv() call: %d\n", errno);
			return (EXIT_FAILURE);
		}

		printf("Result of recv() call: %zd\n", result);

		// UDP is more complex than TCP and Pipes. It is not a stream, nor in order, nor reliable.
		// So we do split AEDAT packets up into small messages to send over UDP, because some packets
		// are simply too big (Frames) to fit one message, and even then, 64K messages are almost
		// guaranteed to be lost in transit.
		// Each packet's first message has the highest-bit of the sequence number set to 1, all
		// subsequent messages related to that packet have it set to 0. The sequence number is
		// continuous, increased by one on each successive message.
		// So, an example reader could work this way:
		// - wait for first UDP message with a sequence number with highest bit set to 1
		// - possible new packet: store current content and sequence number, wait on new messages
		//   with the appropriate sequence numbers (highest bit 0) to complete it
		// - if you get another "Start-of-Packet" message, also do the above
		// - continue accumulating and completing packets, with completely rebuilt packets being
		//   sent for processing in-order, with some timeout (sequence-number or time based) to
		//   invalidate incomplete packets and avoid waiting forever

		// Decode network header.
		struct aedat3_network_header networkHeader = caerParseNetworkHeader(dataBuffer);

		printf("Magic number: %" PRIi64 "\n", networkHeader.magicNumber);
		printf("Sequence number: %" PRIi64 "\n", networkHeader.sequenceNumber);
		printf("Version number: %" PRIi8 "\n", networkHeader.versionNumber);
		printf("Format number: %" PRIi8 "\n", networkHeader.formatNumber);
		printf("Source ID: %" PRIi16 "\n", networkHeader.sourceID);

		analyzeUDPMessage(highestParsedSequenceNumber, &incompleteUDPPackets, &unassignedUDPMessages,
			networkHeader.sequenceNumber, dataBuffer + AEDAT3_NETWORK_HEADER_LENGTH,
			(size_t) result - AEDAT3_NETWORK_HEADER_LENGTH);
	}

	// Close connection.
	close(listenUDPSocket);

	udpMessage msg = NULL;
	LL_FOREACH(unassignedUDPMessages, msg)
	{
		free(msg);
	}

	udpPacket pkt = NULL;
	LL_FOREACH(incompleteUDPPackets, pkt)
	{
		free(pkt);
	}

	free(dataBuffer);

	return (EXIT_SUCCESS);
}

static void analyzeUDPMessage(int64_t highestParsedSequenceNumber, udpPacket *incompleteUDPPackets,
	udpMessage *unassignedUDPMessages, int64_t sequenceNumber, uint8_t *data, size_t dataLength) {
	// Is this a start message or an intermediate/end one?
	bool startMessage = (U64T(sequenceNumber) & 0x8000000000000000LL);
	sequenceNumber = sequenceNumber & 0x7FFFFFFFFFFFFFFFLL;

	// If the sequence number is smaller or equal of the highest already parsed
	// UDP packet, we discard it right away. The stream reconstruction has already
	// passed this point, so we can't insert this old data anywhere anyway.
	if (sequenceNumber <= highestParsedSequenceNumber) {
		return;
	}

	// First check if this is a start message. If yes, we allocate a new packet for it and
	// put it at the right place. Also detect duplicate start messages here.
	if (startMessage) {
		// First we check if we already have an UDP packet with this particular starting
		// sequence number. Duplicate messages are possible with UDP!
		udpPacket packet = NULL;
		LL_FOREACH(*incompleteUDPPackets, packet)
		{
			if (packet->startSequenceNumber == sequenceNumber) {
				// Duplicate, ignore!
				return;
			}
		}

		// This is the first time we see this start packet, and all seems valid, so we
		// parse the header and populate the other fields, then insert the packet at
		// its right place in the list based upon the sequence number.
		caerEventPacketHeader eventHeader = (caerEventPacketHeader) data;

		int32_t eventSize = caerEventPacketHeaderGetEventSize(eventHeader);
		int32_t eventNumber = caerEventPacketHeaderGetEventNumber(eventHeader);

		// Determine event packe size in bytes, so we can allocate space for it and
		// calculate how many UDP packets are used to send this.
		size_t eventPacketSize = CAER_EVENT_PACKET_HEADER_SIZE + (size_t) (eventSize * eventNumber);

		// Allocate memory for new event packet.
		caerEventPacketHeader newEventPacket = calloc(1, eventPacketSize);
		if (newEventPacket == NULL) {
			return;
		}

		// Calculate over how many UDP packets this event packet is split up.
		size_t numUDPPackets = eventPacketSize / MAX_OUTPUT_UDP_SIZE;
		if ((eventPacketSize % MAX_OUTPUT_UDP_SIZE) != 0) {
			numUDPPackets++;
		}

		// Allocate memory for tracking the reconstruction of the full event packet from
		// lots of smaller UDP packets.
		udpPacket newPacket = calloc(1, sizeof(*newPacket) + (numUDPPackets * sizeof(bool)));
		if (newPacket == NULL) {
			free(newEventPacket);
			return;
		}

		// Copy content at start, set all required fields.
		newPacket->content = newEventPacket;
		memcpy(newPacket->content, data, dataLength);
		newPacket->startSequenceNumber = sequenceNumber;
		newPacket->endSequenceNumber = I64T((size_t ) sequenceNumber + numUDPPackets - 1);
		newPacket->numUDPPackets = numUDPPackets;
		newPacket->udpPacketsReceived[0] = true;

		// Find out where to link new UDP packet into ordered list.
		// We already checked and excluded equality above, so it can only be smaller
		// or bigger, never equal.
		udpPacket prevPacket = NULL;
		LL_FOREACH(*incompleteUDPPackets, packet)
		{
			if (packet->startSequenceNumber > newPacket->startSequenceNumber) {
				break;
			}

			prevPacket = packet;
		}

		if (prevPacket == NULL) {
			// Insert at HEAD of list.
			newPacket->next = *incompleteUDPPackets;
			*incompleteUDPPackets = newPacket;
		}
		else {
			// Insert somewhere inside list.
			newPacket->next = prevPacket->next;
			prevPacket->next = newPacket;
		}

		// TODO: scan unassigned messages list and try to pair up.
	}
	else {
		// Not a start message, but an intermediate/end one!
		// Let's find out if we can assign it to any existing UDP packet.
		// If not, we put it into the unassigned messages list.
		udpPacket packet = NULL;
		LL_FOREACH(*incompleteUDPPackets, packet)
		{
			if (sequenceNumber > packet->startSequenceNumber && sequenceNumber <= packet->endSequenceNumber) {
				// This message is part of this UDP packet!

			}
		}
	}
}

static void printPacketInfo(caerEventPacketHeader header) {
	// Decode successfully received data.
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
