#include "output_common.h"
#include "base/mainloop.h"
#include "ext/ringbuffer/ringbuffer.h"
#include "ext/c11threads_posix.h"

#include <stdatomic.h>
#include <unistd.h>
#include <sys/uio.h>

#include <libcaer/events/common.h>

struct event_packet_mapper {
	int16_t typeID;
	RingBuffer transferRing;
};

typedef struct event_packet_mapper *eventPacketMapper;

struct output_common_state {
	/// The output handling thread (separate as to not hold up processing).
	thrd_t outputThread;
	/// The file descriptor for send().
	int fileDescriptor;
	/// Filter out invalidated events or not.
	bool validOnly;
	/// Size of each write buffer, in bytes.
	size_t maxBufferSize;
	/// Maximum interval without sending data, in µs.
	/// How long to wait if buffer not full before committing it anyway.
	size_t maxBufferInterval;
	/// Size of ring-buffer for main->output thread communication, in packets.
	size_t transferBufferSize;
	/// Track source ID (cannot change!).
	int16_t sourceID;
	/// Track maximum number of different types allowed (cannot change!).
	size_t packetAmount;
	/// Map packets to ring-buffers based on type information.
	eventPacketMapper packetMapper;
};

typedef struct output_common_state *outputCommonState;

static void copyPacketToTransferRing(outputCommonState state, caerEventPacketHeader eventPacket);
static eventPacketMapper initializePacketMapper(size_t packetAmount, size_t transferBufferSize);
static void destroyPacketMapper(eventPacketMapper packetMapper, size_t packetAmount);
static int outputHandlerThread(void *stateArg);

/**
 * Copy event packets to the right ring buffer for transfer
 * to the external output handling thread based on type information.
 *
 * @param packetMapper array of packet mapper structures: Type -> Transfer-Ring.
 * @param packetAmount length of array, amount of expected different event packets.
 * @param eventPacket an event packet.
 */
static void copyPacketToTransferRing(outputCommonState state, caerEventPacketHeader eventPacket) {
	// Skip empty event packets.
	if (eventPacket == NULL) {
		return;
	}

	// Get source and type information from the event packet.
	int16_t eventSource = caerEventPacketHeaderGetEventSource(eventPacket);
	int16_t eventType = caerEventPacketHeaderGetEventType(eventPacket);

	// Check that source is unique.
	if (state->sourceID == -1) {
		state->sourceID = eventSource; // Remember this!
	}
	else if (state->sourceID != eventSource) {
		caerLog(CAER_LOG_ERROR, "Data Output", "An output module can only handle packets from the same source! "
			"A packet with source %d was sent, but this output module expects only packets from source %d.",
			eventSource, state->sourceID);
		return;
	}

	RingBuffer transferRing = NULL;

	// Map it to a transfer ring buffer.
	for (size_t i = 0; i < state->packetAmount; i++) {
		// Check that there is a unique mapping to a transfer ring, or if not,
		// create a new one in a free mapper slot. Slots are filled up in increasing
		// index order, so if we reach empty slots, there can't be a match afterwards.
		if (state->packetMapper[i].typeID == eventType) {
			// Found match, use it.
			transferRing = state->packetMapper[i].transferRing;
			break;
		}

		// Reached empty slot, use it.
		if (state->packetMapper[i].typeID == -1) {
			state->packetMapper[i].typeID = eventType;

			transferRing = state->packetMapper[i].transferRing;
			break;
		}
	}

	// Check that a valid index was found, else complain.
	if (transferRing == NULL) {
		caerLog(CAER_LOG_ERROR, "Data Output",
			"New packet type %d for source %d and no more free slots available; this means an unexpected event "
				"packet made its way to this output module, one that was not declared at compile time.", eventType,
			eventSource);
		return;
	}

	// Now that we know where to copy the event packet to, let's do it.
	caerEventPacketHeader eventPacketCopy = NULL;

	// Handle the valid only flag here, that way we don't have to do another copy and
	// process it in the output handling thread if this is requested.
	if (state->validOnly) {
		eventPacketCopy = caerCopyEventPacketOnlyValidEvents(eventPacket);
	}
	else {
		eventPacketCopy = caerCopyEventPacketOnlyEvents(eventPacket);
	}

	if (eventPacketCopy == NULL) {
		// Failed to copy packet.
		caerLog(CAER_LOG_ERROR, "Data Output", "Failed to copy packet.");
		return;
	}

	if (!ringBufferPut(transferRing, eventPacketCopy)) {
		free(eventPacketCopy);
		// TODO: handle ring buffer full, maybe block on setting?

		caerLog(CAER_LOG_INFO, "Data Output", "Failed to put packet copy on transfer ring-buffer: full.");
		return;
	}
}

static eventPacketMapper initializePacketMapper(size_t packetAmount, size_t transferBufferSize) {
	eventPacketMapper mapper = calloc(packetAmount, sizeof(*mapper));
	if (mapper == NULL) {
		// Allocation error.
		return (NULL);
	}

	// Initialize all the ring-buffers.
	bool initFail = false;

	for (size_t i = 0; i < packetAmount; i++) {
		mapper[i].transferRing = ringBufferInit(transferBufferSize);
		if (mapper[i].transferRing == NULL) {
			// Failed to initialize.
			initFail = true;
			break;
		}
	}

	// Check that all ring-buffers were initialized correctly.
	if (initFail) {
		// Free everything.
		destroyPacketMapper(mapper, packetAmount);

		return (NULL);
	}

	return (mapper);
}

static void destroyPacketMapper(eventPacketMapper packetMapper, size_t packetAmount) {
	// Free additional memory used for ring-buffers.
	for (size_t i = 0; i < packetAmount; i++) {
		if (packetMapper[i].transferRing != NULL) {
			caerEventPacketHeader header;
			while ((header = ringBufferGet(packetMapper[i].transferRing)) != NULL) {
				free(header); // Free unused packet copies.
			}

			ringBufferFree(packetMapper[i].transferRing);
		}
	}

	free(packetMapper);
}

static int outputHandlerThread(void *stateArg) {
	outputCommonState state = stateArg;

	while (true) {

	}

	return (thrd_success);
}

bool caerOutputCommonInit(caerModuleData moduleData, int fd) {
	outputCommonState state = moduleData->moduleState;

	// Check for invalid file descriptor.
	if (fd < 0) {
		caerLog(CAER_LOG_ERROR, "Data Output", "Invalid file descriptor.");
		return (false);
	}

	state->fileDescriptor = fd;

	// Handle configuration.
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "validOnly", false);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "maxBufferSize", 8192); // in bytes, size of each buffer
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "maxBufferInterval", 10000); // in µs, max. interval without sending data
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "transferBufferSize", 128); // in packets

	state->validOnly = sshsNodeGetBool(moduleData->moduleNode, "validOnly");
	state->maxBufferSize = (size_t) sshsNodeGetInt(moduleData->moduleNode, "maxBufferSize");
	state->maxBufferInterval = (size_t) sshsNodeGetInt(moduleData->moduleNode, "maxBufferInterval");
	state->transferBufferSize = (size_t) sshsNodeGetInt(moduleData->moduleNode, "transferBufferSize");

	// Initial source ID has to be -1 (invalid).
	state->sourceID = -1;

	// Start output handling thread.
	if (thrd_create(&state->outputThread, &outputHandlerThread, state) != thrd_success) {
		caerLog(CAER_LOG_ERROR, "Data Output", "Failed to start output handling thread.");
		return (false);
	}

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	return (true);
}

void caerOutputCommonExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	outputCommonState state = moduleData->moduleState;

	if ((errno = thrd_join(state->outputThread, NULL)) != thrd_success) {
		// This should never happen!
		caerLog(CAER_LOG_CRITICAL, "Data Output", "Failed to join output handling thread. Error: %d.", errno);
	}

	if (state->packetMapper != NULL) {
		destroyPacketMapper(state->packetMapper, state->packetAmount);
		state->packetMapper = NULL;
	}

	// Close file descriptor.
	close(state->fileDescriptor);
}

void caerOutputCommonRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	outputCommonState state = moduleData->moduleState;

	// Initialize packet mappers array if first run.
	if (state->packetMapper == NULL) {
		state->packetMapper = initializePacketMapper(argsNumber, state->transferBufferSize);
		if (state->packetMapper == NULL) {
			caerLog(CAER_LOG_ERROR, "Data Output", "Failed to allocate memory for output packet mapper.");
			return; // Skip on failure.
		}

		state->packetAmount = argsNumber;
	}

	// Check event mapper allocation size: must respect argsNumber.
	if (state->packetAmount != argsNumber) {
		caerLog(CAER_LOG_ERROR, "Data Output", "Number of passed packet arguments changed, this is not allowed!");
		return;
	}

	for (size_t i = 0; i < state->packetAmount; i++) {
		caerEventPacketHeader packetHeader = va_arg(args, caerEventPacketHeader);

		copyPacketToTransferRing(state, packetHeader);
	}
}

void caerOutputCommonConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	outputCommonState state = moduleData->moduleState;

}
