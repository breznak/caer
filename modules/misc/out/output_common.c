#include "output_common.h"
#include "base/mainloop.h"
#include "ext/ringbuffer/ringbuffer.h"
#include "ext/c11threads_posix.h"
#include "ext/portable_time.h"
#include "ext/nets.h"

#include <stdatomic.h>
#include <unistd.h>
#include <sys/uio.h>

#include <libcaer/events/common.h>

struct event_packet_mapper {
	int16_t typeID;
	RingBuffer transferRing;
};

struct event_packet_mapper_array {
	mtx_shared_t globalLock;
	size_t mappersUsedSize;
	size_t mappersSize;
	struct event_packet_mapper mappers[];
};

typedef struct event_packet_mapper_array *eventPacketMapperArray;

struct output_common_state {
	/// Control flag for output handling thread.
	atomic_bool running;
	/// The output handling thread (separate as to not hold up processing).
	thrd_t outputThread;
	/// The file descriptor for send().
	int fileDescriptor;
	/// Filter out invalidated events or not.
	atomic_bool validOnly;
	/// Force all incoming packets to be committed to the transfer ring-buffers.
	/// This results in no loss of data, but may slow down processing considerably.
	/// It may also block it altogether, if the output goes away for any reason.
	atomic_bool keepPackets;
	/// Size of ring-buffer for main->output thread communication, in packets.
	size_t transferBufferSize;
	/// Track source ID (cannot change!).
	int16_t sourceID;
	/// Track maximum number of different types allowed (cannot change!).
	size_t packetAmount;
	/// Map packets to ring-buffers based on type information.
	eventPacketMapperArray packetMapper;
	/// Data buffer for writing to file descriptor (buffered I/O).
	uint8_t *buffer;
	/// Size of data write buffer, in bytes.
	size_t bufferSize;
	/// Size of data currently inside data write buffer, in bytes.
	size_t bufferUsedSize;
	/// Maximum interval without sending data, in µs.
	/// How long to wait if buffer not full before committing it anyway.
	size_t bufferMaxInterval;
	/// Flag to signal update to buffer configuration asynchronously.
	atomic_bool bufferUpdate;
	/// Time of last buffer commit to file descriptor (send() call).
	struct timespec bufferLastCommitTime;
	/// Reference to parent module's original data.
	caerModuleData parentModule;
};

typedef struct output_common_state *outputCommonState;

static void copyPacketToTransferRing(outputCommonState state, caerEventPacketHeader eventPacket);
static eventPacketMapperArray initializePacketMapper(size_t packetAmount, size_t transferBufferSize);
static void destroyPacketMapper(eventPacketMapperArray packetMapper);
static int eventPacketMapperCmp(const void *a, const void *b);
static bool newOutputBuffer(outputCommonState state);
static bool commitOutputBuffer(outputCommonState state);
static int outputHandlerThread(void *stateArg);
static void caerOutputCommonConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);

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
		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
			"An output module can only handle packets from the same source! "
				"A packet with source %d was sent, but this output module expects only packets from source %d.",
			eventSource, state->sourceID);
		return;
	}

	RingBuffer transferRing = NULL;

	// Map it to a transfer ring buffer.
	mtx_shared_lock_shared(&state->packetMapper->globalLock);

	size_t mappersUsedSize = state->packetMapper->mappersUsedSize;

	for (size_t i = 0; i < mappersUsedSize; i++) {
		// Check that there is a unique mapping to a transfer ring, or if not,
		// create a new one in a free mapper slot. Slots are filled up in increasing
		// index order, so if we reach empty slots, there can't be a match afterwards.
		if (state->packetMapper->mappers[i].typeID == eventType) {
			// Found match, use it.
			transferRing = state->packetMapper->mappers[i].transferRing;
			break;
		}
	}

	mtx_shared_unlock_shared(&state->packetMapper->globalLock);

	// If no valid mapping was found, if there is still space we go and add a new
	// mapping, else we complain and exit.
	if (transferRing == NULL && state->packetMapper->mappersSize != mappersUsedSize) {
		mtx_shared_lock_exclusive(&state->packetMapper->globalLock);

		// Add new mapping after last used slot.
		state->packetMapper->mappers[mappersUsedSize].typeID = eventType;
		transferRing = state->packetMapper->mappers[mappersUsedSize].transferRing;

		state->packetMapper->mappersUsedSize++;

		// Sort event_packet_mapper structures by their Type ID. This makes later
		// lookups in the output handler thread easier and more efficient.
		qsort(state->packetMapper->mappers, state->packetMapper->mappersUsedSize, sizeof(struct event_packet_mapper),
			&eventPacketMapperCmp);

		mtx_shared_unlock_exclusive(&state->packetMapper->globalLock);
	}
	else {
		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
			"New packet type %d for source %d and no more free slots available; this means an unexpected event "
				"packet made its way to this output module, one that was not declared at compile time.", eventType,
			eventSource);
		return;
	}

	// Now that we know where to copy the event packet to, let's do it.
	caerEventPacketHeader eventPacketCopy = NULL;

	// Handle the valid only flag here, that way we don't have to do another copy and
	// process it in the output handling thread if this is requested.
	if (atomic_load_explicit(&state->validOnly, memory_order_relaxed)) {
		eventPacketCopy = caerCopyEventPacketOnlyValidEvents(eventPacket);
	}
	else {
		eventPacketCopy = caerCopyEventPacketOnlyEvents(eventPacket);
	}

	if (eventPacketCopy == NULL) {
		// Failed to copy packet.
		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Failed to copy packet.");
		return;
	}

	retry: if (!ringBufferPut(transferRing, eventPacketCopy)) {
		if (atomic_load_explicit(&state->keepPackets, memory_order_relaxed)) {
			// Retry forever if requested.
			goto retry;
		}

		free(eventPacketCopy);

		caerLog(CAER_LOG_INFO, state->parentModule->moduleSubSystemString,
			"Failed to put packet copy on transfer ring-buffer: full.");
		return;
	}
}

static eventPacketMapperArray initializePacketMapper(size_t packetAmount, size_t transferBufferSize) {
	eventPacketMapperArray mapper = calloc(1, sizeof(*mapper) + (packetAmount * sizeof(struct event_packet_mapper)));
	if (mapper == NULL) {
		// Allocation error.
		return (NULL);
	}

	// Initialize values and locks.
	mapper->mappersSize = packetAmount;
	mapper->mappersUsedSize = 0;

	if (mtx_shared_init(&mapper->globalLock) != thrd_success) {
		free(mapper);

		return (NULL);
	}

	// Initialize all the event_packet_mapper structures.
	bool initFail = false;

	for (size_t i = 0; i < mapper->mappersSize; i++) {
		mapper->mappers[i].typeID = -1; // Invalid Type ID to start with.

		mapper->mappers[i].transferRing = ringBufferInit(transferBufferSize);
		if (mapper->mappers[i].transferRing == NULL) {
			// Failed to initialize.
			initFail = true;
			break;
		}
	}

	// Check that all ring-buffers were initialized correctly.
	if (initFail) {
		// Free everything.
		destroyPacketMapper(mapper);

		return (NULL);
	}

	return (mapper);
}

static void destroyPacketMapper(eventPacketMapperArray packetMapper) {
	// Free additional memory used for ring-buffers.
	for (size_t i = 0; i < packetMapper->mappersSize; i++) {
		if (packetMapper->mappers[i].transferRing != NULL) {
			caerEventPacketHeader header;
			while ((header = ringBufferGet(packetMapper->mappers[i].transferRing)) != NULL) {
				free(header); // Free unused packet copies.
			}

			ringBufferFree(packetMapper->mappers[i].transferRing);
		}
	}

	mtx_shared_destroy(&packetMapper->globalLock);

	free(packetMapper);
}

static int eventPacketMapperCmp(const void *a, const void *b) {
	const struct event_packet_mapper *aa = a;
	const struct event_packet_mapper *bb = b;

	if (aa->typeID < bb->typeID) {
		return (-1);
	}
	else if (aa->typeID > bb->typeID) {
		return (1);
	}
	else {
		return (0);
	}
}

static bool newOutputBuffer(outputCommonState state) {
	// Allocate new buffer first.
	size_t newBufferSize = (size_t) sshsNodeGetInt(state->parentModule->moduleNode, "bufferSize");

	uint8_t *newBuffer = calloc(newBufferSize, sizeof(uint8_t));
	if (newBuffer == NULL) {
		return (false);
	}

	// Commit previous buffer content and then free the memory.
	if (state->buffer != NULL) {
		commitOutputBuffer(state);

		free(state->buffer);
	}

	// Use new buffer.
	state->buffer = newBuffer;
	state->bufferSize = newBufferSize;

	return (true);
}

static bool commitOutputBuffer(outputCommonState state) {
	if (state->bufferUsedSize != 0) {
		if (!writeUntilDone(state->fileDescriptor, state->buffer, state->bufferUsedSize)) {
			return (false);
		}

		state->bufferUsedSize = 0;
	}

	return (true);
}

static int outputHandlerThread(void *stateArg) {
	outputCommonState state = stateArg;

	while (atomic_load_explicit(&state->running, memory_order_relaxed)) {
		// Handle configuration changes affecting buffer management.
		if (atomic_load_explicit(&state->bufferUpdate, memory_order_relaxed)) {
			atomic_store(&state->bufferUpdate, false);

			state->bufferMaxInterval = (size_t) sshsNodeGetInt(state->parentModule->moduleNode, "bufferMaxInterval");

			if (!newOutputBuffer(state)) {
				caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
					"Failed to allocate new output data buffer. Continue using old one.");
			}
		}

		// Fill output data buffer with data from incoming packets.
		// Respect time order as specified in AEDAT 3.X format: first event's main 64-bit
		// timestamp decides its ordering with regards to other packets. If equal,
		// order by ascending type ID.
		// TODO: implement this.
		mtx_shared_lock_shared(&state->packetMapper->globalLock);

		mtx_shared_unlock_shared(&state->packetMapper->globalLock);
	}

	// TODO: handle shutdown, write out all content of ring-buffers and commit data buffers.

	return (thrd_success);
}

bool caerOutputCommonInit(caerModuleData moduleData, int fd) {
	outputCommonState state = moduleData->moduleState;

	state->parentModule = moduleData;

	// Check for invalid file descriptor.
	if (fd < 0) {
		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Invalid file descriptor.");
		return (false);
	}

	state->fileDescriptor = fd;

	// Handle configuration.
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "validOnly", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "keepPackets", false);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "bufferSize", 16384); // in bytes, size of data buffer
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "bufferMaxInterval", 20000); // in µs, max. interval without sending data
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "transferBufferSize", 128); // in packets

	atomic_store(&state->validOnly, sshsNodeGetBool(moduleData->moduleNode, "validOnly"));
	atomic_store(&state->keepPackets, sshsNodeGetBool(moduleData->moduleNode, "keepPackets"));
	state->bufferMaxInterval = (size_t) sshsNodeGetInt(moduleData->moduleNode, "bufferMaxInterval");
	state->transferBufferSize = (size_t) sshsNodeGetInt(moduleData->moduleNode, "transferBufferSize");
	// transferBufferSize only changes here at init time! bufferSize is updated in newOutputBuffer().

	// Initial source ID has to be -1 (invalid).
	state->sourceID = -1;

	// Allocate data buffer.
	if (!newOutputBuffer(state)) {
		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Failed to allocate output data buffer.");
		return (false);
	}

	// Start output handling thread.
	atomic_store(&state->running, true);

	if (thrd_create(&state->outputThread, &outputHandlerThread, state) != thrd_success) {
		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Failed to start output handling thread.");
		return (false);
	}

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerOutputCommonConfigListener);

	return (true);
}

void caerOutputCommonExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerOutputCommonConfigListener);

	outputCommonState state = moduleData->moduleState;

	// Stop output thread and wait on it.
	atomic_store(&state->running, false);

	if ((errno = thrd_join(state->outputThread, NULL)) != thrd_success) {
		// This should never happen!
		caerLog(CAER_LOG_CRITICAL, state->parentModule->moduleSubSystemString,
			"Failed to join output handling thread. Error: %d.", errno);
	}

	if (state->packetMapper != NULL) {
		destroyPacketMapper(state->packetMapper);
	}

	if (state->buffer != NULL) {
		free(state->buffer);
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
			caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
				"Failed to allocate memory for output packet mapper.");
			return; // Skip on failure.
		}

		state->packetAmount = argsNumber;
	}

	// Check event mapper allocation size: must respect argsNumber.
	if (state->packetAmount != argsNumber) {
		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
			"Number of passed packet arguments changed, this is not allowed!");
		return;
	}

	for (size_t i = 0; i < argsNumber; i++) {
		caerEventPacketHeader packetHeader = va_arg(args, caerEventPacketHeader);

		copyPacketToTransferRing(state, packetHeader);
	}
}

static void caerOutputCommonConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	caerModuleData moduleData = userData;
	outputCommonState state = moduleData->moduleState;

	if (event == ATTRIBUTE_MODIFIED) {
		if (changeType == BOOL && caerStrEquals(changeKey, "validOnly")) {
			// Set valid only flag to given value.
			atomic_store(&state->validOnly, changeValue.boolean);
		}
		else if (changeType == BOOL && caerStrEquals(changeKey, "keepPackets")) {
			// Set keep packets flag to given value.
			atomic_store(&state->keepPackets, changeValue.boolean);
		}
		else if (changeType == INT && caerStrEquals(changeKey, "bufferSize")) {
			// Set buffer update flag.
			atomic_store(&state->bufferUpdate, true);
		}
		else if (changeType == INT && caerStrEquals(changeKey, "bufferMaxInterval")) {
			// Set buffer update flag.
			atomic_store(&state->bufferUpdate, true);
		}
	}
}
