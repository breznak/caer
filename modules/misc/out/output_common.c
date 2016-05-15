/*
 * Here we handle all outputs in a common way, taking in event packets
 * as input and writing a buffer to a file descriptor as output.
 * The main-loop part is responsible for gathering the event packets,
 * copying them and their events (valid or not depending on configuration),
 * and putting them on a transfer ring-buffer. A second thread, called the
 * output handler, gets the packet groups from there, orders them according
 * to the AEDAT 3.X format specification, and breaks them up into chunks as
 * directed to write() them to a file descriptor efficiently (buffered I/O).
 * The AEDAT 3.X format specification specifically states that there is no
 * relation at all between packets from different sources at the output level,
 * that they behave as if independent, which we do here to simplify the system
 * considerably: one output module (or Sink) can only work with packets from
 * one source. Multiple sources will have to go to multiple output modules!
 * The other stipulation in the AEDAT 3.X specifications is on ordering of
 * events from the same source: the first timestamp of a packet determines
 * its order in the packet stream, from smallest timestamp to largest, which
 * is the logical increasing time ordering you'd expect.
 * This kind of ordering is useful and simplifies reading back data later on;
 * if you read a packet of type A with TS A-TS1, when you next read a packet of
 * the same type A, with TS A-TS2, you know you must also have read all other
 * events, of this AND all other present types, with a timestamp between A-TS1
 * and (A-TS2 - 1). This makes time-based reading and replaying of data very easy
 * and efficient, so time-slice playback or real-time playback get relatively
 * simple to implement. Data-amount based playback is always relatively easy.
 * Now, outputting event packets in this particular order from an output module
 * requires some additional processing: before you can write out packet A with TS
 * A-TS1, you need to be sure no other packets with a timestamp equal or smaller
 * to A-TS1 can come afterwards (the only solution would be to discard them at
 * that point to maintain the correct ordering, and you'd want to avoid that).
 * We cannot assume a constant and quick data flow, since at any point during a
 * recording, data producers can be turned off, packet size etc. configuration
 * changed, or some events, like Special ones, are rare to begin with during
 * normal camera operation (one approx. every 35 minutes).
 * But we'd like to write data continuously and as soon as possible!
 * Thankfully cAER/libcaer come to the rescue thanks to a small but important
 * detail of how input modules are implemented (input modules are all those
 * modules that create new data in some way, also called a Source).
 * They either create sequences of single packets, where the ordering is trivial,
 * or so called 'Packet Containers', which do offer timestamp-related guarantees.
 * From the libcaer/events/packetContainer.h documentation:
 *
 * "An EventPacketContainer is a logical construct that contains packets
 * of events (EventPackets) of different event types, with the aim of
 * keeping related events of differing types, such as DVS and IMU data,
 * together. Such a relation is usually based on time intervals, trying
 * to keep groups of event happening in a certain time-slice together.
 * All conforming input modules keep to this rule, with *one* possible
 * exception: for example IMU6 and Frame packets cannot guarantee this
 * always; it is possible that *one* such event is moved to the successive
 * packet container, since these are composite events and can take a long
 * time to be completed/created, and in that time any of the other packet
 * container commit triggers may happen, and waiting is not an option to
 * maintain those contracts, and thus the event may be incomplete at that
 * time and has to be deferred to the next packet container."
 *
 * Understanding this gives a simple solution to the problem above: if we
 * see all the packets contained in a packet container, which is the case
 * for each run through of the cAER mainloop (as it fetches *one* new packet
 * container each time from an input module), we only have to also see the
 * next packet container from that input module; at which point we can order
 * the packets of the first container correctly (with maybe some of the second
 * container interspersed in), and write them out to a file descriptor.
 * Then we just rinse and repeat with the remaining packets of the second
 * container, and its own successor.
 * The assumption of one run of the mainloop getting at most one packet
 * container from each Source is correct with the current implementation,
 * and future designs of Sources should take this into account! Delays in
 * packet containers getting to the output module are still allowed, provided
 * the ordering doesn't change and single packets aren't mixed, which is
 * a sane restriction to impose anyway.
 */

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
#include <libcaer/events/packetContainer.h>

struct output_common_state {
	/// Control flag for output handling thread.
	atomic_bool running;
	/// The output handling thread (separate as to not hold up processing).
	thrd_t outputThread;
	/// Track source ID (cannot change!). One source per I/O module!
	int16_t sourceID;
	/// The file descriptors for send().
	outputCommonFDs fileDescriptors;
	/// Filter out invalidated events or not.
	atomic_bool validOnly;
	/// Force all incoming packets to be committed to the transfer ring-buffers.
	/// This results in no loss of data, but may slow down processing considerably.
	/// It may also block it altogether, if the output goes away for any reason.
	atomic_bool keepPackets;
	/// Transfer packets coming from a mainloop run to the output handling thread.
	/// We use EventPacketContainers as data structure for convenience, they do exactly
	/// keep track of the data we do want to transfer and are part of libcaer.
	RingBuffer transferRing;
	/// Track last event packet container gotten in output handler thread.
	caerEventPacketContainer lastPacketContainer;
	/// Track last event packet timestamp that was sent out.
	int64_t lastPacketTimestamp;
	/// Track last event packet type ID that was sent out.
	int16_t lastPacketTypeID;
	/// Data buffer for writing to file descriptor (buffered I/O).
	uint8_t *buffer;
	/// Size of data write buffer, in bytes.
	size_t bufferSize;
	/// Size of data currently inside data write buffer, in bytes.
	size_t bufferUsedSize;
	/// Maximum interval without sending data, in µs.
	/// How long to wait if buffer not full before committing it anyway.
	size_t bufferMaxInterval;
	/// Time of last buffer commit to file descriptor (send() call).
	struct timespec bufferLastCommitTime;
	/// Flag to signal update to buffer configuration asynchronously.
	atomic_bool bufferUpdate;
	/// Reference to parent module's original data.
	caerModuleData parentModule;
};

typedef struct output_common_state *outputCommonState;

static void copyPacketsToTransferRing(outputCommonState state, size_t packetsListSize, va_list packetsList);
static int packetsTypeCmp(const void *a, const void *b);
static bool newOutputBuffer(outputCommonState state);
static bool commitOutputBuffer(outputCommonState state);
static void sendEventPacket(outputCommonState state, caerEventPacketHeader packet);
static int outputHandlerThread(void *stateArg);
static void caerOutputCommonConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);

/**
 * Copy event packets to the ring buffer for transfer to the output handler thread.
 *
 * @param state output module state.
 * @param packetsListSize the length of the variable-length argument list of event packets.
 * @param packetsList a variable-length argument list of event packets.
 */
static void copyPacketsToTransferRing(outputCommonState state, size_t packetsListSize, va_list packetsList) {
	caerEventPacketHeader packets[packetsListSize];
	size_t packetsSize = 0;

	// Count how many packets are really there, skipping empty event packets.
	for (size_t i = 0; i < packetsListSize; i++) {
		caerEventPacketHeader packetHeader = va_arg(packetsList, caerEventPacketHeader);

		// Found non-empty event packet.
		if (packetHeader != NULL) {
			// Get source information from the event packet.
			int16_t eventSource = caerEventPacketHeaderGetEventSource(packetHeader);

			// Check that source is unique.
			if (state->sourceID == -1) {
				state->sourceID = eventSource; // Remember this!
			}
			else if (state->sourceID != eventSource) {
				caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
					"An output module can only handle packets from the same source! "
						"A packet with source %d was sent, but this output module expects only packets from source %d.",
					eventSource, state->sourceID);
				continue;
			}

			// Source ID is correct, packet is not empty, we got it!
			packets[packetsSize++] = packetHeader;
		}
	}

	// There was nothing in this mainloop run!
	if (packetsSize == 0) {
		return;
	}

	// Allocate memory for event packet array structure that will get passed to output handler thread.
	caerEventPacketContainer eventPackets = caerEventPacketContainerAllocate((int32_t) packetsSize);
	if (eventPackets == NULL) {
		return;
	}

	// Handle the valid only flag here, that way we don't have to do another copy and
	// process it in the output handling thread. We get the value once here, so we do
	// the same for all packets from the same mainloop run, avoiding mid-way changes.
	bool validOnly = atomic_load_explicit(&state->validOnly, memory_order_relaxed);

	// Now copy each event packet and send the array out. We reset the packets number to
	// zero here, so we can increase it as we loop and track how many packets there are.
	caerEventPacketContainerSetEventPacketsNumber(eventPackets, 0);

	for (size_t i = 0; i < packetsSize; i++) {
		int32_t idx = caerEventPacketContainerGetEventPacketsNumber(eventPackets);

		if (validOnly) {
			caerEventPacketContainerSetEventPacket(eventPackets, idx, caerCopyEventPacketOnlyValidEvents(packets[i]));
		}
		else {
			caerEventPacketContainerSetEventPacket(eventPackets, idx, caerCopyEventPacketOnlyEvents(packets[i]));
		}

		if (caerEventPacketContainerGetEventPacket(eventPackets, idx) == NULL) {
			// Failed to copy packet. Signal but try to continue anyway.
			if ((validOnly && (caerEventPacketHeaderGetEventValid(packets[i]) == 0))
				|| (!validOnly && (caerEventPacketHeaderGetEventNumber(packets[i]) == 0))) {
				caerLog(CAER_LOG_NOTICE, state->parentModule->moduleSubSystemString,
					"Submitted empty event packet to output. Ignoring empty event packet.");
			}
			else {
				caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
					"Failed to copy event packet to output.");
			}
		}
		else {
			caerEventPacketContainerSetEventPacketsNumber(eventPackets, idx + 1);
		}
	}

	// We might have failed to copy all packets (unlikely).
	if (caerEventPacketContainerGetEventPacketsNumber(eventPackets) == 0) {
		caerEventPacketContainerFree(eventPackets);

		return;
	}

	retry: if (!ringBufferPut(state->transferRing, eventPackets)) {
		if (atomic_load_explicit(&state->keepPackets, memory_order_relaxed)) {
			// Retry forever if requested.
			goto retry;
		}

		caerEventPacketContainerFree(eventPackets);

		caerLog(CAER_LOG_INFO, state->parentModule->moduleSubSystemString,
			"Failed to put packet's array copy on transfer ring-buffer: full.");
		return;
	}
}

static int packetsTypeCmp(const void *a, const void *b) {
	const caerEventPacketHeader *aa = a;
	const caerEventPacketHeader *bb = b;

	// Sort first by timestamp of the first event.
	int32_t eventTimestampA = caerGenericEventGetTimestamp(caerGenericEventGetEvent(*aa, 0), *aa);
	int32_t eventTimestampB = caerGenericEventGetTimestamp(caerGenericEventGetEvent(*bb, 0), *bb);

	if (eventTimestampA < eventTimestampB) {
		return (-1);
	}
	else if (eventTimestampA > eventTimestampB) {
		return (1);
	}
	else {
		// If equal, further sort by type ID.
		int16_t eventTypeA = caerEventPacketHeaderGetEventType(*aa);
		int16_t eventTypeB = caerEventPacketHeaderGetEventType(*bb);

		if (eventTypeA < eventTypeB) {
			return (-1);
		}
		else if (eventTypeA > eventTypeB) {
			return (1);
		}
		else {
			return (0);
		}
	}
}

static bool newOutputBuffer(outputCommonState state) {
	// First check if the size really changed.
	size_t newBufferSize = (size_t) sshsNodeGetInt(state->parentModule->moduleNode, "bufferSize");

	if (newBufferSize == state->bufferSize) {
		// Yeah, we're already where we want to be!
		return (true);
	}

	// Allocate new buffer.
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
		for (size_t i = 0; i < state->fileDescriptors->fdsSize; i++) {
			if (state->fileDescriptors->fds[i] >= 0) {
				if (!writeUntilDone(state->fileDescriptors->fds[i], state->buffer, state->bufferUsedSize)) {
					// TODO: maybe close() and set to -1 here?
					return (false);
				}
			}
		}

		state->bufferUsedSize = 0;
	}

	// Update last commit time.
	portable_clock_gettime_monotonic(&state->bufferLastCommitTime);

	return (true);
}

static void sendEventPacket(outputCommonState state, caerEventPacketHeader packet) {

}

static int outputHandlerThread(void *stateArg) {
	outputCommonState state = stateArg;

	// If no data is available on the transfer ring-buffer, sleep for 500µs (0.5 ms)
	// to avoid wasting resources in a busy loop.
	struct timespec noDataSleep = { .tv_sec = 0, .tv_nsec = 500000 };

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
		// Respect time order as specified in AEDAT 3.X format: first event's main
		// timestamp decides its ordering with regards to other packets. Smaller
		// comes first. If equal, order by increasing type ID.

		// Get the newest event packet container from the transfer ring-buffer.
		caerEventPacketContainer currPacketContainer = ringBufferGet(state->transferRing);
		if (currPacketContainer == NULL) {
			// There is none, so we can't work on and commit the last container either.
			// We just sleep here a little and then try again, as we need that data!
			thrd_sleep(&noDataSleep, NULL);
			continue;
		}

		// Sort container by first timestamp and by type ID.
		size_t currPacketContainerSize = (size_t) caerEventPacketContainerGetEventPacketsNumber(currPacketContainer);

		qsort(currPacketContainer->eventPackets, currPacketContainerSize, sizeof(caerEventPacketHeader),
			&packetsTypeCmp);

		// We got new data. If this is the first time, we can only store that data, and then
		// go and wait on the second container to arrive, before taking any decisions.
		if (state->lastPacketContainer == NULL) {
			state->lastPacketContainer = currPacketContainer;
			continue;
		}

		// Since we just got new data, let's first check that it does conform to our expectations.
		// This means the timestamp didn't slide back, or if equal, the type ID didn't get smaller.
		// These checks are needed to avoid illegal ordering. Normal operation will never trigger
		// these, as stated in the assumptions at the start of file, but erroneous usage or mixing
		// or reordering of packet containers is possible, and has to be caught here.
		for (size_t cpIdx = 0; cpIdx < currPacketContainerSize; cpIdx++) {
			caerEventPacketHeader cpPacket = caerEventPacketContainerGetEventPacket(currPacketContainer,
				(int32_t) cpIdx);

			void *cpFirstEvent = caerGenericEventGetEvent(cpPacket, 0);
			int64_t cpFirstEventTimestamp = caerGenericEventGetTimestamp64(cpFirstEvent, cpPacket);
			int16_t cpTypeID = caerEventPacketHeaderGetEventType(cpPacket);

			if (cpFirstEventTimestamp < state->lastPacketTimestamp) {
				// Smaller TS than already sent, illegal, delete packet.
				caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
					"Detected timestamp going back, ignoring this packet!");

				// Mark as deleted.
				caerEventPacketContainerSetEventPacket(currPacketContainer, (int32_t) cpIdx, NULL);
				free(cpPacket);
			}
			else if (cpFirstEventTimestamp == state->lastPacketTimestamp) {
				// Same TS, so check that we don't have a smaller type ID, which would be illegal.
				if (cpTypeID < state->lastPacketTypeID) {
					// Smaller Type ID than already sent, illegal, delete packet.
					caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
						"Detected type ID going back, ignoring this packet!");

					// Mark as deleted.
					caerEventPacketContainerSetEventPacket(currPacketContainer, (int32_t) cpIdx, NULL);
					free(cpPacket);
				}
				else {
					// Equal or bigger Type ID, this is good. Strict Type ID ordering ensures
					// that all other packets in this container are the same, so exit loop.
					break;
				}
			}
			else {
				// Bigger TS than already sent, this is good. Strict TS ordering ensures
				// that all other packets in this container are the same, so exit loop.
				break;
			}
		}

		// We have both packet containers, we can now get the needed data, order them, send
		// them out, and put the remaining ones into lastPacketContainer for the next run.
		size_t lastPacketContainerSize = (size_t) caerEventPacketContainerGetEventPacketsNumber(
			state->lastPacketContainer);

		for (size_t lpIdx = 0; lpIdx < lastPacketContainerSize; lpIdx++) {
			caerEventPacketHeader lpPacket = caerEventPacketContainerGetEventPacket(state->lastPacketContainer,
				(int32_t) lpIdx);

			// Packets in the lastPacketContainer can be NULL, as they could have been marked as sent
			// in the following code, when they were part of currPacketContainer.
			if (lpPacket == NULL) {
				continue;
			}

			void *lpFirstEvent = caerGenericEventGetEvent(lpPacket, 0);
			int64_t lpFirstEventTimestamp = caerGenericEventGetTimestamp64(lpFirstEvent, lpPacket);
			int16_t lpTypeID = caerEventPacketHeaderGetEventType(lpPacket);

			// Check, based on first event timestamp, that there is no event packet in the current
			// (second) packet container with lower timestamp than the one in the last (first)
			// packet container. Since they are ordered by timestamp already, simple linear search.
			for (size_t cpIdx = 0; cpIdx < currPacketContainerSize; cpIdx++) {
				caerEventPacketHeader cpPacket = caerEventPacketContainerGetEventPacket(currPacketContainer,
					(int32_t) cpIdx);

				// Packets in the currPacketContainer can be NULL, as they could have been marked as
				// sent in the following code.
				if (cpPacket == NULL) {
					continue;
				}

				void *cpFirstEvent = caerGenericEventGetEvent(cpPacket, 0);
				int64_t cpFirstEventTimestamp = caerGenericEventGetTimestamp64(cpFirstEvent, cpPacket);
				int16_t cpTypeID = caerEventPacketHeaderGetEventType(cpPacket);

				if (cpFirstEventTimestamp < lpFirstEventTimestamp) {
					// Strictly smaller, packet from current container has precedence.
					sendEventPacket(state, cpPacket);
					state->lastPacketTimestamp = cpFirstEventTimestamp;
					state->lastPacketTypeID = cpTypeID;

					// Mark as sent.
					caerEventPacketContainerSetEventPacket(currPacketContainer, (int32_t) cpIdx, NULL);
					free(cpPacket);
				}
				else if (cpFirstEventTimestamp == lpFirstEventTimestamp) {
					// Timestamps are equal between the packets from the two packet containers.
					// We order by Type ID then, as the format specification mandates.
					if (cpTypeID < lpTypeID) {
						// Strictly smaller, packet from current container has precedence.
						sendEventPacket(state, cpPacket);
						state->lastPacketTimestamp = cpFirstEventTimestamp;
						state->lastPacketTypeID = cpTypeID;

						// Mark as sent.
						caerEventPacketContainerSetEventPacket(currPacketContainer, (int32_t) cpIdx, NULL);
						free(cpPacket);
					}
					else {
						// Equal or bigger, in this case we stop and let the packet from the
						// last container be sent. This current packet will be sent on the
						// next iteration.
						break;
					}
				}
				else {
					// Strictly bigger, since ordered by timestamp, all further current packets
					// will also be bigger, so we can stop checking the current packet container.
					break;
				}
			}

			// Send the packet from last event container, there are guaranteed none in the current
			// event container with smaller timestamp, or equal timestamp but smaller type ID.
			sendEventPacket(state, lpPacket);
			state->lastPacketTimestamp = lpFirstEventTimestamp;
			state->lastPacketTypeID = lpTypeID;

			// Mark as sent.
			caerEventPacketContainerSetEventPacket(state->lastPacketContainer, (int32_t) lpIdx, NULL);
			free(lpPacket);
		}

		// Check that lastPacketContainer is empty and everything has been sent (sanity check).
		for (size_t lpIdx = 0; lpIdx < lastPacketContainerSize; lpIdx++) {
			caerEventPacketHeader lpPacket = caerEventPacketContainerGetEventPacket(state->lastPacketContainer,
				(int32_t) lpIdx);

			if (lpPacket != NULL) {
				// This should never happen!
				caerLog(CAER_LOG_CRITICAL, state->parentModule->moduleSubSystemString,
					"Failed to send all packets in lastPacketContainer.");
			}
		}

		// Free all remaining lastPacketContainer memory.
		caerEventPacketContainerFree(state->lastPacketContainer);

		// Switch lastPacketContainer with currPacketContainer for next loop iteration.
		state->lastPacketContainer = currPacketContainer;
	}

	// TODO: handle shutdown, write out all content of ring-buffers and commit data buffers.

	return (thrd_success);
}

bool caerOutputCommonInit(caerModuleData moduleData, outputCommonFDs fds) {
	outputCommonState state = moduleData->moduleState;

	state->parentModule = moduleData;

	// Check for invalid file descriptors.
	if (fds == NULL) {
		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Invalid file descriptor array.");
		return (false);
	}

	if (fds->fdsSize == 0) {
		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Empty file descriptor array.");
		return (false);
	}

	for (size_t i = 0; i < fds->fdsSize; i++) {
		// Allow values of -1 to signal "not in use" slot.
		if (fds->fds[i] < -1) {
			caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Invalid file descriptor.");
			return (false);
		}
	}

	state->fileDescriptors = fds;

	// Initial source ID has to be -1 (invalid).
	state->sourceID = -1;

	// Handle configuration.
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "validOnly", false); // only send valid events
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "keepPackets", false); // ensure all packets are kept
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "bufferSize", 16384); // in bytes, size of data buffer
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "bufferMaxInterval", 20000); // in µs, max. interval without sending data
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "transferBufferSize", 128); // in packet groups

	atomic_store(&state->validOnly, sshsNodeGetBool(moduleData->moduleNode, "validOnly"));
	atomic_store(&state->keepPackets, sshsNodeGetBool(moduleData->moduleNode, "keepPackets"));
	state->bufferMaxInterval = (size_t) sshsNodeGetInt(moduleData->moduleNode, "bufferMaxInterval");

	// Initialize transfer ring-buffer. transferBufferSize only changes here at init time!
	state->transferRing = ringBufferInit((size_t) sshsNodeGetInt(moduleData->moduleNode, "transferBufferSize"));
	if (state->transferRing == NULL) {
		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Failed to allocate transfer ring-buffer.");
		return (false);
	}

	// Allocate data buffer. bufferSize is updated here.
	if (!newOutputBuffer(state)) {
		ringBufferFree(state->transferRing);

		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Failed to allocate output data buffer.");
		return (false);
	}

	// Start output handling thread.
	atomic_store(&state->running, true);

	if (thrd_create(&state->outputThread, &outputHandlerThread, state) != thrd_success) {
		ringBufferFree(state->transferRing);
		free(state->buffer);

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

	free(state->buffer);

	ringBufferFree(state->transferRing);
}

void caerOutputCommonRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	outputCommonState state = moduleData->moduleState;

	copyPacketsToTransferRing(state, argsNumber, args);
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
