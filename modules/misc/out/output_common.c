/*
 * Here we handle all outputs in a common way, taking in event packets
 * as input and writing a buffer to a file descriptor as output.
 * The main-loop part is responsible for gathering the event packets,
 * copying them and their events (valid or not depending on configuration),
 * and putting them on a transfer ring-buffer. A second thread, called the
 * output handler, gets the packet groups from there, orders them according
 * to the AEDAT 3.X format specification, and breaks them up into chunks as
 * directed to write them to a file descriptor efficiently (buffered I/O).
 * The AEDAT 3.X format specification specifically states that there is no
 * relation at all between packets from different sources at the output level,
 * that they behave as if independent, which we do here to simplify the system
 * considerably: one output module (or Sink) can only work with packets from
 * one source. Multiple sources will have to go to multiple output modules!
 * The other stipulation in the AEDAT 3.X specifications is on ordering of
 * events from the same source: the first timestamp of a packet determines
 * its order in the packet stream, from smallest timestamp to largest, which
 * is the logical monotonic increasing time ordering you'd expect.
 * This kind of ordering is useful and simplifies reading back data later on;
 * if you read a packet of type A with TS A-TS1, when you next read a packet of
 * the same type A, with TS A-TS2, you know you must also have read all other
 * events, of this AND all other present types, with a timestamp between A-TS1
 * and (A-TS2 - 1). This makes time-based reading and replaying of data very easy
 * and efficient, so time-slice playback or real-time playback get relatively
 * simple to implement. Data-amount based playback is always relatively easy.
 *
 * Now, outputting event packets in this particular order from an output module
 * requires some additional processing: before you can write out packet A with TS
 * A-TS1, you need to be sure no other packets with a timestamp smaller than
 * A-TS1 can come afterwards (the only solution would be to discard them at
 * that point to maintain the correct ordering, and you'd want to avoid that).
 * We cannot assume a constant and quick data flow, since at any point during a
 * recording, data producers can be turned off, packet size etc. configuration
 * changed, or some events, like Special ones, are rare to begin with during
 * normal camera operation (the TIMESTAMP_WRAP every 35 minutes).
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
 * This time-order is based on the *main* time-stamp of an event, the one
 * whose offset is referenced in the event packet header and that is
 * used by the caerGenericEvent*() functions. It's guaranteed that all
 * conforming input modules keep to this rule, generating containers
 * that include all events from all types within the given time-slice."
 *
 * Understanding this gives a simple solution to the problem above: if we
 * see all the packets contained in a packet container, which is the case
 * for each run through of the cAER mainloop (as it fetches *one* new packet
 * container each time from an input module), we can order the packets of
 * the container correctly, and write them out to a file descriptor.
 * Then we just rinse and repeat for every new packet container.
 * The assumption of one run of the mainloop getting at most one packet
 * container from each Source is correct with the current implementation,
 * and future designs of Sources should take this into account! Delays in
 * packet containers getting to the output module are still allowed, provided
 * the ordering doesn't change and single packets aren't mixed, which is
 * a sane restriction to impose anyway.
 */

#include "output_common.h"
#include "base/mainloop.h"
#include "ext/portable_time.h"
#include "ext/ringbuffer/ringbuffer.h"
#include "ext/buffers.h"
#include <arpa/inet.h>
#include <netinet/in.h>
#ifdef HAVE_PTHREADS
	#include "ext/c11threads_posix.h"
#endif

#define LODEPNG_NO_COMPILE_DISK
#define LODEPNG_NO_COMPILE_ANCILLARY_CHUNKS
#include "ext/lodepng/lodepng.h"

#include <libcaer/events/common.h>
#include <libcaer/events/packetContainer.h>
#include <libcaer/events/frame.h>

// TODO: check handling of timestamp-reset events from camera!

struct output_common_statistics {
	uint64_t packetsNumber;
	uint64_t packetsTotalSize;
	uint64_t packetsHeaderSize;
	uint64_t packetsDataSize;
	uint64_t dataWritten;
};

struct output_common_state {
	/// Control flag for output handling thread.
	atomic_bool running;
	/// The output handling thread (separate as to not hold up processing).
	thrd_t outputThread;
	/// Track source ID (cannot change!). One source per I/O module!
	atomic_int_fast16_t sourceID;
	/// Source information node for that particular source ID.
	/// Must be set by mainloop, external threads cannot get it directly!
	sshsNode sourceInfoNode;
	/// The file descriptors for writing and server mode.
	outputCommonFDs fileDescriptors;
	/// Network-like stream or file-like stream. Matters for header format.
	bool isNetworkStream;
	/// For network-like outputs, we differentiate between stream and message
	/// based protocols, like TCP and UDP. Matters for header/sequence number.
	bool isNetworkMessageBased;
	/// Keep track of the sequence number for message-based protocols.
	int64_t networkSequenceNumber;
	/// Filter out invalidated events or not.
	atomic_bool validOnly;
	/// Force all incoming packets to be committed to the transfer ring-buffer.
	/// This results in no loss of data, but may slow down processing considerably.
	/// It may also block it altogether, if the output goes away for any reason.
	atomic_bool keepPackets;
	/// Transfer packets coming from a mainloop run to the output handling thread.
	/// We use EventPacketContainers as data structure for convenience, they do exactly
	/// keep track of the data we do want to transfer and are part of libcaer.
	RingBuffer transferRing;
	/// Track last packet container's highest event timestamp that was sent out.
	int64_t lastTimestamp;
	/// Data buffer for writing to file descriptor (buffered I/O).
	simpleBuffer dataBuffer;
	/// Maximum interval without sending data, in µs.
	/// How long to wait if buffer not full before committing it anyway.
	uint64_t bufferMaxInterval;
	/// Time of last buffer commit to file descriptor (send() call).
	struct timespec bufferLastCommitTime;
	/// Flag to signal update to buffer configuration asynchronously.
	atomic_bool bufferUpdate;
	/// Support different formats, providing data compression.
	int8_t format;
	/// Output module statistics collection.
	struct output_common_statistics statistics;
	/// Reference to parent module's original data.
	caerModuleData parentModule;
};

typedef struct output_common_state *outputCommonState;

size_t CAER_OUTPUT_COMMON_STATE_STRUCT_SIZE = sizeof(struct output_common_state);

static void copyPacketsToTransferRing(outputCommonState state, size_t packetsListSize, va_list packetsList);
static int packetsFirstTimestampThenTypeCmp(const void *a, const void *b);
static bool newOutputBuffer(outputCommonState state);
static void commitOutputBuffer(outputCommonState state);
static size_t compressEventPacket(outputCommonState state, caerEventPacketHeader packet, size_t packetSize);
static void sendEventPacket(outputCommonState state, caerEventPacketHeader packet);
static void orderAndSendEventPackets(outputCommonState state, caerEventPacketContainer currPacketContainer);
static void handleNewServerConnections(outputCommonState state);
static void sendFileHeader(outputCommonState state);
static void sendNetworkHeader(outputCommonState state, int *onlyOneClientFD);
static int outputHandlerThread(void *stateArg);
static void caerOutputCommonConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);

int caerOutputCommonGetServerFd(void *statePtr) {
	outputCommonState state = statePtr;

	return (state->fileDescriptors->serverFd);
}

outputCommonFDs caerOutputCommonAllocateFdArray(size_t size) {
	// Allocate memory for file descriptor array structure.
	outputCommonFDs fileDescriptors = malloc(sizeof(*fileDescriptors) + (size * sizeof(int)));
	if (fileDescriptors == NULL) {
		return (NULL);
	}

	// Initialize all values, FDs to -1.
	fileDescriptors->fdsSize = size;

	fileDescriptors->serverFd = -1;

	for (size_t i = 0; i < size; i++) {
		fileDescriptors->fds[i] = -1;
	}

	return (fileDescriptors);
}

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
			int16_t sourceID = I16T(atomic_load_explicit(&state->sourceID, memory_order_relaxed));

			if (sourceID == -1) {
				state->sourceInfoNode = caerMainloopGetSourceInfo(U16T(eventSource));
				if (state->sourceInfoNode == NULL) {
					// This should never happen, but we handle it gracefully.
					caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
						"Failed to get source info to setup output module.");
					return;
				}

				atomic_store(&state->sourceID, eventSource); // Remember this!
			}
			else if (sourceID != eventSource) {
				caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
					"An output module can only handle packets from the same source! "
						"A packet with source %" PRIi16 " was sent, but this output module expects only packets from source %" PRIi16 ".",
					eventSource, sourceID);
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

	// Now copy each event packet and send the array out. Track how many packets there are.
	int32_t idx = 0;

	for (size_t i = 0; i < packetsSize; i++) {
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
			idx++;
		}
	}

	// We might have failed to copy all packets (unlikely).
	if (idx == 0) {
		caerEventPacketContainerFree(eventPackets);

		return;
	}

	// Reset packet container size so we only consider the packets we managed
	// to successfully copy.
	caerEventPacketContainerSetEventPacketsNumber(eventPackets, idx);

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

static int packetsFirstTimestampThenTypeCmp(const void *a, const void *b) {
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

	if (state->dataBuffer != NULL && state->dataBuffer->bufferSize == newBufferSize) {
		// Yeah, we're already where we want to be!
		return (true);
	}

	// Allocate new buffer.
	simpleBuffer newBuffer = simpleBufferInit(newBufferSize);
	if (newBuffer == NULL) {
		return (false);
	}

	// Commit previous buffer content and then free the memory.
	if (state->dataBuffer != NULL) {
		commitOutputBuffer(state);

		free(state->dataBuffer);
	}

	// Use new buffer.
	state->dataBuffer = newBuffer;

	return (true);
}

static inline void writeBufferToAll(outputCommonState state, const uint8_t *buffer, size_t bufferSize) {
	for (size_t i = 0; i < state->fileDescriptors->fdsSize; i++) {
		int fd = state->fileDescriptors->fds[i];

		if (fd >= 0) {
			if (!writeUntilDone(fd, buffer, bufferSize)) {
				// Write failed, most of the reasons for that to happen are
				// not recoverable from, so we just disable this file descriptor.
				// This also detects client-side close() for TCP server connections.
				caerLog(CAER_LOG_INFO, state->parentModule->moduleSubSystemString,
					"Disconnect or error on fd %d, closing and removing. Error: %d.", fd, errno);

				close(fd);
				state->fileDescriptors->fds[i] = -1;
			}
		}
	}
}

static void commitOutputBuffer(outputCommonState state) {
	if (state->dataBuffer->bufferUsedSize != 0) {
		writeBufferToAll(state, state->dataBuffer->buffer, state->dataBuffer->bufferUsedSize);

		state->dataBuffer->bufferUsedSize = 0;

		// If message-based protocol, we fill in the now empty buffer with the
		// appropriate header.
		if (state->isNetworkMessageBased) {
			sendNetworkHeader(state, NULL);
		}
	}

	// Update last commit time.
	portable_clock_gettime_monotonic(&state->bufferLastCommitTime);
}

static inline void caerGenericEventSetTimestamp(void *eventPtr, caerEventPacketHeader headerPtr, int32_t timestamp) {
	*((int32_t *) (((uint8_t *) eventPtr) + U64T(caerEventPacketHeaderGetEventTSOffset(headerPtr)))) = htole32(
		timestamp);
}

static inline LodePNGColorType caerFrameEventColorToLodePNG(enum caer_frame_event_color_channels channels) {
	switch (channels) {
		case GRAYSCALE:
			return (LCT_GREY);
			break;

		case RGB:
			return (LCT_RGB);
			break;

		case RGBA:
		default:
			return (LCT_RGBA);
			break;
	}
}

static size_t compressEventPacket(outputCommonState state, caerEventPacketHeader packet, size_t packetSize) {
	// Data compression technique 1: serialize timestamps for event types that tend to repeat them a lot.
	// Currently, this means polarity events.
	if ((state->format & 0x01) && caerEventPacketHeaderGetEventType(packet) == POLARITY_EVENT) {
		// Search for runs of at least 3 events with the same timestamp, and convert them to a special
		// sequence: leave first event unchanged, but mark its timestamp as special by setting the
		// highest bit (bit 31) to one (it is forbidden for timestamps in memory to have that bit set for
		// signed-integer-only language compatibility). Then, for the second event, change its timestamp
		// to a 4-byte integer saying how many more events will follow afterwards with this same timestamp
		// (this is used for decoding), so only their data portion will be given. Then follow with those
		// event's data, back to back, with their timestamps removed.
		// So let's assume there are 6 events with TS=1234. In memory this looks like this:
		// E1(data,ts), E2(data,ts), E3(data,ts), E4(data,ts), E5(data,ts), E6(data,ts)
		// After timestamp serialization compression step:
		// E1(data,ts|0x80000000), E2(data,4), E3(data), E4(data), E5(data), E5(data)
		// This change is only in the data itself, not in the packet headers, so that we can still use the
		// eventCapacity and eventSize fields to calculate memory allocation when doing decompression.
		// As such, to correctly interpret this data, the Format flags must be correctly set. All current
		// file or network formats do specify those as mandatory in their headers, so we can rely on that.
		// Also all event types where this kind of thing makes any sense do have the timestamp as their last
		// data member in their struct, so we can use that information, stored in tsOffset header field,
		// together with eventSize, to come up with a generic implementation applicable to all other event
		// types that satisfy this condition of TS-as-last-member (so we can use that offset as event size).
		// When this is enabled, it requires full iteration thorough the whole event packet, both at
		// compression and at decompression time.
		size_t currPacketOffset = CAER_EVENT_PACKET_HEADER_SIZE; // Start here, no change to header.
		int32_t eventSize = caerEventPacketHeaderGetEventSize(packet);
		int32_t eventTSOffset = caerEventPacketHeaderGetEventTSOffset(packet);

		int32_t lastTS = -1;
		int32_t currTS = -1;
		size_t tsRun = 0;
		bool doMemMove = false; // Initially don't move memory, until we actually shrink the size.

		for (int32_t caerIteratorCounter = 0; caerIteratorCounter <= caerEventPacketHeaderGetEventNumber(packet);
			caerIteratorCounter++) {
			// Iterate until one element past the end, to flush the last run. In that particular case,
			// we don't get a new element or TS, as we'd be past the end of the array.
			if (caerIteratorCounter < caerEventPacketHeaderGetEventNumber(packet)) {
				void *caerIteratorElement = caerGenericEventGetEvent(packet, caerIteratorCounter);

				currTS = caerGenericEventGetTimestamp(caerIteratorElement, packet);
				if (currTS == lastTS) {
					// Increase size of run of same TS events currently being seen.
					tsRun++;
					continue;
				}
			}

			// TS are different, at this point look if the last run was long enough
			// and if it makes sense to compress. It does starting with 3 events.
			if (tsRun >= 3) {
				// First event to remains there, we set its TS highest bit.
				uint8_t *firstEvent = caerGenericEventGetEvent(packet, caerIteratorCounter - (int32_t) tsRun--);
				caerGenericEventSetTimestamp(firstEvent, packet,
					caerGenericEventGetTimestamp(firstEvent, packet) | I32T(0x80000000));

				// Now use second event's timestamp for storing how many further events.
				uint8_t *secondEvent = caerGenericEventGetEvent(packet, caerIteratorCounter - (int32_t) tsRun--);
				caerGenericEventSetTimestamp(secondEvent, packet, I32T(tsRun)); // Is at least 1.

				// Finally move modified memory where it needs to go.
				if (doMemMove) {
					memmove(((uint8_t *) packet) + currPacketOffset, firstEvent, (size_t) eventSize * 2);
				}
				else {
					doMemMove = true; // After first shrink always move memory.
				}
				currPacketOffset += (size_t) eventSize * 2;

				// Now go thorugh remaining events and move their data close together.
				while (tsRun > 0) {
					uint8_t *thirdEvent = caerGenericEventGetEvent(packet, caerIteratorCounter - (int32_t) tsRun--);
					memmove(((uint8_t *) packet) + currPacketOffset, thirdEvent, (size_t) eventTSOffset);
					currPacketOffset += (size_t) eventTSOffset;
				}
			}
			else {
				// Just copy data unchanged if no compression is possible.
				if (doMemMove) {
					uint8_t *startEvent = caerGenericEventGetEvent(packet, caerIteratorCounter - (int32_t) tsRun);
					memmove(((uint8_t *) packet) + currPacketOffset, startEvent, (size_t) eventSize * tsRun);
				}
				currPacketOffset += (size_t) eventSize * tsRun;
			}

			// Reset values for next iteration.
			lastTS = currTS;
			tsRun = 1;
		}

		return (currPacketOffset);
	}

	// Data compression technique 2: do PNG compression on frames, Grayscale and RGB(A).
	if ((state->format & 0x02) && caerEventPacketHeaderGetEventType(packet) == FRAME_EVENT) {
		size_t currPacketOffset = CAER_EVENT_PACKET_HEADER_SIZE; // Start here, no change to header.
		size_t frameHeaderSize = sizeof(struct caer_frame_event);

		CAER_FRAME_ITERATOR_ALL_START((caerFrameEventPacket) packet)
			size_t pixelSize = caerFrameEventGetPixelsSize(caerFrameIteratorElement);

			// Keep frame event header intact, compress image data, move memory close together.
			memmove(((uint8_t *) packet) + currPacketOffset, caerFrameIteratorElement, frameHeaderSize);
			currPacketOffset += frameHeaderSize;

			unsigned char *outBuffer;
			size_t outSize;
			if (lodepng_encode_memory(&outBuffer, &outSize,
				(unsigned char *) caerFrameEventGetPixelArrayUnsafe(caerFrameIteratorElement),
				U32T(caerFrameEventGetLengthX(caerFrameIteratorElement)),
				U32T(caerFrameEventGetLengthY(caerFrameIteratorElement)),
				caerFrameEventColorToLodePNG(caerFrameEventGetChannelNumber(caerFrameIteratorElement)), 16) != 0) {
				// Failed to generate PNG.
				// TODO: how to handle?
			}

			// Check that the image didn't actually grow.
			// Add integer needed for storing PNG block length.
			if ((outSize + sizeof(int32_t)) > pixelSize) {
				caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Failed to compress frame event. "
					"Image actually grew by %zu bytes to a total of %zu bytes.", (outSize - pixelSize), outSize);
				// TODO: how to handle this?
			}

			// Store size of PNG image block as 4 byte integer.
			int32_t outSizeInt = I32T(outSize);
			memcpy(((uint8_t *) packet) + currPacketOffset, &outSizeInt, sizeof(int32_t));
			currPacketOffset += sizeof(int32_t);

			memcpy(((uint8_t *) packet) + currPacketOffset, outBuffer, outSize);
			currPacketOffset += outSize;
		}

		return (currPacketOffset);
	}

	return (packetSize);
}

static void sendEventPacket(outputCommonState state, caerEventPacketHeader packet) {
	// Calculate total size of packet, in bytes.
	size_t packetSize = CAER_EVENT_PACKET_HEADER_SIZE
		+ (size_t) (caerEventPacketHeaderGetEventCapacity(packet) * caerEventPacketHeaderGetEventSize(packet));

	// Statistics support.
	state->statistics.packetsNumber++;
	state->statistics.packetsTotalSize += packetSize;
	state->statistics.packetsHeaderSize += CAER_EVENT_PACKET_HEADER_SIZE;
	state->statistics.packetsDataSize += (size_t) (caerEventPacketHeaderGetEventCapacity(packet)
		* caerEventPacketHeaderGetEventSize(packet));

	if (state->format != 0) {
		packetSize = compressEventPacket(state, packet, packetSize);
	}

	// Statistics support (after compression).
	state->statistics.dataWritten += packetSize;

	// Send it out until none is left!
	size_t packetIndex = 0;

	while (packetSize > 0) {
		// Calculate remaining space in current buffer.
		size_t usableBufferSpace = state->dataBuffer->bufferSize - state->dataBuffer->bufferUsedSize;

		// Let's see how much of it (or all of it!) we need.
		if (packetSize < usableBufferSpace) {
			usableBufferSpace = packetSize;
		}

		// Copy memory from packet to buffer.
		memcpy(state->dataBuffer->buffer + state->dataBuffer->bufferUsedSize, ((uint8_t *) packet) + packetIndex,
			usableBufferSpace);

		// Update indexes.
		state->dataBuffer->bufferUsedSize += usableBufferSpace;
		packetIndex += usableBufferSpace;
		packetSize -= usableBufferSpace;

		if (state->dataBuffer->bufferUsedSize == state->dataBuffer->bufferSize) {
			// Commit buffer once full.
			commitOutputBuffer(state);
		}
	}

	// Each commit operation updates the last committed buffer time.
	// The above code resulted in some commits, with the time being updated,
	// or in no commits at all, with the time remaining as before.
	// Here we check that the time difference between now and the last actual
	// commit doesn't exceed the allowed maximum interval.
	struct timespec currentTime;
	portable_clock_gettime_monotonic(&currentTime);

	uint64_t diffNanoTime = (uint64_t) (((int64_t) (currentTime.tv_sec - state->bufferLastCommitTime.tv_sec)
		* 1000000000LL) + (int64_t) (currentTime.tv_nsec - state->bufferLastCommitTime.tv_nsec));

	// DiffNanoTime is the difference in nanoseconds; we want to trigger after
	// the user provided interval has elapsed (also in nanoseconds).
	if (diffNanoTime >= state->bufferMaxInterval) {
		commitOutputBuffer(state);
	}
}

static void orderAndSendEventPackets(outputCommonState state, caerEventPacketContainer currPacketContainer) {
	// Sort container by first timestamp (required) and by type ID (convenience).
	size_t currPacketContainerSize = (size_t) caerEventPacketContainerGetEventPacketsNumber(currPacketContainer);

	qsort(currPacketContainer->eventPackets, currPacketContainerSize, sizeof(caerEventPacketHeader),
		&packetsFirstTimestampThenTypeCmp);

	// Since we just got new data, let's first check that it does conform to our expectations.
	// This means the timestamp didn't slide back! So new smallest TS is >= than last highest TS.
	// These checks are needed to avoid illegal ordering. Normal operation will never trigger
	// these, as stated in the assumptions at the start of file, but erroneous usage or mixing
	// or reordering of packet containers is possible, and has to be caught here.
	int64_t highestTimestamp = 0;

	for (size_t cpIdx = 0; cpIdx < currPacketContainerSize; cpIdx++) {
		caerEventPacketHeader cpPacket = caerEventPacketContainerGetEventPacket(currPacketContainer, (int32_t) cpIdx);

		void *cpFirstEvent = caerGenericEventGetEvent(cpPacket, 0);
		int64_t cpFirstEventTimestamp = caerGenericEventGetTimestamp64(cpFirstEvent, cpPacket);

		if (cpFirstEventTimestamp < state->lastTimestamp) {
			// Smaller TS than already sent, illegal, ignore packet.
			caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
				"Detected timestamp going back, expected at least %" PRIi64 " but got %" PRIi64 "."
				" Ignoring packet of type %" PRIi16 " from source %" PRIi16 ", with %" PRIi32 " events!",
				state->lastTimestamp, cpFirstEventTimestamp, caerEventPacketHeaderGetEventType(cpPacket),
				caerEventPacketHeaderGetEventSource(cpPacket), caerEventPacketHeaderGetEventNumber(cpPacket));
		}
		else {
			// Bigger or equal TS than already sent, this is good. Strict TS ordering ensures
			// that all other packets in this container are the same, so we can start sending
			// the packets from here on out to the file descriptor.
			sendEventPacket(state, cpPacket);

			// Update highest timestamp for this packet container, based upon its valid packets.
			void *cpLastEvent = caerGenericEventGetEvent(cpPacket, caerEventPacketHeaderGetEventNumber(cpPacket) - 1);
			int64_t cpLastEventTimestamp = caerGenericEventGetTimestamp64(cpLastEvent, cpPacket);

			if (cpLastEventTimestamp > highestTimestamp) {
				highestTimestamp = cpLastEventTimestamp;
			}
		}
	}

	// Remember highest timestamp for check in next iteration.
	state->lastTimestamp = highestTimestamp;
}

static void handleNewServerConnections(outputCommonState state) {
	// First let's see if any new connections are waiting on the listening
	// socket to be accepted. This returns right away (non-blocking).
	socklen_t clientAddrLength = sizeof(struct sockaddr_in);
	struct sockaddr_in clientAddr;
	memset(&clientAddr, 0, clientAddrLength);

	int acceptResult = accept(state->fileDescriptors->serverFd, (struct sockaddr *) &clientAddr, &clientAddrLength);
	if (acceptResult < 0) {
		if (errno != EAGAIN && errno != EWOULDBLOCK) {
			// Only log real failure. EAGAIN/EWOULDBLOCK just means no
			// connections are present for non-blocking accept() right now.
			caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
				"TCP server accept() failed. Error: %d.", errno);
		}

		return;
	}

	// New connection present!
	// Put it in the list of FDs and send header, if there is space left, or close.
	for (size_t i = 0; i < state->fileDescriptors->fdsSize; i++) {
		if (state->fileDescriptors->fds[i] == -1) {
			caerLog(CAER_LOG_DEBUG, state->parentModule->moduleSubSystemString,
				"Accepted new TCP connection from client (fd %d).", acceptResult);

			// Empty place in FD list, add this one.
			state->fileDescriptors->fds[i] = acceptResult;

			// Successfully connected, send header to client.
			sendNetworkHeader(state, &state->fileDescriptors->fds[i]);

			// Add client IP to list. This is a comma separated string.
			char *connectedClientsStr = sshsNodeGetString(state->parentModule->moduleNode, "connectedClients");

			size_t newConnectedClientStrLength = strlen(connectedClientsStr) + 1 + INET_ADDRSTRLEN + 1;
			char *newConnectedClientsStr = realloc(connectedClientsStr, newConnectedClientStrLength);
			if (newConnectedClientsStr == NULL) {
				free(connectedClientsStr);

				caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
					"Failed to update connectedClients information.");
				return;
			}

			const char *clientAddrStr = inet_ntop(AF_INET, &clientAddr.sin_addr, (char[INET_ADDRSTRLEN] ) { 0x00 },
				INET_ADDRSTRLEN);

			if (!caerStrEquals(newConnectedClientsStr, "")) {
				// Only prepend comma if the string wasn't empty before.
				strncat(newConnectedClientsStr, ",", 1);
			}
			strncat(newConnectedClientsStr, clientAddrStr, INET_ADDRSTRLEN);

			sshsNodePutString(state->parentModule->moduleNode, "connectedClients", newConnectedClientsStr);

			free(newConnectedClientsStr);

			return;
		}
	}

	// No space for new connection, just close it (client will exit).
	caerLog(CAER_LOG_DEBUG, state->parentModule->moduleSubSystemString,
		"Rejected TCP client (fd %d), max connections reached.", acceptResult);

	close(acceptResult);
}

static void sendFileHeader(outputCommonState state) {
	// Write AEDAT 3.1 header (RAW format).
	writeBufferToAll(state, (const uint8_t *) "#!AER-DAT" AEDAT3_FILE_VERSION "\r\n", 11 + strlen(AEDAT3_FILE_VERSION));

	writeBufferToAll(state, (const uint8_t *) "#Format: RAW\r\n", 14);

	char *sourceString = sshsNodeGetString(state->sourceInfoNode, "sourceString");
	writeBufferToAll(state, (const uint8_t *) sourceString, strlen(sourceString));
	free(sourceString);

	time_t currentTimeEpoch = time(NULL);
	tzset();
	struct tm currentTime;
	localtime_r(&currentTimeEpoch, &currentTime);

	// Following time format uses exactly 44 characters (25 separators/characters,
	// 4 year, 2 month, 2 day, 2 hours, 2 minutes, 2 seconds, 5 time-zone).
	size_t currentTimeStringLength = 44;
	char currentTimeString[currentTimeStringLength + 1]; // + 1 for terminating NUL byte.
	strftime(currentTimeString, currentTimeStringLength + 1, "#Start-Time: %Y-%m-%d %H:%M:%S (TZ%z)\r\n", &currentTime);

	writeBufferToAll(state, (const uint8_t *) currentTimeString, currentTimeStringLength);

	writeBufferToAll(state, (const uint8_t *) "#!END-HEADER\r\n", 14);
}

static void sendNetworkHeader(outputCommonState state, int *onlyOneClientFD) {
	// Send AEDAT 3.1 header (RAW format) for network streams (20 bytes total).
	int64_t magicNumber = htole64(AEDAT3_NETWORK_MAGIC_NUMBER);

	int64_t sequenceNumber = htole64(state->networkSequenceNumber);

	int8_t versionNumber = AEDAT3_NETWORK_VERSION;
	int8_t formatNumber = 0x00; // RAW format.
	int16_t sourceNumber = htole16(1); // Always one source per output module.

	uint8_t networkHeader[AEDAT3_NETWORK_HEADER_LENGTH] = { 0 };

	*((int64_t *) (networkHeader + 0)) = magicNumber;
	*((int64_t *) (networkHeader + 8)) = sequenceNumber;
	*((int8_t *) (networkHeader + 16)) = versionNumber;
	*((int8_t *) (networkHeader + 17)) = formatNumber;
	*((int16_t *) (networkHeader + 18)) = sourceNumber;

	// If message-based, we copy the header at the start of the buffer,
	// because we want it in each message (and each buffer is a message!).
	if (state->isNetworkMessageBased) {
		memcpy(state->dataBuffer->buffer, networkHeader, AEDAT3_NETWORK_HEADER_LENGTH);
		state->dataBuffer->bufferUsedSize = AEDAT3_NETWORK_HEADER_LENGTH;

		// Increase sequence number for successive headers, if this is a
		// message-based network protocol (UDP for example).
		state->networkSequenceNumber++;
	}
	else {
		// Else, not message-based, so we just write it once at start directly.
		// We support writing to all clients, or only to one specified client.
		// This one-client mode is only used for server mode operation.
		if (onlyOneClientFD != NULL && *onlyOneClientFD >= 0) {
			if (!writeUntilDone(*onlyOneClientFD, networkHeader, AEDAT3_NETWORK_HEADER_LENGTH)) {
				caerLog(CAER_LOG_INFO, state->parentModule->moduleSubSystemString,
					"Disconnect or error on fd %d, closing and removing. Error: %d.", *onlyOneClientFD, errno);

				close(*onlyOneClientFD);
				*onlyOneClientFD = -1;
			}
		}
		else {
			writeBufferToAll(state, networkHeader, AEDAT3_NETWORK_HEADER_LENGTH);
		}
	}
}

static int outputHandlerThread(void *stateArg) {
	outputCommonState state = stateArg;

	// Set thread name.
	thrd_set_name(state->parentModule->moduleSubSystemString);

	bool headerSent = false;

	while (atomic_load_explicit(&state->running, memory_order_relaxed)) {
		// Wait for source to be defined.
		int16_t sourceID = I16T(atomic_load_explicit(&state->sourceID, memory_order_relaxed));
		if (sourceID == -1) {
			continue;
		}

		// Send appropriate header.
		if (state->isNetworkStream) {
			sendNetworkHeader(state, NULL);
		}
		else {
			sendFileHeader(state);
		}

		headerSent = true;
		break;
	}

	// If no header sent, it means we exited (running=false) without ever getting any
	// event packet with a source ID, so we don't have to process anything.
	// But we make sure to empty the transfer ring-buffer, as something may have been
	// put there in the meantime, so we ensure it's checked and freed.
	if (!headerSent) {
		caerEventPacketContainer packetContainer;
		while ((packetContainer = ringBufferGet(state->transferRing)) != NULL) {
			// Free all remaining packet container memory.
			caerEventPacketContainerFree(packetContainer);
		}

		return (thrd_success);
	}

	// If no data is available on the transfer ring-buffer, sleep for 500µs (0.5 ms)
	// to avoid wasting resources in a busy loop.
	struct timespec noDataSleep = { .tv_sec = 0, .tv_nsec = 500000 };

	while (atomic_load_explicit(&state->running, memory_order_relaxed)) {
		// Handle new connections in server mode.
		if (state->isNetworkStream && state->fileDescriptors->serverFd >= 0) {
			handleNewServerConnections(state);
		}

		// Handle configuration changes affecting buffer management.
		if (atomic_load_explicit(&state->bufferUpdate, memory_order_relaxed)) {
			atomic_store(&state->bufferUpdate, false);

			state->bufferMaxInterval = U64T(sshsNodeGetInt(state->parentModule->moduleNode, "bufferMaxInterval"));
			state->bufferMaxInterval *= 1000LLU; // Convert from microseconds to nanoseconds.

			if (!newOutputBuffer(state)) {
				caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
					"Failed to allocate new output data buffer. Continue using old one.");
			}
		}

		// Fill output data buffer with data from incoming packets.
		// Respect time order as specified in AEDAT 3.X format: first event's main
		// timestamp decides its ordering with regards to other packets. Smaller
		// comes first. If equal, order by increasing type ID as a convenience,
		// not strictly required by specification!

		// Get the newest event packet container from the transfer ring-buffer.
		caerEventPacketContainer currPacketContainer = ringBufferGet(state->transferRing);
		if (currPacketContainer == NULL) {
			// There is none, so we can't work on and commit this.
			// We just sleep here a little and then try again, as we need the data!
			thrd_sleep(&noDataSleep, NULL);
			continue;
		}

		orderAndSendEventPackets(state, currPacketContainer);

		// Free all remaining packet container memory.
		caerEventPacketContainerFree(currPacketContainer);
	}

	// Handle shutdown, write out all content remaining in the transfer ring-buffer
	// and write the packets out to the file descriptor.
	caerEventPacketContainer packetContainer;
	while ((packetContainer = ringBufferGet(state->transferRing)) != NULL) {
		orderAndSendEventPackets(state, packetContainer);

		// Free all remaining packet container memory.
		caerEventPacketContainerFree(packetContainer);
	}

	// Make sure last (incomplete) buffer is sent out.
	commitOutputBuffer(state);

	return (thrd_success);
}

bool caerOutputCommonInit(caerModuleData moduleData, outputCommonFDs fds, bool isNetworkStream,
bool isNetworkMessageBased) {
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

	if (fds->serverFd < -1) {
		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Invalid server file descriptor.");
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

	// Store network/file, message-based or not information.
	state->isNetworkStream = isNetworkStream;
	state->isNetworkMessageBased = isNetworkMessageBased;

	// If in server mode, add SSHS attribute to track connected client IPs.
	if (state->fileDescriptors->serverFd >= 0) {
		sshsNodePutString(state->parentModule->moduleNode, "connectedClients", "");
	}

	// Initial source ID has to be -1 (invalid).
	atomic_store(&state->sourceID, -1);

	// Handle configuration.
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "validOnly", false); // only send valid events
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "keepPackets", false); // ensure all packets are kept
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "bufferSize", 16384); // in bytes, size of data buffer
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "bufferMaxInterval", 20000); // in µs, max. interval without sending data
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "transferBufferSize", 128); // in packet groups

	atomic_store(&state->validOnly, sshsNodeGetBool(moduleData->moduleNode, "validOnly"));
	atomic_store(&state->keepPackets, sshsNodeGetBool(moduleData->moduleNode, "keepPackets"));
	state->bufferMaxInterval = U64T(sshsNodeGetInt(moduleData->moduleNode, "bufferMaxInterval"));
	state->bufferMaxInterval *= 1000LLU; // Convert from microseconds to nanoseconds.

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

	// Initialize to current time.
	portable_clock_gettime_monotonic(&state->bufferLastCommitTime);

	// Start output handling thread.
	atomic_store(&state->running, true);

	if (thrd_create(&state->outputThread, &outputHandlerThread, state) != thrd_success) {
		ringBufferFree(state->transferRing);
		free(state->dataBuffer);

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

	// Now clean up the transfer ring-buffer and its contents.
	caerEventPacketContainer packetContainer;
	while ((packetContainer = ringBufferGet(state->transferRing)) != NULL) {
		caerEventPacketContainerFree(packetContainer);

		// This should never happen!
		caerLog(CAER_LOG_CRITICAL, state->parentModule->moduleSubSystemString, "Transfer ring-buffer was not empty!");
	}

	ringBufferFree(state->transferRing);

	// Close file descriptors.
	for (size_t i = 0; i < state->fileDescriptors->fdsSize; i++) {
		int fd = state->fileDescriptors->fds[i];

		if (fd >= 0) {
			close(fd);
		}
	}

	if (state->fileDescriptors->serverFd >= 0) {
		close(state->fileDescriptors->serverFd);

		// Server shut down, no more clients.
		sshsNodePutString(state->parentModule->moduleNode, "connectedClients", "");
	}

	// Free allocated memory.
	free(state->fileDescriptors);

	free(state->dataBuffer);

	// Print final statistics results.
	caerLog(CAER_LOG_INFO, state->parentModule->moduleSubSystemString,
		"Statistics: wrote %" PRIu64 " packets, for a total uncompressed size of %" PRIu64 " bytes (%" PRIu64 " bytes header + %" PRIu64 " bytes data). "
		"Actually written to output were %" PRIu64 " bytes (after compression), resulting in a saving of %" PRIu64 " bytes.",
		state->statistics.packetsNumber, state->statistics.packetsTotalSize, state->statistics.packetsHeaderSize,
		state->statistics.packetsDataSize, state->statistics.dataWritten,
		(state->statistics.packetsTotalSize - state->statistics.dataWritten));
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
