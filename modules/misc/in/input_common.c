#include "input_common.h"
#include "base/mainloop.h"
#include "ext/c11threads_posix.h"
#include "ext/portable_time.h"
#include "ext/ringbuffer/ringbuffer.h"
#include "ext/buffers.h"

#include <libcaer/events/common.h>
#include <libcaer/events/packetContainer.h>

#define MAX_HEADER_LINE_SIZE 1024
#define STD_PACKET_SIZE 10240

struct input_common_header_info {
	bool isValidHeader;
	bool isAEDAT3;
	int16_t majorVersion;
	int8_t minorVersion;
	int8_t formatID;
	int64_t lastSequenceNumber;
};

struct input_common_state {
	/// Control flag for input handling thread.
	atomic_bool running;
	/// The input handling thread (separate as to only wake up mainloop
	/// processing when there is new data available).
	thrd_t inputThread;
	/// Track source ID (cannot change!) to read data for. One source per I/O module!
	int16_t sourceID;
	/// The file descriptor for reading.
	int fileDescriptor;
	/// Network-like stream or file-like stream. Matters for header format.
	bool isNetworkStream;
	/// For network-like inputs, we differentiate between stream and message
	/// based protocols, like TCP and UDP. Matters for header/sequence number.
	bool isNetworkMessageBased;
	/// Keep track of the sequence number for message-based protocols.
	int64_t networkSequenceNumber;
	/// Filter out invalidated events or not.
	atomic_bool validOnly;
	/// Force all incoming packets to be committed to the transfer ring-buffer.
	/// This results in no loss of data, but may deviate from the requested
	/// real-time play-back expectations.
	atomic_bool keepPackets;
	/// Transfer packets coming from the input handling thread to the mainloop.
	/// We use EventPacketContainers, as that is the standard data structure
	/// returned from an input module.
	RingBuffer transferRing;
	/// Header parsing results.
	struct input_common_header_info header;
	/// Track last packet container's highest event timestamp that was sent out.
	int64_t lastTimestamp;
	/// Data buffer for reading from file descriptor (buffered I/O).
	simpleBuffer dataBuffer;
	/// Flag to signal update to buffer configuration asynchronously.
	atomic_bool bufferUpdate;
	/// Reference to parent module's original data.
	caerModuleData parentModule;
	/// Reference to module's mainloop (for data availability signaling).
	caerMainloopData mainloopReference;
};

typedef struct input_common_state *inputCommonState;

size_t CAER_INPUT_COMMON_STATE_STRUCT_SIZE = sizeof(struct input_common_state);

static bool newInputBuffer(inputCommonState state);
static bool parseNetworkHeader(inputCommonState state);
static char *getFileHeaderLine(inputCommonState state);
static bool parseFileHeader(inputCommonState state);
static bool parseHeader(inputCommonState state);
static bool parsePackets(inputCommonState state);
static int inputHandlerThread(void *stateArg);
static void caerInputCommonConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);

static bool newInputBuffer(inputCommonState state) {
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
		// We just free here, there's nothing to do, since the buffer can only get
		// reallocated when it's empty (either at start or after it has been read).
		free(state->dataBuffer);
	}

	// Use new buffer.
	state->dataBuffer = newBuffer;

	return (true);
}

static bool parseNetworkHeader(inputCommonState state) {
	// TODO: handle network header.
	return (true);
}

static char *getFileHeaderLine(inputCommonState state) {
	simpleBuffer buf = state->dataBuffer;

	if (buf->buffer[buf->bufferPosition] == '#') {
		size_t headerLinePos = 0;
		char *headerLine = malloc(MAX_HEADER_LINE_SIZE);
		if (headerLine == NULL) {
			// Failed to allocate memory.
			return (NULL);
		}

		headerLine[headerLinePos++] = '#';
		buf->bufferPosition++;

		while (buf->buffer[buf->bufferPosition] != '\n') {
			if (headerLinePos >= (MAX_HEADER_LINE_SIZE - 2)) { // -1 for terminating new-line, -1 for end NUL char.
				// Overlong header line, refuse it.
				free(headerLine);
				return (NULL);
			}

			headerLine[headerLinePos++] = (char) buf->buffer[buf->bufferPosition];
			buf->bufferPosition++;
		}

		// Found terminating new-line character.
		headerLine[headerLinePos++] = '\n';
		buf->bufferPosition++;

		// Now let's just verify that the previous character was indeed a carriage-return.
		if (headerLine[headerLinePos - 2] == '\r') {
			// Valid, terminate it and return it.
			headerLine[headerLinePos] = '\0';

			return (headerLine);
		}
		else {
			// Invalid header line. No Windows line-ending.
			free(headerLine);
			return (NULL);
		}
	}

	// Invalid header line. Doesn't begin with #.
	return (NULL);
}

static bool parseFileHeader(inputCommonState state) {
	// We expect that the full header part is contained within
	// this one data buffer.
	// File headers are part of the AEDAT 3.X specification.
	// Start with #, go until '\r\n' (Windows EOL). First must be
	// version header !AER-DATx.y, last must be end-of-header
	// marker with !END-HEADER.
	bool versionHeader = false;
	bool formatHeader = false;
	bool endHeader = false;

	while (!endHeader) {
		char *headerLine = getFileHeaderLine(state);
		if (headerLine == NULL) {
			// Failed to parse header line; this is an invalid header for AEDAT 3.1!
			// For AEDAT 2.0, since there is no END-HEADER, this might be
			// the right way for headers to stop, so we consider this valid IFF we
			// already got at least the version header.
			if (versionHeader && !state->header.isAEDAT3) {
				// Got an AEDAT 2.0 version header, that's fine.
				state->header.isValidHeader = true;
				return (true);
			}

			return (false);
		}

		if (!versionHeader) {
			// First thing we expect is the version header. We don't support files not having it.
			if (sscanf(headerLine, "#!AER-DAT%" SCNi16 ".%" SCNi8 "\r\n", &state->header.majorVersion,
				&state->header.minorVersion) == 2) {
				versionHeader = true;

				// Check valid versions.
				switch (state->header.majorVersion) {
					case 2:
						// AEDAT 2.0 is supported. No revisions exist.
						if (state->header.minorVersion != 0) {
							goto noValidVersionHeader;
						}
						break;

					case 3:
						state->header.isAEDAT3 = true;

						// AEDAT 3.1 is supported. AEDAT 3.0 was not in active use.
						if (state->header.minorVersion != 1) {
							goto noValidVersionHeader;
						}
						break;

					default:
						// Versions other than 2.0 and 3.1 are not supported.
						goto noValidVersionHeader;
						break;
				}

				caerLog(CAER_LOG_DEBUG, state->parentModule->moduleSubSystemString,
					"Found AEDAT%" PRIi16 ".%" PRIi8 " version header.", state->header.majorVersion,
					state->header.minorVersion);
			}
			else {
				noValidVersionHeader: free(headerLine);

				caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
					"No compliant AEDAT version header found. Invalid file.");
				return (false);
			}
		}
		else if (state->header.isAEDAT3 && !formatHeader) {
			// Then the format header. Only with AEDAT 3.X.
			char *formatString = NULL;

			if (sscanf(headerLine, "#Format: %ms\r\n", &formatString) == 1) {
				formatHeader = true;

				// Parse format string to format ID.
				if (caerStrEquals(formatString, "RAW")) {
					state->header.formatID = 0x00;
				}
				else if (caerStrEquals(formatString, "Serial-TS")) {
					state->header.formatID = 0x01;
				}
				else if (caerStrEquals(formatString, "Compressed")) {
					state->header.formatID = 0x02;
				}
				else {
					// No valid format found.
					free(headerLine);

					caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
						"No compliant Format type found. Format '%s' is invalid.", formatString);

					free(formatString);
					return (false);
				}

				caerLog(CAER_LOG_DEBUG, state->parentModule->moduleSubSystemString,
					"Found Format header with value '%s', Format ID %" PRIi8 ".", formatString, state->header.formatID);

				free(formatString);
			}
			else {
				free(headerLine);

				caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
					"No compliant Format header found. Invalid file.");
				return (false);
			}
		}
		else {
			// Now we either have other header lines with AEDAT 2.0, or the END-HEADER with
			// AEDAT 3.1. We check this before the other possible header lines.
			if (caerStrEquals(headerLine, "#!END-HEADER\r\n")) {
				endHeader = true;

				caerLog(CAER_LOG_DEBUG, state->parentModule->moduleSubSystemString, "Found END-HEADER header.");
			}
			else {
				// Then other headers, like Source or Start-Time. We only support one active source.
				// TODO: parse these.
				caerLog(CAER_LOG_DEBUG, state->parentModule->moduleSubSystemString, "Header line: '%s'.", headerLine);
			}
		}

		free(headerLine);
	}

	state->header.isValidHeader = true;
	return (true);
}

static bool parseHeader(inputCommonState state) {
	if (state->isNetworkStream) {
		return (parseNetworkHeader(state));
	}
	else {
		return (parseFileHeader(state));
	}
}

static bool parsePackets(inputCommonState state) {
	if (!state->header.isAEDAT3) {
		// TODO: AEDAT 2.0 not yet supported.
		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
			"Reading AEDAT 2.0 data not yet supported.");
		return (false);
	}

	// Signal availability of new data to the mainloop on packet container commit.
	//atomic_fetch_add_explicit(&state->mainloopReference->dataAvailable, 1, memory_order_release);

	return (true);
}

static int inputHandlerThread(void *stateArg) {
	inputCommonState state = stateArg;

	struct timespec waitSleep = { .tv_sec = 0, .tv_nsec = 1000000 }; // Wait 1ms.

	while (atomic_load_explicit(&state->running, memory_order_relaxed)) {
		// Handle configuration changes affecting buffer management.
		if (atomic_load_explicit(&state->bufferUpdate, memory_order_relaxed)) {
			atomic_store(&state->bufferUpdate, false);

			if (!newInputBuffer(state)) {
				caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
					"Failed to allocate new input data buffer. Continue using old one.");
			}
		}

		// Read data from disk or socket.
		if (!simpleBufferRead(state->fileDescriptor, state->dataBuffer)) {
			// Error or EOF with no data. Let's just stop at this point.
			close(state->fileDescriptor);
			state->fileDescriptor = -1;

			// Distinguish EOF from errors based upon errno value.
			if (errno == 0) {
				caerLog(CAER_LOG_INFO, state->parentModule->moduleSubSystemString, "End of file reached.");
			}
			else {
				caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
					"Error while reading data, error: %d.", errno);
			}
			break;
		}

		// Parse header and setup header info structure.
		if (!state->header.isValidHeader && !parseHeader(state)) {
			// Header invalid, exit.
			caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Failed to parse header.");
			break;
		}

		// Parse event packets now.
		if (!parsePackets(state)) {
			// Packets invalid, exit.
			caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Failed to parse event packets.");
			break;
		}

		// Go and get a full buffer on next iteration again, starting at position 0.
		state->dataBuffer->bufferPosition = 0;
	}

	// Ensure parent also shuts down, for example on read failures.
	sshsNodePutBool(state->parentModule->moduleNode, "running", false);

	return (thrd_success);
}

bool caerInputCommonInit(caerModuleData moduleData, int readFd, bool isNetworkStream,
bool isNetworkMessageBased) {
	inputCommonState state = moduleData->moduleState;

	state->parentModule = moduleData;
	state->mainloopReference = caerMainloopGetReference();

	// Check for invalid file descriptors.
	if (readFd < -1) {
		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Invalid file descriptor.");
		return (false);
	}

	state->fileDescriptor = readFd;

	// Store network/file, message-based or not information.
	state->isNetworkStream = isNetworkStream;
	state->isNetworkMessageBased = isNetworkMessageBased;

	// Initial source ID has to be -1 (invalid).
	state->sourceID = -1;

	// Handle configuration.
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "validOnly", false); // only send valid events
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "keepPackets", false); // ensure all packets are kept
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "bufferSize", 65536); // in bytes, size of data buffer
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "transferBufferSize", 128); // in packet groups

	atomic_store(&state->validOnly, sshsNodeGetBool(moduleData->moduleNode, "validOnly"));
	atomic_store(&state->keepPackets, sshsNodeGetBool(moduleData->moduleNode, "keepPackets"));

	// Initialize transfer ring-buffer. transferBufferSize only changes here at init time!
	state->transferRing = ringBufferInit((size_t) sshsNodeGetInt(moduleData->moduleNode, "transferBufferSize"));
	if (state->transferRing == NULL) {
		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Failed to allocate transfer ring-buffer.");
		return (false);
	}

	// Allocate data buffer. bufferSize is updated here.
	if (!newInputBuffer(state)) {
		ringBufferFree(state->transferRing);

		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Failed to allocate input data buffer.");
		return (false);
	}

	// Start input handling thread.
	atomic_store(&state->running, true);

	if (thrd_create(&state->inputThread, &inputHandlerThread, state) != thrd_success) {
		ringBufferFree(state->transferRing);
		free(state->dataBuffer);

		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Failed to start input handling thread.");
		return (false);
	}

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerInputCommonConfigListener);

	return (true);
}

void caerInputCommonExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerInputCommonConfigListener);

	inputCommonState state = moduleData->moduleState;

	// Stop input thread and wait on it.
	atomic_store(&state->running, false);

	if ((errno = thrd_join(state->inputThread, NULL)) != thrd_success) {
		// This should never happen!
		caerLog(CAER_LOG_CRITICAL, state->parentModule->moduleSubSystemString,
			"Failed to join input handling thread. Error: %d.", errno);
	}

	// Now clean up the transfer ring-buffer and its contents.
	caerEventPacketContainer packetContainer;
	while ((packetContainer = ringBufferGet(state->transferRing)) != NULL) {
		caerEventPacketContainerFree(packetContainer);

		atomic_fetch_sub_explicit(&state->mainloopReference->dataAvailable, 1, memory_order_relaxed);
	}

	ringBufferFree(state->transferRing);

	// Close file descriptors.
	if (state->fileDescriptor >= 0) {
		close(state->fileDescriptor);
	}

	// Free allocated memory.
	free(state->dataBuffer);
}

void caerInputCommonRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	inputCommonState state = moduleData->moduleState;

	// Interpret variable arguments (same as above in main function).
	caerEventPacketContainer *container = va_arg(args, caerEventPacketContainer *);

	*container = ringBufferGet(state->transferRing);

	if (*container != NULL) {
		// Got a container, set it up for auto-reclaim and signal it's not available anymore.
		caerMainloopFreeAfterLoop((void (*)(void *)) &caerEventPacketContainerFree, *container);

		// No special memory order for decrease, because the acquire load to even start running
		// through a mainloop already synchronizes with the release store above.
		atomic_fetch_sub_explicit(&state->mainloopReference->dataAvailable, 1, memory_order_relaxed);
	}
}

static void caerInputCommonConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	caerModuleData moduleData = userData;
	inputCommonState state = moduleData->moduleState;

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
	}
}
