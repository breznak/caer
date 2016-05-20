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
	/// Header has been completely read and is valid.
	bool isValidHeader;
	/// Format is AEDAT 3.
	bool isAEDAT3;
	/// Major AEDAT format version (X.y).
	int16_t majorVersion;
	/// Minor AEDAT format version (x.Y)
	int8_t minorVersion;
	/// AEDAT 3 Format ID (from Format header), used for decoding.
	int8_t formatID;
	/// Track source ID (cannot change!) to read data for. One source per I/O module!
	int16_t sourceID;
	/// Keep track of the sequence number for message-based protocols.
	int64_t networkSequenceNumber;
};

struct input_common_packet_data {
	/// Track last packet container's highest event timestamp that was sent out.
	int64_t lastTimestamp;
	/// Current packet header, to support headers being split across buffers.
	uint8_t currPacketHeader[CAER_EVENT_PACKET_HEADER_SIZE];
	/// Current packet header length (determines if complete or not).
	size_t currPacketHeaderSize;
	/// Current packet, to get filled up with data.
	caerEventPacketHeader currPacket;
	/// Current packet data length.
	size_t currPacketDataSize;
	/// Current packet offset, index into data.
	size_t currPacketDataOffset;
	/// Skip over packets coming from other sources. We only support one!
	size_t skipSize;
};

struct input_common_state {
	/// Control flag for input handling thread.
	atomic_bool running;
	/// The input handling thread (separate as to only wake up mainloop
	/// processing when there is new data available).
	thrd_t inputThread;
	/// The file descriptor for reading.
	int fileDescriptor;
	/// Network-like stream or file-like stream. Matters for header format.
	bool isNetworkStream;
	/// For network-like inputs, we differentiate between stream and message
	/// based protocols, like TCP and UDP. Matters for header/sequence number.
	bool isNetworkMessageBased;
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
	/// Packet data parsing structures.
	struct input_common_packet_data packets;
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
static void parseSourceString(char *sourceString, inputCommonState state);
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

	// So we have to change the size, let's see if the new number makes any sense.
	// We want reasonably sized buffers as minimum, that must fit at least the
	// event packet header and the network header fully (so 28 bytes), as well as
	// the standard AEDAT 2.0 and 3.1 headers, so a couple hundred bytes, and that
	// will maintain good performance. 512 seems a good compromise.
	if (newBufferSize < 512) {
		newBufferSize = 512;
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

static void parseSourceString(char *sourceString, inputCommonState state) {
	// TODO: implement sourceString itself for files, and some way to parse sizes then.

	// Create SourceInfo node.
	sshsNode sourceInfoNode = sshsGetRelativeNode(state->parentModule->moduleNode, "sourceInfo/");

	int16_t dvsSizeX = 0, dvsSizeY = 0;
	int16_t apsSizeX = 0, apsSizeY = 0;

	// Determine sizes via known chip information.
	if (caerStrEquals(sourceString, "DVS128")) {
		dvsSizeX = dvsSizeY = 128;
	}
	else if (caerStrEquals(sourceString, "DAVIS240A") || caerStrEquals(sourceString, "DAVIS240B")
		|| caerStrEquals(sourceString, "DAVIS240C")) {
		dvsSizeX = apsSizeX = 240;
		dvsSizeY = apsSizeY = 180;
	}
	else if (caerStrEquals(sourceString, "DAVIS128")) {
		dvsSizeX = apsSizeX = dvsSizeY = apsSizeY = 128;
	}
	else if (caerStrEquals(sourceString, "DAVIS346A") || caerStrEquals(sourceString, "DAVIS346B")
		|| caerStrEquals(sourceString, "DAVIS346Cbsi")) {
		dvsSizeX = apsSizeX = 346;
		dvsSizeY = apsSizeY = 260;
	}
	else if (caerStrEquals(sourceString, "DAVIS640")) {
		dvsSizeX = apsSizeX = 640;
		dvsSizeY = apsSizeY = 480;
	}
	else if (caerStrEquals(sourceString, "DAVISHet640")) {
		dvsSizeX = 320;
		dvsSizeY = 240;
		apsSizeX = 640;
		apsSizeY = 480;
	}
	else if (caerStrEquals(sourceString, "DAVIS208")) {
		dvsSizeX = apsSizeX = 208;
		dvsSizeY = apsSizeY = 192;
	}
	else {
		// Default fall-back of 640x480 (VGA).
		caerLog(CAER_LOG_WARNING, state->parentModule->moduleSubSystemString,
			"Impossible to determine display sizes from Source information/string. Falling back to 640x480 (VGA).");
		dvsSizeX = apsSizeX = 640;
		dvsSizeY = apsSizeY = 480;
	}

	// Put size information inside sourceInfo node.
	sshsNodePutShort(sourceInfoNode, "dvsSizeX", dvsSizeX);
	sshsNodePutShort(sourceInfoNode, "dvsSizeY", dvsSizeY);

	if (apsSizeX != 0 && apsSizeY != 0) {
		sshsNodePutShort(sourceInfoNode, "apsSizeX", apsSizeX);
		sshsNodePutShort(sourceInfoNode, "apsSizeY", apsSizeY);
	}

	sshsNodePutShort(sourceInfoNode, "dataSizeX", (dvsSizeX > apsSizeX) ? (dvsSizeX) : (apsSizeX));
	sshsNodePutShort(sourceInfoNode, "dataSizeY", (dvsSizeY > apsSizeY) ? (dvsSizeY) : (apsSizeY));
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
	bool sourceHeader = false;
	bool endHeader = false;

	while (!endHeader) {
		char *headerLine = getFileHeaderLine(state);
		if (headerLine == NULL) {
			// Failed to parse header line; this is an invalid header for AEDAT 3.1!
			// For AEDAT 2.0, since there is no END-HEADER, this might be
			// the right way for headers to stop, so we consider this valid IFF we
			// already got at least the version header.
			if (versionHeader && !state->header.isAEDAT3) {
				// Parsed AEDAT 2.0 header successfully.
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
		else if (state->header.isAEDAT3 && !sourceHeader) {
			// Then the source header. Only with AEDAT 3.X. We only support one active source.
			char *sourceString = NULL;

			if (sscanf(headerLine, "#Source %" SCNi16 ": %ms\r\n", &state->header.sourceID, &sourceString) == 2) {
				sourceHeader = true;

				// Parse source string to get needed sourceInfo parameters.
				parseSourceString(sourceString, state);

				caerLog(CAER_LOG_DEBUG, state->parentModule->moduleSubSystemString,
					"Found Source header with value '%s', Source ID %" PRIi16 ".", sourceString,
					state->header.sourceID);

				free(sourceString);
			}
			else {
				free(headerLine);

				caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
					"No compliant Source header found. Invalid file.");
				return (false);
			}
		}
		else {
			// Now we either have other header lines with AEDAT 2.0/AEDAT 3.1, or
			// the END-HEADER with AEDAT 3.1. We check this before the other possible,
			// because it terminates the AEDAT 3.1 header, so we stop in that case.
			if (caerStrEquals(headerLine, "#!END-HEADER\r\n")) {
				endHeader = true;

				caerLog(CAER_LOG_DEBUG, state->parentModule->moduleSubSystemString, "Found END-HEADER header.");
			}
			else {
				// Then other headers, like Start-Time.
				if (caerStrEqualsUpTo(headerLine, "#Start-Time: %s", 13)) {
					char *startTimeString = NULL;

					if (sscanf(headerLine, "#Start-Time: %m[^\r]s\n", &startTimeString) == 1) {
						caerLog(CAER_LOG_INFO, state->parentModule->moduleSubSystemString, "Recording was taken on %s.",
							startTimeString);
						free(startTimeString);
					}
				}
				else {
					headerLine[strlen(headerLine) - 2] = '\0'; // Shorten string to avoid printing ending \r\n.
					caerLog(CAER_LOG_DEBUG, state->parentModule->moduleSubSystemString, "Header line: '%s'.",
						headerLine);
				}
			}
		}

		free(headerLine);
	}

	// Parsed AEDAT 3.1 header successfully.
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

	simpleBuffer buf = state->dataBuffer;

	while (buf->bufferPosition < buf->bufferUsedSize) {
		// So now we're somewhere inside the buffer (usually at start), and want to
		// read in a very long sequence of event packets.
		// An event packet is made up of header + data, and the header contains the
		// information needed to decode the data and its length, to know then where
		// the next event packet boundary is. So we get the full header first, then
		// the data, but careful, it can all be split across two (header+data) or
		// more (data) buffers, so we need to reassemble!
		size_t remainingData = buf->bufferUsedSize - buf->bufferPosition;

		// First thing, handle skip packet requests.
		if (state->packets.skipSize != 0) {
			if (state->packets.skipSize >= remainingData) {
				state->packets.skipSize -= remainingData;

				// Go and get next buffer. bufferPosition is reset.
				return (true);
			}
			else {
				buf->bufferPosition += state->packets.skipSize;
				remainingData -= state->packets.skipSize;
				state->packets.skipSize = 0; // Don't skip anymore, continue as usual.
			}
		}

		if (state->packets.currPacketHeaderSize != CAER_EVENT_PACKET_HEADER_SIZE) {
			if (remainingData < CAER_EVENT_PACKET_HEADER_SIZE) {
				// Reaching end of buffer, the header is split across two buffers!
				memcpy(state->packets.currPacketHeader, buf->buffer + buf->bufferPosition, remainingData);

				state->packets.currPacketHeaderSize = remainingData;

				// Go and get next buffer. bufferPosition is reset.
				return (true);
			}
			else {
				// Either a full header, or the second part of one.
				size_t dataToRead = CAER_EVENT_PACKET_HEADER_SIZE - state->packets.currPacketHeaderSize;

				memcpy(state->packets.currPacketHeader + state->packets.currPacketHeaderSize,
					buf->buffer + buf->bufferPosition, dataToRead);

				state->packets.currPacketHeaderSize += dataToRead;
				buf->bufferPosition += dataToRead;
				remainingData -= dataToRead;
			}

			// So now that we have a full header, let's look at it.
			caerEventPacketHeader packet = (caerEventPacketHeader) state->packets.currPacketHeader;

			int16_t eventSource = caerEventPacketHeaderGetEventSource(packet);
			int32_t eventNumber = caerEventPacketHeaderGetEventNumber(packet);
			int32_t eventValid = caerEventPacketHeaderGetEventValid(packet);
			int32_t eventSize = caerEventPacketHeaderGetEventSize(packet);

			// First we verify that the source ID remained unique (only one source per I/O module supported!).
			if (state->header.sourceID != eventSource) {
				caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
					"An input module can only handle packets from the same source! "
						"A packet with source %" PRIi16 " was read, but this input module expects only packets from source %" PRIi16 ".",
					eventSource, state->header.sourceID);

				// Skip packet.
				state->packets.skipSize = (size_t) (eventNumber * eventSize);
				state->packets.currPacketHeaderSize = 0; // Get new header after skipping.

				continue;
			}

			// Now let's get the right number of events, depending on user settings.
			bool validOnly = atomic_load_explicit(&state->validOnly, memory_order_relaxed);
			int32_t eventCapacity = (validOnly) ? (eventValid) : (eventNumber);

			// Allocate space for the full packet, so we can reassemble it.
			state->packets.currPacketDataSize = (size_t) (eventCapacity * eventSize);

			caerLog(CAER_LOG_DEBUG, state->parentModule->moduleSubSystemString,
				"Allocating %zu bytes for newly read event packet.",
				CAER_EVENT_PACKET_HEADER_SIZE + state->packets.currPacketDataSize);

			state->packets.currPacket = malloc(CAER_EVENT_PACKET_HEADER_SIZE + state->packets.currPacketDataSize);
			if (state->packets.currPacket == NULL) {
				caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
					"Failed to allocate memory for newly read event packet.");
				return (false);
			}

			// First we copy the header in.
			memcpy(state->packets.currPacket, state->packets.currPacketHeader, CAER_EVENT_PACKET_HEADER_SIZE);

			state->packets.currPacketDataOffset = CAER_EVENT_PACKET_HEADER_SIZE;

			// Rewrite event source to reflect this module, not the original one.
			caerEventPacketHeaderSetEventSource(state->packets.currPacket, I16T(state->parentModule->moduleID));

			// And set the event numbers to the correct value for the valid-only mode.
			if (validOnly) {
				caerEventPacketHeaderSetEventNumber(state->packets.currPacket, eventCapacity);
				caerEventPacketHeaderSetEventCapacity(state->packets.currPacket, eventCapacity);
			}
		}

		// And then the data, from the buffer to the new event packet. We have to take care of
		// date being split across multiple buffers, as well as what data we want to copy. In
		// validOnly mode, we only want to copy valid events, which means either checking and
		// copying, or in the case of already receiving a packet where eventValid == eventNumber
		// just copying. For the non-validOnly case, we always just copy.
		int32_t eventNumberOriginal = caerEventPacketHeaderGetEventNumber(
			(caerEventPacketHeader) state->packets.currPacketHeader);
		int32_t eventNumber = caerEventPacketHeaderGetEventNumber(state->packets.currPacket);

		if (eventNumberOriginal == eventNumber) {
			// Original packet header and new packet header have the same event number, this
			// means that either there was no change due to validOnly (eventValid == eventNumber),
			// or that validOnly mode is disabled. Just copy data!
			if (state->packets.currPacketDataSize >= remainingData) {
				// We need to copy more data than in this buffer.
				memcpy(((uint8_t *) state->packets.currPacket) + state->packets.currPacketDataOffset,
					buf->buffer + buf->bufferPosition, remainingData);

				state->packets.currPacketDataOffset += remainingData;
				state->packets.currPacketDataSize -= remainingData;

				// Go and get next buffer. bufferPosition is reset.
				return (true);
			}
			else {
				// We copy the last bytes of data and we're done.
				memcpy(((uint8_t *) state->packets.currPacket) + state->packets.currPacketDataOffset,
					buf->buffer + buf->bufferPosition, state->packets.currPacketDataSize);

				// This packet is fully copied and done, so reset variables for next iteration.
				state->packets.currPacketHeaderSize = 0; // Get new header next iteration.
				buf->bufferPosition += state->packets.currPacketDataSize;
			}
		}
		else {
			// Valid-only mode, iterate over events and copy only the valid ones.
			int32_t eventSize = caerEventPacketHeaderGetEventSize(state->packets.currPacket);

			// TODO: implement this.
			return (false);
		}

		// We've got a full event packet, store it.
		// TODO: improve this.
		caerEventPacketContainer eventPackets = caerEventPacketContainerAllocate(1);

		caerEventPacketContainerSetEventPacket(eventPackets, 0, state->packets.currPacket);

		retry: if (!ringBufferPut(state->transferRing, eventPackets)) {
			if (atomic_load_explicit(&state->keepPackets, memory_order_relaxed)) {
				// Retry forever if requested.
				goto retry;
			}

			caerEventPacketContainerFree(eventPackets);

			caerLog(CAER_LOG_INFO, state->parentModule->moduleSubSystemString,
				"Failed to put new packet container on transfer ring-buffer: full.");
		}
		else {
			// Signal availability of new data to the mainloop on packet container commit.
			atomic_fetch_add_explicit(&state->mainloopReference->dataAvailable, 1, memory_order_release);
		}
	}

	return (true);
}

static int inputHandlerThread(void *stateArg) {
	inputCommonState state = stateArg;

	struct timespec waitSleep = { .tv_sec = 0, .tv_nsec = 10000000 }; // Wait 10ms.

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

		// Slow down generation of new packets to near real-time.
		thrd_sleep(&waitSleep, NULL);

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

	// Add auto-restart setting.
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "autoRestart", true);

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

	free(state->packets.currPacket);

	if (sshsNodeGetBool(moduleData->moduleNode, "autoRestart")) {
		// Prime input module again so that it will try to restart if new devices detected.
		sshsNodePutBool(moduleData->moduleNode, "running", true);
	}
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
