#include "input_common.h"
#include "base/mainloop.h"
#include "ext/portable_time.h"
#include "ext/ringbuffer/ringbuffer.h"
#include "ext/uthash/utarray.h"
#include "ext/uthash/utlist.h"
#include "ext/buffers.h"
#ifdef HAVE_PTHREADS
#include "ext/c11threads_posix.h"
#endif
#ifdef ENABLE_INOUT_PNG_COMPRESSION
#include <png.h>
#endif

#include <libcaer/events/common.h>
#include <libcaer/events/packetContainer.h>
#include <libcaer/events/frame.h>
#include <libcaer/events/special.h>

#define MAX_HEADER_LINE_SIZE 1024

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

struct packetData {
	/// Numerical ID of a packet. First packet hast ID 0.
	size_t id;
	/// Data offset, in bytes.
	size_t offset;
	/// Data size, in bytes.
	size_t size;
	/// Is this packet compressed?
	bool isCompressed;
	/// Contained event type.
	int16_t eventType;
	/// Size of contained events, in bytes.
	int32_t eventSize;
	/// Contained number of events.
	int32_t eventNumber;
	/// Contained number of valid events.
	int32_t eventValid;
	/// First (lowest) timestamp.
	int64_t startTimestamp;
	/// Last (highest) timestamp.
	int64_t endTimestamp;
	/// Doubly-linked list pointers.
	struct packetData *prev, *next;
};

struct input_common_packet_data {
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
	/// Current packet data for packet list book-keeping.
	struct packetData *currPacketData;
	/// List of data on all parsed original packets from the input.
	struct packetData *packetsList;
	/// Global packet counter.
	size_t packetCount;
};

struct input_common_packet_container_data {
	/// Current events, merged into packets, sorted by type.
	UT_array *eventPackets;
	/// Smallest timestamp among all events currently held in 'eventPackets'.
	int64_t lowestTimestamp;
	/// Biggest timestamp among all events currently held in 'eventPackets'.
	int64_t highestTimestamp;
	/// Sum of the event number for all packets currently held in 'eventPackets'.
	size_t totalEventNumber;
	/// The first main timestamp (the one relevant for packet ordering in streams)
	/// of the last event packet that was handled.
	int64_t lastSeenPacketTimestamp;
	/// The timestamp up to which we want to (have to!) read, so that we can
	/// output the next packet container (in time-slice mode).
	int64_t newContainerTimestampStart;
	int64_t newContainerTimestampEnd;
	/// Time when the last packet container was sent out, used to calculate
	/// sleep time to reach user configured 'timeDelay'.
	struct timespec lastCommitTime;
	/// Time slice (in µs), for which to generate a packet container.
	atomic_int_fast32_t timeSlice;
	/// Time delay (in µs) between the start of two consecutive time slices.
	/// This is used for real-time slow-down.
	atomic_int_fast32_t timeDelay;
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
	/// Packet container data structure, to generate from packets.
	struct input_common_packet_container_data packetContainer;
	/// Data buffer for reading from file descriptor (buffered I/O).
	simpleBuffer dataBuffer;
	/// Offset for current data buffer.
	size_t dataBufferOffset;
	/// Flag to signal update to buffer configuration asynchronously.
	atomic_bool bufferUpdate;
	/// Reference to parent module's original data.
	caerModuleData parentModule;
	/// Reference to module's mainloop (for data availability signaling).
	caerMainloopData mainloopReference;
};

typedef struct input_common_state *inputCommonState;

size_t CAER_INPUT_COMMON_STATE_STRUCT_SIZE = sizeof(struct input_common_state);

static int packetsTypeCmp(const void *a, const void *b);
static bool newInputBuffer(inputCommonState state);
static bool parseNetworkHeader(inputCommonState state);
static char *getFileHeaderLine(inputCommonState state);
static void parseSourceString(char *sourceString, inputCommonState state);
static bool parseFileHeader(inputCommonState state);
static bool parseHeader(inputCommonState state);
static void addToPacketContainer(inputCommonState state, caerEventPacketHeader newPacket);
static caerEventPacketContainer generatePacketContainer(inputCommonState state);
static bool parseData(inputCommonState state);
static bool parseAEDAT2(inputCommonState state);
static bool parseAEDAT3(inputCommonState state, bool reverseXY);
static int aedat3GetPacket(inputCommonState state);
static bool decompressTimestampSerialize(inputCommonState state, caerEventPacketHeader packet, size_t packetSize);
static bool decompressEventPacket(inputCommonState state, caerEventPacketHeader packet, size_t packetSize);
static int inputHandlerThread(void *stateArg);
static void caerInputCommonConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);

static int packetsTypeCmp(const void *a, const void *b) {
	const caerEventPacketHeader *aa = a;
	const caerEventPacketHeader *bb = b;

	// Sort by type ID.
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
	// Network header is 20 bytes long. Use struct to interpret.
	struct aedat3_network_header networkHeader;

	// Copy data into packet struct.
	memcpy(&networkHeader, state->dataBuffer->buffer, AEDAT3_NETWORK_HEADER_LENGTH);
	state->dataBuffer->bufferPosition += AEDAT3_NETWORK_HEADER_LENGTH;

	// Ensure endianness conversion is done if needed.
	networkHeader.magicNumber = le64toh(networkHeader.magicNumber);
	networkHeader.sequenceNumber = le64toh(networkHeader.sequenceNumber);
	networkHeader.sourceNumber = le16toh(networkHeader.sourceNumber);

	// Check header values.
	if (networkHeader.magicNumber != AEDAT3_NETWORK_MAGIC_NUMBER) {
		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
			"AEDAT 3.X magic number not found. Invalid network stream.");
		return (false);
	}

	state->header.isAEDAT3 = true;
	state->header.majorVersion = 3;

	if (state->isNetworkMessageBased) {
		// For message based streams, use the sequence number.
		// TODO: check this for missing packets in message mode!
		state->header.networkSequenceNumber = networkHeader.sequenceNumber;
	}
	else {
		// For stream based transports, this is always zero.
		if (networkHeader.sequenceNumber != 0) {
			caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
				"SequenceNumber is not zero. Invalid network stream.");
			return (false);
		}
	}

	if (networkHeader.versionNumber != AEDAT3_NETWORK_VERSION) {
		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
			"Unsupported AEDAT version. Invalid network stream.");
		return (false);
	}

	state->header.minorVersion = networkHeader.versionNumber;

	// All formats are supported.
	state->header.formatID = networkHeader.formatNumber;

	// TODO: get sourceInfo node via config-server side-channel.
	state->header.sourceID = 1;
	sshsNode sourceInfoNode = sshsGetRelativeNode(state->parentModule->moduleNode, "sourceInfo/");
	sshsNodePutShort(sourceInfoNode, "dvsSizeX", 240);
	sshsNodePutShort(sourceInfoNode, "dvsSizeY", 180);
	sshsNodePutShort(sourceInfoNode, "apsSizeX", 240);
	sshsNodePutShort(sourceInfoNode, "apsSizeY", 180);

	// We're done!
	state->header.isValidHeader = true;

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
	// Create SourceInfo node.
	sshsNode sourceInfoNode = sshsGetRelativeNode(state->parentModule->moduleNode, "sourceInfo/");

	sshsNodePutLong(sourceInfoNode, "highestTimestamp", -1);

	int16_t dvsSizeX = 0, dvsSizeY = 0;
	int16_t apsSizeX = 0, apsSizeY = 0;
	int16_t dataSizeX = 0, dataSizeY = 0;
	int16_t visualizerSizeX = 0, visualizerSizeY = 0;

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
	else if (caerStrEqualsUpTo(sourceString, "File,", 5)) {
		sscanf(sourceString + 5, "dvsSizeX=%" SCNi16 ",dvsSizeY=%" SCNi16 ",apsSizeX=%" SCNi16 ",apsSizeY=%" SCNi16 ","
		"dataSizeX=%" SCNi16 ",dataSizeY=%" SCNi16 ",visualizerSizeX=%" SCNi16 ",visualizerSizeY=%" SCNi16 "\r\n",
			&dvsSizeX, &dvsSizeY, &apsSizeX, &apsSizeY, &dataSizeX, &dataSizeY, &visualizerSizeX, &visualizerSizeY);
	}
	else if (caerStrEqualsUpTo(sourceString, "Network,", 8)) {
		sscanf(sourceString + 8, "dvsSizeX=%" SCNi16 ",dvsSizeY=%" SCNi16 ",apsSizeX=%" SCNi16 ",apsSizeY=%" SCNi16 ","
		"dataSizeX=%" SCNi16 ",dataSizeY=%" SCNi16 ",visualizerSizeX=%" SCNi16 ",visualizerSizeY=%" SCNi16 "\r\n",
			&dvsSizeX, &dvsSizeY, &apsSizeX, &apsSizeY, &dataSizeX, &dataSizeY, &visualizerSizeX, &visualizerSizeY);
	}
	else if (caerStrEqualsUpTo(sourceString, "Processor,", 10)) {
		sscanf(sourceString + 10, "dvsSizeX=%" SCNi16 ",dvsSizeY=%" SCNi16 ",apsSizeX=%" SCNi16 ",apsSizeY=%" SCNi16 ","
		"dataSizeX=%" SCNi16 ",dataSizeY=%" SCNi16 ",visualizerSizeX=%" SCNi16 ",visualizerSizeY=%" SCNi16 "\r\n",
			&dvsSizeX, &dvsSizeY, &apsSizeX, &apsSizeY, &dataSizeX, &dataSizeY, &visualizerSizeX, &visualizerSizeY);
	}
	else {
		// Default fall-back of 640x480 (VGA).
		caerLog(CAER_LOG_WARNING, state->parentModule->moduleSubSystemString,
			"Impossible to determine display sizes from Source information/string. Falling back to 640x480 (VGA).");
		dvsSizeX = apsSizeX = 640;
		dvsSizeY = apsSizeY = 480;
	}

	// Put size information inside sourceInfo node.
	if (dvsSizeX != 0 && dvsSizeY != 0) {
		sshsNodePutShort(sourceInfoNode, "dvsSizeX", dvsSizeX);
		sshsNodePutShort(sourceInfoNode, "dvsSizeY", dvsSizeY);
	}

	if (apsSizeX != 0 && apsSizeY != 0) {
		sshsNodePutShort(sourceInfoNode, "apsSizeX", apsSizeX);
		sshsNodePutShort(sourceInfoNode, "apsSizeY", apsSizeY);
	}

	if (dataSizeX == 0 && dataSizeY == 0) {
		// Try to auto-discover dataSize, if it was not previously set, based on the
		// presence of DVS or APS sizes. If they don't exist either, this will be 0.
		dataSizeX = (dvsSizeX > apsSizeX) ? (dvsSizeX) : (apsSizeX);
		dataSizeY = (dvsSizeY > apsSizeY) ? (dvsSizeY) : (apsSizeY);
	}

	if (dataSizeX != 0 && dataSizeY != 0) {
		sshsNodePutShort(sourceInfoNode, "dataSizeX", dataSizeX);
		sshsNodePutShort(sourceInfoNode, "dataSizeY", dataSizeY);
	}

	if (visualizerSizeX != 0 && visualizerSizeY != 0) {
		sshsNodePutShort(sourceInfoNode, "visualizerSizeX", visualizerSizeX);
		sshsNodePutShort(sourceInfoNode, "visualizerSizeY", visualizerSizeY);
	}

	// Generate source string for output modules.
	size_t sourceStringFileLength = (size_t) snprintf(NULL, 0, "#Source %" PRIu16 ": File,"
	"dvsSizeX=%" PRIi16 ",dvsSizeY=%" PRIi16 ",apsSizeX=%" PRIi16 ",apsSizeY=%" PRIi16 ","
	"dataSizeX=%" PRIi16 ",dataSizeY=%" PRIi16 ",visualizerSizeX=%" PRIi16 ",visualizerSizeY=%" PRIi16 "\r\n"
	"#-Source %" PRIi16 ": %s\r\n", state->parentModule->moduleID, dvsSizeX, dvsSizeY, apsSizeX, apsSizeY, dataSizeX,
		dataSizeY, visualizerSizeX, visualizerSizeY, state->header.sourceID, sourceString);

	char sourceStringFile[sourceStringFileLength + 1];
	snprintf(sourceStringFile, sourceStringFileLength + 1, "#Source %" PRIu16 ": File,"
	"dvsSizeX=%" PRIi16 ",dvsSizeY=%" PRIi16 ",apsSizeX=%" PRIi16 ",apsSizeY=%" PRIi16 ","
	"dataSizeX=%" PRIi16 ",dataSizeY=%" PRIi16 ",visualizerSizeX=%" PRIi16 ",visualizerSizeY=%" PRIi16 "\r\n"
	"#-Source %" PRIi16 ": %s\r\n", state->parentModule->moduleID, dvsSizeX, dvsSizeY, apsSizeX, apsSizeY, dataSizeX,
		dataSizeY, visualizerSizeX, visualizerSizeY, state->header.sourceID, sourceString);
	sourceStringFile[sourceStringFileLength] = '\0';

	sshsNodePutString(sourceInfoNode, "sourceString", sourceStringFile);
}

static bool parseFileHeader(inputCommonState state) {
	// We expect that the full header part is contained within
	// this one data buffer.
	// File headers are part of the AEDAT 3.X specification.
	// Start with #, go until '\r\n' (Windows EOL). First must be
	// version header !AER-DATx.y, last must be end-of-header
	// marker with !END-HEADER (AEDAT 3.1 only).
	bool versionHeader = false;
	bool formatHeader = false;
	bool sourceHeader = false;
	bool endHeader = false;

	while (!endHeader) {
		char *headerLine = getFileHeaderLine(state);
		if (headerLine == NULL) {
			// Failed to parse header line; this is an invalid header for AEDAT 3.1!
			// For AEDAT 2.0 and 3.0, since there is no END-HEADER, this might be
			// the right way for headers to stop, so we consider this valid IFF we
			// already got the version header for AEDAT 2.0, and for AEDAT 3.0 if we
			// also got the required headers Format and Source at least.
			if ((state->header.majorVersion == 2 && state->header.minorVersion == 0) && versionHeader) {
				// Parsed AEDAT 2.0 header successfully (version).
				state->header.isValidHeader = true;
				return (true);
			}

			if ((state->header.majorVersion == 3 && state->header.minorVersion == 0) && versionHeader && formatHeader
				&& sourceHeader) {
				// Parsed AEDAT 3.0 header successfully (version, format, source).
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

						// AEDAT 3.0 and 3.1 are supported.
						if (state->header.minorVersion != 0 && state->header.minorVersion != 1) {
							goto noValidVersionHeader;
						}
						break;

					default:
						// Versions other than 2.0 and 3.X are not supported.
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
			char formatString[1024 + 1];

			if (sscanf(headerLine, "#Format: %1024s\r\n", formatString) == 1) {
				formatHeader = true;

				// Parse format string to format ID.
				// We support either only RAW, or a mixture of the various compression modes.
				if (caerStrEquals(formatString, "RAW")) {
					state->header.formatID = 0x00;
				}
				else {
					state->header.formatID = 0x00;

					if (strstr(formatString, "TSSerialize") != NULL) {
						state->header.formatID |= 0x01;
					}

					if (strstr(formatString, "PNGFrames") != NULL) {
						state->header.formatID |= 0x02;
					}

					if (!state->header.formatID) {
						// No valid format found.
						free(headerLine);

						caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
							"No compliant Format type found. Format '%s' is invalid.", formatString);

						return (false);
					}
				}

				caerLog(CAER_LOG_DEBUG, state->parentModule->moduleSubSystemString,
					"Found Format header with value '%s', Format ID %" PRIi8 ".", formatString, state->header.formatID);
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
			char sourceString[1024 + 1];

			if (sscanf(headerLine, "#Source %" SCNi16 ": %1024[^\r]s\n", &state->header.sourceID, sourceString) == 2) {
				sourceHeader = true;

				// Parse source string to get needed sourceInfo parameters.
				parseSourceString(sourceString, state);

				caerLog(CAER_LOG_DEBUG, state->parentModule->moduleSubSystemString,
					"Found Source header with value '%s', Source ID %" PRIi16 ".", sourceString,
					state->header.sourceID);
			}
			else {
				free(headerLine);

				caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
					"No compliant Source header found. Invalid file.");
				return (false);
			}
		}
		else {
			// Now we either have other header lines with AEDAT 2.0/AEDAT 3.X, or
			// the END-HEADER with AEDAT 3.1. We check this before the other possible,
			// because it terminates the AEDAT 3.1 header, so we stop in that case.
			if (caerStrEquals(headerLine, "#!END-HEADER\r\n")) {
				endHeader = true;

				caerLog(CAER_LOG_DEBUG, state->parentModule->moduleSubSystemString, "Found END-HEADER header.");
			}
			else {
				// Then other headers, like Start-Time.
				// TODO: parse negative source strings (#-Source) and add them to sourceInfo.
				if (caerStrEqualsUpTo(headerLine, "#Start-Time: ", 13)) {
					char startTimeString[1024 + 1];

					if (sscanf(headerLine, "#Start-Time: %1024[^\r]s\n", startTimeString) == 1) {
						caerLog(CAER_LOG_INFO, state->parentModule->moduleSubSystemString, "Recording was taken on %s.",
							startTimeString);
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

static void addToPacketContainer(inputCommonState state, caerEventPacketHeader newPacket) {
	int16_t newPacketType = caerEventPacketHeaderGetEventType(newPacket);
	int64_t newPacketFirstTimestamp = caerGenericEventGetTimestamp64(caerGenericEventGetEvent(newPacket, 0), newPacket);
	int64_t newPacketLastTimestamp = caerGenericEventGetTimestamp64(
		caerGenericEventGetEvent(newPacket, caerEventPacketHeaderGetEventCapacity(newPacket) - 1), newPacket);

	// Remember the main timestamp of the first event of the new packet. That's the
	// order-relvant timestamp for files/streams.
	state->packetContainer.lastSeenPacketTimestamp = newPacketFirstTimestamp;

	// Initialize with first packet.
	if (state->packetContainer.newContainerTimestampStart == -1) {
		// -1 because newPacketFirstTimestamp is part of the set!
		state->packetContainer.newContainerTimestampStart = newPacketFirstTimestamp;
		state->packetContainer.newContainerTimestampEnd = newPacketFirstTimestamp
			+ (atomic_load_explicit(&state->packetContainer.timeSlice, memory_order_relaxed) - 1);

		portable_clock_gettime_monotonic(&state->packetContainer.lastCommitTime);
	}

	bool packetAlreadyExists = false;
	caerEventPacketHeader *packet = NULL;
	while ((packet = (caerEventPacketHeader *) utarray_next(state->packetContainer.eventPackets, packet)) != NULL) {
		int16_t packetType = caerEventPacketHeaderGetEventType(*packet);

		if (packetType == newPacketType) {
			// Packet of this type already present.
			packetAlreadyExists = true;
			break;
		}
	}

	// Packet with same type as newPacket found, do merge operation.
	if (packetAlreadyExists) {
		// Merge newPacket with '*packet'. Since packets from the same source,
		// and having the same time, are guaranteed to have monotonic timestamps,
		// the merge operation becomes a simple append operation.
		int32_t packetEventSize = caerEventPacketHeaderGetEventSize(*packet);

		int32_t packetEventValid = caerEventPacketHeaderGetEventValid(*packet);
		int32_t packetEventNumber = caerEventPacketHeaderGetEventNumber(*packet);

		int32_t newPacketEventValid = caerEventPacketHeaderGetEventValid(newPacket);
		int32_t newPacketEventNumber = caerEventPacketHeaderGetEventNumber(newPacket);

		caerEventPacketHeader mergedPacket = caerGenericEventPacketGrow(*packet,
			(packetEventNumber + newPacketEventNumber));
		if (mergedPacket == NULL) {
			// Failed to allocate memory for merged packets, free newPacket and jump merge stage.
			free(newPacket);
			return;
		}

		// Memory reallocation was successful, so we can update the header, copy over the new
		// events, and update the main pointer in the event packets array.
		*packet = mergedPacket;

		caerEventPacketHeaderSetEventValid(mergedPacket, packetEventValid + newPacketEventValid);
		caerEventPacketHeaderSetEventNumber(mergedPacket, packetEventNumber + newPacketEventNumber);

		memcpy(((uint8_t *) mergedPacket) + CAER_EVENT_PACKET_HEADER_SIZE + (packetEventSize * packetEventNumber),
			((uint8_t *) newPacket) + CAER_EVENT_PACKET_HEADER_SIZE, (size_t) (packetEventSize * newPacketEventNumber));

		// Merged content with existing packet, data copied: free new one.
		free(newPacket);

		// Update packets statistics. FirstTimestamp can never be smaller than lowestTimestamp,
		// since we already have events of the same type that must come before.
		state->packetContainer.totalEventNumber += (size_t) newPacketEventNumber;

		if (newPacketLastTimestamp > state->packetContainer.highestTimestamp) {
			state->packetContainer.highestTimestamp = newPacketLastTimestamp;
		}
	}
	else {
		// No previous packet of this type found, use this one directly.
		utarray_push_back(state->packetContainer.eventPackets, &newPacket);

		utarray_sort(state->packetContainer.eventPackets, &packetsTypeCmp);

		// Update packets statistics.
		state->packetContainer.totalEventNumber += (size_t) caerEventPacketHeaderGetEventNumber(newPacket);

		if (newPacketFirstTimestamp < state->packetContainer.lowestTimestamp) {
			state->packetContainer.lowestTimestamp = newPacketFirstTimestamp;
		}

		if (newPacketLastTimestamp > state->packetContainer.highestTimestamp) {
			state->packetContainer.highestTimestamp = newPacketLastTimestamp;
		}
	}
}

static inline void doPacketContainerCommit(inputCommonState state, caerEventPacketContainer packetContainer) {
	retry: if (!ringBufferPut(state->transferRing, packetContainer)) {
		if (atomic_load_explicit(&state->keepPackets, memory_order_relaxed)) {
			// Retry forever if requested.
			goto retry;
		}

		caerEventPacketContainerFree(packetContainer);

		caerLog(CAER_LOG_INFO, state->parentModule->moduleSubSystemString,
			"Failed to put new packet container on transfer ring-buffer: full.");
	}
	else {
		// Signal availability of new data to the mainloop on packet container commit.
		atomic_fetch_add_explicit(&state->mainloopReference->dataAvailable, 1, memory_order_release);

		caerLog(CAER_LOG_DEBUG, state->parentModule->moduleSubSystemString, "Submitted packet container successfully.");
	}
}

static inline void doTimeDelay(inputCommonState state) {
	// Got packet container, delay it until user-defined time.
	uint64_t timeDelay = U64T(atomic_load_explicit(&state->packetContainer.timeDelay, memory_order_relaxed));

	if (timeDelay != 0) {
		// Get current time (nanosecond resolution).
		struct timespec currentTime;
		portable_clock_gettime_monotonic(&currentTime);

		// Calculate elapsed time since last commit, based on that then either
		// wait to meet timing, or log that it's impossible with current settings.
		uint64_t diffNanoTime = (uint64_t) (((int64_t) (currentTime.tv_sec
			- state->packetContainer.lastCommitTime.tv_sec) * 1000000000LL)
			+ (int64_t) (currentTime.tv_nsec - state->packetContainer.lastCommitTime.tv_nsec));
		uint64_t diffMicroTime = diffNanoTime / 1000;

		if (diffMicroTime >= timeDelay) {
			caerLog(CAER_LOG_WARNING, state->parentModule->moduleSubSystemString,
				"Impossible to meet timeDelay timing specification with current settings.");

			// Don't delay any more by requesting time again, use old one.
			state->packetContainer.lastCommitTime = currentTime;
		}
		else {
			// Sleep for the remaining time.
			uint64_t sleepMicroTime = timeDelay - diffMicroTime;
			uint64_t sleepMicroTimeSec = sleepMicroTime / 1000000; // Seconds part.
			uint64_t sleepMicroTimeNsec = (sleepMicroTime * 1000) - (sleepMicroTimeSec * 1000000000); // Nanoseconds part.

			struct timespec delaySleep = { .tv_sec = I64T(sleepMicroTimeSec), .tv_nsec = I64T(sleepMicroTimeNsec) };

			thrd_sleep(&delaySleep, NULL);

			// Update stored time.
			portable_clock_gettime_monotonic(&state->packetContainer.lastCommitTime);
		}
	}
}

static caerEventPacketContainer generatePacketContainer(inputCommonState state) {
	// Let's generate a packet container, use the size of the event packets array as upper bound.
	int32_t packetContainerPosition = 0;
	caerEventPacketContainer packetContainer = caerEventPacketContainerAllocate(
		(int32_t) utarray_len(state->packetContainer.eventPackets));
	if (packetContainer == NULL) {
		return (NULL);
	}

	// Iterate over each event packet, and slice out the relevant part in time.
	caerEventPacketHeader *currPacket = NULL;
	while ((currPacket = (caerEventPacketHeader *) utarray_next(state->packetContainer.eventPackets, currPacket))
		!= NULL) {
		// Search for cutoff point in time. Also count valid events encountered for later calculations.
		int32_t cutoffIndex = -1;
		int32_t validEventsSeen = 0;

		CAER_ITERATOR_ALL_START(*currPacket, void *)
			if (caerGenericEventGetTimestamp64(caerIteratorElement, *currPacket)
				> state->packetContainer.newContainerTimestampEnd) {
				cutoffIndex = caerIteratorCounter;
				break;
			}

			if (caerGenericEventIsValid(caerIteratorElement)) {
				validEventsSeen++;
			}CAER_ITERATOR_ALL_END

		// TODO: handle TS_WRAP and TS_RESET as split points.

		// If there is no cutoff point, we can just send on the whole packet with no changes.
		if (cutoffIndex == -1) {
			caerEventPacketContainerSetEventPacket(packetContainer, packetContainerPosition++, *currPacket);

			// Erase slot from packets array.
			utarray_erase(state->packetContainer.eventPackets,
				(size_t) utarray_eltidx(state->packetContainer.eventPackets, currPacket), 1);
			currPacket = (caerEventPacketHeader *) utarray_prev(state->packetContainer.eventPackets, currPacket);
			continue;
		}

		// If there is one on the other hand, we can only send up to that event.
		// Special case is if the cutoff point is zero, meaning there's nothing to send.
		if (cutoffIndex == 0) {
			continue;
		}

		int32_t currPacketEventSize = caerEventPacketHeaderGetEventSize(*currPacket);
		int32_t currPacketEventValid = caerEventPacketHeaderGetEventValid(*currPacket);
		int32_t currPacketEventNumber = caerEventPacketHeaderGetEventNumber(*currPacket);

		// Allocate a new packet, with space for the remaining events that we don't send off
		// (the ones after cutoff point).
		int32_t nextPacketEventNumber = currPacketEventNumber - cutoffIndex;
		caerEventPacketHeader nextPacket = malloc(
		CAER_EVENT_PACKET_HEADER_SIZE + (size_t) (currPacketEventSize * nextPacketEventNumber));
		if (nextPacket == NULL) {
			// TODO: handle allocation failure.
			caerLog(CAER_LOG_CRITICAL, state->parentModule->moduleSubSystemString,
				"Failed memory allocation for nextPacket.");
			exit(EXIT_FAILURE);
		}

		// Copy header and remaining events to new packet, set header sizes correctly.
		memcpy(nextPacket, *currPacket, CAER_EVENT_PACKET_HEADER_SIZE);
		memcpy(((uint8_t *) nextPacket) + CAER_EVENT_PACKET_HEADER_SIZE,
			((uint8_t *) *currPacket) + CAER_EVENT_PACKET_HEADER_SIZE + (currPacketEventSize * cutoffIndex),
			(size_t) (currPacketEventSize * nextPacketEventNumber));

		caerEventPacketHeaderSetEventValid(nextPacket, currPacketEventValid - validEventsSeen);
		caerEventPacketHeaderSetEventNumber(nextPacket, nextPacketEventNumber);
		caerEventPacketHeaderSetEventCapacity(nextPacket, nextPacketEventNumber);

		// Resize current packet to include only the events up until cutoff point.
		caerEventPacketHeader currPacketResized = realloc(*currPacket,
		CAER_EVENT_PACKET_HEADER_SIZE + (size_t) (currPacketEventSize * cutoffIndex));
		if (currPacketResized == NULL) {
			// TODO: handle allocation failure.
			caerLog(CAER_LOG_CRITICAL, state->parentModule->moduleSubSystemString,
				"Failed memory allocation for currPacketResized.");
			exit(EXIT_FAILURE);
		}

		// Set header sizes for resized packet correctly.
		caerEventPacketHeaderSetEventValid(currPacketResized, validEventsSeen);
		caerEventPacketHeaderSetEventNumber(currPacketResized, cutoffIndex);
		caerEventPacketHeaderSetEventCapacity(currPacketResized, cutoffIndex);

		// Update references: the nextPacket goes into the eventPackets array at the currPacket position.
		// The currPacket, after resizing, goes into the packet container for output.
		*currPacket = nextPacket;
		caerEventPacketContainerSetEventPacket(packetContainer, packetContainerPosition++, currPacketResized);
	}

	// Update wanted timestamp for next time slice.
	state->packetContainer.newContainerTimestampStart += atomic_load_explicit(&state->packetContainer.timeSlice,
		memory_order_relaxed);
	state->packetContainer.newContainerTimestampEnd += atomic_load_explicit(&state->packetContainer.timeSlice,
		memory_order_relaxed);

	return (packetContainer);
}

static bool parseData(inputCommonState state) {
	if (state->header.majorVersion == 2 && state->header.minorVersion == 0) {
		return (parseAEDAT2(state));
	}
	else if (state->header.majorVersion == 3 && state->header.minorVersion == 0) {
		return (parseAEDAT3(state, true));
	}
	else if (state->header.majorVersion == 3 && state->header.minorVersion == 1) {
		return (parseAEDAT3(state, false));
	}

	// No parseable format found!
	return (false);
}

static bool parseAEDAT2(inputCommonState state) {
	// TODO: AEDAT 2.0 not yet supported.
	caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Reading AEDAT 2.0 data not yet supported.");
	return (false);
}

static bool parseAEDAT3(inputCommonState state, bool reverseXY) {
	while (state->dataBuffer->bufferPosition < state->dataBuffer->bufferUsedSize) {
		int pRes = aedat3GetPacket(state);
		if (pRes < 0) {
			// Error in parsing buffer to get packet.
			return (false);
		}
		else if (pRes == 1) {
			// Finished parsing this buffer with no new packet.
			// Exit to get next buffer.
			break;
		}
		else if (pRes == 2) {
			// Skip requested, run again.
			continue;
		}

		// New packet from stream, process it.
		caerLog(CAER_LOG_DEBUG, state->parentModule->moduleSubSystemString,
			"New packet read - ID: %zu, Offset: %zu, Size: %zu, Events: %" PRIi32 ", Type: %" PRIi16 ", StartTS: %" PRIi64 ", EndTS: %" PRIi64 ".",
			state->packets.currPacketData->id, state->packets.currPacketData->offset,
			state->packets.currPacketData->size, state->packets.currPacketData->eventNumber,
			state->packets.currPacketData->eventType, state->packets.currPacketData->startTimestamp,
			state->packets.currPacketData->endTimestamp);
	}

	// All good, get next buffer.
	return (true);
}

/**
 * Parse the current buffer and try to extract the AEDAT 3.X
 * packet contained within, as well as updating the packet
 * meta-data list.
 *
 * @param state common input data structure.
 *
 * @return 0 on successful packet extraction.
 * Positive numbers for special conditions:
 * 1 if more data needed.
 * 2 if skip requested (call again).
 * Negative numbers on error conditions:
 * -1 on memory allocation failure.
 * -2 on decompression failure.
 */
static int aedat3GetPacket(inputCommonState state) {
	simpleBuffer buf = state->dataBuffer;

	// So now we're somewhere inside the buffer (usually at start), and want to
	// read in a very long sequence of event packets.
	// An event packet is made up of header + data, and the header contains the
	// information needed to decode the data and its length, to know then where
	// the next event packet boundary is. So we get the full header first, then
	// the data, but careful, it can all be split across two (header+data) or
	// more (data) buffers, so we need to reassemble!
	size_t remainingData = buf->bufferUsedSize - buf->bufferPosition;

	// First thing, handle skip packet requests. This can happen if packets
	// from another source are mixed in, or we forbid some packet types.
	// In that case, we just skip over all their bytes and try to get the next
	// good packet (header).
	if (state->packets.skipSize != 0) {
		if (state->packets.skipSize >= remainingData) {
			state->packets.skipSize -= remainingData;

			// Go and get next buffer. bufferPosition is at end of buffer.
			buf->bufferPosition += remainingData;
			return (1);
		}
		else {
			buf->bufferPosition += state->packets.skipSize;
			remainingData -= state->packets.skipSize;
			state->packets.skipSize = 0; // Don't skip anymore, continue as usual.
		}
	}

	// Get 28 bytes common packet header.
	if (state->packets.currPacketHeaderSize != CAER_EVENT_PACKET_HEADER_SIZE) {
		if (remainingData < CAER_EVENT_PACKET_HEADER_SIZE) {
			// Reaching end of buffer, the header is split across two buffers!
			memcpy(state->packets.currPacketHeader, buf->buffer + buf->bufferPosition, remainingData);

			state->packets.currPacketHeaderSize = remainingData;

			// Go and get next buffer. bufferPosition is at end of buffer.
			buf->bufferPosition += remainingData;
			return (1);
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

		int16_t eventType = caerEventPacketHeaderGetEventType(packet);
		bool isCompressed = (eventType & 0x8000);
		int16_t eventSource = caerEventPacketHeaderGetEventSource(packet);
		int32_t eventCapacity = caerEventPacketHeaderGetEventCapacity(packet);
		int32_t eventNumber = caerEventPacketHeaderGetEventNumber(packet);
		int32_t eventValid = caerEventPacketHeaderGetEventValid(packet);
		int32_t eventSize = caerEventPacketHeaderGetEventSize(packet);

		// First we verify that the source ID remained unique (only one source per I/O module supported!).
		if (state->header.sourceID != eventSource) {
			caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
				"An input module can only handle packets from the same source! "
					"A packet with source %" PRIi16 " was read, but this input module expects only packets from source %" PRIi16 ". "
				"Discarding event packet.", eventSource, state->header.sourceID);

			// Skip packet. If packet is compressed, eventCapacity carries the size.
			state->packets.skipSize = (isCompressed) ? (size_t) (eventCapacity) : (size_t) (eventNumber * eventSize);
			state->packets.currPacketHeaderSize = 0; // Get new header after skipping.

			// Run function again to skip data. bufferPosition is already up-to-date.
			return (2);
		}

		// If packet is compressed, eventCapacity carries the size in bytes to read.
		state->packets.currPacketDataSize =
			(isCompressed) ? (size_t) (eventCapacity) : (size_t) (eventNumber * eventSize);

		// Allocate space for the full packet, so we can reassemble it (and decompress it later).
		state->packets.currPacket = malloc(CAER_EVENT_PACKET_HEADER_SIZE + (size_t) (eventNumber * eventSize));
		if (state->packets.currPacket == NULL) {
			caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
				"Failed to allocate memory for new event packet.");
			return (-1);
		}

		// First we copy the header in.
		memcpy(state->packets.currPacket, state->packets.currPacketHeader, CAER_EVENT_PACKET_HEADER_SIZE);

		state->packets.currPacketDataOffset = CAER_EVENT_PACKET_HEADER_SIZE;

		// Rewrite event source to reflect this module, not the original one.
		caerEventPacketHeaderSetEventSource(state->packets.currPacket, I16T(state->parentModule->moduleID));

		// If packet was compressed, restore original eventType and eventCapacity,
		// for in-memory usage (no mark bit, eventCapacity == eventNumber).
		if (isCompressed) {
			state->packets.currPacket->eventType = htole16(
				le16toh(state->packets.currPacket->eventType) & I16T(0x7FFF));
			state->packets.currPacket->eventCapacity = htole32(eventNumber);
		}

		// Now we can also start keeping track of this packet's meta-data.
		state->packets.currPacketData = calloc(1, sizeof(struct packetData));
		if (state->packets.currPacketData == NULL) {
			free(state->packets.currPacket);
			state->packets.currPacket = NULL;

			caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
				"Failed to allocate memory for new event packet meta-data.");
			return (-1);
		}

		// Fill out meta-data fields with proper information gained from current event packet.
		state->packets.currPacketData->id = state->packets.packetCount++;
		state->packets.currPacketData->offset =
			(state->isNetworkStream) ?
				(0) : (state->dataBufferOffset + buf->bufferPosition - CAER_EVENT_PACKET_HEADER_SIZE);
		state->packets.currPacketData->size = CAER_EVENT_PACKET_HEADER_SIZE + state->packets.currPacketDataSize;
		state->packets.currPacketData->isCompressed = isCompressed;
		state->packets.currPacketData->eventType = caerEventPacketHeaderGetEventType(state->packets.currPacket);
		state->packets.currPacketData->eventSize = eventSize;
		state->packets.currPacketData->eventNumber = eventNumber;
		state->packets.currPacketData->eventValid = eventValid;
		state->packets.currPacketData->startTimestamp = -1; // Invalid for now.
		state->packets.currPacketData->endTimestamp = -1; // Invalid for now.
	}

	// Now get the data from the buffer to the new event packet. We have to take care of
	// data being split across multiple buffers, as above.
	if (state->packets.currPacketDataSize > remainingData) {
		// We need to copy more data than in this buffer.
		memcpy(((uint8_t *) state->packets.currPacket) + state->packets.currPacketDataOffset,
			buf->buffer + buf->bufferPosition, remainingData);

		state->packets.currPacketDataOffset += remainingData;
		state->packets.currPacketDataSize -= remainingData;

		// Go and get next buffer. bufferPosition is at end of buffer.
		buf->bufferPosition += remainingData;
		return (1);
	}
	else {
		// We copy the last bytes of data and we're done.
		memcpy(((uint8_t *) state->packets.currPacket) + state->packets.currPacketDataOffset,
			buf->buffer + buf->bufferPosition, state->packets.currPacketDataSize);

		// This packet is fully copied and done, so reset variables for next iteration.
		state->packets.currPacketHeaderSize = 0; // Get new header next iteration.
		buf->bufferPosition += state->packets.currPacketDataSize;

		// Decompress packet.
		if (state->packets.currPacketData->isCompressed) {
			if (!decompressEventPacket(state, state->packets.currPacket, state->packets.currPacketData->size)) {
				// Failed to decompress packet. Error exit.
				free(state->packets.currPacket);
				state->packets.currPacket = NULL;
				free(state->packets.currPacketData);
				state->packets.currPacketData = NULL;

				caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
					"Failed to decompress event packet.");
				return (-2);
			}
		}

		// Update timestamp information and insert packet into meta-data list.
		void *firstEvent = caerGenericEventGetEvent(state->packets.currPacket, 0);
		state->packets.currPacketData->startTimestamp = caerGenericEventGetTimestamp64(firstEvent,
			state->packets.currPacket);

		void *lastEvent = caerGenericEventGetEvent(state->packets.currPacket,
			state->packets.currPacketData->eventNumber - 1);
		state->packets.currPacketData->endTimestamp = caerGenericEventGetTimestamp64(lastEvent,
			state->packets.currPacket);

		DL_APPEND(state->packets.packetsList, state->packets.currPacketData);

		// New packet parsed!
		return (0);
	}
}

#ifdef ENABLE_INOUT_PNG_COMPRESSION

static void caerLibPNGReadBuffer(png_structp png_ptr, png_bytep data, png_size_t length);
static bool decompressFramePNG(inputCommonState state, caerEventPacketHeader packet, size_t packetSize);

// Simple structure to store PNG image bytes.
struct caer_libpng_buffer {
	uint8_t *buffer;
	size_t size;
	size_t pos;
};

static void caerLibPNGReadBuffer(png_structp png_ptr, png_bytep data, png_size_t length) {
	struct caer_libpng_buffer *p = (struct caer_libpng_buffer *) png_get_io_ptr(png_ptr);
	size_t newPos = p->pos + length;

	// Detect attempts to read past buffer end.
	if (newPos > p->size) {
		png_error(png_ptr, "Read Buffer Error");
	}

	memcpy(data, p->buffer + p->pos, length);
	p->pos += length;
}

static inline enum caer_frame_event_color_channels caerFrameEventColorFromLibPNG(int channels) {
	switch (channels) {
		case PNG_COLOR_TYPE_GRAY:
			return (GRAYSCALE);
			break;

		case PNG_COLOR_TYPE_RGB:
			return (RGB);
			break;

		case PNG_COLOR_TYPE_RGBA:
		default:
			return (RGBA);
			break;
	}
}

static inline bool caerFrameEventPNGDecompress(uint8_t *inBuffer, size_t inSize, uint16_t *outBuffer, int32_t xSize,
	int32_t ySize, enum caer_frame_event_color_channels channels) {
	png_structp png_ptr = NULL;
	png_infop info_ptr = NULL;

	// Initialize the write struct.
	png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
	if (png_ptr == NULL) {
		return (false);
	}

	// Initialize the info struct.
	info_ptr = png_create_info_struct(png_ptr);
	if (info_ptr == NULL) {
		png_destroy_read_struct(&png_ptr, NULL, NULL);
		return (false);
	}

	// Set up error handling.
	if (setjmp(png_jmpbuf(png_ptr))) {
		png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
		return (false);
	}

	// Handle endianness of 16-bit depth pixels correctly.
	// PNG assumes big-endian, our Frame Event is always little-endian.
	png_set_swap(png_ptr);

	// Set read function to buffer one.
	struct caer_libpng_buffer state = { .buffer = inBuffer, .size = inSize, .pos = 0 };
	png_set_read_fn(png_ptr, &state, &caerLibPNGReadBuffer);

	// Read the whole PNG image.
	png_read_png(png_ptr, info_ptr, PNG_TRANSFORM_IDENTITY, NULL);

	// Extract header info.
	png_uint_32 width = 0, height = 0;
	int bitDepth = 0;
	int color = -1;
	png_get_IHDR(png_ptr, info_ptr, &width, &height, &bitDepth, &color, NULL, NULL, NULL);

	// Check header info against known values from our frame event header.
	if ((I32T(width) != xSize) || (I32T(height) != ySize) || (bitDepth != 16)
		|| (caerFrameEventColorFromLibPNG(color) != channels)) {
		png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
		return (false);
	}

	// Extract image data, row by row.
	png_size_t row_bytes = png_get_rowbytes(png_ptr, info_ptr);
	png_bytepp row_pointers = png_get_rows(png_ptr, info_ptr);

	for (size_t y = 0; y < (size_t) ySize; y++) {
		memcpy(&outBuffer[y * row_bytes], row_pointers[y], row_bytes);
	}

	// Destroy main structs.
	png_destroy_read_struct(&png_ptr, &info_ptr, NULL);

	return (true);
}

static bool decompressFramePNG(inputCommonState state, caerEventPacketHeader packet, size_t packetSize) {
	// We want to avoid allocating new memory for each PNG decompression, and moving around things
	// to much. So we first go through the compressed header+data blocks, and move them to their
	// correct position for an in-memory frame packet (so at N*eventSize). Then we decompress the
	// PNG block and directly copy the results into the space that it was occupying (plus extra for
	// the uncompressed pixels).
	// First we go once through the events to know where they are, and where they should go.
	// Then we do memory move + PNG decompression, starting from the last event (back-side), so as
	// to not overwrite memory we still need and haven't moved yet.
	int32_t eventSize = caerEventPacketHeaderGetEventSize(packet);
	int32_t eventNumber = caerEventPacketHeaderGetEventNumber(packet);

	struct {
		size_t offsetDestination;
		size_t offset;
		size_t size;
		bool isCompressed;
	} eventMemory[eventNumber];

	size_t currPacketOffset = CAER_EVENT_PACKET_HEADER_SIZE; // Start here, no change to header.
	size_t frameEventHeaderSize = sizeof(struct caer_frame_event);

	// Gather information on events.
	for (int32_t i = 0; i < eventNumber; i++) {
		// In-memory packet's events have to go where N*eventSize is.
		eventMemory[i].offsetDestination = CAER_EVENT_PACKET_HEADER_SIZE + (size_t) (i * eventSize);
		eventMemory[i].offset = currPacketOffset;

		caerFrameEvent frameEvent = (caerFrameEvent) (((uint8_t *) packet) + currPacketOffset);

		// Bit 31 of info signals if event is PNG-compressed or not.
		eventMemory[i].isCompressed = GET_NUMBITS32(frameEvent->info, 31, 0x01);

		if (eventMemory[i].isCompressed) {
			// Clear compression enabled bit.
			CLEAR_NUMBITS32(frameEvent->info, 31, 0x01);

			// Compressed block size is held in an integer right after the header.
			int32_t pngSize = le32toh(*((int32_t * ) (((uint8_t * ) frameEvent) + frameEventHeaderSize)));

			// PNG size is header plus integer plus compressed block size.
			eventMemory[i].size = frameEventHeaderSize + sizeof(int32_t) + (size_t) pngSize;
		}
		else {
			// Normal size is header plus uncompressed pixels.
			eventMemory[i].size = frameEventHeaderSize + caerFrameEventGetPixelsSize(frameEvent);
		}

		// Update counter.
		currPacketOffset += eventMemory[i].size;
	}

	// Check that we indeed parsed everything correctly.
	if (currPacketOffset != packetSize) {
		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Failed to decompress frame event. "
			"Size after event parsing and packet size don't match.");
		return (false);
	}

	// Now move memory and decompress in reverse order.
	for (int32_t i = eventNumber; i >= 0; i--) {
		// Move memory from compressed position to uncompressed, in-memory position.
		memmove(((uint8_t *) packet) + eventMemory[i].offsetDestination, ((uint8_t *) packet) + eventMemory[i].offset,
			eventMemory[i].size);

		// If event is PNG-compressed, decompress it now.
		if (eventMemory[i].isCompressed) {
			uint8_t *pngBuffer = ((uint8_t *) packet) + eventMemory[i].offsetDestination + frameEventHeaderSize
				+ sizeof(int32_t);
			size_t pngBufferSize = eventMemory[i].size - frameEventHeaderSize - sizeof(int32_t);

			caerFrameEvent frameEvent = (caerFrameEvent) (((uint8_t *) packet) + eventMemory[i].offsetDestination);

			if (!caerFrameEventPNGDecompress(pngBuffer, pngBufferSize, caerFrameEventGetPixelArrayUnsafe(frameEvent),
				caerFrameEventGetLengthX(frameEvent), caerFrameEventGetLengthY(frameEvent),
				caerFrameEventGetChannelNumber(frameEvent))) {
				// Failed to decompress PNG.
				caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Failed to decompress frame event. "
					"PNG decompression failure.");
				return (false);
			}
		}

		// Uncompressed size will always be header + uncompressed pixels.
		caerFrameEvent frameEvent = (caerFrameEvent) (((uint8_t *) packet) + eventMemory[i].offsetDestination);
		size_t uncompressedSize = frameEventHeaderSize + caerFrameEventGetPixelsSize(frameEvent);

		// Initialize the rest of the memory of the event to zeros, to comply with spec
		// that says non-pixels at the end, if they exist, are always zero.
		memset(((uint8_t *) packet) + eventMemory[i].offsetDestination + uncompressedSize, 0,
			(size_t) eventSize - uncompressedSize);
	}

	return (true);
}

#endif

static bool decompressTimestampSerialize(inputCommonState state, caerEventPacketHeader packet, size_t packetSize) {
	// To decompress this, we have to allocate memory to hold the expanded events. There is
	// no efficient way to avoid this; working backwards from the last compressed event might
	// be an option, but you'd have to track where all the events are during a first forward
	// pass, and keeping track of offset, ts, numEvents for each group would incur similar
	// memory consumption, while considerably increasing complexity. So let's just do the
	// simple thing.
	int32_t eventSize = caerEventPacketHeaderGetEventSize(packet);
	int32_t eventNumber = caerEventPacketHeaderGetEventNumber(packet);
	int32_t eventTSOffset = caerEventPacketHeaderGetEventTSOffset(packet);

	uint8_t *events = malloc((size_t) (eventNumber * eventSize));
	if (events == NULL) {
		// Memory allocation failure.
		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Failed to decode serialized timestamp. "
			"Memory allocation failure.");
		return (false);
	}

	size_t currPacketOffset = CAER_EVENT_PACKET_HEADER_SIZE; // Start here, no change to header.
	size_t recoveredEventsPosition = 0;
	size_t recoveredEventsNumber = 0;

	while (currPacketOffset < packetSize) {
		void *firstEvent = ((uint8_t *) packet) + currPacketOffset;
		int32_t currTS = caerGenericEventGetTimestamp(firstEvent, packet);

		if (currTS & I32T(0x80000000)) {
			// Compressed run starts here! Must clear the compression bit from
			// this first timestamp and restore the timestamp to the others.
			// So first we clean the timestamp.
			currTS &= I32T(0x7FFFFFFF);

			// Then we fix the first event timestamp and copy it over.
			caerGenericEventSetTimestamp(firstEvent, packet, currTS);
			memcpy(events + recoveredEventsPosition, firstEvent, (size_t) eventSize);

			currPacketOffset += (size_t) eventSize;
			recoveredEventsPosition += (size_t) eventSize;
			recoveredEventsNumber++;

			// Then we get the second event, and get its timestamp, which is
			// actually the size of the following compressed run.
			void *secondEvent = ((uint8_t *) packet) + currPacketOffset;
			int32_t tsRun = caerGenericEventGetTimestamp(secondEvent, packet);

			// And fix its own timestamp back to what it should be, and copy it.
			caerGenericEventSetTimestamp(secondEvent, packet, currTS);
			memcpy(events + recoveredEventsPosition, secondEvent, (size_t) eventSize);

			currPacketOffset += (size_t) eventSize;
			recoveredEventsPosition += (size_t) eventSize;
			recoveredEventsNumber++;

			// Now go through the compressed, data-only events, and restore their
			// timestamp. We do this by copying the data and then adding the timestamp,
			// which is always the last in an event.
			while (tsRun > 0) {
				void *thirdEvent = ((uint8_t *) packet) + currPacketOffset;
				memcpy(events + recoveredEventsPosition, thirdEvent, (size_t) eventTSOffset);

				currPacketOffset += (size_t) eventTSOffset;
				recoveredEventsPosition += (size_t) eventTSOffset;

				int32_t *newTS = (int32_t *) (events + recoveredEventsPosition);
				*newTS = currTS;

				recoveredEventsPosition += sizeof(int32_t);

				recoveredEventsNumber++;
				tsRun--;
			}
		}
		else {
			// Normal event, nothing compressed.
			// Just copy and advance.
			memcpy(events + recoveredEventsPosition, firstEvent, (size_t) eventSize);

			currPacketOffset += (size_t) eventSize;
			recoveredEventsPosition += (size_t) eventSize;
			recoveredEventsNumber++;
		}
	}

	// Check we really recovered all events from compression.
	if (currPacketOffset != packetSize) {
		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Failed to decode serialized timestamp. "
			"Length of compressed packet and read data don't match.");
		return (false);
	}

	if ((size_t) (eventNumber * eventSize) != recoveredEventsPosition) {
		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Failed to decode serialized timestamp. "
			"Length of uncompressed packet and uncompressed data don't match.");
		return (false);
	}

	if ((size_t) eventNumber != recoveredEventsNumber) {
		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Failed to decode serialized timestamp. "
			"Number of expected and recovered events don't match.");
		return (false);
	}

	// Copy recovered event packet into original.
	memcpy(((uint8_t *) packet) + CAER_EVENT_PACKET_HEADER_SIZE, events, recoveredEventsPosition);

	free(events);

	return (true);
}

static bool decompressEventPacket(inputCommonState state, caerEventPacketHeader packet, size_t packetSize) {
	bool retVal = false;

	// Data compression technique 1: serialized timestamps.
	if ((state->header.formatID & 0x01) && caerEventPacketHeaderGetEventType(packet) == POLARITY_EVENT) {
		retVal = decompressTimestampSerialize(state, packet, packetSize);
	}

#ifdef ENABLE_INOUT_PNG_COMPRESSION
	// Data compression technique 2: frame PNG compression.
	if ((state->header.formatID & 0x02) && caerEventPacketHeaderGetEventType(packet) == FRAME_EVENT) {
		retVal = decompressFramePNG(state, packet, packetSize);
	}
#endif

	return (retVal);
}

static bool processPacket(inputCommonState state) {
	// We've got a full event packet, store it. It will later appear in the packet
	// container in some form.
	addToPacketContainer(state, state->packets.currPacket);

	// Check if we have read and accumulated all the event packets with a main first timestamp smaller
	// or equal than what we want. We know this is the case when the last seen main timestamp is clearly
	// bigger than the wanted one. If this is true, it means we do have all the possible events of all
	// types that happen up until that point, and we can split that time range off into a packet container.
	// If not, we just go get the next event packet.
	if (state->packetContainer.lastSeenPacketTimestamp <= state->packetContainer.newContainerTimestampEnd) {
		return (false);
	}

	caerEventPacketContainer packetContainer = generatePacketContainer(state);
	if (packetContainer == NULL) {
		// On failure, just continue.
		return (false);
	}

	doTimeDelay(state);

	doPacketContainerCommit(state, packetContainer);

	return (true);
}

static int inputHandlerThread(void *stateArg) {
	inputCommonState state = stateArg;

	// Set thread name.
	thrd_set_name(state->parentModule->moduleSubSystemString);

	// Set thread priority to high. This may fail depending on your OS configuration.
	if (thrd_set_priority(-1) != thrd_success) {
		caerLog(CAER_LOG_INFO, state->parentModule->moduleSubSystemString,
			"Failed to raise thread priority for Input Handler thread. You may experience lags and delays.");
	}

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
				// Flush last event packets/packet container on EOF.
				caerEventPacketContainer packetContainer = generatePacketContainer(state);
				if (packetContainer != NULL) {
					doTimeDelay(state);

					doPacketContainerCommit(state, packetContainer);
				}

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

		// Parse event data now.
		if (!parseData(state)) {
			// Packets invalid, exit.
			caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Failed to parse event data.");
			break;
		}

		// Go and get a full buffer on next iteration again, starting at position 0.
		state->dataBuffer->bufferPosition = 0;

		// Update offset. Makes sense for files only.
		if (!state->isNetworkStream) {
			state->dataBufferOffset += state->dataBuffer->bufferUsedSize;
		}
	}

	// At this point we either got terminated (running=false) or we stopped for some
	// reason: parsing error or End-of-File.
	// If we got hard-terminated, we empty the ring-buffer in the Exit() state.
	// If we hit EOF/parse errors though, we want the consumers to be able to finish
	// consuming the already produced data, so we wait for the ring-buffer to be empty.
	if (atomic_load(&state->running)) {
		while (atomic_load(&state->mainloopReference->dataAvailable) != 0) {
			;
		}

		// Ensure parent also shuts down, for example on read failures.
		sshsNodePutBool(state->parentModule->moduleNode, "running", false);
	}

	return (thrd_success);
}

static const UT_icd ut_caerEventPacketHeader_icd = { sizeof(caerEventPacketHeader), NULL, NULL, NULL };

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

	sshsNodePutIntIfAbsent(moduleData->moduleNode, "timeSlice", 10000); // in µs, size of time slice to generate
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "timeDelay", 10000); // in µs, delay between consecutive slices

	atomic_store(&state->validOnly, sshsNodeGetBool(moduleData->moduleNode, "validOnly"));
	atomic_store(&state->keepPackets, sshsNodeGetBool(moduleData->moduleNode, "keepPackets"));

	atomic_store(&state->packetContainer.timeSlice, sshsNodeGetInt(moduleData->moduleNode, "timeSlice"));
	atomic_store(&state->packetContainer.timeDelay, sshsNodeGetInt(moduleData->moduleNode, "timeDelay"));

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

	// Initialize array for packets -> packet container.
	utarray_new(state->packetContainer.eventPackets, &ut_caerEventPacketHeader_icd);

	state->packetContainer.newContainerTimestampStart = -1;
	state->packetContainer.newContainerTimestampEnd = -1;

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

		// If we're here, then nobody will consume this data afterwards.
		atomic_fetch_sub_explicit(&state->mainloopReference->dataAvailable, 1, memory_order_relaxed);
	}

	ringBufferFree(state->transferRing);

	// Free all waiting packets.
	caerEventPacketHeader *packet = NULL;
	while ((packet = (caerEventPacketHeader *) utarray_next(state->packetContainer.eventPackets, packet)) != NULL) {
		free(*packet);
	}

	// Clear and free packet array used for packet container construction.
	utarray_free(state->packetContainer.eventPackets);

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

		sshsNodePutLong(sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/"), "highestTimestamp",
			caerEventPacketContainerGetHighestEventTimestamp(*container));

		caerSpecialEventPacket special = (caerSpecialEventPacket) caerEventPacketContainerGetEventPacket(*container,
			SPECIAL_EVENT);

		if (special != NULL) {
			caerSpecialEvent tsResetEvent = caerSpecialEventPacketFindEventByType(special, TIMESTAMP_RESET);

			if (tsResetEvent != NULL) {
				caerMainloopResetProcessors();
				caerMainloopResetOutputs();
			}
		}
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
		else if (changeType == INT && caerStrEquals(changeKey, "timeSlice")) {
			atomic_store(&state->packetContainer.timeSlice, changeValue.iint);
		}
		else if (changeType == INT && caerStrEquals(changeKey, "timeDelay")) {
			atomic_store(&state->packetContainer.timeDelay, changeValue.iint);
		}
	}
}
