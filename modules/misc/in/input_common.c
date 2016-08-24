#include "input_common.h"
#include "input_visualizer_eventhandler.h"
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
#include <libcaer/events/special.h>
#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>

#define MAX_HEADER_LINE_SIZE 1024

enum input_reader_state {
	READER_OK = 0, EOF_REACHED = 1, ERROR_READ = -1, ERROR_HEADER = -2, ERROR_DATA = -3,
};

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

struct input_packet_data {
	/// Numerical ID of a packet. First packet has ID 0.
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
	struct input_packet_data *prev, *next;
};

typedef struct input_packet_data *packetData;

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
	packetData currPacketData;
	/// List of data on all parsed original packets from the input.
	packetData packetsList;
	/// Global packet counter.
	size_t packetCount;
};

struct input_common_packet_container_data {
	/// Current events, merged into packets, sorted by type.
	UT_array *eventPackets;
	/// The first main timestamp (the one relevant for packet ordering in streams)
	/// of the last event packet that was handled.
	int64_t lastPacketTimestamp;
	/// Track tsOverflow value. On change, we must commit the current packet
	/// container content and empty it out.
	int32_t lastTimestampOverflow;
	/// Size limit reached in any packet.
	bool sizeLimitHit;
	/// The timestamp that needs to be read up to, so that the size limit can
	/// actually be committed, because we know no other events are around.
	int64_t sizeLimitTimestamp;
	/// The timestamp up to which we want to (have to!) read, so that we can
	/// output the next packet container (in time-slice mode).
	int64_t newContainerTimestampEnd;
	/// The size limit that triggered the hit above.
	int32_t newContainerSizeLimit;
	/// Size slice (in events), for which to generate a packet container.
	atomic_int_fast32_t sizeSlice;
	/// Time slice (in µs), for which to generate a packet container.
	atomic_int_fast32_t timeSlice;
	/// Time delay (in µs) between the start of two consecutive time slices.
	/// This is used for real-time slow-down.
	atomic_int_fast32_t timeDelay;
	/// Time when the last packet container was sent out, used to calculate
	/// sleep time to reach user configured 'timeDelay'.
	struct timespec lastCommitTime;
};

struct input_common_state {
	/// Control flag for input handling threads.
	atomic_bool running;
	/// Reader thread state, to signal conditions like EOF or error to
	/// the assembler thread.
	atomic_int_fast32_t inputReaderThreadState;
	/// The first input handling thread (separate as to only wake up mainloop
	/// processing when there is new data available): takes care of data
	/// reading and parsing, decompression from the input channel.
	thrd_t inputReaderThread;
	/// The first input handling thread (separate as to only wake up mainloop
	/// processing when there is new data available): takes care of assembling
	/// packet containers that respect the specs using the packets read by
	/// the inputReadThread. This is separate so that delay operations don't
	/// use up resources that could be doing read/decompression work.
	thrd_t inputAssemblerThread;
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
	/// Pause support.
	atomic_bool pause;
	/// Transfer packets coming from the input reading thread to the assembly
	/// thread. Normal EventPackets are used here.
	RingBuffer transferRingPackets;
	/// Transfer packet containers coming from the input assembly thread to
	/// the mainloop. We use EventPacketContainers, as that is the standard
	/// data structure returned from an input module.
	RingBuffer transferRingPacketContainers;
	/// Track how many packet containers are in the ring-buffer, ready for
	/// consumption by the user. The Mainloop's 'dataAvailable' variable already
	/// does this at a global level, but we also need to keep track at a local
	/// (module) level of this, to avoid confusion in the case multiple Inputs
	/// are inside the same Mainloop, which is entirely possible and supported.
	atomic_uint_fast32_t dataAvailableModule;
	/// Header parsing results.
	struct input_common_header_info header;
	/// Packet data parsing structures.
	struct input_common_packet_data packets;
	/// Packet container data structure, to generate from packets.
	struct input_common_packet_container_data packetContainer;
	/// The file descriptor for reading.
	int fileDescriptor;
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
	/// Reference to sourceInfo node (to avoid getting it each time again).
	sshsNode sourceInfoNode;
};

typedef struct input_common_state *inputCommonState;

size_t CAER_INPUT_COMMON_STATE_STRUCT_SIZE = sizeof(struct input_common_state);

static bool newInputBuffer(inputCommonState state);
static bool parseNetworkHeader(inputCommonState state);
static char *getFileHeaderLine(inputCommonState state);
static void parseSourceString(char *sourceString, inputCommonState state);
static bool parseFileHeader(inputCommonState state);
static bool parseHeader(inputCommonState state);
static bool parseData(inputCommonState state);
static int aedat2GetPacket(inputCommonState state, int16_t chipID);
static int aedat3GetPacket(inputCommonState state, bool isAEDAT30);
static void aedat30ChangeOrigin(inputCommonState state, caerEventPacketHeader packet);
static bool decompressTimestampSerialize(inputCommonState state, caerEventPacketHeader packet, size_t packetSize);
static bool decompressEventPacket(inputCommonState state, caerEventPacketHeader packet, size_t packetSize);
static int inputReaderThread(void *stateArg);

static bool addToPacketContainer(inputCommonState state, caerEventPacketHeader newPacket, packetData newPacketData);
static caerEventPacketContainer generatePacketContainer(inputCommonState state, bool forceFlush);
static void commitPacketContainer(inputCommonState state, bool forceFlush);
static void doTimeDelay(inputCommonState state);
static void doPacketContainerCommit(inputCommonState state, caerEventPacketContainer packetContainer, bool force);
static bool handleTSReset(inputCommonState state);
static void getPacketInfo(caerEventPacketHeader packet, packetData packetInfoData);
static int inputAssemblerThread(void *stateArg);

static void caerInputCommonConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static int packetsFirstTypeThenSizeCmp(const void *a, const void *b);

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
	networkHeader.sourceID = le16toh(networkHeader.sourceID);

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
		// TODO: Network: check this for missing packets in message mode!
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

	// TODO: Network: get sourceInfo node info via config-server side-channel.
	state->header.sourceID = networkHeader.sourceID;
	sshsNodePutShort(state->sourceInfoNode, "dvsSizeX", 240);
	sshsNodePutShort(state->sourceInfoNode, "dvsSizeY", 180);
	sshsNodePutShort(state->sourceInfoNode, "apsSizeX", 240);
	sshsNodePutShort(state->sourceInfoNode, "apsSizeY", 180);

	// TODO: Network: add sourceString.

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
		sshsNodePutShort(state->sourceInfoNode, "dvsSizeX", dvsSizeX);
		sshsNodePutShort(state->sourceInfoNode, "dvsSizeY", dvsSizeY);
	}

	if (apsSizeX != 0 && apsSizeY != 0) {
		sshsNodePutShort(state->sourceInfoNode, "apsSizeX", apsSizeX);
		sshsNodePutShort(state->sourceInfoNode, "apsSizeY", apsSizeY);
	}

	if (dataSizeX == 0 && dataSizeY == 0) {
		// Try to auto-discover dataSize, if it was not previously set, based on the
		// presence of DVS or APS sizes. If they don't exist either, this will be 0.
		dataSizeX = (dvsSizeX > apsSizeX) ? (dvsSizeX) : (apsSizeX);
		dataSizeY = (dvsSizeY > apsSizeY) ? (dvsSizeY) : (apsSizeY);
	}

	if (dataSizeX != 0 && dataSizeY != 0) {
		sshsNodePutShort(state->sourceInfoNode, "dataSizeX", dataSizeX);
		sshsNodePutShort(state->sourceInfoNode, "dataSizeY", dataSizeY);
	}

	if (visualizerSizeX != 0 && visualizerSizeY != 0) {
		sshsNodePutShort(state->sourceInfoNode, "visualizerSizeX", visualizerSizeX);
		sshsNodePutShort(state->sourceInfoNode, "visualizerSizeY", visualizerSizeY);
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

	sshsNodePutString(state->sourceInfoNode, "sourceString", sourceStringFile);
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

					if (strstr(formatString, "SerializedTS") != NULL) {
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
				if (caerStrEqualsUpTo(headerLine, "#Start-Time: ", 13)) {
					char startTimeString[1024 + 1];

					if (sscanf(headerLine, "#Start-Time: %1024[^\r]s\n", startTimeString) == 1) {
						caerLog(CAER_LOG_INFO, state->parentModule->moduleSubSystemString, "Recording was taken on %s.",
							startTimeString);
					}
				}
				else if (caerStrEqualsUpTo(headerLine, "#-Source ", 9)) {
					// Detect negative source strings (#-Source) and add them to sourceInfo.
					// Previous sources are simply appended to the sourceString string in order.
					char *currSourceString = sshsNodeGetString(state->sourceInfoNode, "sourceString");
					size_t currSourceStringLength = strlen(currSourceString);

					size_t addSourceStringLength = strlen(headerLine);

					char *newSourceString = realloc(currSourceString,
						currSourceStringLength + addSourceStringLength + 1); // +1 for NUL termination.
					if (newSourceString == NULL) {
						// Memory reallocation failure, skip this negative source string.
						free(currSourceString);
					}
					else {
						// Concatenate negative source string and commit as new sourceString.
						memcpy(newSourceString + currSourceStringLength, headerLine, addSourceStringLength);
						newSourceString[currSourceStringLength + addSourceStringLength] = '\0';

						sshsNodePutString(state->sourceInfoNode, "sourceString", newSourceString);
						free(newSourceString);
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

static bool parseData(inputCommonState state) {
	while (state->dataBuffer->bufferPosition < state->dataBuffer->bufferUsedSize) {
		int pRes = -1;

		// Try getting packet and packetData from buffer.
		if (state->header.majorVersion == 2 && state->header.minorVersion == 0) {
			pRes = aedat2GetPacket(state, 0);
		}
		else if (state->header.majorVersion == 3) {
			pRes = aedat3GetPacket(state, (state->header.minorVersion == 0));
		}
		else {
			// No parseable format found!
			return (false);
		}

		// Check packet parser return value.
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

		caerLog(CAER_LOG_DEBUG, state->parentModule->moduleSubSystemString,
			"New packet read - ID: %zu, Offset: %zu, Size: %zu, Events: %" PRIi32 ", Type: %" PRIi16 ", StartTS: %" PRIi64 ", EndTS: %" PRIi64 ".",
			state->packets.currPacketData->id, state->packets.currPacketData->offset,
			state->packets.currPacketData->size, state->packets.currPacketData->eventNumber,
			state->packets.currPacketData->eventType, state->packets.currPacketData->startTimestamp,
			state->packets.currPacketData->endTimestamp);

		// New packet information, add it to the global packet info list.
		// This is done here to prevent ambiguity about the ownership of the involved memory block:
		// it either is inside the global list with state->packets.currPacketData NULL, or it is not
		// in the list, but in state->packets.currPacketData itself. So if, on exit, we clear both,
		// we'll free all the memory and have no fear of a double-free happening.
		DL_APPEND(state->packets.packetsList, state->packets.currPacketData);
		state->packets.currPacketData = NULL;

		// New packet from stream, send it off to the input assembler thread. Same memory
		// related considerations as above for state->packets.currPacketData apply here too!
		while (!ringBufferPut(state->transferRingPackets, state->packets.currPacket)) {
			// We ensure all read packets are sent to the Assembler stage.
			if (!atomic_load_explicit(&state->running, memory_order_relaxed)) {
				// On normal termination, just return without errors. The Reader thread
				// will then also exit without errors and clean up in Exit().
				return (true);
			}

			// Delay by 10 µs if no change, to avoid a wasteful busy loop.
			struct timespec retrySleep = { .tv_sec = 0, .tv_nsec = 10000 };
			thrd_sleep(&retrySleep, NULL);
		}

		state->packets.currPacket = NULL;
	}

	// All good, get next buffer.
	return (true);
}

/**
 * Parse the current buffer and try to extract the AEDAT 2.0
 * data contained within, to form a compliant AEDAT 3.1 packet,
 * and then update the packet meta-data list with it.
 *
 * @param state common input data structure.
 * @param chipID chip identifier to decide sizes, ordering and
 * features of the data stream for conversion to AEDAT 3.1.
 *
 * @return 0 on successful packet extraction.
 * Positive numbers for special conditions:
 * 1 if more data needed.
 * Negative numbers on error conditions:
 * -1 on memory allocation failure.
 */
static int aedat2GetPacket(inputCommonState state, int16_t chipID) {
	UNUSED_ARGUMENT(chipID);

	// TODO: AEDAT 2.0 not yet supported.
	caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Reading AEDAT 2.0 data not yet supported.");
	return (-1);
}

/**
 * Parse the current buffer and try to extract the AEDAT 3.X
 * packet contained within, as well as updating the packet
 * meta-data list.
 *
 * @param state common input data structure.
 * @param isAEDAT30 change the X/Y coordinate origin for Frames and Polarity
 * events, as this changed from 3.0 (lower left) to 3.1 (upper left).
 *
 * @return 0 on successful packet extraction.
 * Positive numbers for special conditions:
 * 1 if more data needed.
 * 2 if skip requested (call again).
 * Negative numbers on error conditions:
 * -1 on memory allocation failure.
 * -2 on decompression failure.
 */
static int aedat3GetPacket(inputCommonState state, bool isAEDAT30) {
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
		state->packets.currPacketData = calloc(1, sizeof(struct input_packet_data));
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

		// If the file was in AEDAT 3.0 format, we must change X/Y coordinate origin
		// for Polarity and Frame events. We do this after parsing and decompression.
		if (isAEDAT30) {
			aedat30ChangeOrigin(state, state->packets.currPacket);
		}

		// New packet parsed!
		return (0);
	}
}

static void aedat30ChangeOrigin(inputCommonState state, caerEventPacketHeader packet) {
	if (caerEventPacketHeaderGetEventType(packet) == POLARITY_EVENT) {
		// We need to know the DVS resolution to invert the polarity Y address.
		int16_t dvsSizeY = I16T(sshsNodeGetShort(state->sourceInfoNode, "dvsSizeY") - 1);

		CAER_POLARITY_ITERATOR_ALL_START((caerPolarityEventPacket) packet)
			uint16_t newYAddress = U16T(dvsSizeY - caerPolarityEventGetY(caerPolarityIteratorElement));
			caerPolarityEventSetY(caerPolarityIteratorElement, newYAddress);
		}
	}

	if (caerEventPacketHeaderGetEventType(packet) == FRAME_EVENT) {
		// For frames, the resolution and size information is carried in each event.
		CAER_FRAME_ITERATOR_ALL_START((caerFrameEventPacket) packet)
			int32_t lengthX = caerFrameEventGetLengthX(caerFrameIteratorElement);
			int32_t lengthY = caerFrameEventGetLengthY(caerFrameIteratorElement);
			enum caer_frame_event_color_channels channels = caerFrameEventGetChannelNumber(caerFrameIteratorElement);

			size_t rowSize = (size_t) lengthX * channels;

			uint16_t *pixels = caerFrameEventGetPixelArrayUnsafe(caerFrameIteratorElement);

			// Invert position of entire rows.
			for (size_t y = 0; y < (size_t) lengthY; y++) {
				size_t invY = (size_t) lengthY - 1 - y;

				// Don't invert if no position change, this happens in the exact
				// middle if lengthY is uneven.
				if (y != invY) {
					memcpy(&pixels[y * rowSize], &pixels[invY * rowSize], rowSize);
				}
			}
		}
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

static int inputReaderThread(void *stateArg) {
	inputCommonState state = stateArg;

	// Set thread name.
	size_t threadNameLength = strlen(state->parentModule->moduleSubSystemString);
	char threadName[threadNameLength + 1 + 8]; // +1 for NUL character.
	strcpy(threadName, state->parentModule->moduleSubSystemString);
	strcat(threadName, "[Reader]");
	thrd_set_name(threadName);

	// Set thread priority to high. This may fail depending on your OS configuration.
	if (thrd_set_priority(-1) != thrd_success) {
		caerLog(CAER_LOG_INFO, state->parentModule->moduleSubSystemString,
			"Failed to raise thread priority for Input Reader thread. You may experience lags and delays.");
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
				caerLog(CAER_LOG_INFO, state->parentModule->moduleSubSystemString, "Reached End of File.");
				atomic_store(&state->inputReaderThreadState, EOF_REACHED); // EOF
			}
			else {
				caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
					"Error while reading data, error: %d.", errno);
				atomic_store(&state->inputReaderThreadState, ERROR_READ); // Error
			}
			break;
		}

		// Parse header and setup header info structure.
		if (!state->header.isValidHeader && !parseHeader(state)) {
			// Header invalid, exit.
			caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Failed to parse header.");
			atomic_store(&state->inputReaderThreadState, ERROR_HEADER); // Error in Header
			break;
		}

		// Parse event data now.
		if (!parseData(state)) {
			// Packets invalid, exit.
			caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Failed to parse event data.");
			atomic_store(&state->inputReaderThreadState, ERROR_DATA); // Error in Data
			break;
		}

		// Go and get a full buffer on next iteration again, starting at position 0.
		state->dataBuffer->bufferPosition = 0;

		// Update offset. Makes sense for files only.
		if (!state->isNetworkStream) {
			state->dataBufferOffset += state->dataBuffer->bufferUsedSize;
		}
	}

	return (thrd_success);
}

static inline void updateSizeCommitCriteria(inputCommonState state, caerEventPacketHeader newPacket) {
	if ((state->packetContainer.newContainerSizeLimit > 0)
		&& (caerEventPacketHeaderGetEventNumber(newPacket) >= state->packetContainer.newContainerSizeLimit)) {
		void *sizeLimitEvent = caerGenericEventGetEvent(newPacket, state->packetContainer.newContainerSizeLimit - 1);
		int64_t sizeLimitTimestamp = caerGenericEventGetTimestamp64(sizeLimitEvent, newPacket);

		// Reject the size limit if its corresponding timestamp isn't smaller than the time limit.
		// If not (>=), then the time limit will hit first anyway and take precedence.
		if (sizeLimitTimestamp < state->packetContainer.newContainerTimestampEnd) {
			state->packetContainer.sizeLimitHit = true;

			if (sizeLimitTimestamp < state->packetContainer.sizeLimitTimestamp) {
				state->packetContainer.sizeLimitTimestamp = sizeLimitTimestamp;
			}
		}
	}
}

/**
 * Add the given packet to a packet container that acts as accumulator. This way all
 * events are in a common place, from which the right event amounts/times can be sliced.
 * Packets are unique by type and event size, since for a packet of the same type, the
 * only global things that can change are the source ID and the event size (like for
 * Frames). The source ID is guaranteed to be the same from one source only when using
 * the input module, so we only have to check for the event size in addition to the type.
 *
 * @param state common input data structure.
 * @param newPacket packet to add/merge with accumulator packet container.
 * @param newPacketData information on the new packet.
 *
 * @return true on successful packet merge, false on failure (memory allocation).
 */
static bool addToPacketContainer(inputCommonState state, caerEventPacketHeader newPacket, packetData newPacketData) {
	bool packetAlreadyExists = false;
	caerEventPacketHeader *packet = NULL;
	while ((packet = (caerEventPacketHeader *) utarray_next(state->packetContainer.eventPackets, packet)) != NULL) {
		int16_t packetEventType = caerEventPacketHeaderGetEventType(*packet);
		int32_t packetEventSize = caerEventPacketHeaderGetEventSize(*packet);

		if (packetEventType == newPacketData->eventType && packetEventSize == newPacketData->eventSize) {
			// Packet with this type and event size already present.
			packetAlreadyExists = true;
			break;
		}
	}

	// Packet with same type and event size as newPacket found, do merge operation.
	if (packetAlreadyExists) {
		// Merge newPacket with '*packet'. Since packets from the same source,
		// and having the same time, are guaranteed to have monotonic timestamps,
		// the merge operation becomes a simple append operation.
		caerEventPacketHeader mergedPacket = caerGenericEventPacketAppend(*packet, newPacket);
		if (mergedPacket == NULL) {
			caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
				"%s: Failed to allocate memory for packet merge operation.", __func__);
			return (false);
		}

		// Merged content with existing packet, data copied: free new one.
		// Update references to old/new packets to point to merged one.
		free(newPacket);
		*packet = mergedPacket;
		newPacket = mergedPacket;
	}
	else {
		// No previous packet of this type and event size found, use this one directly.
		utarray_push_back(state->packetContainer.eventPackets, &newPacket);

		utarray_sort(state->packetContainer.eventPackets, &packetsFirstTypeThenSizeCmp);
	}

	// Update size commit criteria, if size limit is enabled and not already hit by a previous packet.
	updateSizeCommitCriteria(state, newPacket);

	return (true);
}

static caerEventPacketContainer generatePacketContainer(inputCommonState state, bool forceFlush) {
	// Let's generate a packet container, use the size of the event packets array as upper bound.
	int32_t packetContainerPosition = 0;
	caerEventPacketContainer packetContainer = caerEventPacketContainerAllocate(
		(int32_t) utarray_len(state->packetContainer.eventPackets));
	if (packetContainer == NULL) {
		return (NULL);
	}

	// When we force a flush commit, we put everything currently there in the packet
	// container and return it, with no slicing being done at all.
	if (forceFlush) {
		caerEventPacketHeader *currPacket = NULL;
		while ((currPacket = (caerEventPacketHeader *) utarray_next(state->packetContainer.eventPackets, currPacket))
			!= NULL) {
			caerEventPacketContainerSetEventPacket(packetContainer, packetContainerPosition++, *currPacket);
		}

		// Clean packets array, they are all being sent out now.
		utarray_clear(state->packetContainer.eventPackets);
	}
	else {
		// Iterate over each event packet, and slice out the relevant part in time.
		caerEventPacketHeader *currPacket = NULL;
		while ((currPacket = (caerEventPacketHeader *) utarray_next(state->packetContainer.eventPackets, currPacket))
			!= NULL) {
			// Search for cutoff point, either reaching the size limit first, or then the time limit.
			// Also count valid events encountered for later setting the right values in the packet.
			int32_t cutoffIndex = -1;
			int32_t validEventsSeen = 0;

			CAER_ITERATOR_ALL_START(*currPacket, void *)
				int64_t caerIteratorElementTimestamp = caerGenericEventGetTimestamp64(caerIteratorElement, *currPacket);

				if ((state->packetContainer.sizeLimitHit
					&& ((caerIteratorCounter >= state->packetContainer.newContainerSizeLimit)
						|| (caerIteratorElementTimestamp > state->packetContainer.sizeLimitTimestamp)))
					|| (caerIteratorElementTimestamp > state->packetContainer.newContainerTimestampEnd)) {
					cutoffIndex = caerIteratorCounter;
					break;
				}

				if (caerGenericEventIsValid(caerIteratorElement)) {
					validEventsSeen++;
				}
			}

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
				caerLog(CAER_LOG_CRITICAL, state->parentModule->moduleSubSystemString,
					"Failed memory allocation for nextPacket. Discarding remaining data.");
			}
			else {
				// Copy header and remaining events to new packet, set header sizes correctly.
				memcpy(nextPacket, *currPacket, CAER_EVENT_PACKET_HEADER_SIZE);
				memcpy(((uint8_t *) nextPacket) + CAER_EVENT_PACKET_HEADER_SIZE,
					((uint8_t *) *currPacket) + CAER_EVENT_PACKET_HEADER_SIZE + (currPacketEventSize * cutoffIndex),
					(size_t) (currPacketEventSize * nextPacketEventNumber));

				caerEventPacketHeaderSetEventValid(nextPacket, currPacketEventValid - validEventsSeen);
				caerEventPacketHeaderSetEventNumber(nextPacket, nextPacketEventNumber);
				caerEventPacketHeaderSetEventCapacity(nextPacket, nextPacketEventNumber);
			}

			// Resize current packet to include only the events up until cutoff point.
			caerEventPacketHeader currPacketResized = realloc(*currPacket,
			CAER_EVENT_PACKET_HEADER_SIZE + (size_t) (currPacketEventSize * cutoffIndex));
			if (currPacketResized == NULL) {
				// This is unlikely to happen as we always shrink here!
				caerLog(CAER_LOG_CRITICAL, state->parentModule->moduleSubSystemString,
					"Failed memory allocation for currPacketResized. Discarding current data.");
				free(*currPacket);
			}
			else {
				// Set header sizes for resized packet correctly.
				caerEventPacketHeaderSetEventValid(currPacketResized, validEventsSeen);
				caerEventPacketHeaderSetEventNumber(currPacketResized, cutoffIndex);
				caerEventPacketHeaderSetEventCapacity(currPacketResized, cutoffIndex);

				// Update references: the currPacket, after resizing, goes into the packet container for output.
				caerEventPacketContainerSetEventPacket(packetContainer, packetContainerPosition++, currPacketResized);
			}

			// Update references: the nextPacket goes into the eventPackets array at the currPacket position,
			// if it exists. Else we just delete that position.
			if (nextPacket != NULL) {
				*currPacket = nextPacket;
			}
			else {
				// Erase slot from packets array.
				utarray_erase(state->packetContainer.eventPackets,
					(size_t) utarray_eltidx(state->packetContainer.eventPackets, currPacket), 1);
				currPacket = (caerEventPacketHeader *) utarray_prev(state->packetContainer.eventPackets, currPacket);
			}
		}
	}

	return (packetContainer);
}

static void commitPacketContainer(inputCommonState state, bool forceFlush) {
	// Check if we hit any of the size limits (no more than X events per packet type).
	// Check if we have read and accumulated all the event packets with a main first timestamp smaller
	// or equal than what we want. We know this is the case when the last seen main timestamp is clearly
	// bigger than the wanted one. If this is true, it means we do have all the possible events of all
	// types that happen up until that point, and we can split that time range off into a packet container.
	// If not, we just go get the next event packet.
	bool sizeCommit = false, timeCommit = false;

	redo: sizeCommit = state->packetContainer.sizeLimitHit
		&& (state->packetContainer.lastPacketTimestamp > state->packetContainer.sizeLimitTimestamp);
	timeCommit = (state->packetContainer.lastPacketTimestamp > state->packetContainer.newContainerTimestampEnd);

	if (!forceFlush && !sizeCommit && !timeCommit) {
		return;
	}

	caerEventPacketContainer packetContainer = generatePacketContainer(state, forceFlush);
	if (packetContainer == NULL) {
		// Memory allocation or other error.
		return;
	}

	// Update wanted timestamp for next time slice.
	// Only do this if size limit was not active, since size limit can only be active
	// if the slice would (in time) be smaller than the time limit end, so the next run
	// must again check and comb through the same time window.
	// Also don't update if forceFlush is true, for the same reason of the next call
	// having to again comb through the same time window for any of the size or time
	// limits to hit again (on TS Overflow, on TS Reset everything just resets anyway).
	if (!sizeCommit && !forceFlush) {
		state->packetContainer.newContainerTimestampEnd += I32T(
			atomic_load_explicit(&state->packetContainer.timeSlice, memory_order_relaxed));

		// Only do time delay operation if time is actually changing. On size hits or
		// full flushes, this would slow down everything incorrectly as it would be an
		// extra delay operation inside the same time window.
		doTimeDelay(state);
	}

	doPacketContainerCommit(state, packetContainer, atomic_load_explicit(&state->keepPackets, memory_order_relaxed));

	// Update size slice for next packet container.
	state->packetContainer.newContainerSizeLimit = I32T(
		atomic_load_explicit(&state->packetContainer.sizeSlice, memory_order_relaxed));

	state->packetContainer.sizeLimitHit = false;
	state->packetContainer.sizeLimitTimestamp = INT32_MAX;

	if (!forceFlush) {
		// Check if any of the remaining packets still would trigger an early size limit.
		caerEventPacketHeader *currPacket = NULL;
		while ((currPacket = (caerEventPacketHeader *) utarray_next(state->packetContainer.eventPackets, currPacket))
			!= NULL) {
			updateSizeCommitCriteria(state, *currPacket);
		}

		// Run the above again, to make sure we do exhaust all possible size and time commits
		// possible with the data we have now, before going and getting new data.
		goto redo;
	}
}

static void doTimeDelay(inputCommonState state) {
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

static void doPacketContainerCommit(inputCommonState state, caerEventPacketContainer packetContainer, bool force) {
	// Could be that the packet container is empty of events. Don't commit empty containers.
	if (caerEventPacketContainerGetEventsNumber(packetContainer) == 0) {
		return;
	}

	retry: if (!ringBufferPut(state->transferRingPacketContainers, packetContainer)) {
		if (force && atomic_load_explicit(&state->running, memory_order_relaxed)) {
			// Retry forever if requested, at least while the module is running.
			goto retry;
		}

		caerEventPacketContainerFree(packetContainer);

		caerLog(CAER_LOG_INFO, state->parentModule->moduleSubSystemString,
			"Failed to put new packet container on transfer ring-buffer: full.");
	}
	else {
		// Signal availability of new data to the mainloop on packet container commit.
		atomic_fetch_add_explicit(&state->dataAvailableModule, 1, memory_order_release);
		atomic_fetch_add_explicit(&state->mainloopReference->dataAvailable, 1, memory_order_release);

		caerLog(CAER_LOG_DEBUG, state->parentModule->moduleSubSystemString, "Submitted packet container successfully.");
	}
}

static bool handleTSReset(inputCommonState state) {
	// Commit all current content.
	commitPacketContainer(state, true);

	// Send lone packet container with just TS_RESET.
	// Allocate packet container just for this event.
	caerEventPacketContainer tsResetContainer = caerEventPacketContainerAllocate(1);
	if (tsResetContainer == NULL) {
		caerLog(CAER_LOG_CRITICAL, state->parentModule->moduleSubSystemString,
			"Failed to allocate tsReset event packet container.");
		return (false);
	}

	// Allocate special packet just for this event.
	caerSpecialEventPacket tsResetPacket = caerSpecialEventPacketAllocate(1, I16T(state->parentModule->moduleID),
		state->packetContainer.lastTimestampOverflow);
	if (tsResetPacket == NULL) {
		caerLog(CAER_LOG_CRITICAL, state->parentModule->moduleSubSystemString,
			"Failed to allocate tsReset special event packet.");
		return (false);
	}

	// Create timestamp reset event.
	caerSpecialEvent tsResetEvent = caerSpecialEventPacketGetEvent(tsResetPacket, 0);
	caerSpecialEventSetTimestamp(tsResetEvent, INT32_MAX);
	caerSpecialEventSetType(tsResetEvent, TIMESTAMP_RESET);
	caerSpecialEventValidate(tsResetEvent, tsResetPacket);

	// Assign special packet to packet container.
	caerEventPacketContainerSetEventPacket(tsResetContainer, SPECIAL_EVENT, (caerEventPacketHeader) tsResetPacket);

	// Guaranteed commit of timestamp reset container.
	doPacketContainerCommit(state, tsResetContainer, true);

	// Prepare for the new event timeline coming with the next packet.
	// Reset all time related counters to initial state.
	state->packetContainer.lastPacketTimestamp = 0;
	state->packetContainer.lastTimestampOverflow = 0;
	state->packetContainer.newContainerTimestampEnd = -1;

	return (true);
}

static void getPacketInfo(caerEventPacketHeader packet, packetData packetInfoData) {
	// Get data from new packet.
	packetInfoData->eventType = caerEventPacketHeaderGetEventType(packet);
	packetInfoData->eventSize = caerEventPacketHeaderGetEventSize(packet);
	packetInfoData->eventValid = caerEventPacketHeaderGetEventValid(packet);
	packetInfoData->eventNumber = caerEventPacketHeaderGetEventNumber(packet);

	void *firstEvent = caerGenericEventGetEvent(packet, 0);
	packetInfoData->startTimestamp = caerGenericEventGetTimestamp64(firstEvent, packet);

	void *lastEvent = caerGenericEventGetEvent(packet, packetInfoData->eventNumber - 1);
	packetInfoData->endTimestamp = caerGenericEventGetTimestamp64(lastEvent, packet);
}

static int inputAssemblerThread(void *stateArg) {
	inputCommonState state = stateArg;

	// Set thread name.
	size_t threadNameLength = strlen(state->parentModule->moduleSubSystemString);
	char threadName[threadNameLength + 1 + 11]; // +1 for NUL character.
	strcpy(threadName, state->parentModule->moduleSubSystemString);
	strcat(threadName, "[Assembler]");
	thrd_set_name(threadName);

	// Set thread priority to high. This may fail depending on your OS configuration.
	if (thrd_set_priority(-1) != thrd_success) {
		caerLog(CAER_LOG_INFO, state->parentModule->moduleSubSystemString,
			"Failed to raise thread priority for Input Assembler thread. You may experience lags and delays.");
	}

	// Delay by 1 µs if no data, to avoid a wasteful busy loop.
	struct timespec noDataSleep = { .tv_sec = 0, .tv_nsec = 1000 };

	while (atomic_load_explicit(&state->running, memory_order_relaxed)) {
		// Support pause: don't get and send out new data while in pause mode.
		if (atomic_load_explicit(&state->pause, memory_order_relaxed)) {
			// Wait for 1 ms in pause mode, to avoid a wasteful busy loop.
			struct timespec pauseSleep = { .tv_sec = 0, .tv_nsec = 1000000 };
			thrd_sleep(&pauseSleep, NULL);

			continue;
		}

		// Get parsed packets from Reader thread.
		caerEventPacketHeader currPacket = ringBufferGet(state->transferRingPackets);
		if (currPacket == NULL) {
			// Let's see why there are no more packets to read, maybe the reader failed.
			// Also EOF could have been reached, in which case the reader would have committed its last
			// packet before setting the flag, and so we must have already seen that one too, due to
			// visibility between threads of the data put on a ring-buffer.
			if (atomic_load_explicit(&state->inputReaderThreadState, memory_order_relaxed) != READER_OK) {
				break;
			}

			// Delay by 1 µs if no data, to avoid a wasteful busy loop.
			thrd_sleep(&noDataSleep, NULL);

			continue;
		}

		// If validOnly flag is enabled, clean the packets up here, removing all
		// invalid events prior to the get info and merge steps.
		if (atomic_load_explicit(&state->validOnly, memory_order_relaxed)) {
			caerCleanEventPacket(currPacket);
		}

		// Get info on the new packet.
		struct input_packet_data currPacketData;
		getPacketInfo(currPacket, &currPacketData);

		// Check timestamp constraints as per AEDAT 3.X format: order-relevant timestamps
		// of each packet (the first timestamp) must be smaller or equal than next packet's.
		if (currPacketData.startTimestamp < state->packetContainer.lastPacketTimestamp) {
			// Discard non-compliant packets.
			free(currPacket);

			caerLog(CAER_LOG_INFO, state->parentModule->moduleSubSystemString,
				"Dropping packet due to incorrect timestamp order. "
					"Order-relevant timestamp is %" PRIi64 ", but expected was at least %" PRIi64 ".",
				currPacketData.startTimestamp, state->packetContainer.lastPacketTimestamp);
			continue;
		}

		// Remember the main timestamp of the first event of the new packet. That's the
		// order-relevant timestamp for files/streams.
		state->packetContainer.lastPacketTimestamp = currPacketData.startTimestamp;

		// Initialize time slice counters with first packet.
		if (state->packetContainer.newContainerTimestampEnd == -1) {
			// -1 because newPacketFirstTimestamp itself is part of the set!
			state->packetContainer.newContainerTimestampEnd = currPacketData.startTimestamp
				+ (atomic_load_explicit(&state->packetContainer.timeSlice, memory_order_relaxed) - 1);

			portable_clock_gettime_monotonic(&state->packetContainer.lastCommitTime);
		}

		// If it's a special packet, it might contain TIMESTAMP_RESET as event, which affects
		// how things are mixed and parsed. This needs to be detected first, before merging.
		bool tsReset = false;

		if ((currPacketData.eventType == SPECIAL_EVENT)
			&& (caerSpecialEventPacketFindValidEventByType((caerSpecialEventPacket) currPacket, TIMESTAMP_RESET) != NULL)) {
			tsReset = true;
			caerLog(CAER_LOG_INFO, state->parentModule->moduleSubSystemString, "Timestamp Reset detected in stream.");

			if (currPacketData.eventNumber != 1) {
				caerLog(CAER_LOG_WARNING, state->parentModule->moduleSubSystemString,
					"Timpestamp Reset detected, but it is not alone in its Special Event packet. "
						"This may lead to issues and should never happen.");
			}
		}

		// Support the big timestamp wrap, which changes tsOverflow, and affects
		// how things are mixed and parsed. This needs to be detected first, before merging.
		// TS Overflow can either be equal (okay) or bigger (detected here), never smaller due
		// to monotonic timestamps, as checked above (lastPacketTimestamp check).
		bool tsOverflow = false;

		if (caerEventPacketHeaderGetEventTSOverflow(currPacket) > state->packetContainer.lastTimestampOverflow) {
			state->packetContainer.lastTimestampOverflow = caerEventPacketHeaderGetEventTSOverflow(currPacket);

			tsOverflow = true;
			caerLog(CAER_LOG_INFO, state->parentModule->moduleSubSystemString,
				"Timestamp Overflow detected in stream.");
		}

		// Now we have all the information and must do some merge and commit operations.
		// There are four cases:
		// a) no reset and no overflow - just merge and follow usual commit scheme
		// b) reset and no overflow - commit current content, commit lone container with just reset
		// c) no reset and overflow - commit current content, re-initialize merger, follow usual scheme
		// d) reset and overflow - the reset would be the first (and last!) thing in the new overflow
		//                         epoch, so we just follow standard reset procedure (above)

		if (tsReset) {
			// Current packet not used.
			free(currPacket);

			// We don't merge the current packet, that should only contain the timestamp reset,
			// but instead generate one to ensure that's the case. Also, all counters are reset.
			if (!handleTSReset(state)) {
				// Critical error, exit.
				break;
			}

			continue;
		}

		if (tsOverflow) {
			// On TS Overflow, commit all current data, and then afterwards normally
			// merge the current packet witht the (now empty) packet container.
			commitPacketContainer(state, true);
		}

		// We've got a full event packet, store it (merge with current).
		if (!addToPacketContainer(state, currPacket, &currPacketData)) {
			// Discard on merge failure.
			free(currPacket);

			continue;
		}

		// Generate a proper packet container and commit it to the Mainloop
		// if the user-set time/size limits are reached.
		commitPacketContainer(state, false);
	}

	// At this point we either got terminated (running=false) or we stopped for some
	// reason: no more packets due to error or End-of-File.
	// If we got hard-terminated, we empty the ring-buffer in the Exit() state.
	// If we hit EOF/errors though, we want the consumers to be able to finish
	// consuming the already produced data, so we wait for the ring-buffer to be empty.
	if (atomic_load(&state->running)) {
		while (atomic_load(&state->running) && atomic_load(&state->dataAvailableModule) != 0) {
			// Delay by 1 ms if no change, to avoid a wasteful busy loop.
			struct timespec waitSleep = { .tv_sec = 0, .tv_nsec = 1000000 };
			thrd_sleep(&waitSleep, NULL);
		}

		// Ensure parent also shuts down, for example on read failures or EOF.
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
	state->sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");
	sshsNodePutLong(state->sourceInfoNode, "highestTimestamp", -1);

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
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "pause", false); // support pausing a stream
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "bufferSize", 65536); // in bytes, size of data buffer
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "transferBufferSize", 128); // in packet groups

	sshsNodePutIntIfAbsent(moduleData->moduleNode, "sizeSlice", 8192); // in events, size of slice to generate
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "timeSlice", 10000); // in µs, size of time slice to generate
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "timeDelay", 10000); // in µs, delay between consecutive slices

	atomic_store(&state->validOnly, sshsNodeGetBool(moduleData->moduleNode, "validOnly"));
	atomic_store(&state->keepPackets, sshsNodeGetBool(moduleData->moduleNode, "keepPackets"));
	atomic_store(&state->pause, sshsNodeGetBool(moduleData->moduleNode, "pause"));

	atomic_store(&state->packetContainer.sizeSlice, sshsNodeGetInt(moduleData->moduleNode, "sizeSlice"));
	atomic_store(&state->packetContainer.timeSlice, sshsNodeGetInt(moduleData->moduleNode, "timeSlice"));
	atomic_store(&state->packetContainer.timeDelay, sshsNodeGetInt(moduleData->moduleNode, "timeDelay"));

	// Initialize transfer ring-buffers. transferBufferSize only changes here at init time!
	state->transferRingPackets = ringBufferInit((size_t) sshsNodeGetInt(moduleData->moduleNode, "transferBufferSize"));
	if (state->transferRingPackets == NULL) {
		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
			"Failed to allocate packets transfer ring-buffer.");
		return (false);
	}

	state->transferRingPacketContainers = ringBufferInit(
		(size_t) sshsNodeGetInt(moduleData->moduleNode, "transferBufferSize"));
	if (state->transferRingPacketContainers == NULL) {
		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
			"Failed to allocate packet containers transfer ring-buffer.");
		return (false);
	}

	// Allocate data buffer. bufferSize is updated here.
	if (!newInputBuffer(state)) {
		ringBufferFree(state->transferRingPackets);
		ringBufferFree(state->transferRingPacketContainers);

		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Failed to allocate input data buffer.");
		return (false);
	}

	// Initialize array for packets -> packet container.
	utarray_new(state->packetContainer.eventPackets, &ut_caerEventPacketHeader_icd);

	state->packetContainer.newContainerTimestampEnd = -1;
	state->packetContainer.newContainerSizeLimit = I32T(
		atomic_load_explicit(&state->packetContainer.sizeSlice, memory_order_relaxed));
	state->packetContainer.sizeLimitTimestamp = INT32_MAX;

	// Start input handling threads.
	atomic_store(&state->running, true);

	if (thrd_create(&state->inputAssemblerThread, &inputAssemblerThread, state) != thrd_success) {
		ringBufferFree(state->transferRingPackets);
		ringBufferFree(state->transferRingPacketContainers);
		free(state->dataBuffer);

		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Failed to start input assembler thread.");
		return (false);
	}

	if (thrd_create(&state->inputReaderThread, &inputReaderThread, state) != thrd_success) {
		ringBufferFree(state->transferRingPackets);
		ringBufferFree(state->transferRingPacketContainers);
		free(state->dataBuffer);

		// Stop assembler thread (started just above) and wait on it.
		atomic_store(&state->running, false);

		if ((errno = thrd_join(state->inputAssemblerThread, NULL)) != thrd_success) {
			// This should never happen!
			caerLog(CAER_LOG_CRITICAL, state->parentModule->moduleSubSystemString,
				"Failed to join input assembler thread. Error: %d.", errno);
		}

		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Failed to start input reader thread.");
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

	// Stop input threads and wait on them.
	atomic_store(&state->running, false);

	if ((errno = thrd_join(state->inputReaderThread, NULL)) != thrd_success) {
		// This should never happen!
		caerLog(CAER_LOG_CRITICAL, state->parentModule->moduleSubSystemString,
			"Failed to join input reader thread. Error: %d.", errno);
	}

	if ((errno = thrd_join(state->inputAssemblerThread, NULL)) != thrd_success) {
		// This should never happen!
		caerLog(CAER_LOG_CRITICAL, state->parentModule->moduleSubSystemString,
			"Failed to join input assembler thread. Error: %d.", errno);
	}

	// Now clean up the transfer ring-buffers and its contents.
	caerEventPacketContainer packetContainer;
	while ((packetContainer = ringBufferGet(state->transferRingPacketContainers)) != NULL) {
		caerEventPacketContainerFree(packetContainer);

		// If we're here, then nobody will (or even can) consume this data afterwards.
		atomic_fetch_sub_explicit(&state->mainloopReference->dataAvailable, 1, memory_order_relaxed);
		atomic_fetch_sub_explicit(&state->dataAvailableModule, 1, memory_order_relaxed);
	}

	ringBufferFree(state->transferRingPacketContainers);

	// Check we indeed removed all data and counters match this expectation.
	if (atomic_load(&state->dataAvailableModule) != 0) {
		// This should never happen!
		caerLog(CAER_LOG_CRITICAL, state->parentModule->moduleSubSystemString,
			"After cleanup, data is still available for consumption. Counter value: %" PRIu32 ".",
			U32T(atomic_load(&state->dataAvailableModule)));
	}

	caerEventPacketHeader packet;
	while ((packet = ringBufferGet(state->transferRingPackets)) != NULL) {
		free(packet);
	}

	ringBufferFree(state->transferRingPackets);

	// Free all waiting packets.
	caerEventPacketHeader *packetPtr = NULL;
	while ((packetPtr = (caerEventPacketHeader *) utarray_next(state->packetContainer.eventPackets, packetPtr)) != NULL) {
		free(*packetPtr);
	}

	// Clear and free packet array used for packet container construction.
	utarray_free(state->packetContainer.eventPackets);

	// Close file descriptors.
	if (state->fileDescriptor >= 0) {
		close(state->fileDescriptor);
	}

	// Free allocated memory.
	free(state->dataBuffer);

	// Remove lingering packet parsing data.
	packetData curr, curr_tmp;
	DL_FOREACH_SAFE(state->packets.packetsList, curr, curr_tmp)
	{
		DL_DELETE(state->packets.packetsList, curr);
		free(curr);
	}

	free(state->packets.currPacketData);
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

	*container = ringBufferGet(state->transferRingPacketContainers);

	if (*container != NULL) {
		// Got a container, set it up for auto-reclaim and signal it's not available anymore.
		caerMainloopFreeAfterLoop((void (*)(void *)) &caerEventPacketContainerFree, *container);

		// No special memory order for decrease, because the acquire load to even start running
		// through a mainloop already synchronizes with the release store above.
		atomic_fetch_sub_explicit(&state->mainloopReference->dataAvailable, 1, memory_order_relaxed);
		atomic_fetch_sub_explicit(&state->dataAvailableModule, 1, memory_order_relaxed);

		sshsNodePutLong(state->sourceInfoNode, "highestTimestamp",
			caerEventPacketContainerGetHighestEventTimestamp(*container));

		caerEventPacketHeader special = caerEventPacketContainerGetEventPacket(*container, SPECIAL_EVENT);

		if ((special != NULL) && (caerEventPacketHeaderGetEventType(special) == SPECIAL_EVENT)
			&& (caerEventPacketHeaderGetEventNumber(special) == 1)
			&& (caerSpecialEventPacketFindEventByType((caerSpecialEventPacket) special, TIMESTAMP_RESET) != NULL)) {
			caerMainloopResetProcessors(moduleData->moduleID);
			caerMainloopResetOutputs(moduleData->moduleID);
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
		else if (changeType == BOOL && caerStrEquals(changeKey, "pause")) {
			// Set pause flag to given value.
			atomic_store(&state->pause, changeValue.boolean);
		}
		else if (changeType == INT && caerStrEquals(changeKey, "bufferSize")) {
			// Set buffer update flag.
			atomic_store(&state->bufferUpdate, true);
		}
		else if (changeType == INT && caerStrEquals(changeKey, "sizeSlice")) {
			atomic_store(&state->packetContainer.sizeSlice, changeValue.iint);
		}
		else if (changeType == INT && caerStrEquals(changeKey, "timeSlice")) {
			atomic_store(&state->packetContainer.timeSlice, changeValue.iint);
		}
		else if (changeType == INT && caerStrEquals(changeKey, "timeDelay")) {
			atomic_store(&state->packetContainer.timeDelay, changeValue.iint);
		}
	}
}

static int packetsFirstTypeThenSizeCmp(const void *a, const void *b) {
	const caerEventPacketHeader *aa = a;
	const caerEventPacketHeader *bb = b;

	// Sort first by type ID.
	int16_t eventTypeA = caerEventPacketHeaderGetEventType(*aa);
	int16_t eventTypeB = caerEventPacketHeaderGetEventType(*bb);

	if (eventTypeA < eventTypeB) {
		return (-1);
	}
	else if (eventTypeA > eventTypeB) {
		return (1);
	}
	else {
		// If equal, further sort by event size.
		int32_t eventSizeA = caerEventPacketHeaderGetEventSize(*aa);
		int32_t eventSizeB = caerEventPacketHeaderGetEventSize(*bb);

		if (eventSizeA < eventSizeB) {
			return (-1);
		}
		else if (eventSizeA > eventSizeB) {
			return (1);
		}
		else {
			return (0);
		}
	}
}

#ifdef ENABLE_VISUALIZER
void caerInputVisualizerEventHandler(caerVisualizerPublicState state, ALLEGRO_EVENT event) {
	if (event.type == ALLEGRO_EVENT_KEY_DOWN && event.keyboard.keycode == ALLEGRO_KEY_SPACE) {
		bool pause = sshsNodeGetBool(state->eventSourceConfigNode, "pause");

		sshsNodePutBool(state->eventSourceConfigNode, "pause", !pause);
	}
}
#endif
