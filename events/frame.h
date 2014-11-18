/*
 * frame.h
 *
 *  Created on: Jan 6, 2014
 *      Author: llongi
 */

#ifndef FRAME_H_
#define FRAME_H_

#include "common.h"
#include "base/misc.h"

// All pixels are always normalized to 16bit depth.
// Multiple channels (such as RGB) are possible.
#define CHANNEL_NUMBER_SHIFT 1
#define CHANNEL_NUMBER_MASK 0x0000001F

struct caer_frame_event {
	uint32_t info;
	uint16_t lengthX;
	uint16_t lengthY;
	uint32_t ts_startframe;
	uint32_t ts_endframe;
	uint32_t ts_startexposure;
	uint32_t ts_endexposure;
	uint16_t pixels[];
}__attribute__((__packed__));

typedef struct caer_frame_event *caerFrameEvent;

struct caer_frame_event_packet {
	struct caer_event_packet_header packetHeader;
	struct caer_frame_event events[];
}__attribute__((__packed__));

typedef struct caer_frame_event_packet *caerFrameEventPacket;

// Need pixel info too here, so storage requirement for pixel data can be determined.
static inline caerFrameEventPacket caerFrameEventPacketAllocate(uint32_t eventCapacity, uint16_t eventSource,
	uint16_t maxLengthX, uint16_t maxLengthY, uint8_t maxChannels) {
	// Calculate maximum needed bytes for storing pixel data.
	uint32_t pixelBytes = U32T(sizeof(uint16_t)) * maxLengthX * maxLengthY * maxChannels;

	uint32_t eventSize = U32T(sizeof(struct caer_frame_event)) + pixelBytes;
	size_t eventPacketSize = sizeof(struct caer_frame_event_packet) + (eventCapacity * eventSize);

	caerFrameEventPacket packet = malloc(eventPacketSize);
	if (packet == NULL) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_CRITICAL, "caerFrameEventPacketAllocate()",
#endif
			"Failed to allocate %zu bytes of memory for Frame Event Packet of capacity %"
			PRIu32 " from source %" PRIu16 ". Error: %s (%d).", eventPacketSize, eventCapacity, eventSource,
			caerLogStrerror(errno), errno);
#endif
		return (NULL);
	}

	// Zero out event memory (all events invalid).
	memset(packet, 0, eventPacketSize);

	// Fill in header fields.
	caerEventPacketHeaderSetEventType(&packet->packetHeader, FRAME_EVENT);
	caerEventPacketHeaderSetEventSource(&packet->packetHeader, eventSource);
	caerEventPacketHeaderSetEventSize(&packet->packetHeader, eventSize);
	caerEventPacketHeaderSetEventTSOffset(&packet->packetHeader, offsetof(struct caer_frame_event, ts_startexposure));
	caerEventPacketHeaderSetEventCapacity(&packet->packetHeader, eventCapacity);
	caerEventPacketHeaderSetEventNumber(&packet->packetHeader, 0);
	caerEventPacketHeaderSetEventValid(&packet->packetHeader, 0);

	return (packet);
}

static inline caerFrameEvent caerFrameEventPacketGetEvent(caerFrameEventPacket packet, uint32_t n) {
	// Check that we're not out of bounds.
	if (n >= caerEventPacketHeaderGetEventCapacity(&packet->packetHeader)) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_ERROR, "caerFrameEventPacketGetEvent()",
#endif
			"Called caerFrameEventPacketGetEvent() with invalid event offset %" PRIu32 ", while maximum allowed value is %" PRIu32 ".",
			n, caerEventPacketHeaderGetEventCapacity(&packet->packetHeader));
#endif
		return (NULL);
	}

	// Return a pointer to the specified event.
	return ((caerFrameEvent) (((uint8_t *) packet->events)
		+ (n * caerEventPacketHeaderGetEventSize(&packet->packetHeader))));
}

static inline uint32_t caerFrameEventGetTSStartOfFrame(caerFrameEvent event) {
	return (le32toh(event->ts_startframe));
}

static inline void caerFrameEventSetTSStartOfFrame(caerFrameEvent event, uint32_t startFrame) {
	event->ts_startframe = htole32(startFrame);
}

static inline uint32_t caerFrameEventGetTSEndOfFrame(caerFrameEvent event) {
	return (le32toh(event->ts_endframe));
}

static inline void caerFrameEventSetTSEndOfFrame(caerFrameEvent event, uint32_t endFrame) {
	event->ts_endframe = htole32(endFrame);
}

static inline uint32_t caerFrameEventGetTSStartOfExposure(caerFrameEvent event) {
	return (le32toh(event->ts_startexposure));
}

static inline void caerFrameEventSetTSStartOfExposure(caerFrameEvent event, uint32_t startExposure) {
	event->ts_startexposure = htole32(startExposure);
}

static inline uint32_t caerFrameEventGetTSEndOfExposure(caerFrameEvent event) {
	return (le32toh(event->ts_endexposure));
}

static inline void caerFrameEventSetTSEndOfExposure(caerFrameEvent event, uint32_t endExposure) {
	event->ts_endexposure = htole32(endExposure);
}

static inline bool caerFrameEventIsValid(caerFrameEvent event) {
	return ((le32toh(event->info) >> VALID_MARK_SHIFT) & VALID_MARK_MASK);
}

static inline void caerFrameEventValidate(caerFrameEvent event, caerFrameEventPacket packet) {
	if (!caerFrameEventIsValid(event)) {
		event->info |= htole32(U32T(1) << VALID_MARK_SHIFT);

		// Also increase number of events and valid events.
		// Only call this on (still) invalid events!
		caerEventPacketHeaderSetEventNumber(&packet->packetHeader,
			caerEventPacketHeaderGetEventNumber(&packet->packetHeader) + 1);
		caerEventPacketHeaderSetEventValid(&packet->packetHeader,
			caerEventPacketHeaderGetEventValid(&packet->packetHeader) + 1);
	}
	else {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_ERROR, "caerFrameEventValidate()",
#endif
			"Called caerFrameEventValidate() on already valid event.");
#endif
	}
}

static inline void caerFrameEventInvalidate(caerFrameEvent event, caerFrameEventPacket packet) {
	if (caerFrameEventIsValid(event)) {
		event->info &= htole32(~(U32T(1) << VALID_MARK_SHIFT));

		// Also decrease number of valid events. Number of total events doesn't change.
		// Only call this on valid events!
		caerEventPacketHeaderSetEventValid(&packet->packetHeader,
			caerEventPacketHeaderGetEventValid(&packet->packetHeader) - 1);
	}
	else {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_ERROR, "caerFrameEventInvalidate()",
#endif
			"Called caerFrameEventInvalidate() on already invalid event.");
#endif
	}
}

static inline uint8_t caerFrameEventGetChannelNumber(caerFrameEvent event) {
	return U8T((le32toh(event->info) >> CHANNEL_NUMBER_SHIFT) & CHANNEL_NUMBER_MASK);
}

// Always set channel before setting X/Y lengths.
static inline void caerFrameEventSetChannelNumber(caerFrameEvent event, uint8_t channelNumber) {
	event->info |= htole32((U32T(channelNumber) & CHANNEL_NUMBER_MASK) << CHANNEL_NUMBER_SHIFT);
}

static inline uint16_t caerFrameEventGetLengthX(caerFrameEvent event) {
	return (le16toh(event->lengthX));
}

static inline uint16_t caerFrameEventGetLengthY(caerFrameEvent event) {
	return (le16toh(event->lengthY));
}

// Always set channel before setting X/Y lengths.
static inline void caerFrameEventSetLengthXY(caerFrameEvent event, caerFrameEventPacket packet, uint16_t lengthX,
	uint16_t lengthY) {
	// Check value against maximum allowed in this packet.
	uint32_t maxBytes = (caerEventPacketHeaderGetEventSize(&packet->packetHeader)
		- U32T(sizeof(struct caer_frame_event)));
	uint32_t neededBytes = U32T(lengthX * lengthY * caerFrameEventGetChannelNumber(event));

	if (neededBytes > maxBytes) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_ERROR, "caerFrameEventSetLengthXY()",
#endif
			"Called caerFrameEventSetLengthXY() with lengthX=%" PRIu16 " and lengthY=%" PRIu16 ", needing %" PRIu32 " bytes, while storage is possible only for up to %" PRIu32 " bytes.",
			lengthX, lengthY, neededBytes, maxBytes);
#endif
		return;
	}

	event->lengthX = htole16(lengthX);
	event->lengthY = htole16(lengthY);
}

static inline uint16_t caerFrameEventGetPixel(caerFrameEvent event, uint16_t xAddress, uint16_t yAddress) {
	// Check frame bounds first.
	if (yAddress >= caerFrameEventGetLengthY(event)) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_ERROR, "caerFrameEventGetPixel()",
#endif
			"Called caerFrameEventGetPixel() with invalid Y address of %" PRIu16 ", should be between 0 and %" PRIu16 ".",
			yAddress, caerFrameEventGetLengthY(event) - 1);
#endif
		return (0);
	}

	uint16_t xLength = caerFrameEventGetLengthX(event);

	if (xAddress >= xLength) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_ERROR, "caerFrameEventGetPixel()",
#endif
			"Called caerFrameEventGetPixel() with invalid X address of %" PRIu16 ", should be between 0 and %" PRIu16 ".",
			xAddress, xLength - 1);
#endif
		return (0);
	}

	// Get pixel value at specified position.
	return (le16toh(event->pixels[(yAddress * xLength) + xAddress]));
}

static inline void caerFrameEventSetPixel(caerFrameEvent event, uint16_t xAddress, uint16_t yAddress,
	uint16_t pixelValue) {
	// Check frame bounds first.
	if (yAddress >= caerFrameEventGetLengthY(event)) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_ERROR, "caerFrameEventSetPixel()",
#endif
			"Called caerFrameEventSetPixel() with invalid Y address of %" PRIu16 ", should be between 0 and %" PRIu16 ".",
			yAddress, caerFrameEventGetLengthY(event) - 1);
#endif
		return;
	}

	uint16_t xLength = caerFrameEventGetLengthX(event);

	if (xAddress >= xLength) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_ERROR, "caerFrameEventSetPixel()",
#endif
			"Called caerFrameEventSetPixel() with invalid X address of %" PRIu16 ", should be between 0 and %" PRIu16 ".",
			xAddress, xLength - 1);
#endif
		return;
	}

	// Set pixel value at specified position.
	event->pixels[(yAddress * xLength) + xAddress] = htole16(pixelValue);
}

static inline uint16_t caerFrameEventGetPixelForChannel(caerFrameEvent event, uint16_t xAddress, uint16_t yAddress,
	uint8_t channel) {
	// Check frame bounds first.
	if (yAddress >= caerFrameEventGetLengthY(event)) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_ERROR, "caerFrameEventGetPixelForChannel()",
#endif
			"Called caerFrameEventGetPixelForChannel() with invalid Y address of %" PRIu16 ", should be between 0 and %" PRIu16 ".",
			yAddress, caerFrameEventGetLengthY(event) - 1);
#endif
		return (0);
	}

	uint16_t xLength = caerFrameEventGetLengthX(event);

	if (xAddress >= xLength) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_ERROR, "caerFrameEventGetPixelForChannel()",
#endif
			"Called caerFrameEventGetPixelForChannel() with invalid X address of %" PRIu16 ", should be between 0 and %" PRIu16 ".",
			xAddress, xLength - 1);
#endif
		return (0);
	}

	uint16_t channelNumber = caerFrameEventGetChannelNumber(event);

	if (channel >= channelNumber) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_ERROR, "caerFrameEventGetPixelForChannel()",
#endif
			"Called caerFrameEventGetPixelForChannel() with invalid channel number of %" PRIu16 ", should be between 0 and %" PRIu16 ".",
			channel, channelNumber - 1);
#endif
		return (0);
	}

	// Get pixel value at specified position.
	return (le16toh(event->pixels[(((yAddress * xLength) + xAddress) * channelNumber) + channel]));
}

static inline void caerFrameEventSetPixelForChannel(caerFrameEvent event, uint16_t xAddress, uint16_t yAddress,
	uint8_t channel, uint16_t pixelValue) {
	// Check frame bounds first.
	if (yAddress >= caerFrameEventGetLengthY(event)) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_ERROR, "caerFrameEventSetPixelForChannel()",
#endif
			"Called caerFrameEventSetPixelForChannel() with invalid Y address of %" PRIu16 ", should be between 0 and %" PRIu16 ".",
			yAddress, caerFrameEventGetLengthY(event) - 1);
#endif
		return;
	}

	uint16_t xLength = caerFrameEventGetLengthX(event);

	if (xAddress >= xLength) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_ERROR, "caerFrameEventSetPixelForChannel()",
#endif
			"Called caerFrameEventSetPixelForChannel() with invalid X address of %" PRIu16 ", should be between 0 and %" PRIu16 ".",
			xAddress, xLength - 1);
#endif
		return;
	}

	uint16_t channelNumber = caerFrameEventGetChannelNumber(event);

	if (channel >= channelNumber) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_ERROR, "caerFrameEventSetPixelForChannel()",
#endif
			"Called caerFrameEventSetPixelForChannel() with invalid channel number of %" PRIu16 ", should be between 0 and %" PRIu16 ".",
			channel, channelNumber - 1);
#endif
		return;
	}

	// Set pixel value at specified position.
	event->pixels[(((yAddress * xLength) + xAddress) * channelNumber) + channel] = htole16(pixelValue);
}

static inline uint16_t caerFrameEventGetPixelUnsafe(caerFrameEvent event, uint16_t xAddress, uint16_t yAddress) {
	// Get pixel value at specified position.
	return (le16toh(event->pixels[(yAddress * caerFrameEventGetLengthX(event)) + xAddress]));
}

static inline void caerFrameEventSetPixelUnsafe(caerFrameEvent event, uint16_t xAddress, uint16_t yAddress,
	uint16_t pixelValue) {
	// Set pixel value at specified position.
	event->pixels[(yAddress * caerFrameEventGetLengthX(event)) + xAddress] = htole16(pixelValue);
}

static inline uint16_t caerFrameEventGetPixelForChannelUnsafe(caerFrameEvent event, uint16_t xAddress, uint16_t yAddress,
	uint8_t channel) {
	// Get pixel value at specified position.
	return (le16toh(event->pixels[(((yAddress * caerFrameEventGetLengthX(event)) + xAddress) * caerFrameEventGetChannelNumber(event)) + channel]));
}

static inline void caerFrameEventSetPixelForChannelUnsafe(caerFrameEvent event, uint16_t xAddress, uint16_t yAddress,
	uint8_t channel, uint16_t pixelValue) {
	// Set pixel value at specified position.
	event->pixels[(((yAddress * caerFrameEventGetLengthX(event)) + xAddress) * caerFrameEventGetChannelNumber(event)) + channel] = htole16(pixelValue);
}

static inline uint16_t *caerFrameEventGetPixelArrayUnsafe(caerFrameEvent event) {
	// Get pixel array.
	return (event->pixels);
}

#endif /* FRAME_H_ */
