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

#define ADC_DEPTH_SHIFT 1
#define ADC_DEPTH_MASK 0x0000001F

struct caer_frame_event {
	uint32_t info;
	uint16_t lengthX;
	uint16_t lengthY;
	uint32_t ts_startframe;
	uint32_t ts_endframe;
	uint32_t ts_startexposure;
	uint32_t ts_endexposure;
	uint8_t pixels[];
}__attribute__((__packed__));

typedef struct caer_frame_event *caerFrameEvent;

struct caer_frame_event_packet {
	struct caer_event_packet_header packetHeader;
	struct caer_frame_event events[];
}__attribute__((__packed__));

typedef struct caer_frame_event_packet *caerFrameEventPacket;

// Need pixel info too here, so storage requirement for pixel data can be determined.
static inline caerFrameEventPacket caerFrameEventPacketAllocate(uint32_t eventCapacity, uint16_t eventSource,
	uint8_t maxADCDepth, uint16_t maxYLength, uint16_t maxXLength) {
	// Align pixels to either 8bits or 16bits, depending on their ADC value length.
	// 16bits is the maximum supported for now, though this could be trivially expanded.
	uint8_t ADCDepthBytes = 0;

	if (maxADCDepth <= 8) {
		ADCDepthBytes = 1;
	}
	else if (maxADCDepth <= 16) {
		ADCDepthBytes = 2;
	}
	else {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_ERROR,
#endif
			"Maximum ADC size exceeds 16, the highest value currently supported.");
#endif
	}

	// Calculate maximum needed bytes for storing pixel data.
	uint32_t pixelBytes = (uint32_t) ADCDepthBytes * maxYLength * maxXLength;

	uint32_t eventSize = (uint32_t) sizeof(struct caer_frame_event) + pixelBytes;
	size_t eventPacketSize = sizeof(struct caer_frame_event_packet) + (eventCapacity * eventSize);

	caerFrameEventPacket packet = malloc(eventPacketSize);
	if (packet == NULL) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_CRITICAL,
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
		caerLog(LOG_ERROR,
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
		caerLog(LOG_ERROR,
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
		caerLog(LOG_ERROR,
#endif
			"Called caerFrameEventInvalidate() on already invalid event.");
#endif
	}
}

static inline uint8_t caerFrameEventGetADCDepth(caerFrameEvent event) {
	return U8T((le32toh(event->info) >> ADC_DEPTH_SHIFT) & ADC_DEPTH_MASK);
}

static inline uint16_t caerFrameEventGetLengthX(caerFrameEvent event) {
	return U16T(le32toh(event->lengthX));
}

static inline uint16_t caerFrameEventGetLengthY(caerFrameEvent event) {
	return U16T(le32toh(event->lengthY));
}

static inline void caerFrameEventSetADCDepth(caerFrameEvent event, caerFrameEventPacket packet, uint8_t adcDepth) {
	// Check value against maximum allowed in this packet.
	uint32_t maxBits = (caerEventPacketHeaderGetEventSize(&packet->packetHeader)
		- (uint32_t) sizeof(struct caer_frame_event)) * 8;
	uint32_t neededBits = (uint32_t) adcDepth * caerFrameEventGetYLength(event) * caerFrameEventGetXLength(event);

	if (neededBits > maxBits) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_ERROR,
#endif
			"Called caerFrameEventSetADCDepth() with adcDepth=%" PRIu8 ", needing %" PRIu32 " bits, while storage is possible only for up to %" PRIu32 " bits.",
			adcDepth, neededBits, maxBits);
#endif
		return;
	}

	event->info |= htole32((U32T(adcDepth) & ADC_DEPTH_MASK) << ADC_DEPTH_SHIFT);
}

static inline void caerFrameEventSetYXLength(caerFrameEvent event, caerFrameEventPacket packet, uint16_t xLength,
	uint16_t yLength) {
	// Check value against maximum allowed in this packet.
	uint32_t maxBits = (caerEventPacketHeaderGetEventSize(&packet->packetHeader)
		- (uint32_t) sizeof(struct caer_frame_event)) * 8;
	uint32_t neededBits = (uint32_t) caerFrameEventGetADCDepth(event) * xLength * yLength;

	if (neededBits > maxBits) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_ERROR,
#endif
			"Called caerFrameEventSetYXLength() with yLength=%" PRIu16 " and xLength=%" PRIu16 ", needing %" PRIu32 " bits, while storage is possible only for up to %" PRIu32 " bits.",
			yLength, xLength, neededBits, maxBits);
#endif
		return;
	}

	event->lengthX = htole32(xLength);
	event->lengthY = htole32(yLength);
}

static inline uint32_t caerFrameEventGetPixel(caerFrameEvent event, uint16_t yAddress, uint16_t xAddress) {
	// Check frame bounds first.
	if (yAddress >= caerFrameEventGetLengthY(event)) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_ERROR,
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
		caerLog(LOG_ERROR,
#endif
			"Called caerFrameEventGetPixel() with invalid X address of %" PRIu16 ", should be between 0 and %" PRIu16 ".",
			xAddress, xLength - 1);
#endif
		return (0);
	}

	// Need adcDepth to calculate stride.
	uint8_t adcDepth = caerFrameEventGetADCDepth(event);
	uint32_t pixelValue = 0;

	// Get pixel value at specified position.
	caerBitArrayCopy(event->pixels, (yAddress * xLength * adcDepth) + (xAddress * adcDepth), &pixelValue, 32 - adcDepth,
		adcDepth);

	// The copy algorithm works as if the integer was big endian (byte-wise increasing).
	// So convert it back to host format here.
	return (be32toh(pixelValue));
}

static inline void caerFrameEventSetPixel(caerFrameEvent event, uint16_t yAddress, uint16_t xAddress,
	uint32_t pixelValue) {
	// Check frame bounds first.
	if (yAddress >= caerFrameEventGetYLength(event)) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_ERROR,
#endif
			"Called caerFrameEventSetPixel() with invalid Y address of %" PRIu16 ", should be between 0 and %" PRIu16 ".",
			yAddress, caerFrameEventGetYLength(event) - 1);
#endif
		return;
	}

	uint16_t xLength = caerFrameEventGetXLength(event);

	if (xAddress >= xLength) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_ERROR,
#endif
			"Called caerFrameEventSetPixel() with invalid X address of %" PRIu16 ", should be between 0 and %" PRIu16 ".",
			xAddress, xLength - 1);
#endif
		return;
	}

	// Check that value isn't above the depth bound.
	uint8_t adcDepth = caerFrameEventGetADCDepth(event);

	if (pixelValue >= (U32T(1) << adcDepth)) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_ERROR,
#endif
			"Called caerFrameEventSetPixel() with invalid pixel value of %" PRIu32 ", should be between 0 and %" PRIu32 ".",
			pixelValue, (U32T(1) << adcDepth) - 1);
#endif
		return;
	}

	// Working with the pixel value as big endian is much easier, as the bits
	// are in the byte order the copy algorithm can most easily deal with.
	pixelValue = htobe32(pixelValue);

	// Set the pixel value at the specified position to the given value.
	caerBitArrayCopy(&pixelValue, 32 - adcDepth, event->pixels, (yAddress * xLength * adcDepth) + (xAddress * adcDepth),
		adcDepth);
}

#endif /* FRAME_H_ */
