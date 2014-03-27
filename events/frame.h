/*
 * frame.h
 *
 *  Created on: Jan 6, 2014
 *      Author: llongi
 */

#ifndef FRAME_H_
#define FRAME_H_

#include "common.h"

#define ADC_DEPTH_SHIFT 1
#define ADC_DEPTH_MASK 0x0000001F
#define Y_LENGTH_SHIFT 6
#define Y_LENGTH_MASK 0x00001FFF
#define X_LENGTH_SHIFT 19
#define X_LENGTH_MASK 0x00001FFF

struct caer_frame_event {
	uint32_t info;
	uint32_t ts_sorr;
	uint32_t ts_eorr;
	uint32_t ts_soe;
	uint32_t ts_eoe;
	uint32_t ts_sosr;
	uint32_t ts_eosr;
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
	// Calculate maximum needed bits for storing pixel data.
	uint32_t pixelBits = (uint32_t) maxADCDepth * maxYLength * maxXLength;

	// Round up (ceil) to bytes (8 bits per byte).
	uint32_t pixelBytes = (pixelBits + 7) / 8;

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
	caerEventPacketHeaderSetEventTSOffset(&packet->packetHeader, offsetof(struct caer_frame_event, ts_sorr));
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

static inline uint32_t caerFrameEventGetTSStartOfResetRead(caerFrameEvent event) {
	return (le32toh(event->ts_sorr));
}

static inline void caerFrameEventSetTSStartOfResetRead(caerFrameEvent event, uint32_t sorr) {
	event->ts_sorr = htole32(sorr);
}

static inline uint32_t caerFrameEventGetTSEndOfResetRead(caerFrameEvent event) {
	return (le32toh(event->ts_eorr));
}

static inline void caerFrameEventSetTSEndOfResetRead(caerFrameEvent event, uint32_t eorr) {
	event->ts_eorr = htole32(eorr);
}

static inline uint32_t caerFrameEventGetTSStartOfExposure(caerFrameEvent event) {
	return (le32toh(event->ts_soe));
}

static inline void caerFrameEventSetTSStartOfExposure(caerFrameEvent event, uint32_t soe) {
	event->ts_soe = htole32(soe);
}

static inline uint32_t caerFrameEventGetTSEndOfExposure(caerFrameEvent event) {
	return (le32toh(event->ts_eoe));
}

static inline void caerFrameEventSetTSEndOfExposure(caerFrameEvent event, uint32_t eoe) {
	event->ts_eoe = htole32(eoe);
}

static inline uint32_t caerFrameEventGetTSStartOfSignalRead(caerFrameEvent event) {
	return (le32toh(event->ts_sosr));
}

static inline void caerFrameEventSetTSStartOfSignalRead(caerFrameEvent event, uint32_t sosr) {
	event->ts_sosr = htole32(sosr);
}

static inline uint32_t caerFrameEventGetTSEndOfSignalRead(caerFrameEvent event) {
	return (le32toh(event->ts_eosr));
}

static inline void caerFrameEventSetTSEndOfSignalRead(caerFrameEvent event, uint32_t eosr) {
	event->ts_eosr = htole32(eosr);
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

static inline uint16_t caerFrameEventGetYLength(caerFrameEvent event) {
	return U16T((le32toh(event->info) >> Y_LENGTH_SHIFT) & Y_LENGTH_MASK);
}

static inline uint16_t caerFrameEventGetXLength(caerFrameEvent event) {
	return U16T((le32toh(event->info) >> X_LENGTH_SHIFT) & X_LENGTH_MASK);
}

static inline void caerFrameEventSetADCDepth(caerFrameEvent event, caerFrameEventPacket packet, uint8_t adcDepth) {
	// Check value against maximum allowed in this packet.
	uint32_t maxBits = (caerEventPacketHeaderGetEventSize(&packet->packetHeader) - sizeof(struct caer_frame_event)) * 8;
	uint32_t neededBits = adcDepth * caerFrameEventGetYLength(event) * caerFrameEventGetXLength(event);

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

static inline void caerFrameEventSetYXLength(caerFrameEvent event, caerFrameEventPacket packet, uint16_t yLength,
	uint16_t xLength) {
	// Check value against maximum allowed in this packet.
	uint32_t maxBits = (caerEventPacketHeaderGetEventSize(&packet->packetHeader) - sizeof(struct caer_frame_event)) * 8;
	uint32_t neededBits = caerFrameEventGetADCDepth(event) * yLength * xLength;

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

	event->info |= htole32((U32T(yLength) & Y_LENGTH_MASK) << Y_LENGTH_SHIFT);
	event->info |= htole32((U32T(xLength) & X_LENGTH_MASK) << X_LENGTH_SHIFT);
}

static inline uint32_t caerFrameEventGetPixel(caerFrameEvent event, uint16_t yAddress, uint16_t xAddress) {
	// Check frame bounds first.
	if (yAddress >= caerFrameEventGetYLength(event)) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_ERROR,
#endif
			"Called caerFrameEventGetPixel() with invalid Y address of %" PRIu16 ", should be between 0 and %" PRIu16 ".",
			yAddress, caerFrameEventGetYLength(event) - 1);
#endif
		return (0);
	}

	uint16_t xLength = caerFrameEventGetXLength(event);

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
