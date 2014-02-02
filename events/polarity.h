/*
 * polarity.h
 *
 *  Created on: Nov 26, 2013
 *      Author: chtekk
 */

#ifndef POLARITY_H_
#define POLARITY_H_

#include "common.h"

// 0 in the 0th bit means invalid, 1 means valid.
// This way zeroing-out an event packet sets all its events to invalid.
#define VALID_MARK_SHIFT 0
#define VALID_MARK_MASK 0x00000001
#define POLARITY_SHIFT 1
#define POLARITY_MASK 0x00000001
#define Y_ADDR_SHIFT 6
#define Y_ADDR_MASK 0x00001FFF
#define X_ADDR_SHIFT 19
#define X_ADDR_MASK 0x00001FFF

struct caer_polarity_event {
	uint32_t data;
	uint32_t timestamp;
}__attribute__((__packed__));

typedef struct caer_polarity_event *caerPolarityEvent;

struct caer_polarity_event_packet {
	struct caer_event_packet_header packetHeader;
	struct caer_polarity_event events[];
}__attribute__((__packed__));

typedef struct caer_polarity_event_packet *caerPolarityEventPacket;

static inline caerPolarityEventPacket caerPolarityEventPacketAllocate(uint32_t eventCapacity, uint16_t eventSource) {
	uint32_t eventSize = sizeof(struct caer_polarity_event);
	size_t eventPacketSize = sizeof(struct caer_polarity_event_packet) + (eventCapacity * eventSize);

	caerPolarityEventPacket packet = malloc(eventPacketSize);
	if (packet == NULL) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_CRITICAL,
#endif
			"Failed to allocate %zu bytes of memory for Polarity Event Packet of capacity %"
			PRIu32 " from source %" PRIu16 ". Error: %s (%d).", eventPacketSize, eventCapacity, eventSource,
			caerLogStrerror(errno), errno);
#endif
		return (NULL);
	}

	// Zero out event memory (all events invalid).
	memset(packet, 0, eventPacketSize);

	// Fill in header fields.
	caerEventPacketHeaderSetEventType(&packet->packetHeader, POLARITY_EVENT);
	caerEventPacketHeaderSetEventSource(&packet->packetHeader, eventSource);
	caerEventPacketHeaderSetEventSize(&packet->packetHeader, eventSize);
	caerEventPacketHeaderSetEventTSOffset(&packet->packetHeader, offsetof(struct caer_polarity_event, timestamp));
	caerEventPacketHeaderSetEventCapacity(&packet->packetHeader, eventCapacity);
	caerEventPacketHeaderSetEventNumber(&packet->packetHeader, 0);
	caerEventPacketHeaderSetEventValid(&packet->packetHeader, 0);

	return (packet);
}

static inline caerPolarityEvent caerPolarityEventPacketGetEvent(caerPolarityEventPacket packet, uint32_t n) {
	// Check that we're not out of bounds.
	if (n >= caerEventPacketHeaderGetEventCapacity(&packet->packetHeader)) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_ERROR,
#endif
			"Called caerPolarityEventPacketGetEvent() with invalid event offset %" PRIu32 ", while maximum allowed value is %" PRIu32 ".",
			n, caerEventPacketHeaderGetEventCapacity(&packet->packetHeader));
#endif
		return (NULL);
	}

	// Return a pointer to the specified event.
	return (packet->events + n);
}

static inline uint32_t caerPolarityEventGetTimestamp(caerPolarityEvent event) {
	return (le32toh(event->timestamp));
}

static inline void caerPolarityEventSetTimestamp(caerPolarityEvent event, uint32_t timestamp) {
	event->timestamp = htole32(timestamp);
}

static inline bool caerPolarityEventIsValid(caerPolarityEvent event) {
	return ((le32toh(event->data) >> VALID_MARK_SHIFT) & VALID_MARK_MASK);
}

static inline void caerPolarityEventValidate(caerPolarityEvent event, caerPolarityEventPacket packet) {
	if (!caerPolarityEventIsValid(event)) {
		event->data |= htole32(U32T(1) << VALID_MARK_SHIFT);

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
			"Called caerPolarityEventValidate() on already valid event.");
#endif
	}
}

static inline void caerPolarityEventInvalidate(caerPolarityEvent event, caerPolarityEventPacket packet) {
	if (caerPolarityEventIsValid(event)) {
		event->data &= htole32(~(U32T(1) << VALID_MARK_SHIFT));

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
			"Called caerPolarityEventInvalidate() on already invalid event.");
#endif
	}
}

static inline bool caerPolarityEventGetPolarity(caerPolarityEvent event) {
	return ((le32toh(event->data) >> POLARITY_SHIFT) & POLARITY_MASK);
}

static inline void caerPolarityEventSetPolarity(caerPolarityEvent event, bool polarity) {
	event->data |= htole32((U32T(polarity) & POLARITY_MASK) << POLARITY_SHIFT);
}

static inline uint16_t caerPolarityEventGetY(caerPolarityEvent event) {
	return U16T((le32toh(event->data) >> Y_ADDR_SHIFT) & Y_ADDR_MASK);
}

static inline void caerPolarityEventSetY(caerPolarityEvent event, uint16_t yAddress) {
	event->data |= htole32((U32T(yAddress) & Y_ADDR_MASK) << Y_ADDR_SHIFT);
}

static inline uint16_t caerPolarityEventGetX(caerPolarityEvent event) {
	return U16T((le32toh(event->data) >> X_ADDR_SHIFT) & X_ADDR_MASK);
}

static inline void caerPolarityEventSetX(caerPolarityEvent event, uint16_t xAddress) {
	event->data |= htole32((U32T(xAddress) & X_ADDR_MASK) << X_ADDR_SHIFT);
}

#endif /* POLARITY_H_ */
