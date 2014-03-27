/*
 * special.h
 *
 *  Created on: Nov 28, 2013
 *      Author: llongi
 */

#ifndef SPECIAL_H_
#define SPECIAL_H_

#include "common.h"

#define TYPE_SHIFT 1
#define TYPE_MASK 0x0000001F
#define DATA_SHIFT 6
#define DATA_MASK 0x03FFFFFF

enum caer_special_event_types {
	TIMESTAMP_WRAP = 0, TIMESTAMP_RESET = 1, EXTERNAL_TRIGGER = 2, ROW_ONLY = 3,
};

struct caer_special_event {
	uint32_t data;
	uint32_t timestamp;
}__attribute__((__packed__));

typedef struct caer_special_event *caerSpecialEvent;

struct caer_special_event_packet {
	struct caer_event_packet_header packetHeader;
	struct caer_special_event events[];
}__attribute__((__packed__));

typedef struct caer_special_event_packet *caerSpecialEventPacket;

static inline caerSpecialEventPacket caerSpecialEventPacketAllocate(uint32_t eventCapacity, uint16_t eventSource) {
	uint32_t eventSize = sizeof(struct caer_special_event);
	size_t eventPacketSize = sizeof(struct caer_special_event_packet) + (eventCapacity * eventSize);

	caerSpecialEventPacket packet = malloc(eventPacketSize);
	if (packet == NULL) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_CRITICAL,
#endif
			"Failed to allocate %zu bytes of memory for Special Event Packet of capacity %"
			PRIu32 " from source %" PRIu16 ". Error: %s (%d).", eventPacketSize, eventCapacity, eventSource,
			caerLogStrerror(errno), errno);
#endif
		return (NULL);
	}

	// Zero out event memory (all events invalid).
	memset(packet, 0, eventPacketSize);

	// Fill in header fields.
	caerEventPacketHeaderSetEventType(&packet->packetHeader, SPECIAL_EVENT);
	caerEventPacketHeaderSetEventSource(&packet->packetHeader, eventSource);
	caerEventPacketHeaderSetEventSize(&packet->packetHeader, eventSize);
	caerEventPacketHeaderSetEventTSOffset(&packet->packetHeader, offsetof(struct caer_special_event, timestamp));
	caerEventPacketHeaderSetEventCapacity(&packet->packetHeader, eventCapacity);
	caerEventPacketHeaderSetEventNumber(&packet->packetHeader, 0);
	caerEventPacketHeaderSetEventValid(&packet->packetHeader, 0);

	return (packet);
}

static inline caerSpecialEvent caerSpecialEventPacketGetEvent(caerSpecialEventPacket packet, uint32_t n) {
	// Check that we're not out of bounds.
	if (n >= caerEventPacketHeaderGetEventCapacity(&packet->packetHeader)) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_ERROR,
#endif
			"Called caerSpecialEventPacketGetEvent() with invalid event offset %" PRIu32 ", while maximum allowed value is %" PRIu32 ".",
			n, caerEventPacketHeaderGetEventCapacity(&packet->packetHeader));
#endif
		return (NULL);
	}

	// Return a pointer to the specified event.
	return (packet->events + n);
}

static inline uint32_t caerSpecialEventGetTimestamp(caerSpecialEvent event) {
	return (le32toh(event->timestamp));
}

static inline void caerSpecialEventSetTimestamp(caerSpecialEvent event, uint32_t timestamp) {
	event->timestamp = htole32(timestamp);
}

static inline bool caerSpecialEventIsValid(caerSpecialEvent event) {
	return ((le32toh(event->data) >> VALID_MARK_SHIFT) & VALID_MARK_MASK);
}

static inline void caerSpecialEventValidate(caerSpecialEvent event, caerSpecialEventPacket packet) {
	if (!caerSpecialEventIsValid(event)) {
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
			"Called caerSpecialEventValidate() on already valid event.");
#endif
	}
}

static inline void caerSpecialEventInvalidate(caerSpecialEvent event, caerSpecialEventPacket packet) {
	if (caerSpecialEventIsValid(event)) {
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
			"Called caerSpecialEventInvalidate() on already invalid event.");
#endif
	}
}

static inline uint8_t caerSpecialEventGetType(caerSpecialEvent event) {
	return U8T((le32toh(event->data) >> TYPE_SHIFT) & TYPE_MASK);
}

static inline void caerSpecialEventSetType(caerSpecialEvent event, uint8_t type) {
	event->data |= htole32((U32T(type) & TYPE_MASK) << TYPE_SHIFT);
}

static inline uint32_t caerSpecialEventGetData(caerSpecialEvent event) {
	return U32T((le32toh(event->data) >> DATA_SHIFT) & DATA_MASK);
}

static inline void caerSpecialEventSetData(caerSpecialEvent event, uint32_t data) {
	event->data |= htole32((U32T(data) & DATA_MASK) << DATA_SHIFT);
}

#endif /* SPECIAL_H_ */
