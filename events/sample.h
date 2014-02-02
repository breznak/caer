/*
 * sample.h
 *
 *  Created on: Jan 6, 2014
 *      Author: llongi
 */

#ifndef SAMPLE_H_
#define SAMPLE_H_

#include "common.h"

// 0 in the 0th bit means invalid, 1 means valid.
// This way zeroing-out an event packet sets all its events to invalid.
#define VALID_MARK_SHIFT 0
#define VALID_MARK_MASK 0x00000001
#define TYPE_SHIFT 1
#define TYPE_MASK 0x0000001F
#define SAMPLE_SHIFT 8
#define SAMPLE_MASK 0x00FFFFFF

struct caer_sample_event {
	uint32_t data;
	uint32_t timestamp;
}__attribute__((__packed__));

typedef struct caer_sample_event *caerSampleEvent;

struct caer_sample_event_packet {
	struct caer_event_packet_header packetHeader;
	struct caer_sample_event events[];
}__attribute__((__packed__));

typedef struct caer_sample_event_packet *caerSampleEventPacket;

static inline caerSampleEventPacket caerSampleEventPacketAllocate(uint32_t eventCapacity, uint16_t eventSource) {
	uint32_t eventSize = sizeof(struct caer_sample_event);
	size_t eventPacketSize = sizeof(struct caer_sample_event_packet) + (eventCapacity * eventSize);

	caerSampleEventPacket packet = malloc(eventPacketSize);
	if (packet == NULL) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_CRITICAL,
#endif
			"Failed to allocate %zu bytes of memory for Sample Event Packet of capacity %"
			PRIu32 " from source %" PRIu16 ". Error: %s (%d).", eventPacketSize, eventCapacity, eventSource,
			caerLogStrerror(errno), errno);
#endif
		return (NULL);
	}

	// Zero out event memory (all events invalid).
	memset(packet, 0, eventPacketSize);

	// Fill in header fields.
	caerEventPacketHeaderSetEventType(&packet->packetHeader, SAMPLE_EVENT);
	caerEventPacketHeaderSetEventSource(&packet->packetHeader, eventSource);
	caerEventPacketHeaderSetEventSize(&packet->packetHeader, eventSize);
	caerEventPacketHeaderSetEventTSOffset(&packet->packetHeader, offsetof(struct caer_sample_event, timestamp));
	caerEventPacketHeaderSetEventCapacity(&packet->packetHeader, eventCapacity);
	caerEventPacketHeaderSetEventNumber(&packet->packetHeader, 0);
	caerEventPacketHeaderSetEventValid(&packet->packetHeader, 0);

	return (packet);
}

static inline caerSampleEvent caerSampleEventPacketGetEvent(caerSampleEventPacket packet, uint32_t n) {
	// Check that we're not out of bounds.
	if (n >= caerEventPacketHeaderGetEventCapacity(&packet->packetHeader)) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_ERROR,
#endif
			"Called caerSampleEventPacketGetEvent() with invalid event offset %" PRIu32 ", while maximum allowed value is %" PRIu32 ".",
			n, caerEventPacketHeaderGetEventCapacity(&packet->packetHeader));
#endif
		return (NULL);
	}

	// Return a pointer to the specified event.
	return (packet->events + n);
}

static inline uint32_t caerSampleEventGetTimestamp(caerSampleEvent event) {
	return (le32toh(event->timestamp));
}

static inline void caerSampleEventSetTimestamp(caerSampleEvent event, uint32_t timestamp) {
	event->timestamp = htole32(timestamp);
}

static inline bool caerSampleEventIsValid(caerSampleEvent event) {
	return ((le32toh(event->data) >> VALID_MARK_SHIFT) & VALID_MARK_MASK);
}

static inline void caerSampleEventValidate(caerSampleEvent event, caerSampleEventPacket packet) {
	if (!caerSampleEventIsValid(event)) {
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
			"Called caerSampleEventValidate() on already valid event.");
#endif
	}
}

static inline void caerSampleEventInvalidate(caerSampleEvent event, caerSampleEventPacket packet) {
	if (caerSampleEventIsValid(event)) {
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
			"Called caerSampleEventInvalidate() on already invalid event.");
#endif
	}
}

static inline uint8_t caerSampleEventGetType(caerSampleEvent event) {
	return U8T((le32toh(event->data) >> TYPE_SHIFT) & TYPE_MASK);
}

static inline void caerSampleEventSetType(caerSampleEvent event, uint8_t type) {
	event->data |= htole32((U32T(type) & TYPE_MASK) << TYPE_SHIFT);
}

static inline uint32_t caerSampleEventGetSample(caerSampleEvent event) {
	return U32T((le32toh(event->data) >> SAMPLE_SHIFT) & SAMPLE_MASK);
}

static inline void caerSampleEventSetSample(caerSampleEvent event, uint32_t sample) {
	event->data |= htole32((U32T(sample) & SAMPLE_MASK) << SAMPLE_SHIFT);
}

#endif /* SAMPLE_H_ */
