/*
 * common.h
 *
 *  Created on: Nov 26, 2013
 *      Author: chtekk
 */

#ifndef COMMON_H_
#define COMMON_H_

#include "main.h"

enum caer_event_types {
	SPECIAL_EVENT = 0,
	POLARITY_EVENT = 1,
	SAMPLE_EVENT = 2,
	EAR_EVENT = 3,
	FRAME_EVENT = 4,
	IMU6_EVENT = 5,
	IMU9_EVENT = 6,
};

struct caer_event_packet_header {
	uint16_t eventType; // Numerical type ID, unique to each event type (see enum).
	uint16_t eventSource; // Numerical source ID, unique inside a process.
	uint32_t eventSize; // Size of one event in bytes.
	uint32_t eventTSOffset; // Offset in bytes at which the main 32bit time-stamp can be found.
	uint32_t eventCapacity; // Maximum number of events this packet can store.
	uint32_t eventNumber; // Total number of events present in this packet (valid + invalid).
	uint32_t eventValid; // Total number of valid events present in this packet.
}__attribute__((__packed__));

typedef struct caer_event_packet_header *caerEventPacketHeader;

static inline uint16_t caerEventPacketHeaderGetEventType(caerEventPacketHeader header) {
	return (le16toh(header->eventType));
}

static inline void caerEventPacketHeaderSetEventType(caerEventPacketHeader header, uint16_t eventType) {
	header->eventType = htole16(eventType);
}

static inline uint16_t caerEventPacketHeaderGetEventSource(caerEventPacketHeader header) {
	return (le16toh(header->eventSource));
}

static inline void caerEventPacketHeaderSetEventSource(caerEventPacketHeader header, uint16_t eventSource) {
	header->eventSource = htole16(eventSource);
}

static inline uint32_t caerEventPacketHeaderGetEventSize(caerEventPacketHeader header) {
	return (le32toh(header->eventSize));
}

static inline void caerEventPacketHeaderSetEventSize(caerEventPacketHeader header, uint32_t eventSize) {
	header->eventSize = htole32(eventSize);
}

static inline uint32_t caerEventPacketHeaderGetEventTSOffset(caerEventPacketHeader header) {
	return (le32toh(header->eventTSOffset));
}

static inline void caerEventPacketHeaderSetEventTSOffset(caerEventPacketHeader header, uint32_t eventTSOffset) {
	header->eventTSOffset = htole32(eventTSOffset);
}

static inline uint32_t caerEventPacketHeaderGetEventCapacity(caerEventPacketHeader header) {
	return (le32toh(header->eventCapacity));
}

static inline void caerEventPacketHeaderSetEventCapacity(caerEventPacketHeader header, uint32_t eventsCapacity) {
	header->eventCapacity = htole32(eventsCapacity);
}

static inline uint32_t caerEventPacketHeaderGetEventNumber(caerEventPacketHeader header) {
	return (le32toh(header->eventNumber));
}

static inline void caerEventPacketHeaderSetEventNumber(caerEventPacketHeader header, uint32_t eventsNumber) {
	header->eventNumber = htole32(eventsNumber);
}

static inline uint32_t caerEventPacketHeaderGetEventValid(caerEventPacketHeader header) {
	return (le32toh(header->eventValid));
}

static inline void caerEventPacketHeaderSetEventValid(caerEventPacketHeader header, uint32_t eventsValid) {
	header->eventValid = htole32(eventsValid);
}

static inline void *caerGenericEventGetEvent(void *headerPtr, uint32_t n) {
	// Check that we're not out of bounds.
	if (n >= caerEventPacketHeaderGetEventCapacity(headerPtr)) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_ERROR,
#endif
			"Called caerGenericEventGetEvent() with invalid event offset %" PRIu32 ", while maximum allowed value is %" PRIu32 ".",
			n, caerEventPacketHeaderGetEventCapacity(headerPtr));
#endif
		return (NULL);
	}

	// Return a pointer to the specified event.
	return (((uint8_t *) headerPtr)
		+ (sizeof(struct caer_event_packet_header) + (n * caerEventPacketHeaderGetEventSize(headerPtr))));
}

static inline uint32_t caerGenericEventGetTimestamp(void *eventPtr, void *headerPtr) {
	return (le32toh(*((uint32_t *) (((uint8_t *) eventPtr) + caerEventPacketHeaderGetEventTSOffset(headerPtr)))));
}

static inline bool caerGenericEventIsValid(void *eventPtr) {
	return (*((uint8_t *) eventPtr) & 0x01);
}

#endif /* COMMON_H_ */
