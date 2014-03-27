/*
 * imu9.h
 *
 *  Created on: Jan 6, 2014
 *      Author: llongi
 */

#ifndef IMU9_H_
#define IMU9_H_

#include "common.h"

struct caer_imu9_event {
	uint16_t info;
	uint16_t accel_x;
	uint16_t accel_y;
	uint16_t accel_z;
	uint16_t gyro_x;
	uint16_t gyro_y;
	uint16_t gyro_z;
	uint16_t temp;
	uint32_t timestamp;
	uint16_t comp_x;
	uint16_t comp_y;
	uint16_t comp_z;
}__attribute__((__packed__));

typedef struct caer_imu9_event *caerIMU9Event;

struct caer_imu9_event_packet {
	struct caer_event_packet_header packetHeader;
	struct caer_imu9_event events[];
}__attribute__((__packed__));

typedef struct caer_imu9_event_packet *caerIMU9EventPacket;

static inline caerIMU9EventPacket caerIMU9EventPacketAllocate(uint32_t eventCapacity, uint16_t eventSource) {
	uint32_t eventSize = sizeof(struct caer_imu9_event);
	size_t eventPacketSize = sizeof(struct caer_imu9_event_packet) + (eventCapacity * eventSize);

	caerIMU9EventPacket packet = malloc(eventPacketSize);
	if (packet == NULL) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_CRITICAL,
#endif
			"Failed to allocate %zu bytes of memory for IMU9 Event Packet of capacity %"
			PRIu32 " from source %" PRIu16 ". Error: %s (%d).", eventPacketSize, eventCapacity, eventSource,
			caerLogStrerror(errno), errno);
#endif
		return (NULL);
	}

	// Zero out event memory (all events invalid).
	memset(packet, 0, eventPacketSize);

	// Fill in header fields.
	caerEventPacketHeaderSetEventType(&packet->packetHeader, IMU9_EVENT);
	caerEventPacketHeaderSetEventSource(&packet->packetHeader, eventSource);
	caerEventPacketHeaderSetEventSize(&packet->packetHeader, eventSize);
	caerEventPacketHeaderSetEventTSOffset(&packet->packetHeader, offsetof(struct caer_imu9_event, timestamp));
	caerEventPacketHeaderSetEventCapacity(&packet->packetHeader, eventCapacity);
	caerEventPacketHeaderSetEventNumber(&packet->packetHeader, 0);
	caerEventPacketHeaderSetEventValid(&packet->packetHeader, 0);

	return (packet);
}

static inline caerIMU9Event caerIMU9EventPacketGetEvent(caerIMU9EventPacket packet, uint32_t n) {
	// Check that we're not out of bounds.
	if (n >= caerEventPacketHeaderGetEventCapacity(&packet->packetHeader)) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_ERROR,
#endif
			"Called caerIMU9EventPacketGetEvent() with invalid event offset %" PRIu32 ", while maximum allowed value is %" PRIu32 ".",
			n, caerEventPacketHeaderGetEventCapacity(&packet->packetHeader));
#endif
		return (NULL);
	}

	// Return a pointer to the specified event.
	return (packet->events + n);
}

static inline uint32_t caerIMU9EventGetTimestamp(caerIMU9Event event) {
	return (le32toh(event->timestamp));
}

static inline void caerIMU9EventSetTimestamp(caerIMU9Event event, uint32_t timestamp) {
	event->timestamp = htole32(timestamp);
}

static inline bool caerIMU9EventIsValid(caerIMU9Event event) {
	return ((le16toh(event->info) >> VALID_MARK_SHIFT) & VALID_MARK_MASK);
}

static inline void caerIMU9EventValidate(caerIMU9Event event, caerIMU9EventPacket packet) {
	if (!caerIMU9EventIsValid(event)) {
		event->info |= htole16(U16T(1) << VALID_MARK_SHIFT);

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
			"Called caerIMU9EventValidate() on already valid event.");
#endif
	}
}

static inline void caerIMU9EventInvalidate(caerIMU9Event event, caerIMU9EventPacket packet) {
	if (caerIMU9EventIsValid(event)) {
		event->info &= htole16(~(U16T(1) << VALID_MARK_SHIFT));

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
			"Called caerIMU9EventInvalidate() on already invalid event.");
#endif
	}
}

static inline uint16_t caerIMU9EventGetAccelX(caerIMU9Event event) {
	return le16toh(event->accel_x);
}

static inline void caerIMU9EventSetAccelX(caerIMU9Event event, uint16_t accelX) {
	event->accel_x = htole16(accelX);
}

static inline uint16_t caerIMU9EventGetAccelY(caerIMU9Event event) {
	return le16toh(event->accel_y);
}

static inline void caerIMU9EventSetAccelY(caerIMU9Event event, uint16_t accelY) {
	event->accel_y = htole16(accelY);
}

static inline uint16_t caerIMU9EventGetAccelZ(caerIMU9Event event) {
	return le16toh(event->accel_z);
}

static inline void caerIMU9EventSetAccelZ(caerIMU9Event event, uint16_t accelZ) {
	event->accel_z = htole16(accelZ);
}

static inline uint16_t caerIMU9EventGetGyroX(caerIMU9Event event) {
	return le16toh(event->gyro_x);
}

static inline void caerIMU9EventSetGyroX(caerIMU9Event event, uint16_t gyroX) {
	event->gyro_x = htole16(gyroX);
}

static inline uint16_t caerIMU9EventGetGyroY(caerIMU9Event event) {
	return le16toh(event->gyro_y);
}

static inline void caerIMU9EventSetGyroY(caerIMU9Event event, uint16_t gyroY) {
	event->gyro_y = htole16(gyroY);
}

static inline uint16_t caerIMU9EventGetGyroZ(caerIMU9Event event) {
	return le16toh(event->gyro_z);
}

static inline void caerIMU9EventSetGyroZ(caerIMU9Event event, uint16_t gyroZ) {
	event->gyro_z = htole16(gyroZ);
}

static inline uint16_t caerIMU9EventGetCompX(caerIMU9Event event) {
	return le16toh(event->comp_x);
}

static inline void caerIMU9EventSetCompX(caerIMU9Event event, uint16_t compX) {
	event->comp_x = htole16(compX);
}

static inline uint16_t caerIMU9EventGetCompY(caerIMU9Event event) {
	return le16toh(event->comp_y);
}

static inline void caerIMU9EventSetCompY(caerIMU9Event event, uint16_t compY) {
	event->comp_y = htole16(compY);
}

static inline uint16_t caerIMU9EventGetCompZ(caerIMU9Event event) {
	return le16toh(event->comp_z);
}

static inline void caerIMU9EventSetCompZ(caerIMU9Event event, uint16_t compZ) {
	event->comp_z = htole16(compZ);
}

static inline uint16_t caerIMU9EventGetTemp(caerIMU9Event event) {
	return le16toh(event->temp);
}

static inline void caerIMU9EventSetTemp(caerIMU9Event event, uint16_t temp) {
	event->temp = htole16(temp);
}

#endif /* IMU9_H_ */
