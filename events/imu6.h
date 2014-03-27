/*
 * imu6.h
 *
 *  Created on: Jan 6, 2014
 *      Author: llongi
 */

#ifndef IMU6_H_
#define IMU6_H_

#include "common.h"

struct caer_imu6_event {
	uint16_t info;
	uint16_t accel_x;
	uint16_t accel_y;
	uint16_t accel_z;
	uint16_t gyro_x;
	uint16_t gyro_y;
	uint16_t gyro_z;
	uint16_t temp;
	uint32_t timestamp;
}__attribute__((__packed__));

typedef struct caer_imu6_event *caerIMU6Event;

struct caer_imu6_event_packet {
	struct caer_event_packet_header packetHeader;
	struct caer_imu6_event events[];
}__attribute__((__packed__));

typedef struct caer_imu6_event_packet *caerIMU6EventPacket;

static inline caerIMU6EventPacket caerIMU6EventPacketAllocate(uint32_t eventCapacity, uint16_t eventSource) {
	uint32_t eventSize = sizeof(struct caer_imu6_event);
	size_t eventPacketSize = sizeof(struct caer_imu6_event_packet) + (eventCapacity * eventSize);

	caerIMU6EventPacket packet = malloc(eventPacketSize);
	if (packet == NULL) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_CRITICAL,
#endif
			"Failed to allocate %zu bytes of memory for IMU6 Event Packet of capacity %"
			PRIu32 " from source %" PRIu16 ". Error: %s (%d).", eventPacketSize, eventCapacity, eventSource,
			caerLogStrerror(errno), errno);
#endif
		return (NULL);
	}

	// Zero out event memory (all events invalid).
	memset(packet, 0, eventPacketSize);

	// Fill in header fields.
	caerEventPacketHeaderSetEventType(&packet->packetHeader, IMU6_EVENT);
	caerEventPacketHeaderSetEventSource(&packet->packetHeader, eventSource);
	caerEventPacketHeaderSetEventSize(&packet->packetHeader, eventSize);
	caerEventPacketHeaderSetEventTSOffset(&packet->packetHeader, offsetof(struct caer_imu6_event, timestamp));
	caerEventPacketHeaderSetEventCapacity(&packet->packetHeader, eventCapacity);
	caerEventPacketHeaderSetEventNumber(&packet->packetHeader, 0);
	caerEventPacketHeaderSetEventValid(&packet->packetHeader, 0);

	return (packet);
}

static inline caerIMU6Event caerIMU6EventPacketGetEvent(caerIMU6EventPacket packet, uint32_t n) {
	// Check that we're not out of bounds.
	if (n >= caerEventPacketHeaderGetEventCapacity(&packet->packetHeader)) {
#if !defined(LOG_NONE)
#if defined(PRINTF_LOG)
		fprintf(stderr,
#else
		caerLog(LOG_ERROR,
#endif
			"Called caerIMU6EventPacketGetEvent() with invalid event offset %" PRIu32 ", while maximum allowed value is %" PRIu32 ".",
			n, caerEventPacketHeaderGetEventCapacity(&packet->packetHeader));
#endif
		return (NULL);
	}

	// Return a pointer to the specified event.
	return (packet->events + n);
}

static inline uint32_t caerIMU6EventGetTimestamp(caerIMU6Event event) {
	return (le32toh(event->timestamp));
}

static inline void caerIMU6EventSetTimestamp(caerIMU6Event event, uint32_t timestamp) {
	event->timestamp = htole32(timestamp);
}

static inline bool caerIMU6EventIsValid(caerIMU6Event event) {
	return ((le16toh(event->info) >> VALID_MARK_SHIFT) & VALID_MARK_MASK);
}

static inline void caerIMU6EventValidate(caerIMU6Event event, caerIMU6EventPacket packet) {
	if (!caerIMU6EventIsValid(event)) {
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
			"Called caerIMU6EventValidate() on already valid event.");
#endif
	}
}

static inline void caerIMU6EventInvalidate(caerIMU6Event event, caerIMU6EventPacket packet) {
	if (caerIMU6EventIsValid(event)) {
		event->info &= htole16((uint16_t)(~(U16T(1) << VALID_MARK_SHIFT)));

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
			"Called caerIMU6EventInvalidate() on already invalid event.");
#endif
	}
}

static inline uint16_t caerIMU6EventGetAccelX(caerIMU6Event event) {
	return le16toh(event->accel_x);
}

static inline void caerIMU6EventSetAccelX(caerIMU6Event event, uint16_t accelX) {
	event->accel_x = htole16(accelX);
}

static inline uint16_t caerIMU6EventGetAccelY(caerIMU6Event event) {
	return le16toh(event->accel_y);
}

static inline void caerIMU6EventSetAccelY(caerIMU6Event event, uint16_t accelY) {
	event->accel_y = htole16(accelY);
}

static inline uint16_t caerIMU6EventGetAccelZ(caerIMU6Event event) {
	return le16toh(event->accel_z);
}

static inline void caerIMU6EventSetAccelZ(caerIMU6Event event, uint16_t accelZ) {
	event->accel_z = htole16(accelZ);
}

static inline uint16_t caerIMU6EventGetGyroX(caerIMU6Event event) {
	return le16toh(event->gyro_x);
}

static inline void caerIMU6EventSetGyroX(caerIMU6Event event, uint16_t gyroX) {
	event->gyro_x = htole16(gyroX);
}

static inline uint16_t caerIMU6EventGetGyroY(caerIMU6Event event) {
	return le16toh(event->gyro_y);
}

static inline void caerIMU6EventSetGyroY(caerIMU6Event event, uint16_t gyroY) {
	event->gyro_y = htole16(gyroY);
}

static inline uint16_t caerIMU6EventGetGyroZ(caerIMU6Event event) {
	return le16toh(event->gyro_z);
}

static inline void caerIMU6EventSetGyroZ(caerIMU6Event event, uint16_t gyroZ) {
	event->gyro_z = htole16(gyroZ);
}

static inline uint16_t caerIMU6EventGetTemp(caerIMU6Event event) {
	return le16toh(event->temp);
}

static inline void caerIMU6EventSetTemp(caerIMU6Event event, uint16_t temp) {
	event->temp = htole16(temp);
}

#endif /* IMU6_H_ */
