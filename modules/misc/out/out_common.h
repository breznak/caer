/*
 * out_common.h
 *
 *  Created on: Jan 9, 2014
 *      Author: chtekk
 */

#ifndef OUT_COMMON_H_
#define OUT_COMMON_H_

#include "main.h"
#include "events/common.h"
#include <unistd.h>
#include <sys/uio.h>

#define IOVEC_SIZE 512

static inline void caerOutputCommonSend(caerEventPacketHeader packetHeader, int fileDescriptor, bool validOnly,
	struct iovec *sgioMemory) {
	// If validOnly is not specified, we can just send the whole packet
	// in one go directly.
	if (!validOnly) {
		// First we need to fix the event capacity, since we don't want to
		// send the zeroed-out tail of the packet to conserve bandwidth.
		uint32_t oldCapacity = caerEventPacketHeaderGetEventCapacity(packetHeader);

		// Set it to the event number, which we'll use when writing the packet.
		uint32_t eventNumber = caerEventPacketHeaderGetEventNumber(packetHeader);
		caerEventPacketHeaderSetEventCapacity(packetHeader, eventNumber);

		// Write the whole packet, up to the last event.
		write(fileDescriptor, packetHeader,
			sizeof(*packetHeader) + (eventNumber * caerEventPacketHeaderGetEventSize(packetHeader)));

		// Reset to old value.
		caerEventPacketHeaderSetEventCapacity(packetHeader, oldCapacity);
	}
	else {
		// To conserve bandwidth, we only transmit the valid events here, so
		// the values for capacity and number will have to be adjusted.
		uint32_t oldCapacity = caerEventPacketHeaderGetEventCapacity(packetHeader);
		uint32_t oldNumber = caerEventPacketHeaderGetEventNumber(packetHeader);

		uint32_t eventValid = caerEventPacketHeaderGetEventValid(packetHeader);

		// Use scatter/gather IO to write only the valid events out more
		// efficiently if possible, this is limited by the number of iovec
		// structs available, and so we have to determine if it's possible
		// to actually satisfy the request this way, by looking at how many
		// invalid events there are, each of which could be a split point
		// in the event packet buffer. +1 for the packet header part.
		uint32_t eventSize = caerEventPacketHeaderGetEventSize(packetHeader);

		if (sgioMemory != NULL && (oldNumber - eventValid + 1) <= IOVEC_SIZE) {
			size_t iovecUsed = 0;

			// Scan thorough packet and commit valid runs.
			sgioMemory[iovecUsed].iov_base = packetHeader;
			sgioMemory[iovecUsed].iov_len = sizeof(struct caer_event_packet_header);

			for (uint32_t i = 0; i < oldNumber; i++) {
				void *currEvent = caerGenericEventGetEvent(packetHeader, i);

				if (caerGenericEventIsValid(currEvent)) {
					// If this is the first valid packet after an invalid run,
					// set the data for the new run, else just make current longer.
					if (sgioMemory[iovecUsed].iov_base == NULL) {
						sgioMemory[iovecUsed].iov_base = currEvent;
						sgioMemory[iovecUsed].iov_len = eventSize;
					}
					else {
						sgioMemory[iovecUsed].iov_len += eventSize;
					}
				}
				else {
					// Start a new run, if not already done!
					if (sgioMemory[iovecUsed].iov_base != NULL) {
						sgioMemory[++iovecUsed].iov_base = NULL;
					}
				}
			}

			caerEventPacketHeaderSetEventCapacity(packetHeader, eventValid);
			caerEventPacketHeaderSetEventNumber(packetHeader, eventValid);

			// Done, do the call.
			writev(fileDescriptor, sgioMemory, (int) iovecUsed + 1);
		}
		else {
			// Else we use a much slower allocate-copy-free approach.
			uint8_t *tmpValidEvents = malloc(sizeof(struct caer_event_packet_header) + (eventValid * eventSize));

			if (tmpValidEvents == NULL) {
				// Failure to allocate memory, just don't send packet and log this.
				caerLog(LOG_ALERT, "Output: failed to allocate memory for valid event copy.");
			}
			else {
				// Go through all valid events and copy them.
				size_t currOffset = sizeof(struct caer_event_packet_header);

				for (uint32_t i = 0; i < oldNumber; i++) {
					void *currEvent = caerGenericEventGetEvent(packetHeader, i);

					if (caerGenericEventIsValid(currEvent)) {
						memcpy(tmpValidEvents + currOffset, currEvent, eventSize);
						currOffset += eventSize;
					}
				}

				caerEventPacketHeaderSetEventCapacity(packetHeader, eventValid);
				caerEventPacketHeaderSetEventNumber(packetHeader, eventValid);

				// Last, copy the header, _after_ it's been manipulated/updated.
				memcpy(tmpValidEvents, packetHeader, sizeof(struct caer_event_packet_header));

				write(fileDescriptor, tmpValidEvents, currOffset);

				free(tmpValidEvents);
			}
		}

		// Reset to old value.
		caerEventPacketHeaderSetEventCapacity(packetHeader, oldCapacity);
		caerEventPacketHeaderSetEventNumber(packetHeader, oldNumber);
	}
}

#endif /* OUT_COMMON_H_ */
