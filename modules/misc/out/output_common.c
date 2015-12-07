#include "output_common.h"
#include <unistd.h>
#include <sys/uio.h>

#include <libcaer/events/common.h>
#include "ext/ringbuffer/ringbuffer.h"

struct output_common_state {
	int fileDescriptor;
	bool validOnly;
	size_t maxBytesPerPacket;
	size_t packetAmount;
	struct eventPacketMapper packetMapper;
	RingBuffer transferRing[];
};

typedef struct output_common_state *outputCommonState;

static void copyPacketToTransferRing(RingBuffer *transferRing, struct eventPacketMapper *packetMapper,
	size_t packetAmount, void *eventPacket);

/**
 * Copy event packets to the right ring buffer for transfer
 * to the external output handling thread.
 *
 * @param transferRing array of ring buffers for data transfer to thread.
 * @param packetMapper array of packet mapper structures.
 * @param packetAmount length of arrays, amount of expected different event packets.
 * @param eventPacket an event packet.
 */
static void copyPacketToTransferRing(RingBuffer *transferRing, struct eventPacketMapper *packetMapper,
	size_t packetAmount, void *eventPacket) {
	// Skip empty event packets.
	if (eventPacket == NULL) {
		return;
	}

	// Get type and source information from the event packet.
	caerEventPacketHeader header = eventPacket;
	int16_t eventSource = caerEventPacketHeaderGetEventSource(header);
	int16_t eventType = caerEventPacketHeaderGetEventType(header);

	int16_t transferRingIndex = -1;

	// Map it to a transfer ring buffer.
	for (size_t i = 0; i < packetAmount; i++) {
		// Check that there is a unique mapping to a transfer ring, or if not,
		// create a new one in a free mapper slot. Slots are filled up in increasing
		// index order, so if we reach empty slots, there can't be a match afterwards.
		if (packetMapper[i].sourceID == eventSource && packetMapper[i].typeID == eventType) {
			// Found match, use it.
			transferRingIndex = packetMapper[i].transferRingID;
			break;
		}

		// Reached empty slot, use it.
		if (packetMapper[i].sourceID == -1) {
			packetMapper[i].sourceID = eventSource;
			packetMapper[i].typeID = eventType;
			packetMapper[i].transferRingID = (int16_t) i;

			transferRingIndex = packetMapper[i].transferRingID;
			break;
		}
	}

	// Check that a valid index was found, else complain.
	if (transferRingIndex == -1) {
		caerLog(CAER_LOG_ERROR, "Data Output",
			"New packet source/type and no more free slots available, this means an unexpected event packet made its way to this output module, one that was not declared at call time.");
		return;
	}

	// Now that we know where to copy the event packet to, let's do it.
	caerEventPacketHeader eventPacketCopy = caerCopyEventPacket(eventPacket);
	if (eventPacketCopy == NULL) {
		// Failed to copy packet, ignore.
		return;
	}

	if (!ringBufferPut(transferRing[transferRingIndex], eventPacketCopy)) {
		// TODO: handle ring buffer full, maybe block on setting?
		caerLog(CAER_LOG_INFO, "Data Output", "Failed to put new packet on transfer ring: ring full.");
	}
}

bool caerOutputCommonInit(caerModuleData moduleData) {
	outputCommonState state = moduleData->moduleState;

	return (true);
}

void caerOutputCommonExit(caerModuleData moduleData) {
	outputCommonState state = moduleData->moduleState;

}

void caerOutputCommonRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	outputCommonState state = moduleData->moduleState;

	for (size_t i = 0; i < argsNumber; i++) {
		caerEventPacketHeader packetHeader = va_arg(args, caerEventPacketHeader);

		copyPacketToTransferRing(state->transferRing, &state->packetMapper, state->packetAmount, packetHeader);
	}
}

void caerOutputCommonConfig(caerModuleData moduleData) {
	outputCommonState state = moduleData->moduleState;

}
