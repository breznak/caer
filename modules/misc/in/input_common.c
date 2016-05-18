#include "input_common.h"
#include "base/mainloop.h"
#include "ext/c11threads_posix.h"
#include "ext/portable_time.h"
#include "ext/ringbuffer/ringbuffer.h"
#include "ext/buffers.h"

#include <libcaer/events/common.h>
#include <libcaer/events/packetContainer.h>

struct input_common_state {
	/// Control flag for input handling thread.
	atomic_bool running;
	/// The input handling thread (separate as to only wake up mainloop
	/// processing when there is new data available).
	thrd_t inputThread;
	/// Track source ID (cannot change!) to read data for. One source per I/O module!
	int16_t sourceID;
	/// The file descriptor for reading.
	int fileDescriptor;
	/// Network-like stream or file-like stream. Matters for header format.
	bool isNetworkStream;
	/// For network-like inputs, we differentiate between stream and message
	/// based protocols, like TCP and UDP. Matters for header/sequence number.
	bool isNetworkMessageBased;
	/// Keep track of the sequence number for message-based protocols.
	int64_t networkSequenceNumber;
	/// Filter out invalidated events or not.
	atomic_bool validOnly;
	/// Force all incoming packets to be committed to the transfer ring-buffer.
	/// This results in no loss of data, but may deviate from the requested
	/// real-time play-back expectations.
	atomic_bool keepPackets;
	/// Transfer packets coming from the input handling thread to the mainloop.
	/// We use EventPacketContainers, as that is the standard data structure
	/// returned from an input module.
	RingBuffer transferRing;
	/// Track last packet container's highest event timestamp that was sent out.
	int64_t lastTimestamp;
	/// Data buffer for reading from file descriptor (buffered I/O).
	simpleBuffer dataBuffer;

	/// Flag to signal update to buffer configuration asynchronously.
	atomic_bool bufferUpdate;
	/// Reference to parent module's original data.
	caerModuleData parentModule;
};

typedef struct input_common_state *inputCommonState;

size_t CAER_INPUT_COMMON_STATE_STRUCT_SIZE = sizeof(struct input_common_state);

bool caerInputCommonInit(caerModuleData moduleData, int readFd, bool isNetworkStream,
bool isNetworkMessageBased) {
	return (true);
}

void caerInputCommonExit(caerModuleData moduleData) {

}

void caerInputCommonRun(caerModuleData moduleData, size_t argsNumber, va_list args) {

}
