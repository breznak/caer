/*
 * dvs128.c
 *
 *  Created on: Nov 26, 2013
 *      Author: chtekk
 */

#include "dvs128.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "ext/ringbuffer/ringbuffer.h"
#include <pthread.h>
#include <libusb.h>

struct dvs128_state {
	// Data Acquisition Thread -> Mainloop Exchange
	pthread_t dataAcquisitionThread;
	RingBuffer dataExchangeBuffer;
	caerMainloopData mainloopNotify;
	uint16_t sourceID;
	// USB Device State
	libusb_context *deviceContext;
	libusb_device_handle *deviceHandle;
	// Data Acquisition Thread State
	struct libusb_transfer **transfers;
	atomic_ops_uint transfersLength;
	uint32_t wrapAdd;
	uint32_t lastTimestamp;
	// Polarity Packet State
	caerPolarityEventPacket currentPolarityPacket;
	uint32_t currentPolarityPacketPosition;
	uint32_t maxPolarityPacketSize;
	uint32_t maxPolarityPacketInterval;
	// Special Packet State
	caerSpecialEventPacket currentSpecialPacket;
	uint32_t currentSpecialPacketPosition;
	uint32_t maxSpecialPacketSize;
	uint32_t maxSpecialPacketInterval;
};

typedef struct dvs128_state *dvs128State;

static bool caerInputDVS128Init(caerModuleData moduleData);
static void caerInputDVS128Run(caerModuleData moduleData, size_t argsNumber, va_list args);
// CONFIG: Nothing to do here in the main thread!
// Biases are configured asynchronously, and buffer sizes in the data
// acquisition thread itself. Resetting the main config_refresh flag
// will also happen there.
static void caerInputDVS128Exit(caerModuleData moduleData);

static struct caer_module_functions caerInputDVS128Functions = { .moduleInit = &caerInputDVS128Init, .moduleRun =
	&caerInputDVS128Run, .moduleConfig = NULL, .moduleExit = &caerInputDVS128Exit };

void caerInputDVS128(uint16_t moduleID, caerPolarityEventPacket *polarity, caerSpecialEventPacket *special) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "DVS128");

	// IMPORTANT: THE CONTENT OF OUTPUT ARGUMENTS MUST BE SET TO NULL!
	if (polarity != NULL) {
		*polarity = NULL;
	}
	if (special != NULL) {
		*special = NULL;
	}

	caerModuleSM(&caerInputDVS128Functions, moduleData, sizeof(struct dvs128_state), 2, polarity, special);
}

static void *dvs128DataAcquisitionThread(void *inPtr);
static void dvs128DataAcquisitionThreadConfig(caerModuleData data);
static void dvs128AllocateTransfers(dvs128State state, uint32_t bufferNum, uint32_t bufferSize);
static void dvs128DeallocateTransfers(dvs128State state);
static void LIBUSB_CALL dvs128LibUsbCallback(struct libusb_transfer *transfer);
static void dvs128EventTranslator(dvs128State state, uint8_t *buffer, size_t bytesSent);
static void dvs128SendBiases(sshsNode biasNode, libusb_device_handle *devHandle);
static libusb_device_handle *dvs128Open(libusb_context *devContext);
static void dvs128Close(libusb_device_handle *devHandle);
static void caerInputDVS128ConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);

static inline void freeAllPackets(dvs128State state) {
	free(state->currentPolarityPacket);
	free(state->currentSpecialPacket);
}

static bool caerInputDVS128Init(caerModuleData moduleData) {
	caerLog(LOG_DEBUG, "Initializing DVS128 module ...");

	// First, always create all needed setting nodes, set their default values
	// and add their listeners.
	// Set default biases, from DVS128Fast.xml settings.
	sshsNode biasNode = sshsGetRelativeNode(moduleData->moduleNode, "bias/");
	sshsNodePutIntIfAbsent(biasNode, "cas", 1992);
	sshsNodePutIntIfAbsent(biasNode, "injGnd", 1108364);
	sshsNodePutIntIfAbsent(biasNode, "reqPd", 16777215);
	sshsNodePutIntIfAbsent(biasNode, "puX", 8159221);
	sshsNodePutIntIfAbsent(biasNode, "diffOff", 132);
	sshsNodePutIntIfAbsent(biasNode, "req", 309590);
	sshsNodePutIntIfAbsent(biasNode, "refr", 969);
	sshsNodePutIntIfAbsent(biasNode, "puY", 16777215);
	sshsNodePutIntIfAbsent(biasNode, "diffOn", 209996);
	sshsNodePutIntIfAbsent(biasNode, "diff", 13125);
	sshsNodePutIntIfAbsent(biasNode, "foll", 271);
	sshsNodePutIntIfAbsent(biasNode, "pr", 217);

	// USB buffer settings.
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "bufferNumber", 8);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "bufferSize", 4096);

	// Packet settings (size (in events) and time interval (in µs)).
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "polarityPacketMaxSize", 4096);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "polarityPacketMaxInterval", 5000);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "specialPacketMaxSize", 128);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "specialPacketMaxInterval", 1000);

	// Ring-buffer setting (only changes value on module init/shutdown cycles).
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "dataExchangeBufferSize", 64);

	// Install default listener to signal configuration updates asynchronously.
	sshsNodeAddAttrListener(biasNode, moduleData, &caerInputDVS128ConfigListener);
	sshsNodeAddAttrListener(moduleData->moduleNode, moduleData, &caerInputDVS128ConfigListener);

	dvs128State state = moduleData->moduleState;

	// Data source is the same as the module ID (but accessible in state-space).
	state->sourceID = moduleData->moduleID;

	// Put global source information into SSHS.
	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");
	sshsNodePutShort(sourceInfoNode, "sizeX", 128);
	sshsNodePutShort(sourceInfoNode, "sizeY", 128);

	// Initialize state fields.
	state->maxPolarityPacketSize = sshsNodeGetInt(moduleData->moduleNode, "polarityPacketMaxSize");
	state->maxPolarityPacketInterval = sshsNodeGetInt(moduleData->moduleNode, "polarityPacketMaxInterval");

	state->maxSpecialPacketSize = sshsNodeGetInt(moduleData->moduleNode, "specialPacketMaxSize");
	state->maxSpecialPacketInterval = sshsNodeGetInt(moduleData->moduleNode, "specialPacketMaxInterval");

	state->currentPolarityPacket = caerPolarityEventPacketAllocate(state->maxPolarityPacketSize, state->sourceID);
	state->currentPolarityPacketPosition = 0;

	state->currentSpecialPacket = caerSpecialEventPacketAllocate(state->maxSpecialPacketSize, state->sourceID);
	state->currentSpecialPacketPosition = 0;

	state->wrapAdd = 0;
	state->lastTimestamp = 0;

	// Store reference to parent mainloop, so that we can correctly notify
	// the availability or not of data to consume.
	state->mainloopNotify = caerMainloopGetReference();

	// Create data exchange buffers.
	state->dataExchangeBuffer = ringBufferInit(sshsNodeGetInt(moduleData->moduleNode, "dataExchangeBufferSize"));
	if (state->dataExchangeBuffer == NULL) {
		free(state->currentPolarityPacket);
		free(state->currentSpecialPacket);

		caerLog(LOG_CRITICAL, "Failed to initialize data exchange buffer.");
		return (false);
	}

	// Initialize libusb using a separate context for each device.
	// This is to correctly support one thread per device.
	if ((errno = libusb_init(&state->deviceContext)) != LIBUSB_SUCCESS) {
		freeAllPackets(state);
		ringBufferFree(state->dataExchangeBuffer);

		caerLog(LOG_CRITICAL, "Failed to initialize libusb context. Error: %s (%d).", libusb_strerror(errno), errno);
		return (false);
	}

	// Try to open a DVS128 device.
	state->deviceHandle = dvs128Open(state->deviceContext);
	if (state->deviceHandle == NULL) {
		freeAllPackets(state);
		ringBufferFree(state->dataExchangeBuffer);
		libusb_exit(state->deviceContext);

		caerLog(LOG_CRITICAL, "Failed to open DVS128 device.");
		return (false);
	}

	// Start data acquisition thread.
	if ((errno = pthread_create(&state->dataAcquisitionThread, NULL, &dvs128DataAcquisitionThread, moduleData)) != 0) {
		freeAllPackets(state);
		ringBufferFree(state->dataExchangeBuffer);
		dvs128Close(state->deviceHandle);
		libusb_exit(state->deviceContext);

		caerLog(LOG_CRITICAL, "Failed to start data acquisition thread. Error: %s (%d).", caerLogStrerror(errno),
		errno);
		return (false);
	}

	caerLog(LOG_DEBUG, "Initialized DVS128 module successfully with device Bus=%" PRIu8 ":Addr=%" PRIu8 ".",
		libusb_get_bus_number(libusb_get_device(state->deviceHandle)),
		libusb_get_device_address(libusb_get_device(state->deviceHandle)));
	return (true);
}

static void caerInputDVS128Exit(caerModuleData moduleData) {
	caerLog(LOG_DEBUG, "Shutting down DVS128 module ...");

	dvs128State state = moduleData->moduleState;

	// Wait for data acquisition thread to terminate...
	if ((errno = pthread_join(state->dataAcquisitionThread, NULL)) != 0) {
		// This should never happen!
		caerLog(LOG_CRITICAL, "Failed to join data acquisition thread. Error: %s (%d).", caerLogStrerror(errno),
		errno);
	}

	// Finally, close the device fully.
	dvs128Close(state->deviceHandle);

	// Destroy libusb context.
	libusb_exit(state->deviceContext);

	// Empty ringbuffer.
	void *packet;
	while ((packet = ringBufferGet(state->dataExchangeBuffer)) != NULL) {
		caerMainloopDataAvailableDecrease(state->mainloopNotify);
		free(packet);
	}

	// And destroy it.
	ringBufferFree(state->dataExchangeBuffer);

	// Free remaining incomplete packets.
	freeAllPackets(state);

	caerLog(LOG_DEBUG, "Shutdown DVS128 module successfully.");
}

static void caerInputDVS128Run(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerPolarityEventPacket *polarity = va_arg(args, caerPolarityEventPacket *);
	caerSpecialEventPacket *special = va_arg(args, caerSpecialEventPacket *);

	dvs128State state = moduleData->moduleState;

	// Check what the user wants.
	bool wantPolarity = false, havePolarity = false;
	bool wantSpecial = false, haveSpecial = false;

	if (polarity != NULL) {
		wantPolarity = true;
	}

	if (special != NULL) {
		wantSpecial = true;
	}

	void *packet;
	while ((packet = ringBufferLook(state->dataExchangeBuffer)) != NULL) {
		// Check what kind it is and assign accordingly.
		caerEventPacketHeader packetHeader = packet;

		// Check polarity events first, then the special ones.
		if (packetHeader->eventType == POLARITY_EVENT) {
			// Throw away unwanted packets first.
			if (!wantPolarity) {
				caerMainloopDataAvailableDecrease(state->mainloopNotify);
				free(ringBufferGet(state->dataExchangeBuffer));
				continue;
			}

			// At this point packet is something we want, so we see if we can
			// assign it to one of the output pointers. This will be possible if
			// the output is still free, if not, we have to wait until next loop
			// iteration to fit this somewhere, and so we exit.
			if (!havePolarity) {
				caerMainloopDataAvailableDecrease(state->mainloopNotify);
				*polarity = ringBufferGet(state->dataExchangeBuffer);
				havePolarity = true;

				// Ensure memory gets recycled after the loop is over.
				caerMainloopFreeAfterLoop(*polarity);

				continue;
			}

			// Couldn't fit the current packet into any free output pointers,
			// break off and defer to next iteration of mainloop.
			break;
		}
		else if (packetHeader->eventType == SPECIAL_EVENT) {
			// Throw away unwanted packets first.
			if (!wantSpecial) {
				caerMainloopDataAvailableDecrease(state->mainloopNotify);
				free(ringBufferGet(state->dataExchangeBuffer));
				continue;
			}

			// At this point packet is something we want, so we see if we can
			// assign it to one of the output pointers. This will be possible if
			// the output is still free, if not, we have to wait until next loop
			// iteration to fit this somewhere, and so we exit.
			if (!haveSpecial) {
				caerMainloopDataAvailableDecrease(state->mainloopNotify);
				*special = ringBufferGet(state->dataExchangeBuffer);
				haveSpecial = true;

				// Ensure memory gets recycled after the loop is over.
				caerMainloopFreeAfterLoop(*special);

				continue;
			}

			// Couldn't fit the current packet into any free output pointers,
			// break off and defer to next iteration of mainloop.
			break;
		}
	}
}

static void *dvs128DataAcquisitionThread(void *inPtr) {
	caerLog(LOG_DEBUG, "DVS128: initializing data acquisition thread ...");

	// inPtr is a pointer to module data.
	caerModuleData data = inPtr;
	dvs128State state = data->moduleState;

	// Send default start-up biases to device before enabling it.
	dvs128SendBiases(sshsGetRelativeNode(data->moduleNode, "bias/"), state->deviceHandle);

	// Create buffers as specified in config file.
	dvs128AllocateTransfers(state, sshsNodeGetInt(data->moduleNode, "bufferNumber"),
		sshsNodeGetInt(data->moduleNode, "bufferSize"));

	// Enable AER data transfer on USB end-point 6.
	libusb_control_transfer(state->deviceHandle,
		LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		VENDOR_REQUEST_START_TRANSFER, 0, 0, NULL, 0, 0);

	// Handle USB events (1 second timeout).
	struct timeval te = { .tv_sec = 0, .tv_usec = 1000000 };

	caerLog(LOG_DEBUG, "DVS128: data acquisition thread ready to process events.");

	while (atomic_ops_uint_load(&data->running, ATOMIC_OPS_FENCE_NONE) != 0
		&& atomic_ops_uint_load(&state->transfersLength, ATOMIC_OPS_FENCE_NONE) > 0) {
		// Check config refresh, in this case to adjust buffer sizes.
		if (atomic_ops_uint_load(&data->configUpdate, ATOMIC_OPS_FENCE_NONE) != 0) {
			dvs128DataAcquisitionThreadConfig(data);
		}

		libusb_handle_events_timeout(state->deviceContext, &te);
	}

	caerLog(LOG_DEBUG, "DVS128: shutting down data acquisition thread ...");

	// Disable AER data transfer on USB end-point 6.
	libusb_control_transfer(state->deviceHandle,
		LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		VENDOR_REQUEST_STOP_TRANSFER, 0, 0, NULL, 0, 0);

	// Cancel all transfers and handle them.
	dvs128DeallocateTransfers(state);

	// Ensure parent also shuts down (on disconnected device for example).
	sshsNodePutBool(data->moduleNode, "shutdown", true);

	caerLog(LOG_DEBUG, "DVS128: data acquisition thread shut down.");

	return (NULL);
}

static void dvs128DataAcquisitionThreadConfig(caerModuleData moduleData) {
	dvs128State state = moduleData->moduleState;

	// Get the current value to examine by atomic exchange, since we don't
	// want there to be any possible store between a load/store pair.
	uintptr_t configUpdate = atomic_ops_uint_swap(&moduleData->configUpdate, 0, ATOMIC_OPS_FENCE_NONE);

	if (configUpdate & (0x01 << 0)) {
		// Bias update required.
		dvs128SendBiases(sshsGetRelativeNode(moduleData->moduleNode, "bias/"), state->deviceHandle);
	}

	if (configUpdate & (0x01 << 1)) {
		// Do buffer size change: cancel all and recreate them.
		dvs128DeallocateTransfers(state);
		dvs128AllocateTransfers(state, sshsNodeGetInt(moduleData->moduleNode, "bufferNumber"),
			sshsNodeGetInt(moduleData->moduleNode, "bufferSize"));
	}

	if (configUpdate & (0x01 << 2)) {
		// Update maximum size and interval settings for packets.
		state->maxPolarityPacketSize = sshsNodeGetInt(moduleData->moduleNode, "polarityPacketMaxSize");
		state->maxPolarityPacketInterval = sshsNodeGetInt(moduleData->moduleNode, "polarityPacketMaxInterval");

		state->maxSpecialPacketSize = sshsNodeGetInt(moduleData->moduleNode, "specialPacketMaxSize");
		state->maxSpecialPacketInterval = sshsNodeGetInt(moduleData->moduleNode, "specialPacketMaxInterval");
	}
}

static void dvs128AllocateTransfers(dvs128State state, uint32_t bufferNum, uint32_t bufferSize) {
	atomic_ops_uint_store(&state->transfersLength, 0, ATOMIC_OPS_FENCE_NONE);

	// Set number of transfers and allocate memory for the main transfer array.
	state->transfers = calloc(bufferNum, sizeof(struct libusb_transfer *));
	if (state->transfers == NULL) {
		caerLog(LOG_CRITICAL, "Failed to allocate memory for %" PRIu32 " libusb transfers. Error: %s (%d).", bufferNum,
			caerLogStrerror(errno), errno);
		return;
	}

	// Allocate transfers and set them up.
	for (size_t i = 0; i < bufferNum; i++) {
		state->transfers[i] = libusb_alloc_transfer(0);
		if (state->transfers[i] == NULL) {
			caerLog(LOG_CRITICAL, "Unable to allocate further libusb transfers (%zu of %" PRIu32 ").", i, bufferNum);
			return;
		}

		// Create data buffer.
		state->transfers[i]->length = (int) bufferSize;
		state->transfers[i]->buffer = malloc(bufferSize);
		if (state->transfers[i]->buffer == NULL) {
			caerLog(LOG_CRITICAL, "Unable to allocate buffer for libusb transfer %zu. Error: %s (%d).", i,
				caerLogStrerror(errno), errno);

			libusb_free_transfer(state->transfers[i]);
			state->transfers[i] = NULL;

			return;
		}

		// Initialize Transfer.
		state->transfers[i]->dev_handle = state->deviceHandle;
		state->transfers[i]->endpoint = USB_IO_ENDPOINT;
		state->transfers[i]->type = LIBUSB_TRANSFER_TYPE_BULK;
		state->transfers[i]->callback = &dvs128LibUsbCallback;
		state->transfers[i]->user_data = state;
		state->transfers[i]->timeout = 0;
		state->transfers[i]->flags = LIBUSB_TRANSFER_FREE_BUFFER;

		if ((errno = libusb_submit_transfer(state->transfers[i])) == LIBUSB_SUCCESS) {
			atomic_ops_uint_inc(&state->transfersLength, ATOMIC_OPS_FENCE_NONE);
		}
		else {
			caerLog(LOG_CRITICAL, "Unable to submit libusb transfer %zu. Error: %s (%d).", i, libusb_strerror(errno),
			errno);

			// The transfer buffer is freed automatically here thanks to
			// the LIBUSB_TRANSFER_FREE_BUFFER flag set above.
			libusb_free_transfer(state->transfers[i]);
			state->transfers[i] = NULL;

			return;
		}
	}
}

static void dvs128DeallocateTransfers(dvs128State state) {
	// This will change later on, but we still need it.
	uint32_t transfersNum = (uint32_t) atomic_ops_uint_load(&state->transfersLength, ATOMIC_OPS_FENCE_NONE);

	// Cancel all current transfers first.
	for (size_t i = 0; i < transfersNum; i++) {
		errno = libusb_cancel_transfer(state->transfers[i]);
		if (errno != LIBUSB_SUCCESS && errno != LIBUSB_ERROR_NOT_FOUND) {
			caerLog(LOG_CRITICAL, "Unable to cancel libusb transfer %zu. Error: %s (%d).", i, libusb_strerror(errno),
			errno);
			// Proceed with canceling all transfers regardless of errors.
		}
	}

	// Wait for all transfers to go away (0.1 seconds timeout).
	struct timeval te = { .tv_sec = 0, .tv_usec = 100000 };

	while (atomic_ops_uint_load(&state->transfersLength, ATOMIC_OPS_FENCE_NONE) > 0) {
		libusb_handle_events_timeout(state->deviceContext, &te);
	}

	// The buffers and transfers have been deallocated in the callback.
	// Only the transfers array remains.
	free(state->transfers);
}

static void LIBUSB_CALL dvs128LibUsbCallback(struct libusb_transfer *transfer) {
	dvs128State state = transfer->user_data;

	if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
		// Handle data.
		dvs128EventTranslator(state, transfer->buffer, (size_t) transfer->actual_length);
	}

	if (transfer->status != LIBUSB_TRANSFER_CANCELLED && transfer->status != LIBUSB_TRANSFER_NO_DEVICE) {
		// Submit transfer again.
		if (libusb_submit_transfer(transfer) == LIBUSB_SUCCESS) {
			return;
		}
	}

	// Cannot recover (cancelled, no device, or other critical error).
	// Signal this by adjusting the counter, free and exit.
	atomic_ops_uint_dec(&state->transfersLength, ATOMIC_OPS_FENCE_NONE);
	libusb_free_transfer(transfer);
}

#define DVS128_POLARITY_SHIFT 0
#define DVS128_POLARITY_MASK 0x0001
#define DVS128_Y_ADDR_SHIFT 8
#define DVS128_Y_ADDR_MASK 0x007F
#define DVS128_X_ADDR_SHIFT 1
#define DVS128_X_ADDR_MASK 0x007F
#define DVS128_SYNC_EVENT_MASK 0x8000

static void dvs128EventTranslator(dvs128State state, uint8_t *buffer, size_t bytesSent) {
	// Truncate off any extra partial event.
	bytesSent &= (size_t) ~0x03;

	for (size_t i = 0; i < bytesSent; i += 4) {
		bool forcePacketCommit = false;

		if ((buffer[i + 3] & 0x80) == 0x80) {
			// timestamp bit 15 is one -> wrap: now we need to increment
			// the wrapAdd, uses only 14 bit timestamps
			state->wrapAdd += 0x4000;

			// Detect big timestamp wrap-around.
			if (state->wrapAdd == 0) {
				// Reset lastTimestamp to zero at this point, so we can again
				// start detecting overruns of the 32bit value.
				state->lastTimestamp = 0;

				caerSpecialEvent currentEvent = caerSpecialEventPacketGetEvent(state->currentSpecialPacket,
					state->currentSpecialPacketPosition++);
				caerSpecialEventSetTimestamp(currentEvent, UINT32_MAX);
				caerSpecialEventSetType(currentEvent, TIMESTAMP_WRAP);
				caerSpecialEventValidate(currentEvent, state->currentSpecialPacket);

				// Commit packets to separate before wrap from after cleanly.
				forcePacketCommit = true;
			}
		}
		else if ((buffer[i + 3] & 0x40) == 0x40) {
			// timestamp bit 14 is one -> wrapAdd reset: this firmware
			// version uses reset events to reset timestamps
			state->wrapAdd = 0;
			state->lastTimestamp = 0;

			// Create timestamp reset event.
			caerSpecialEvent currentEvent = caerSpecialEventPacketGetEvent(state->currentSpecialPacket,
				state->currentSpecialPacketPosition++);
			caerSpecialEventSetTimestamp(currentEvent, UINT32_MAX);
			caerSpecialEventSetType(currentEvent, TIMESTAMP_RESET);
			caerSpecialEventValidate(currentEvent, state->currentSpecialPacket);

			// Commit packets when doing a reset to clearly separate them.
			forcePacketCommit = true;
		}
		else {
			// address is LSB MSB (USB is LE)
			uint16_t addressUSB = le16toh(*((uint16_t * ) (&buffer[i])));

			// same for timestamp, LSB MSB (USB is LE)
			// 15 bit value of timestamp in 1 us tick
			uint16_t timestampUSB = le16toh(*((uint16_t * ) (&buffer[i + 2])));

			// Expand to 32 bits. (Tick is 1µs already.)
			uint32_t timestamp = timestampUSB + state->wrapAdd;

			// Check monotonicity of timestamps.
			if (timestamp < state->lastTimestamp) {
				caerLog(LOG_ALERT,
					"DVS128: non-monotonic time-stamp detected: lastTimestamp=%" PRIu32 ", timestamp=%" PRIu32 ".",
					state->lastTimestamp, timestamp);
			}

			state->lastTimestamp = timestamp;

			if ((addressUSB & DVS128_SYNC_EVENT_MASK) != 0) {
				// Special Trigger Event (MSB is set)
				caerSpecialEvent currentEvent = caerSpecialEventPacketGetEvent(state->currentSpecialPacket,
					state->currentSpecialPacketPosition++);
				caerSpecialEventSetTimestamp(currentEvent, timestamp);
				caerSpecialEventSetType(currentEvent, EXTERNAL_TRIGGER);
				caerSpecialEventValidate(currentEvent, state->currentSpecialPacket);
			}
			else {
				// Invert x values (flip along the x axis).
				uint16_t x = (uint16_t) (127 - ((uint16_t) ((addressUSB >> DVS128_X_ADDR_SHIFT) & DVS128_X_ADDR_MASK)));
				uint16_t y = (uint16_t) ((addressUSB >> DVS128_Y_ADDR_SHIFT) & DVS128_Y_ADDR_MASK);
				bool polarity = (((addressUSB >> DVS128_POLARITY_SHIFT) & DVS128_POLARITY_MASK) == 0) ? (1) : (0);

				caerPolarityEvent currentEvent = caerPolarityEventPacketGetEvent(state->currentPolarityPacket,
					state->currentPolarityPacketPosition++);
				caerPolarityEventSetTimestamp(currentEvent, timestamp);
				caerPolarityEventSetPolarity(currentEvent, polarity);
				caerPolarityEventSetY(currentEvent, y);
				caerPolarityEventSetX(currentEvent, x);
				caerPolarityEventValidate(currentEvent, state->currentPolarityPacket);
			}
		}

		// Commit packet to the ring-buffer, so they can be processed by the
		// main-loop, when their stated conditions are met.
		if (forcePacketCommit
			|| (state->currentPolarityPacketPosition
				>= caerEventPacketHeaderGetEventCapacity(&state->currentPolarityPacket->packetHeader))
			|| ((state->currentPolarityPacketPosition > 1)
				&& (caerPolarityEventGetTimestamp(
					caerPolarityEventPacketGetEvent(state->currentPolarityPacket,
						state->currentPolarityPacketPosition - 1))
					- caerPolarityEventGetTimestamp(caerPolarityEventPacketGetEvent(state->currentPolarityPacket, 0))
					>= state->maxPolarityPacketInterval))) {
			if (!ringBufferPut(state->dataExchangeBuffer, state->currentPolarityPacket)) {
				// Failed to forward packet, drop it.
				free(state->currentPolarityPacket);
				caerLog(LOG_DEBUG, "Dropped Polarity Event Packet because ring-buffer full!");
			}
			else {
				caerMainloopDataAvailableIncrease(state->mainloopNotify);
			}

			// Allocate new packet for next iteration.
			state->currentPolarityPacket = caerPolarityEventPacketAllocate(state->maxPolarityPacketSize,
				state->sourceID);
			state->currentPolarityPacketPosition = 0;
		}

		if (forcePacketCommit
			|| (state->currentSpecialPacketPosition
				>= caerEventPacketHeaderGetEventCapacity(&state->currentSpecialPacket->packetHeader))
			|| ((state->currentSpecialPacketPosition > 1)
				&& (caerSpecialEventGetTimestamp(
					caerSpecialEventPacketGetEvent(state->currentSpecialPacket,
						state->currentSpecialPacketPosition - 1))
					- caerSpecialEventGetTimestamp(caerSpecialEventPacketGetEvent(state->currentSpecialPacket, 0))
					>= state->maxSpecialPacketInterval))) {
			retry_special: if (!ringBufferPut(state->dataExchangeBuffer, state->currentSpecialPacket)) {
				// Failed to forward packet, drop it, unless it contains a timestamp
				// related change, those are critical, so we just spin until we can
				// deliver that one. (Easily detected by forcePacketCommit!)
				if (forcePacketCommit) {
					goto retry_special;
				}
				else {
					// Failed to forward packet, drop it.
					free(state->currentSpecialPacket);
					caerLog(LOG_DEBUG, "Dropped Special Event Packet because ring-buffer full!");
				}
			}
			else {
				caerMainloopDataAvailableIncrease(state->mainloopNotify);
			}

			// Allocate new packet for next iteration.
			state->currentSpecialPacket = caerSpecialEventPacketAllocate(state->maxSpecialPacketSize, state->sourceID);
			state->currentSpecialPacketPosition = 0;
		}
	}
}

static void dvs128SendBiases(sshsNode biasNode, libusb_device_handle *devHandle) {
	// 12 biases, a 24 bits = 3 bytes each.
	uint8_t biases[12 * 3];

	uint32_t cas = sshsNodeGetInt(biasNode, "cas");
	biases[0] = U8T(cas >> 16);
	biases[1] = U8T(cas >> 8);
	biases[2] = U8T(cas >> 0);

	uint32_t injGnd = sshsNodeGetInt(biasNode, "injGnd");
	biases[3] = U8T(injGnd >> 16);
	biases[4] = U8T(injGnd >> 8);
	biases[5] = U8T(injGnd >> 0);

	uint32_t reqPd = sshsNodeGetInt(biasNode, "reqPd");
	biases[6] = U8T(reqPd >> 16);
	biases[7] = U8T(reqPd >> 8);
	biases[8] = U8T(reqPd >> 0);

	uint32_t puX = sshsNodeGetInt(biasNode, "puX");
	biases[9] = U8T(puX >> 16);
	biases[10] = U8T(puX >> 8);
	biases[11] = U8T(puX >> 0);

	uint32_t diffOff = sshsNodeGetInt(biasNode, "diffOff");
	biases[12] = U8T(diffOff >> 16);
	biases[13] = U8T(diffOff >> 8);
	biases[14] = U8T(diffOff >> 0);

	uint32_t req = sshsNodeGetInt(biasNode, "req");
	biases[15] = U8T(req >> 16);
	biases[16] = U8T(req >> 8);
	biases[17] = U8T(req >> 0);

	uint32_t refr = sshsNodeGetInt(biasNode, "refr");
	biases[18] = U8T(refr >> 16);
	biases[19] = U8T(refr >> 8);
	biases[20] = U8T(refr >> 0);

	uint32_t puY = sshsNodeGetInt(biasNode, "puY");
	biases[21] = U8T(puY >> 16);
	biases[22] = U8T(puY >> 8);
	biases[23] = U8T(puY >> 0);

	uint32_t diffOn = sshsNodeGetInt(biasNode, "diffOn");
	biases[24] = U8T(diffOn >> 16);
	biases[25] = U8T(diffOn >> 8);
	biases[26] = U8T(diffOn >> 0);

	uint32_t diff = sshsNodeGetInt(biasNode, "diff");
	biases[27] = U8T(diff >> 16);
	biases[28] = U8T(diff >> 8);
	biases[29] = U8T(diff >> 0);

	uint32_t foll = sshsNodeGetInt(biasNode, "foll");
	biases[30] = U8T(foll >> 16);
	biases[31] = U8T(foll >> 8);
	biases[32] = U8T(foll >> 0);

	uint32_t pr = sshsNodeGetInt(biasNode, "pr");
	biases[33] = U8T(pr >> 16);
	biases[34] = U8T(pr >> 8);
	biases[35] = U8T(pr >> 0);

	libusb_control_transfer(devHandle, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
	VENDOR_REQUEST_SEND_BIASES, 0, 0, biases, sizeof(biases), 0);
}

static libusb_device_handle *dvs128Open(libusb_context *devContext) {
	libusb_device_handle *devHandle = NULL;
	libusb_device **devicesList;

	ssize_t result = libusb_get_device_list(devContext, &devicesList);

	if (result >= 0) {
		// Cycle thorough all discovered devices and find a match.
		for (size_t i = 0; i < (size_t) result; i++) {
			struct libusb_device_descriptor devDesc;

			if (libusb_get_device_descriptor(devicesList[i], &devDesc) != LIBUSB_SUCCESS) {
				continue;
			}

			// Check if this is the device we want (VID/PID).
			if (devDesc.idVendor == DVS128_VID && devDesc.idProduct == DVS128_PID
				&& (uint8_t) ((devDesc.bcdDevice & 0xFF00) >> 8) == DVS128_DID_TYPE) {
				if (libusb_open(devicesList[i], &devHandle) != LIBUSB_SUCCESS) {
					devHandle = NULL;

					continue;
				}

				// Check that the active configuration is set to number 1. If not, do so.
				int activeConfiguration;
				if (libusb_get_configuration(devHandle, &activeConfiguration) != LIBUSB_SUCCESS) {
					libusb_close(devHandle);
					devHandle = NULL;

					continue;
				}

				if (activeConfiguration != 1) {
					if (libusb_set_configuration(devHandle, 1) != LIBUSB_SUCCESS) {
						libusb_close(devHandle);
						devHandle = NULL;

						continue;
					}
				}

				// Claim interface 0 (default).
				if (libusb_claim_interface(devHandle, 0) != LIBUSB_SUCCESS) {
					libusb_close(devHandle);
					devHandle = NULL;

					continue;
				}

				// Found and configured it!
				break;
			}
		}

		libusb_free_device_list(devicesList, true);
	}

	return (devHandle);
}

static void dvs128Close(libusb_device_handle *devHandle) {
	// Release interface 0 (default).
	libusb_release_interface(devHandle, 0);

	libusb_close(devHandle);
}

static void caerInputDVS128ConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(changeValue);

	caerModuleData data = userData;

	// Distinguish changes to biases, or USB transfers, or others, by
	// using configUpdate like a bit-field.
	if (event == ATTRIBUTE_MODIFIED) {
		// Changes to the bias node.
		if (str_equals(sshsNodeGetName(node), "bias") && changeType == INT) {
			atomic_ops_uint_or(&data->configUpdate, (0x01 << 0), ATOMIC_OPS_FENCE_NONE);
		}

		// Changes to the USB transfer settings (requires reallocation).
		if (changeType == INT && (str_equals(changeKey, "bufferNumber") || str_equals(changeKey, "bufferSize"))) {
			atomic_ops_uint_or(&data->configUpdate, (0x01 << 1), ATOMIC_OPS_FENCE_NONE);
		}

		// Changes to packet size and interval.
		if (changeType == INT
			&& (str_equals(changeKey, "polarityPacketMaxSize") || str_equals(changeKey, "polarityPacketMaxInterval")
				|| str_equals(changeKey, "specialPacketMaxSize") || str_equals(changeKey, "specialPacketMaxInterval"))) {
			atomic_ops_uint_or(&data->configUpdate, (0x01 << 2), ATOMIC_OPS_FENCE_NONE);
		}
	}
}
