#include "davis_common.h"
#include "davis_fx2.h"
#include "base/module.h"
#include <pthread.h>
#include <unistd.h>

struct davisFX2_state {
	// State for data management, common to all DAVISes.
	struct davisCommon_state cstate;
};

typedef struct davisFX2_state *davisFX2State;

static bool caerInputDAVISFX2Init(caerModuleData moduleData);
// RUN: common to all DAVIS systems.
// CONFIG: Nothing to do here in the main thread!
// Biases are configured asynchronously, and buffer sizes in the data
// acquisition thread itself. Resetting the main config_refresh flag
// will also happen there.
// EXIT: common to all DAVIS systems.

static struct caer_module_functions caerInputDAVISFX2Functions = { .moduleInit = &caerInputDAVISFX2Init, .moduleRun =
	&caerInputDAVISCommonRun, .moduleConfig = NULL, .moduleExit = &caerInputDAVISCommonExit };

void caerInputDAVISFX2(uint16_t moduleID, caerPolarityEventPacket *polarity, caerFrameEventPacket *frame,
	caerIMU6EventPacket *imu6, caerSpecialEventPacket *special) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "DAVISFX2");

	// IMPORTANT: THE CONTENT OF OUTPUT ARGUMENTS MUST BE SET TO NULL!
	if (polarity != NULL) {
		*polarity = NULL;
	}
	if (frame != NULL) {
		*frame = NULL;
	}
	if (imu6 != NULL) {
		*imu6 = NULL;
	}
	if (special != NULL) {
		*special = NULL;
	}

	caerModuleSM(&caerInputDAVISFX2Functions, moduleData, sizeof(struct davisFX2_state), 4, polarity, frame, imu6,
		special);
}

static void *dataAcquisitionThread(void *inPtr);
static void sendBias(libusb_device_handle *devHandle, uint16_t biasAddress, uint16_t biasValue);
static void sendBiases(sshsNode moduleNode, libusb_device_handle *devHandle);
static void sendChipSR(sshsNode moduleNode, libusb_device_handle *devHandle);
static void BiasesListener(sshsNode node, void *userData, enum sshs_node_attribute_events event, const char *changeKey,
	enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void ChipSRListener(sshsNode node, void *userData, enum sshs_node_attribute_events event, const char *changeKey,
	enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);

static bool caerInputDAVISFX2Init(caerModuleData moduleData) {
	caerLog(LOG_DEBUG, moduleData->moduleSubSystemString, "Initializing module ...");

	davisFX2State state = moduleData->moduleState;
	davisCommonState cstate = &state->cstate;

	// Data source is the same as the module ID (but accessible in cstate-space).
	// Same thing for subSystemString.
	cstate->sourceID = moduleData->moduleID;
	cstate->sourceSubSystemString = moduleData->moduleSubSystemString;

	// First, we need to connect to the device and ask it what chip it's got,
	// and retain that information for later stages.
	if (!deviceOpenInfo(moduleData, cstate, DAVIS_FX2_VID, DAVIS_FX2_PID, DAVIS_FX2_DID_TYPE)) {
		return (false);
	}

	createCommonConfiguration(moduleData, cstate);

	// Install default listeners to signal configuration updates asynchronously.
	sshsNodeAddAttrListener(sshsGetRelativeNode(moduleData->moduleNode, "chip/"), moduleData, &ChipSRListener);
	// The chip SR needs to be updated also when GlobalShutter in APS changes.
	sshsNodeAddAttrListener(sshsGetRelativeNode(moduleData->moduleNode, "aps/"), moduleData, &ChipSRListener);

	// Walk all bias nodes and install the default handler for changes.
	size_t numBiasNodes;
	sshsNode *biasNodes = sshsNodeGetChildren(sshsGetRelativeNode(moduleData->moduleNode, "bias/"), &numBiasNodes);

	for (size_t i = 0; i < numBiasNodes; i++) {
		sshsNodeAddAttrListener(biasNodes[i], cstate->deviceHandle, &BiasesListener);
	}

	free(biasNodes);

	if (!initializeCommonConfiguration(moduleData, cstate, &dataAcquisitionThread)) {
		return (false);
	}

	caerLog(LOG_DEBUG, moduleData->moduleSubSystemString, "Initialized DAVISFX2 module successfully.");
	return (true);
}

static void *dataAcquisitionThread(void *inPtr) {
	// inPtr is a pointer to module data.
	caerModuleData data = inPtr;
	davisFX2State state = data->moduleState;
	davisCommonState cstate = &state->cstate;

	caerLog(LOG_DEBUG, data->moduleSubSystemString, "Initializing data acquisition thread ...");

	// Create buffers as specified in config file.
	sshsNode usbNode = sshsGetRelativeNode(data->moduleNode, "usb/");
	allocateDataTransfers(cstate, sshsNodeGetInt(usbNode, "BufferNumber"), sshsNodeGetInt(usbNode, "BufferSize"));

	// Send default start-up biases and config values to device before enabling it.
	sendBiases(data->moduleNode, cstate->deviceHandle);
	sendChipSR(data->moduleNode, cstate->deviceHandle);
	sendEnableDataConfig(data->moduleNode, cstate->deviceHandle);

	// Handle USB events (1 second timeout).
	struct timeval te = { .tv_sec = 0, .tv_usec = 1000000 };

	caerLog(LOG_DEBUG, data->moduleSubSystemString, "Data acquisition thread ready to process events.");

	while (atomic_ops_uint_load(&data->running, ATOMIC_OPS_FENCE_NONE) != 0
		&& atomic_ops_uint_load(&cstate->dataTransfersLength, ATOMIC_OPS_FENCE_NONE) > 0) {
		// Check config refresh, in this case to adjust buffer sizes.
		if (atomic_ops_uint_load(&data->configUpdate, ATOMIC_OPS_FENCE_NONE) != 0) {
			dataAcquisitionThreadConfig(data);
		}

		libusb_handle_events_timeout(cstate->deviceContext, &te);
	}

	caerLog(LOG_DEBUG, data->moduleSubSystemString, "Shutting down data acquisition thread ...");

	// Disable all data transfer on USB end-point.
	sendDisableDataConfig(cstate->deviceHandle);

	// Cancel all transfers and handle them.
	deallocateDataTransfers(cstate);

	// Ensure parent also shuts down (on disconnected device for example).
	sshsNodePutBool(data->moduleNode, "shutdown", true);

	caerLog(LOG_DEBUG, data->moduleSubSystemString, "Data acquisition thread shut down.");

	return (NULL);
}

static void sendBias(libusb_device_handle *devHandle, uint16_t biasAddress, uint16_t biasValue) {
	// All biases are two byte quantities.
	uint8_t bias[2];

	// Put the value in.
	bias[0] = U8T(biasValue >> 8);
	bias[1] = U8T(biasValue >> 0);

	libusb_control_transfer(devHandle, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
	VR_CHIP_BIAS, biasAddress, 0, bias, sizeof(bias), 0);
}

static void BiasesListener(sshsNode node, void *userData, enum sshs_node_attribute_events event, const char *changeKey,
	enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(changeKey);
	UNUSED_ARGUMENT(changeType);
	UNUSED_ARGUMENT(changeValue);

	libusb_device_handle *devHandle = userData;

	if (event == ATTRIBUTE_MODIFIED) {
		if (str_equals(sshsNodeGetName(node), "DiffBn")) {
			sendBias(devHandle, 0, generateAddressedCoarseFineBias(node, "DiffBn"));
		}
		else if (str_equals(sshsNodeGetName(node), "OnBn")) {
			sendBias(devHandle, 1, generateAddressedCoarseFineBias(node, "OnBn"));
		}
		else if (str_equals(sshsNodeGetName(node), "OffBn")) {
			sendBias(devHandle, 2, generateAddressedCoarseFineBias(node, "OffBn"));
		}
		else if (str_equals(sshsNodeGetName(node), "ApsCasEpc")) {
			sendBias(devHandle, 3, generateAddressedCoarseFineBias(node, "ApsCasEpc"));
		}
		else if (str_equals(sshsNodeGetName(node), "DiffCasBnc")) {
			sendBias(devHandle, 4, generateAddressedCoarseFineBias(node, "DiffCasBnc"));
		}
		else if (str_equals(sshsNodeGetName(node), "ApsROSFBn")) {
			sendBias(devHandle, 5, generateAddressedCoarseFineBias(node, "ApsROSFBn"));
		}
		else if (str_equals(sshsNodeGetName(node), "LocalBufBn")) {
			sendBias(devHandle, 6, generateAddressedCoarseFineBias(node, "LocalBufBn"));
		}
		else if (str_equals(sshsNodeGetName(node), "PixInvBn")) {
			sendBias(devHandle, 7, generateAddressedCoarseFineBias(node, "PixInvBn"));
		}
		else if (str_equals(sshsNodeGetName(node), "PrBp")) {
			sendBias(devHandle, 8, generateAddressedCoarseFineBias(node, "PrBp"));
		}
		else if (str_equals(sshsNodeGetName(node), "PrSFBp")) {
			sendBias(devHandle, 9, generateAddressedCoarseFineBias(node, "PrSFBp"));
		}
		else if (str_equals(sshsNodeGetName(node), "RefrBp")) {
			sendBias(devHandle, 10, generateAddressedCoarseFineBias(node, "RefrBp"));
		}
		else if (str_equals(sshsNodeGetName(node), "AEPdBn")) {
			sendBias(devHandle, 11, generateAddressedCoarseFineBias(node, "AEPdBn"));
		}
		else if (str_equals(sshsNodeGetName(node), "LcolTimeoutBn")) {
			sendBias(devHandle, 12, generateAddressedCoarseFineBias(node, "LcolTimeoutBn"));
		}
		else if (str_equals(sshsNodeGetName(node), "AEPuXBp")) {
			sendBias(devHandle, 13, generateAddressedCoarseFineBias(node, "AEPuXBp"));
		}
		else if (str_equals(sshsNodeGetName(node), "AEPuYBp")) {
			sendBias(devHandle, 14, generateAddressedCoarseFineBias(node, "AEPuYBp"));
		}
		else if (str_equals(sshsNodeGetName(node), "IFThrBn")) {
			sendBias(devHandle, 15, generateAddressedCoarseFineBias(node, "IFThrBn"));
		}
		else if (str_equals(sshsNodeGetName(node), "IFRefrBn")) {
			sendBias(devHandle, 16, generateAddressedCoarseFineBias(node, "IFRefrBn"));
		}
		else if (str_equals(sshsNodeGetName(node), "PadFollBn")) {
			sendBias(devHandle, 17, generateAddressedCoarseFineBias(node, "PadFollBn"));
		}
		else if (str_equals(sshsNodeGetName(node), "ApsOverflowLevel")) {
			sendBias(devHandle, 18, generateAddressedCoarseFineBias(node, "ApsOverflowLevel"));
		}
		else if (str_equals(sshsNodeGetName(node), "BiasBuffer")) {
			sendBias(devHandle, 19, generateAddressedCoarseFineBias(node, "BiasBuffer"));
		}
		else if (str_equals(sshsNodeGetName(node), "SSP")) {
			sendBias(devHandle, 20, generateShiftedSourceBias(node, "SSP"));
		}
		else if (str_equals(sshsNodeGetName(node), "SSN")) {
			sendBias(devHandle, 21, generateShiftedSourceBias(node, "SSN"));
		}
	}
}

static void sendBiases(sshsNode moduleNode, libusb_device_handle *devHandle) {
	// Only DAVIS240 can be used with the FX2 boards.
	sshsNode biasNode = sshsGetRelativeNode(moduleNode, "bias/");

	// Biases are addressable now!
	sendBias(devHandle, 0, generateAddressedCoarseFineBias(biasNode, "DiffBn"));
	sendBias(devHandle, 1, generateAddressedCoarseFineBias(biasNode, "OnBn"));
	sendBias(devHandle, 2, generateAddressedCoarseFineBias(biasNode, "OffBn"));
	sendBias(devHandle, 3, generateAddressedCoarseFineBias(biasNode, "ApsCasEpc"));
	sendBias(devHandle, 4, generateAddressedCoarseFineBias(biasNode, "DiffCasBnc"));
	sendBias(devHandle, 5, generateAddressedCoarseFineBias(biasNode, "ApsROSFBn"));
	sendBias(devHandle, 6, generateAddressedCoarseFineBias(biasNode, "LocalBufBn"));
	sendBias(devHandle, 7, generateAddressedCoarseFineBias(biasNode, "PixInvBn"));
	sendBias(devHandle, 8, generateAddressedCoarseFineBias(biasNode, "PrBp"));
	sendBias(devHandle, 9, generateAddressedCoarseFineBias(biasNode, "PrSFBp"));
	sendBias(devHandle, 10, generateAddressedCoarseFineBias(biasNode, "RefrBp"));
	sendBias(devHandle, 11, generateAddressedCoarseFineBias(biasNode, "AEPdBn"));
	sendBias(devHandle, 12, generateAddressedCoarseFineBias(biasNode, "LcolTimeoutBn"));
	sendBias(devHandle, 13, generateAddressedCoarseFineBias(biasNode, "AEPuXBp"));
	sendBias(devHandle, 14, generateAddressedCoarseFineBias(biasNode, "AEPuYBp"));
	sendBias(devHandle, 15, generateAddressedCoarseFineBias(biasNode, "IFThrBn"));
	sendBias(devHandle, 16, generateAddressedCoarseFineBias(biasNode, "IFRefrBn"));
	sendBias(devHandle, 17, generateAddressedCoarseFineBias(biasNode, "PadFollBn"));
	sendBias(devHandle, 18, generateAddressedCoarseFineBias(biasNode, "ApsOverflowLevel"));

	sendBias(devHandle, 19, generateAddressedCoarseFineBias(biasNode, "BiasBuffer"));

	sendBias(devHandle, 20, generateShiftedSourceBias(biasNode, "SSP"));
	sendBias(devHandle, 21, generateShiftedSourceBias(biasNode, "SSN"));
}

static void ChipSRListener(sshsNode node, void *userData, enum sshs_node_attribute_events event, const char *changeKey,
	enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(changeValue);

	caerModuleData moduleData = userData;
	sshsNode moduleNode = moduleData->moduleNode;
	libusb_device_handle *devHandle = ((davisFX2State) moduleData->moduleState)->cstate.deviceHandle;

	if (event == ATTRIBUTE_MODIFIED) {
		if (str_equals(sshsNodeGetName(node), "aps")) {
			if (changeType == BOOL && str_equals(changeKey, "GlobalShutter")) {
				sendChipSR(moduleNode, devHandle);
			}
		}
		else if (str_equals(sshsNodeGetName(node), "chip")) {
			sendChipSR(moduleNode, devHandle);
		}
	}
}

static void sendChipSR(sshsNode moduleNode, libusb_device_handle *devHandle) {
	// Only DAVIS240 can be used with the FX2 boards.
	sshsNode chipNode = sshsGetRelativeNode(moduleNode, "chip/");
	sshsNode apsNode = sshsGetRelativeNode(moduleNode, "aps/");

	// A total of 56 bits (7 bytes) of configuration.
	uint8_t chipSR[7] = { 0 };

	// Debug muxes control.
	chipSR[0] |= U8T((sshsNodeGetByte(chipNode, "DigitalMux3") & 0x0F) << 4);
	chipSR[0] |= U8T((sshsNodeGetByte(chipNode, "DigitalMux2") & 0x0F) << 0);
	chipSR[1] |= U8T((sshsNodeGetByte(chipNode, "DigitalMux1") & 0x0F) << 4);
	chipSR[1] |= U8T((sshsNodeGetByte(chipNode, "DigitalMux0") & 0x0F) << 0);

	chipSR[5] |= U8T((sshsNodeGetByte(chipNode, "AnalogMux2") & 0x0F) << 4);
	chipSR[5] |= U8T((sshsNodeGetByte(chipNode, "AnalogMux1") & 0x0F) << 0);
	chipSR[6] |= U8T((sshsNodeGetByte(chipNode, "AnalogMux0") & 0x0F) << 4);

	chipSR[6] |= U8T((sshsNodeGetByte(chipNode, "BiasMux") & 0x0F) << 0);

	// Bytes 2-4 contain the actual 24 configuration bits. 17 are unused.
	bool globalShutter = sshsNodeGetBool(apsNode, "GlobalShutter");
	if (globalShutter) {
		// Flip bit on if enabled.
		chipSR[4] |= (1 << 6);
	}

	bool useAout = sshsNodeGetBool(chipNode, "UseAout");
	if (useAout) {
		// Flip bit on if enabled.
		chipSR[4] |= (1 << 5);
	}

	bool nArow = sshsNodeGetBool(chipNode, "nArow");
	if (nArow) {
		// Flip bit on if enabled.
		chipSR[4] |= (1 << 4);
	}

	bool hotPixelSuppression = sshsNodeGetBool(chipNode, "HotPixelSuppression");
	if (hotPixelSuppression) {
		// Flip bit on if enabled.
		chipSR[4] |= (1 << 3);
	}

	bool resetTestpixel = sshsNodeGetBool(chipNode, "ResetTestPixel");
	if (resetTestpixel) {
		// Flip bit on if enabled.
		chipSR[4] |= (1 << 2);
	}

	bool typeNCalib = sshsNodeGetBool(chipNode, "TypeNCalibNeuron");
	if (typeNCalib) {
		// Flip bit on if enabled.
		chipSR[4] |= (1 << 1);
	}

	bool resetCalib = sshsNodeGetBool(chipNode, "ResetCalibNeuron");
	if (resetCalib) {
		// Flip bit on if enabled.
		chipSR[4] |= (1 << 0);
	}

	libusb_control_transfer(devHandle, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
	VR_CHIP_DIAG, 0, 0, chipSR, sizeof(chipSR), 0);
}
