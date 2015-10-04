/*
 * dvs128.c
 *
 *  Created on: Nov 26, 2013
 *      Author: chtekk
 */

#include "dvs128.h"
#include "base/mainloop.h"
#include "base/module.h"

#include <libcaer/devices/dvs128.h>

static bool caerInputDVS128Init(caerModuleData moduleData);
static void caerInputDVS128Run(caerModuleData moduleData, size_t argsNumber, va_list args);
// CONFIG: Nothing to do here in the main thread!
// Biases are configured asynchronously, and buffer sizes in the data
// acquisition thread itself. Resetting the main config_refresh flag
// will also happen there.
static void caerInputDVS128Exit(caerModuleData moduleData);

static struct caer_module_functions caerInputDVS128Functions = { .moduleInit = &caerInputDVS128Init, .moduleRun =
	&caerInputDVS128Run, .moduleConfig = NULL, .moduleExit = &caerInputDVS128Exit };

caerEventPacketContainer caerInputDVS128(uint16_t moduleID) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "DVS128");

	caerEventPacketContainer result = NULL;

	caerModuleSM(&caerInputDVS128Functions, moduleData, 0, 1, &result);

	return (result);
}

static void caerInputDVS128ConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void mainloopDataNotifyIncrease(void *p);
static void mainloopDataNotifyDecrease(void *p);
static void moduleShutdownNotify(void *p);

static bool caerInputDVS128Init(caerModuleData moduleData) {
	caerLog(CAER_LOG_DEBUG, moduleData->moduleSubSystemString, "Initializing module ...");

	// First, always create all needed setting nodes, set their default values
	// and add their listeners.

	// USB port/bus/SN settings/restrictions.
	// These can be used to force connection to one specific device.
	sshsNode selectorNode = sshsGetRelativeNode(moduleData->moduleNode, "usbDevice/");

	sshsNodePutByteIfAbsent(selectorNode, "BusNumber", 0);
	sshsNodePutByteIfAbsent(selectorNode, "DevAddress", 0);
	sshsNodePutStringIfAbsent(selectorNode, "SerialNumber", "");

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

	// DVS settings.
	sshsNode dvsNode = sshsGetRelativeNode(moduleData->moduleNode, "dvs/");
	sshsNodePutBoolIfAbsent(dvsNode, "Run", true);
	sshsNodePutBoolIfAbsent(dvsNode, "TimestampReset", false);
	sshsNodePutBoolIfAbsent(dvsNode, "ArrayReset", false);

	// USB buffer settings.
	sshsNode usbNode = sshsGetRelativeNode(moduleData->moduleNode, "usb/");

	sshsNodePutIntIfAbsent(moduleData->moduleNode, "BufferNumber", 8);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "BufferSize", 4096);

	sshsNode sysNode = sshsGetRelativeNode(moduleData->moduleNode, "system/");

	// Packet settings (size (in events) and time interval (in µs)).
	sshsNodePutIntIfAbsent(sysNode, "PolarityPacketMaxSize", 4096);
	sshsNodePutIntIfAbsent(sysNode, "PolarityPacketMaxInterval", 5000);
	sshsNodePutIntIfAbsent(sysNode, "SpecialPacketMaxSize", 128);
	sshsNodePutIntIfAbsent(sysNode, "SpecialPacketMaxInterval", 1000);

	// Ring-buffer setting (only changes value on module init/shutdown cycles).
	sshsNodePutIntIfAbsent(sysNode, "DataExchangeBufferSize", 64);

	// Add auto-restart setting.
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "auto-restart", true);

	// Install default listener to signal configuration updates asynchronously.
	sshsNodeAddAttrListener(biasNode, moduleData, &caerInputDVS128ConfigListener);
	sshsNodeAddAttrListener(moduleData->moduleNode, moduleData, &caerInputDVS128ConfigListener);

	// Start data acquisition, and correctly notify mainloop of new data and module of exceptional
	// shutdown cases (device pulled, ...).
	char *serialNumber = sshsNodeGetString(selectorNode, "SerialNumber");
	moduleData->moduleState = caerDeviceOpen(moduleData->moduleID, CAER_DEVICE_DVS128,
		sshsNodeGetByte(selectorNode, "BusNumber"), sshsNodeGetByte(selectorNode, "DevAddress"), serialNumber);
	free(serialNumber);

	// Put global source information into SSHS.
	struct caer_dvs128_info devInfo = caerDVS128InfoGet(moduleData->moduleState);

	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");

	sshsNodePutShort(sourceInfoNode, "dvsSizeX", devInfo.dvsSizeX);
	sshsNodePutShort(sourceInfoNode, "dvsSizeY", devInfo.dvsSizeY);

	// Put source information for "virtual" APS frame that can be used to display and debug filter information.
	sshsNodePutShort(sourceInfoNode, "apsSizeX", devInfo.dvsSizeX);
	sshsNodePutShort(sourceInfoNode, "apsSizeY", devInfo.dvsSizeY);

	caerModuleSetSubSystemString(moduleData, devInfo.deviceString);

	sshsNodePutShort(sourceInfoNode, "logicVersion", devInfo.logicVersion);
	sshsNodePutBool(sourceInfoNode, "deviceIsMaster", devInfo.deviceIsMaster);

	caerDeviceDataStart(moduleData->moduleState, &mainloopDataNotifyIncrease, &mainloopDataNotifyDecrease, NULL,
		&moduleShutdownNotify, moduleData->moduleNode);

	return (true);
}

static void caerInputDVS128Exit(caerModuleData moduleData) {
	caerDeviceDataStop(moduleData->moduleState);

	caerDeviceClose((caerDeviceHandle *) &moduleData->moduleState);

	if (sshsNodeGetBool(moduleData->moduleNode, "auto-restart")) {
		// Prime input module again so that it will try to restart if new devices detected.
		sshsNodePutBool(moduleData->moduleNode, "shutdown", false);
	}
}

static void caerInputDVS128Run(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerEventPacketContainer *container = va_arg(args, caerEventPacketContainer *);

	*container = caerDeviceDataGet(moduleData->moduleState);

	if (*container != NULL) {
		caerMainloopFreeAfterLoop((void (*)(void *)) &caerEventPacketContainerFree, *container);
	}
}

static void caerInputDVS128ConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(changeValue);

	caerModuleData data = userData;

}

static void mainloopDataNotifyIncrease(void *p) {
	UNUSED_ARGUMENT(p);

	caerMainloopDataAvailableIncrease();
}

static void mainloopDataNotifyDecrease(void *p) {
	UNUSED_ARGUMENT(p);

	caerMainloopDataAvailableDecrease();
}

static void moduleShutdownNotify(void *p) {
	sshsNode moduleNode = p;

	// Ensure parent also shuts down (on disconnected device for example).
	sshsNodePutBool(moduleNode, "shutdown", true);
}
