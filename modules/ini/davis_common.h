#ifndef DAVIS_COMMON_H_
#define DAVIS_COMMON_H_

#include "main.h"
#include "events/polarity.h"
#include "events/special.h"
#include "events/frame.h"
#include "events/imu6.h"
#include "base/mainloop.h"
#include "ext/ringbuffer/ringbuffer.h"
#include <libusb.h>

#define APS_READOUT_TYPES_NUM 2
#define APS_READOUT_RESET 0
#define APS_READOUT_SIGNAL 1

#define DAVIS_ARRAY_SIZE_X 240
#define DAVIS_ARRAY_SIZE_Y 180
#define DAVIS_ADC_DEPTH 10
#define DAVIS_COLOR_CHANNELS 1

#define DATA_ENDPOINT 0x82

#define VR_FPGA_CONFIG 0xBF

struct davisCommon_state {
	// Data Acquisition Thread -> Mainloop Exchange
	uint16_t sourceID;
	RingBuffer dataExchangeBuffer;
	caerMainloopData mainloopNotify;
	// USB Device State
	libusb_context *deviceContext;
	libusb_device_handle *deviceHandle;
	// Data Acquisition Thread State
	struct libusb_transfer **dataTransfers;
	atomic_ops_uint dataTransfersLength;
	uint32_t wrapAdd;
	uint32_t lastTimestamp;
	uint32_t currentTimestamp;
	uint32_t dvsTimestamp;
	uint32_t imuTimestamp;
	uint16_t lastY;
	bool gotY;
	bool translateRowOnlyEvents;
	bool apsGlobalShutter;
	uint16_t apsCurrentReadoutType;
	uint16_t apsCountX[APS_READOUT_TYPES_NUM];
	uint16_t apsCountY[APS_READOUT_TYPES_NUM];
	uint16_t apsCurrentResetFrame[DAVIS_ARRAY_SIZE_X * DAVIS_ARRAY_SIZE_Y * DAVIS_COLOR_CHANNELS];
	// Polarity Packet State
	caerPolarityEventPacket currentPolarityPacket;
	uint32_t currentPolarityPacketPosition;
	uint32_t maxPolarityPacketSize;
	uint32_t maxPolarityPacketInterval;
	// Frame Packet State
	caerFrameEventPacket currentFramePacket;
	uint32_t currentFramePacketPosition;
	uint32_t maxFramePacketSize;
	uint32_t maxFramePacketInterval;
	// IMU6 Packet State
	caerIMU6EventPacket currentIMU6Packet;
	uint32_t currentIMU6PacketPosition;
	uint32_t maxIMU6PacketSize;
	uint32_t maxIMU6PacketInterval;
	// Special Packet State
	caerSpecialEventPacket currentSpecialPacket;
	uint32_t currentSpecialPacketPosition;
	uint32_t maxSpecialPacketSize;
	uint32_t maxSpecialPacketInterval;
};

typedef struct davisCommon_state *davisCommonState;

void freeAllPackets(davisCommonState state);
void createAddressedCoarseFineBiasSetting(sshsNode biasNode, const char *biasName, const char *type,
	const char *sex, uint8_t coarseValue, uint8_t fineValue, bool enabled);
uint16_t generateAddressedCoarseFineBias(sshsNode biasNode, const char *biasName);
void createShiftedSourceBiasSetting(sshsNode biasNode, const char *biasName, uint8_t regValue,
	uint8_t refValue, const char *operatingMode, const char *voltageLevel);
uint16_t generateShiftedSourceBias(sshsNode biasNode, const char *biasName);
void sendSpiConfigCommand(libusb_device_handle *devHandle, uint8_t moduleAddr, uint8_t paramAddr, uint32_t param);
void caerInputDAVISCommonRun(caerModuleData moduleData, size_t argsNumber, va_list args);
void allocateDataTransfers(davisCommonState state, uint32_t bufferNum, uint32_t bufferSize);
void deallocateDataTransfers(davisCommonState state);
libusb_device_handle *deviceOpen(libusb_context *devContext, uint16_t devVID, uint16_t devPID,
		uint8_t devType, uint8_t busNumber, uint8_t devAddress);
void deviceClose(libusb_device_handle *devHandle);
void caerInputDAVISCommonConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);

#endif /* DAVIS_COMMON_H_ */
