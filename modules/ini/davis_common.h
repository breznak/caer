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

#define DAVIS_ADC_DEPTH 10
#define DAVIS_COLOR_CHANNELS 1

#define DATA_ENDPOINT 0x82

#define VR_FPGA_CONFIG 0xBF

#define FPGA_MUX 0
#define FPGA_DVS 1
#define FPGA_APS 2
#define FPGA_IMU 3
#define FPGA_EXTINPUT 4
#define FPGA_USB 9

#define EXT_ADC_FREQ 30

#define CHIP_DAVIS240A 0
#define CHIP_DAVIS240B 1
#define CHIP_DAVIS240C 2
#define CHIP_DAVIS128  3
#define CHIP_DAVIS346A 4
#define CHIP_DAVIS346B 5
#define CHIP_DAVIS640  6
#define CHIP_DAVISRGB  7

struct davisCommon_state {
	// Data Acquisition Thread -> Mainloop Exchange
	uint16_t sourceID;
	char *sourceSubSystemString;
	RingBuffer dataExchangeBuffer;
	caerMainloopData mainloopNotify;
	// USB Device State
	libusb_context *deviceContext;
	libusb_device_handle *deviceHandle;
	// Data Acquisition Thread
	pthread_t dataAcquisitionThread;
	struct libusb_transfer **dataTransfers;
	atomic_ops_uint dataTransfersLength;
	uint32_t wrapAdd;
	uint32_t lastTimestamp;
	uint32_t currentTimestamp;
	uint16_t chipID;
	// DVS specific fields
	uint32_t dvsTimestamp;
	uint16_t dvsSizeX;
	uint16_t dvsSizeY;
	uint16_t dvsLastY;
	bool dvsGotY;
	// APS specific fields
	uint16_t apsSizeX;
	uint16_t apsSizeY;
	uint16_t apsWindow0StartX;
	uint16_t apsWindow0StartY;
	uint16_t apsWindow0SizeX;
	uint16_t apsWindow0SizeY;
	bool apsGlobalShutter;
	bool apsResetRead;
	uint16_t apsCurrentReadoutType;
	uint16_t apsCountX[APS_READOUT_TYPES_NUM];
	uint16_t apsCountY[APS_READOUT_TYPES_NUM];
	uint16_t *apsCurrentResetFrame;
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

uint16_t generateVDACBias(sshsNode biasNode, const char *biasName);
uint16_t generateAddressedCoarseFineBias(sshsNode biasNode, const char *biasName);
uint16_t generateShiftedSourceBias(sshsNode biasNode, const char *biasName);
void spiConfigSend(libusb_device_handle *devHandle, uint8_t moduleAddr, uint8_t paramAddr, uint32_t param);
uint32_t spiConfigReceive(libusb_device_handle *devHandle, uint8_t moduleAddr, uint8_t paramAddr);
bool deviceOpenInfo(caerModuleData moduleData, davisCommonState cstate, uint16_t VID, uint16_t PID, uint8_t DID_TYPE);
void createCommonConfiguration(caerModuleData moduleData, davisCommonState cstate);
bool initializeCommonConfiguration(caerModuleData moduleData, davisCommonState cstate,
	void *dataAcquisitionThread(void *inPtr));
void caerInputDAVISCommonRun(caerModuleData moduleData, size_t argsNumber, va_list args);
void caerInputDAVISCommonExit(caerModuleData moduleData);
void allocateDataTransfers(davisCommonState state, uint32_t bufferNum, uint32_t bufferSize);
void deallocateDataTransfers(davisCommonState state);
void sendEnableDataConfig(sshsNode moduleNode, libusb_device_handle *devHandle);
void sendDisableDataConfig(libusb_device_handle *devHandle);
void dataAcquisitionThreadConfig(caerModuleData moduleData);

#endif /* DAVIS_COMMON_H_ */
