#ifndef DAVIS_COMMON_H_
#define DAVIS_COMMON_H_

#include "main.h"
#include "events/polarity.h"
#include "events/special.h"
#include "events/frame.h"
#include "events/imu6.h"

#define APS_READOUT_TYPES_NUM 2
#define APS_READOUT_RESET 0
#define APS_READOUT_SIGNAL 1

#define DAVIS_ARRAY_SIZE_X 240
#define DAVIS_ARRAY_SIZE_Y 180
#define DAVIS_ADC_DEPTH 10
#define DAVIS_COLOR_CHANNELS 1

#define DATA_ENDPOINT 0x82

#define VR_FPGA_CONFIG 0xBF

typedef struct davisCommon_state *davisCommonState;

void freeAllPackets(davisFX3State state);
void *dataAcquisitionThread(void *inPtr);
void dataAcquisitionThreadConfig(caerModuleData data);
void allocateDataTransfers(davisCommonState state, uint32_t bufferNum, uint32_t bufferSize);
void deallocateDataTransfers(davisCommonState state);
void LIBUSB_CALL libUsbDataCallback(struct libusb_transfer *transfer);
void dataTranslator(davisCommonState state, uint8_t *buffer, size_t bytesSent);
void sendSpiConfigCommand(libusb_device_handle *devHandle, uint8_t moduleAddr, uint8_t paramAddr, uint32_t param);
libusb_device_handle *deviceOpen(libusb_context *devContext, uint8_t busNumber, uint8_t devAddress);
void deviceClose(libusb_device_handle *devHandle);

#endif /* DAVIS_COMMON_H_ */
