#ifndef DAVIS_COMMON_H_
#define DAVIS_COMMON_H_

#include "main.h"
#include "base/mainloop.h"

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
