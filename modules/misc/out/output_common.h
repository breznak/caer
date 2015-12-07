#ifndef OUTPUT_COMMON_H_
#define OUTPUT_COMMON_H_

#include "main.h"
#include "base/mainloop.h"
#include "base/module.h"

struct eventPacketMapper {
	int16_t sourceID;
	int16_t typeID;
	int16_t transferRingID;
};

bool caerOutputCommonInit(caerModuleData moduleData);
void caerOutputCommonExit(caerModuleData moduleData);
void caerOutputCommonRun(caerModuleData moduleData, size_t argsNumber, va_list args);
void caerOutputCommonConfig(caerModuleData moduleData);

#endif /* OUTPUT_COMMON_H_ */
