#ifndef DYNAPSE_COMMON_H_
#define DYNAPSE_COMMON_H_

#include "main.h"
#include "base/mainloop.h"
#include "base/module.h"
#include <limits.h>

#include <libcaer/devices/dynapse.h>

bool caerInputDYNAPSEInit(caerModuleData moduleData, uint16_t deviceType);
void caerInputDYNAPSEExit(caerModuleData moduleData);
void caerInputDYNAPSERun(caerModuleData moduleData, size_t argsNumber, va_list args);

#endif /* DYNAPSE_COMMON_H_ */
