#ifndef INPUT_UNIXS_H_
#define INPUT_UNIXS_H_

#include "main.h"

#include <libcaer/events/packetContainer.h>
#include <libcaer/events/special.h>
#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>
#include <libcaer/events/imu6.h>

caerEventPacketContainer caerInputUnixS(uint16_t moduleID);

#endif /* INPUT_UNIXS_H_ */
