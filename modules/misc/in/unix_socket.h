#ifndef INPUT_UNIX_SOCKET_H_
#define INPUT_UNIX_SOCKET_H_

#include "main.h"
#include "input_visualizer_eventhandler.h"

#include <libcaer/events/packetContainer.h>
#include <libcaer/events/special.h>
#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>
#include <libcaer/events/imu6.h>

caerEventPacketContainer caerInputUnixSocket(uint16_t moduleID);

#endif /* INPUT_UNIX_SOCKET_H_ */
