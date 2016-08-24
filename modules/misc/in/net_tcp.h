#ifndef INPUT_NET_TCP_H_
#define INPUT_NET_TCP_H_

#include "main.h"
#include "input_visualizer_eventhandler.h"

#include <libcaer/events/packetContainer.h>
#include <libcaer/events/special.h>
#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>
#include <libcaer/events/imu6.h>

caerEventPacketContainer caerInputNetTCP(uint16_t moduleID);

#endif /* INPUT_NET_TCP_H_ */
