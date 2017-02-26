/*
 * effectivetransferfunction.h
 *
 *  Created on: Feb 2017 - http://www.nature.com/articles/srep14730
 *      Author: federico
 */

#ifndef ETF_H_
#define ETF_H_

#include "main.h"
#include "ext/portable_time.h"
#include "modules/ini/dynapse_common.h"	// useful constants

#include <libcaer/events/spike.h>
#include <libcaer/events/frame.h>
#include <libcaer/events/point4d.h>

caerPoint4DEventPacket caerEffectiveTransferFunction(uint16_t moduleID, caerSpikeEventPacket spike);

#endif /* ETF_H_ */
