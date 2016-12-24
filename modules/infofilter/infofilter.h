/*
 *
 *  Created on: Dec , 2016
 *      Author: federico.corradi@inilabs.com
 */

#ifndef INFOFILTER_H_
#define INFOFILTER_H_

#include "main.h"
#include "base/mainloop.h"
#include "base/module.h"

#include <libcaer/events/polarity.h>
#include <libcaer/events/spike.h>
#include <libcaer/events/frame.h>

#include <allegro5/allegro.h>
#include <allegro5/allegro_font.h>
#include <allegro5/allegro_ttf.h>

#define TXTLEN 2048

void caerInfoFilter(uint16_t moduleID, caerEventPacketContainer container);

#endif /* INFOFILTER_H_ */
