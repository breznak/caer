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
#include <allegro5/allegro_image.h>

#ifdef HAVE_PTHREADS
#include "ext/c11threads_posix.h"
#endif
#include <stdatomic.h>

#define TXTLEN 2048
#define BITMAP_SIZE_X 320
#define BITMAP_SIZE_Y 240
#define BUTTONS_SIZE 40
#define NUM_BUTTONS 8
#define TIME_REMOVE 0.3
#define CHANGE_FACTOR 20	// change factor % (plus or minus of actual value)

void caerInfoFilter(uint16_t moduleID, caerEventPacketContainer container, uint16_t fileInputID);

#endif /* INFOFILTER_H_ */
