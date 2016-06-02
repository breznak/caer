#ifndef IMAGESTREAMERVISUALIZER_H_
#define IMAGESTREAMERVISUALIZER_H_

#include "main.h"

#define AUDIO_BEEP_FILE "modules/imagestreamerbeeper/beep5.ogg"

typedef struct imagestreamerbeeper_state *caerImagestreamerBeeperState;

void caerImagestreamerBeeper(uint16_t moduleID, double classific_results);

#endif /* IMAGESTREAMERVISUALIZER_H_ */
