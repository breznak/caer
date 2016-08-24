#ifndef INPUT_VISUALIZER_EVENTHANDLER_H_
#define INPUT_VISUALIZER_EVENTHANDLER_H_

// Visualizer event handler support, for keyboard commands.
#ifdef ENABLE_VISUALIZER

#include "modules/visualizer/visualizer.h"

void caerInputVisualizerEventHandler(caerVisualizerPublicState state, ALLEGRO_EVENT event);

#endif

#endif /* INPUT_VISUALIZER_EVENTHANDLER_H_ */
