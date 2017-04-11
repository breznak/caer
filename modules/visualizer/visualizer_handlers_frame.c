#include "visualizer.h"
#include "base/mainloop.h"

#include <math.h>
#include <allegro5/allegro_primitives.h>
#include <allegro5/allegro_ttf.h>


void caerVisualizerEventHandlerFrameEvents(caerVisualizerPublicState state, ALLEGRO_EVENT event) {
	if (event.type == ALLEGRO_EVENT_MOUSE_BUTTON_UP) {

		double posx, posy;
		posx = (double) U32T(event.mouse.x);
		posy = (double) U32T(event.mouse.y);

//		// adjust coordinates according to zoom
		double currentZoomFactor = (double) sshsNodeGetFloat(state->visualizerConfigNode, "zoomFactor");
		posx = (double)floor( posx / currentZoomFactor);
		posy = (double)floor( posy / currentZoomFactor);

		caerLog(CAER_LOG_NOTICE, "Visualizer", "pos x %f, pos y %f Zoom %f \n", posx, posy, currentZoomFactor);

	}
}
