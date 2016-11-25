#include "visualizer.h"
#include "base/mainloop.h"

#include <math.h>
#include <allegro5/allegro_primitives.h>
#include <allegro5/allegro_ttf.h>

#include <libcaer/events/spike.h>
#include <libcaer/devices/dynapse.h>

void caerVisualizerEventHandlerSpikeEvents(caerVisualizerPublicState state, ALLEGRO_EVENT event) {
	if (event.type == ALLEGRO_EVENT_MOUSE_BUTTON_UP) {
		uint32_t posx, posy;
		posx = U32T(event.mouse.x);
		posy = U32T(event.mouse.y);

		float currentZoomFactor = sshsNodeGetFloat(state->visualizerConfigNode, "zoomFactor");

		uint8_t coreid = 0;
		if (posx > (int) DYNAPSE_CONFIG_NEUROW * currentZoomFactor
			&& posy > (int) DYNAPSE_CONFIG_NEUCOL * currentZoomFactor) {
			coreid = 3;
		}
		else if (posx < (int) DYNAPSE_CONFIG_NEUROW * currentZoomFactor
			&& posy > (int) DYNAPSE_CONFIG_NEUCOL * currentZoomFactor) {
			coreid = 2;
		}
		else if (posx > (int) DYNAPSE_CONFIG_NEUROW * currentZoomFactor
			&& posy < (int) DYNAPSE_CONFIG_NEUCOL * currentZoomFactor) {
			coreid = 1;
		}
		else if (posx < (int) DYNAPSE_CONFIG_NEUROW * currentZoomFactor
			&& posy < (int) DYNAPSE_CONFIG_NEUCOL * currentZoomFactor) {
			coreid = 0;
		}

		// Which chip is it?
		uint16_t chipid = DYNAPSE_CONFIG_DYNAPSE_U2;

		if (chipid == DYNAPSE_CONFIG_DYNAPSE_U2) {
			uint32_t indexLin = posy / U32T(currentZoomFactor) * DYNAPSE_CONFIG_NEUCOL / U32T(currentZoomFactor)
				+ posx / U32T(currentZoomFactor);

			caerDeviceConfigSet(state->eventSourceModuleState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U2);
			caerDeviceConfigSet(state->eventSourceModuleState, DYNAPSE_CONFIG_MONITOR_NEU, coreid, indexLin);

			caerLog(CAER_LOG_NOTICE, "Visualizer", "Monitoring neuron %d of core %d\n", indexLin, coreid);
		}
	}
}
