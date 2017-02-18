#include "visualizer.h"
#include "base/mainloop.h"
#include "modules/ini/dynapse_common.h"

#include <math.h>
#include <allegro5/allegro_primitives.h>
#include <allegro5/allegro_ttf.h>

#include <libcaer/events/spike.h>
#include <libcaer/devices/dynapse.h>

void caerVisualizerEventHandlerSpikeEvents(caerVisualizerPublicState state, ALLEGRO_EVENT event) {
	if (event.type == ALLEGRO_EVENT_MOUSE_BUTTON_UP) {
		int posx, posy;
		posx = U32T(event.mouse.x);
		posy = U32T(event.mouse.y);

		// adjust coordinates according to zoom
		float currentZoomFactor = sshsNodeGetFloat(state->visualizerConfigNode, "zoomFactor");
		if (currentZoomFactor > 1) {
			posx = (int) floor((float) posx / currentZoomFactor);
			posy = (int) floor((float) posy / currentZoomFactor);
		}
		else if (currentZoomFactor < 1) {
			posx = (int) floor((float) posx * currentZoomFactor);
			posy = (int) floor((float) posy * currentZoomFactor);
		}
		//caerLog(CAER_LOG_NOTICE, "Visualizer", "pos x %d, pos y %d Zoom %f \n", posx, posy, currentZoomFactor);

		// select chip
		uint16_t chipId = 0;
		if (posx >= (int) DYNAPSE_CONFIG_XCHIPSIZE && posy >= (int) DYNAPSE_CONFIG_YCHIPSIZE) {
			chipId = DYNAPSE_CONFIG_DYNAPSE_U3;
		}
		else if (posx < (int) DYNAPSE_CONFIG_XCHIPSIZE && posy >= (int) DYNAPSE_CONFIG_YCHIPSIZE) {
			chipId = DYNAPSE_CONFIG_DYNAPSE_U2;
		}
		else if (posx >= (int) DYNAPSE_CONFIG_XCHIPSIZE && posy < (int) DYNAPSE_CONFIG_YCHIPSIZE) {
			chipId = DYNAPSE_CONFIG_DYNAPSE_U1;
		}
		else if (posx < (int) DYNAPSE_CONFIG_XCHIPSIZE && posy < (int) DYNAPSE_CONFIG_YCHIPSIZE) {
			chipId = DYNAPSE_CONFIG_DYNAPSE_U0;
		}
		// adjust coordinate for chip
		if (chipId == DYNAPSE_CONFIG_DYNAPSE_U3) {
			posx = posx - DYNAPSE_CONFIG_XCHIPSIZE;
			posy = posy - DYNAPSE_CONFIG_YCHIPSIZE;
		}
		else if (chipId == DYNAPSE_CONFIG_DYNAPSE_U2) {
			posy = posy - DYNAPSE_CONFIG_YCHIPSIZE;
		}
		else if (chipId == DYNAPSE_CONFIG_DYNAPSE_U1) {
			posx = posx - DYNAPSE_CONFIG_XCHIPSIZE;
		}

		// select core
		uint8_t coreid = 0;
		if (posx >= 16 && posy < 16) {
			coreid = 2;
		}
		else if (posx >= 16 && posy >= 16) {
			coreid = 3;
		}
		else if (posx < 16 && posy < 16) {
			coreid = 0;
		}
		else if (posx < 16 && posy >= 16) {
			coreid = 1;
		}
		// adjust coordinates for cores
		if (coreid == 1) {
			posy = posy - DYNAPSE_CONFIG_NEUCOL;
		}
		else if (coreid == 0) {
			;
		}
		else if (coreid == 2) {
			posx = posx - DYNAPSE_CONFIG_NEUCOL;
		}
		else if (coreid == 3) {
			posx = posx - DYNAPSE_CONFIG_NEUCOL;
			posy = posy - DYNAPSE_CONFIG_NEUCOL;
		}

		// linear index
		uint32_t indexLin = posx * DYNAPSE_CONFIG_NEUCOL + posy;
		if (indexLin > 255) {
			indexLin = 255;
		}
		caerDeviceConfigSet(((caerInputDynapseState) state->eventSourceModuleState)->deviceState, DYNAPSE_CONFIG_CHIP,
			DYNAPSE_CONFIG_CHIP_ID, (uint32_t) chipId);
		caerDeviceConfigSet(((caerInputDynapseState) state->eventSourceModuleState)->deviceState,
			DYNAPSE_CONFIG_MONITOR_NEU, coreid, indexLin);

		if (chipId == 0) {
			caerLog(CAER_LOG_NOTICE, "Visualizer",
				"Monitoring neuron from DYNAPSE_U0 id %d, neuron number %d of core %d\n", chipId, indexLin, coreid);
		}
		if (chipId == 4) {
			caerLog(CAER_LOG_NOTICE, "Visualizer",
				"Monitoring neuron from DYNAPSE_U2 id %d, neuron number %d of core %d\n", chipId, indexLin, coreid);
		}
		if (chipId == 8) {
			caerLog(CAER_LOG_NOTICE, "Visualizer",
				"Monitoring neuron from DYNAPSE_U1 id %d, neuron number %d of core %d\n", chipId, indexLin, coreid);
		}
		if (chipId == 12) {
			caerLog(CAER_LOG_NOTICE, "Visualizer",
				"Monitoring neuron from DYNAPSE_U3 id %d, neuron number %d of core %d\n", chipId, indexLin, coreid);
		}

	}
}
