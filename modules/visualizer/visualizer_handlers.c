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
		// Check events come from an actual device.
		if (strstr(sshsNodeGetName(state->eventSourceConfigNode), "DYNAPSEFX2") == NULL) {
			return;
		}

		double posx, posy;
		posx = (double) U32T(event.mouse.x);
		posy = (double) U32T(event.mouse.y);

		// adjust coordinates according to zoom
		double currentZoomFactor = (double) sshsNodeGetFloat(state->visualizerConfigNode, "zoomFactor");
		if (currentZoomFactor > 1) {
			posx = (double)floor((double) posx / currentZoomFactor);
			posy = (double)floor((double) posy / currentZoomFactor);
		}
		else if (currentZoomFactor < 1) {
			posx = (double)floor((double) posx * currentZoomFactor);
			posy = (double)floor((double) posy * currentZoomFactor);
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
		uint32_t indexLin = (uint32_t) posx * DYNAPSE_CONFIG_NEUCOL + (uint32_t) posy;
		if (indexLin > 255) {
			indexLin = 255;
		}

		caerDeviceConfigSet(state->eventSourceModuleState, DYNAPSE_CONFIG_CHIP,DYNAPSE_CONFIG_CHIP_ID, (uint32_t) chipId);
		caerDeviceConfigSet(state->eventSourceModuleState, DYNAPSE_CONFIG_MONITOR_NEU, coreid, indexLin);

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
