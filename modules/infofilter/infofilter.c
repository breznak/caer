/*
 *
 *  Created on: Dec, 2016
 *      Author: federico.corradi@inilabs.com
 */

#include <time.h>
#include "infofilter.h"

struct INFilter_state {
	unsigned long microseconds;
	unsigned long seconds;
	unsigned long minutes;
	unsigned long hours;
	unsigned long time_in_micros;
	unsigned long started;
	ALLEGRO_DISPLAY *display;
	ALLEGRO_FONT *font;
	char * txt_string;
};

typedef struct INFilter_state *INFilterState;

static bool caerInfoFilterInit(caerModuleData moduleData);
static void caerInfoFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerInfoFilterExit(caerModuleData moduleData);
static void caerInfoFilterReset(caerModuleData moduleData, uint16_t resetCallSourceID);

static struct caer_module_functions caerInfoFilterFunctions = { .moduleInit =
	&caerInfoFilterInit, .moduleRun = &caerInfoFilterRun, .moduleExit = &caerInfoFilterExit, .moduleReset =
	&caerInfoFilterReset };

void caerInfoFilter(uint16_t moduleID, caerEventPacketContainer container) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "InfoFilter", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return;
	}

	caerModuleSM(&caerInfoFilterFunctions, moduleData, sizeof(struct INFilter_state), 1, container );
}

static bool caerInfoFilterInit(caerModuleData moduleData) {
	INFilterState state = moduleData->moduleState;

	state->minutes = 0;
	state->time_in_micros = 0;
	state->started = -1;

	state->display = NULL;

	if(!al_init()) {
	  fprintf(stderr, "failed to initialize allegro!\n");
	  return -1;
	}

	state->display = al_create_display(640, 320);
	if(!state->display) {
	  fprintf(stderr, "failed to create display!\n");
	  return -1;
	}

	al_init_font_addon(); // initialize the font addon
	al_init_ttf_addon();// initialize the ttf (True Type Font) addon

	state->font = al_load_ttf_font("modules/infofilter/whitrabt.ttf",62,0 );

	if (!state->font){
	  fprintf(stderr, "Could not load 'whitrabt.ttf'.\n");
	  exit(-1);
	}

	state->txt_string = (char*)malloc(TXTLEN * sizeof(char));

	// Nothing that can fail here.
	return (true);
}

static void caerInfoFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerEventPacketContainer container = va_arg(args, caerEventPacketContainer);

	if(container == NULL){
		//nothing to do
		return;
	}

	INFilterState state = moduleData->moduleState;

	int ts = caerEventPacketContainerGetHighestEventTimestamp(container);
	caerEventPacketHeaderGetEventSource(container);

	// get time of the recordings / live
	if(state->started == -1){
		state->started = ts;
	}
	unsigned long current =  (state->time_in_micros + ts) - state->started;
	state->microseconds = current - (state->minutes*60)*1e6 - state->seconds*1e6;
	state->minutes = (current / 60e6);
	state->seconds = ( ((int)current % (int)60e6) / 1e6);
	state->hours = ( ((int)current % (int)60e6) / 1e6) / 1e6;

	//al_clear_to_color(al_map_rgb(50,10,70));
	al_clear_to_color(al_map_rgb(0,0,0));
	sprintf(state->txt_string, " %02d:%02d:%02d:%04d", state->hours,
			state->minutes, state->seconds, state->microseconds);

	al_draw_text(state->font, al_map_rgb(255,255,255), 640/2, (320/4),ALLEGRO_ALIGN_CENTRE, state->txt_string);

	al_flip_display();

}

static void caerInfoFilterExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.

	INFilterState state = moduleData->moduleState;

	// free stuff
	al_destroy_display(state->display);
	free(state->txt_string);
}

static void caerInfoFilterReset(caerModuleData moduleData, uint16_t resetCallSourceID) {
	UNUSED_ARGUMENT(resetCallSourceID);

	INFilterState state = moduleData->moduleState;

	state->minutes = 0;
	state->time_in_micros = 0;
	state->started = -1;

}


