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
	unsigned long started;
	ALLEGRO_BITMAP *bitmap;
	ALLEGRO_DISPLAY *display;
	ALLEGRO_FONT *font;
	ALLEGRO_EVENT_QUEUE *event_queue;
	ALLEGRO_EVENT event;
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
	state->seconds = 0;
	state->hours = 0;
	state->microseconds = 0;
	state->started = -1;

	state->display = NULL;

	if(!al_init()) {
	  fprintf(stderr, "failed to initialize allegro!\n");
	  return -1;
	}

	state->display = al_create_display(BITMAP_SIZE_X, BITMAP_SIZE_Y);
	if(!state->display) {
	  fprintf(stderr, "failed to create display!\n");
	  return -1;
	}

	// Now load addons: primitives to draw, fonts (and TTF) to write text.
	if (al_init_primitives_addon()) {
		// Successfully initialized Allegro primitives addon.
		caerLog(CAER_LOG_DEBUG, "Visualizer", "Allegro primitives addon initialized successfully.");
	}
	else {
		// Failed to initialize Allegro primitives addon.
		caerLog(CAER_LOG_EMERGENCY, "Visualizer", "Failed to initialize Allegro primitives addon.");
		exit(EXIT_FAILURE);
	}

	al_init_font_addon();

	if (al_init_ttf_addon()) {
		// Successfully initialized Allegro TTF addon.
		caerLog(CAER_LOG_DEBUG, "Visualizer", "Allegro TTF addon initialized successfully.");
	}
	else {
		// Failed to initialize Allegro TTF addon.
		caerLog(CAER_LOG_EMERGENCY, "Visualizer", "Failed to initialize Allegro TTF addon.");
		exit(EXIT_FAILURE);
	}

	if(!al_init_image_addon()) {
		caerLog(CAER_LOG_EMERGENCY, "Visualizer", "Failed to initialize Allegro Image addon.");
		exit(EXIT_FAILURE);
	}

	state->font = al_load_ttf_font("modules/infofilter/whitrabt.ttf",32,0 );

	if (!state->font){
	  caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Could not load 'whitrabt.ttf'.\n");
	  exit(EXIT_FAILURE);
	}

	state->txt_string = (char*)malloc(TXTLEN * sizeof(char));

	al_set_new_display_refresh_rate(60.0);

	// Set correct names.
	al_set_org_name("iniLabs");
	al_set_app_name("infoFilter");

	//state->bitmap = al_create_bitmap(BITMAP_SIZE_X, BITMAP_SIZE_Y);
	state->bitmap = al_load_bitmap("modules/infofilter/info_caer_320x240.png");
	if(!state->bitmap) {
	      fprintf(stderr, "failed to create bitmap!\n");
	  	  al_destroy_bitmap(state->bitmap);
	      al_destroy_display(state->display);
	      free(state->txt_string);
	      exit(EXIT_FAILURE);
	}

	// events
	state->event_queue = NULL;
	state->event_queue = al_create_event_queue();
	if(!state->event_queue) {
	  fprintf(stderr, "failed to create event_queue!\n");
	  al_destroy_bitmap(state->bitmap);
	  al_destroy_display(state->display);
	  exit(EXIT_FAILURE);
	}
	al_register_event_source(state->event_queue, al_get_mouse_event_source());

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
	unsigned long current =  ts - state->started;
	state->microseconds = current - (state->minutes*60)*1e6 - state->seconds*1e6;
	state->minutes = (current / 60e6);
	state->seconds = ( ((int)current % (int)60e6) / 1e6);
	state->hours = ( ((int)current % (int)60e6) / 1e6) / 1e6;

	sprintf(state->txt_string, " %02d:%02d:%02d:%04d", state->hours,
			state->minutes, state->seconds, state->microseconds);

	al_set_target_bitmap(state->bitmap);
	al_set_target_bitmap(al_get_backbuffer(state->display));
	al_draw_bitmap(state->bitmap, 0, 0, 0);

	/* timer */
	al_draw_text(state->font, al_map_rgb(0,0,0), BITMAP_SIZE_X/2, (BITMAP_SIZE_Y/4),ALLEGRO_ALIGN_CENTRE, state->txt_string);

	//al_draw_rectangle(0.0, 0.0, 10.0, 10.0, al_map_rgb(0,0,0), 10.0);
	int size = (int) round(BITMAP_SIZE_X / NUM_BUTTONS);
	al_put_pixel(0,(BITMAP_SIZE_Y - BUTTONS_SIZE),al_map_rgb(255,0,0));
	al_put_pixel(size,(BITMAP_SIZE_Y - BUTTONS_SIZE),al_map_rgb(255,0,0));
	al_put_pixel(size*2,(BITMAP_SIZE_Y - BUTTONS_SIZE),al_map_rgb(255,0,0));
	al_put_pixel(size*3,(BITMAP_SIZE_Y - BUTTONS_SIZE),al_map_rgb(255,0,0));
	al_put_pixel(size*4,(BITMAP_SIZE_Y - BUTTONS_SIZE),al_map_rgb(255,0,0));
	al_put_pixel(size*5,(BITMAP_SIZE_Y - BUTTONS_SIZE),al_map_rgb(255,0,0));
	al_put_pixel(size*6,(BITMAP_SIZE_Y - BUTTONS_SIZE),al_map_rgb(255,0,0));
	al_put_pixel(size*7,(BITMAP_SIZE_Y - BUTTONS_SIZE),al_map_rgb(255,0,0));

	bool get_mouse = al_get_next_event(state->event_queue, &state->event);
	if (get_mouse) {
		// mouse pressed
		if(state->event.type == ALLEGRO_EVENT_MOUSE_BUTTON_DOWN){
			//caerLog(CAER_LOG_INFO, moduleData->moduleSubSystemString,
			//		"Mouse position: (%d, %d)\n", state->event.mouse.x, state->event.mouse.y);
			// buttons area
			if(state->event.mouse.y > (BITMAP_SIZE_Y - BUTTONS_SIZE)){
				// determines which button has been pressed
				//caerLog(CAER_LOG_INFO, moduleData->moduleSubSystemString,
				//	"Button pressed position: (%d, %d)\n", state->event.mouse.x, state->event.mouse.y);
				int button_number = (int) ((int) state->event.mouse.x / (int) size);
				switch(button_number){
					case 0:
						caerLog(CAER_LOG_INFO, moduleData->moduleSubSystemString, "Play!");
						break;
					case 1:
						caerLog(CAER_LOG_INFO, moduleData->moduleSubSystemString, "Rewind");
						break;
					case 2:
						caerLog(CAER_LOG_INFO, moduleData->moduleSubSystemString, "Forward");
						break;
					case 3:
						caerLog(CAER_LOG_INFO, moduleData->moduleSubSystemString, "Pause");
						break;
					case 4:
						caerLog(CAER_LOG_INFO, moduleData->moduleSubSystemString, "Rewind from start");
						break;
					case 5:
						caerLog(CAER_LOG_INFO, moduleData->moduleSubSystemString, "Start Recording");
						break;
					case 6:
						caerLog(CAER_LOG_INFO, moduleData->moduleSubSystemString, "Stop Recording");
						break;
				}

			}
		}
	}
	al_flip_display();
	al_clear_to_color(al_map_rgb(0,0,0));
}

static void caerInfoFilterExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.

	INFilterState state = moduleData->moduleState;

	// free stuff
	al_destroy_bitmap(state->bitmap);
	al_destroy_display(state->display);
	free(state->txt_string);
}

static void caerInfoFilterReset(caerModuleData moduleData, uint16_t resetCallSourceID) {
	UNUSED_ARGUMENT(resetCallSourceID);

	INFilterState state = moduleData->moduleState;

	state->minutes = 0;
	state->seconds = 0;
	state->hours = 0;
	state->microseconds = 0;
	state->started = -1;

}


