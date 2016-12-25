/*
 *
 *  Created on: Dec, 2016
 *      Author: federico.corradi@inilabs.com
 */

#include <time.h>
#include "infofilter.h"

struct INFilter_state {
	atomic_int_fast32_t microseconds;
	atomic_int_fast32_t seconds;
	atomic_int_fast32_t minutes;
	atomic_int_fast32_t hours;
	atomic_int_fast32_t started;
	atomic_int_fast32_t fileInputID;
	atomic_bool running;
	thrd_t eventChecker;
	ALLEGRO_BITMAP *bitmap;
	ALLEGRO_DISPLAY *display;
	ALLEGRO_FONT *font;
	ALLEGRO_EVENT_QUEUE *event_queue;
	ALLEGRO_EVENT event;
	char * txt_string;
	sshsNode fileInputConfigNode;
};

typedef struct INFilter_state *INFilterState;

static bool caerInfoFilterInit(caerModuleData moduleData);
static void caerInfoFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerInfoFilterExit(caerModuleData moduleData);
static void caerInfoFilterReset(caerModuleData moduleData, uint16_t resetCallSourceID);
int eventCheckerThread(void *stateInfo);
static void destroyThread(void *infostate);

static struct caer_module_functions caerInfoFilterFunctions = { .moduleInit =
	&caerInfoFilterInit, .moduleRun = &caerInfoFilterRun, .moduleExit = &caerInfoFilterExit, .moduleReset =
	&caerInfoFilterReset };

void caerInfoFilter(uint16_t moduleID, caerEventPacketContainer container, uint16_t fileInputID) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "InfoFilter", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return;
	}

	caerModuleSM(&caerInfoFilterFunctions, moduleData, sizeof(struct INFilter_state), 2, container, fileInputID );
}

static bool caerInfoFilterInit(caerModuleData moduleData) {
	INFilterState state = moduleData->moduleState;

	atomic_store(&state->minutes,0);
	atomic_store(&state->seconds,0);
	atomic_store(&state->hours,0);
	atomic_store(&state->microseconds,0);
	atomic_store(&state->started,-1);

	state->display = NULL;

	if(!al_init()) {
	  caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "failed to initialize allegro!\n");
	  exit(EXIT_FAILURE);
	}

	state->display = al_create_display(BITMAP_SIZE_X, BITMAP_SIZE_Y);
	if(!state->display) {
	  caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "failed to create display!\n");
	  exit(EXIT_FAILURE);
	}

	// Now load addons: primitives to draw, fonts (and TTF) to write text.
	if (al_init_primitives_addon()) {
		// Successfully initialized Allegro primitives addon.
		caerLog(CAER_LOG_DEBUG, moduleData->moduleSubSystemString, "Allegro primitives addon initialized successfully.");
	}
	else {
		// Failed to initialize Allegro primitives addon.
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to initialize Allegro primitives addon.");
		exit(EXIT_FAILURE);
	}

	al_init_font_addon();

	if (al_init_ttf_addon()) {
		// Successfully initialized Allegro TTF addon.
		caerLog(CAER_LOG_DEBUG,  moduleData->moduleSubSystemString, "Allegro TTF addon initialized successfully.");
	}
	else {
		// Failed to initialize Allegro TTF addon.
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to initialize Allegro TTF addon.");
		exit(EXIT_FAILURE);
	}

	if(!al_init_image_addon()) {
		caerLog(CAER_LOG_EMERGENCY, moduleData->moduleSubSystemString, "Failed to initialize Allegro Image addon.");
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
	  caerLog(CAER_LOG_ERROR,  moduleData->moduleSubSystemString, "failed to create event_queue!\n");
	  al_destroy_bitmap(state->bitmap);
	  al_destroy_display(state->display);
	  exit(EXIT_FAILURE);
	}
	al_register_event_source(state->event_queue, al_get_mouse_event_source());

	state->fileInputConfigNode = NULL;
	atomic_store(&state->running, true);

	/* start thread */
	if (thrd_create(&state->eventChecker, &eventCheckerThread, state) != thrd_success) {
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
				"eventCheckerThread: Failed to start thread.");
		return (NULL);
	}

	// Nothing that can fail here.
	return (true);
}

int eventCheckerThread(void *stateInfo) {
	INFilterState state = stateInfo;

	//if (stateInfo == NULL) {
	//	return (thrd_error);
	//}
	thrd_set_name("eventCheckerThread");
	while (atomic_load_explicit(&state->running, memory_order_relaxed)) {
		//caerVisualizerUpdateScreen(state);
		//caerLog(CAER_LOG_NOTICE, "tt",  "from the thread\n");
		int size = (int) round(BITMAP_SIZE_X / NUM_BUTTONS);
		bool get_mouse = al_get_next_event(state->event_queue, &state->event);
		if (get_mouse) {
			HandleEvent:
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
							caerLog(CAER_LOG_INFO, "InfoThread", "Play!");
	#ifdef ENABLE_FILE_INPUT
							if(state->fileInputConfigNode != NULL){
								sshsNodePutBool(state->fileInputConfigNode, "pause", false);
							}
	#else
							caerLog(CAER_LOG_WARNING, "InfoThread",
									"Not in file input mode! Impossible to Play the stream.");
	#endif
							break;
						case 1:
							caerLog(CAER_LOG_INFO, "InfoThread", "Rewind");
							break;
						case 2:
							caerLog(CAER_LOG_INFO, "InfoThread", "Forward");
							break;
						case 3:
							caerLog(CAER_LOG_INFO, "InfoThread", "Pause");
	#ifdef ENABLE_FILE_INPUT
							if(state->fileInputConfigNode != NULL){
								sshsNodePutBool(state->fileInputConfigNode, "pause", true);
							}
							//al_wait_for_event(state->event_queue, &state->event); // play or else
							//goto HandleEvent;
	#else
							caerLog(CAER_LOG_WARNING, "InfoThread",
									"Not in file input mode! Impossible to Pause the stream.");
	#endif
							break;
						case 4:
							caerLog(CAER_LOG_INFO, "InfoThread", "Rewind from start");
							break;
						case 5:
							caerLog(CAER_LOG_INFO, "InfoThread", "Start Recording");
							break;
						case 6:
							caerLog(CAER_LOG_INFO, "InfoThread", "Stop Recording");
							break;
					}

				}
			}
		}

		/*update timing*/
		sprintf(state->txt_string, " %02d:%02d:%02d:%04d", atomic_load(&state->hours),
				 atomic_load(&state->minutes),  atomic_load(&state->seconds),  atomic_load(&state->microseconds));

		al_set_target_bitmap(state->bitmap);
		al_set_target_bitmap(al_get_backbuffer(state->display));
		al_draw_bitmap(state->bitmap, 0, 0, 0);

		/* timer */
		al_draw_text(state->font, al_map_rgb(0,0,0), BITMAP_SIZE_X/2, (BITMAP_SIZE_Y/4),ALLEGRO_ALIGN_CENTRE, state->txt_string);

		al_flip_display();
		al_clear_to_color(al_map_rgb(0,0,0));

	}

	// free stuff
	destroyThread(state);

	return (thrd_success);
}

static void destroyThread(void *infostate) {
	INFilterState state = infostate;

	al_destroy_bitmap(state->bitmap);
	al_destroy_display(state->display);
	al_destroy_font(state->font);

}

static void caerInfoFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerEventPacketContainer container = va_arg(args, caerEventPacketContainer);
	uint16_t fileInputID = va_arg(args, int);

	INFilterState state = moduleData->moduleState;

	if(container == NULL){
		//nothing to do
		return;
	}

	atomic_store(&state->fileInputID, fileInputID);

	int ts = caerEventPacketContainerGetHighestEventTimestamp(container);
	caerEventPacketHeaderGetEventSource(container);

	// get time of the recordings / live
	if(atomic_load(&state->started) == -1){
		atomic_store(&state->started,ts);
	}

	unsigned long current =  ts - atomic_load(&state->started);
	atomic_store(&state->microseconds, current - (state->minutes*60)*1e6 - state->seconds*1e6);
	atomic_store(&state->minutes, (current / 60e6));
	atomic_store(&state->seconds, ( ((int)current % (int)60e6) / 1e6));
	atomic_store(&state->hours, ( ((int)current % (int)60e6) / 1e6) / 1e6);

	if(atomic_load(&state->fileInputID) != NULL){
		state->fileInputConfigNode = caerMainloopGetSourceNode(U16T(atomic_load(&state->fileInputID)));
	}

}

static void caerInfoFilterExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.

	INFilterState state = moduleData->moduleState;

	// free display stuff is done in the thread exit
	atomic_store(&state->running, false);
	// string free
	free(state->txt_string);
}

static void caerInfoFilterReset(caerModuleData moduleData, uint16_t resetCallSourceID) {
	UNUSED_ARGUMENT(resetCallSourceID);

	INFilterState state = moduleData->moduleState;

	atomic_store(&state->minutes,0);
	atomic_store(&state->seconds,0);
	atomic_store(&state->hours,0);
	atomic_store(&state->microseconds,0);
	atomic_store(&state->started,-1);

}


