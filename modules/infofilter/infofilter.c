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

	state->font = al_load_ttf_font("modules/infofilter/fonts/whitrabt.ttf",32,0 );

	if (!state->font){
	  caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Could not load 'whitrabt.ttf'.\n");
	  exit(EXIT_FAILURE);
	}

	state->txt_string = (char*)malloc(TXTLEN * sizeof(char));

	al_set_new_display_refresh_rate(60.0);

	//state->bitmap = al_create_bitmap(BITMAP_SIZE_X, BITMAP_SIZE_Y);
	state->bitmap = al_load_bitmap("modules/infofilter/skin/info_caer_320x240.png");
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

	if (stateInfo == NULL) {
		return (thrd_error);
	}

	float seconds = 0.0f;
	bool pressed = false;
	int last_state = -1;

	clock_t start;
	clock_t end;
	end = clock();
	start = clock();

	//check startup status
#ifdef ENABLE_FILE_INPUT
	bool paused = false;
	if(state->fileInputConfigNode != NULL){
		paused = sshsNodeGetBool(state->fileInputConfigNode, "pause");
	}
	if(paused){
		state->bitmap = al_load_bitmap("modules/infofilter/skin/info_caer_320x240_pause.png");
	}
#endif
	thrd_set_name("eventCheckerThread");
	while (atomic_load_explicit(&state->running, memory_order_relaxed)) {

		// check time from last button press... if passed put default skin
		end = clock();
		seconds = (float)(end - start) / CLOCKS_PER_SEC;
		// if it is not paused (==1)
		//  	or recording (== 6)
		// then remove the contour after time t
		if( (seconds >= TIME_REMOVE) && pressed && (last_state != 1) && (last_state != 6)){
			state->bitmap = al_load_bitmap("modules/infofilter/skin/info_caer_320x240.png");
			pressed = false;
		}

		// check pressed button
 		int size = (int) round(BITMAP_SIZE_X / NUM_BUTTONS);
		bool get_mouse = al_get_next_event(state->event_queue, &state->event);
		if (get_mouse) {
			HandleEvent:
			// mouse pressed
			if(state->event.type == ALLEGRO_EVENT_MOUSE_BUTTON_DOWN){
				if(state->event.mouse.y > (BITMAP_SIZE_Y - BUTTONS_SIZE)){
					// determines which button has been pressed
					int button_number = (int) ((int) state->event.mouse.x / (int) size);
					last_state = button_number;
					switch(button_number){
						case 0:
							start = clock();
							pressed = true;
							caerLog(CAER_LOG_INFO, "InfoThread", "Play!");
#ifdef ENABLE_FILE_INPUT
							if(state->fileInputConfigNode != NULL){
								sshsNodePutBool(state->fileInputConfigNode, "pause", false);
							}
							state->bitmap = al_load_bitmap("modules/infofilter/skin/info_caer_320x240_play.png");
#else
							caerLog(CAER_LOG_WARNING, "InfoThread",
									"Not in file input mode! Impossible to Play the stream.");
#endif
							break;
						case 1:
							start = clock();
							pressed = true;
#ifdef ENABLE_FILE_INPUT
							if(state->fileInputConfigNode != NULL){
								sshsNodePutBool(state->fileInputConfigNode, "pause", true);
							}
							state->bitmap = al_load_bitmap("modules/infofilter/skin/info_caer_320x240_pause.png");
#else
							caerLog(CAER_LOG_WARNING, "InfoThread",
								"Not in file input mode! Impossible to Pause the stream.");
#endif
							break;
						case 2:
							start = clock();
							pressed = true;
							state->bitmap = al_load_bitmap("modules/infofilter/skin/info_caer_320x240_rewind.png");
							caerLog(CAER_LOG_INFO, "InfoThread", "Rewind");
							break;
						case 3:
							start = clock();
							pressed = true;
#ifdef ENABLE_FILE_INPUT
							if(state->fileInputConfigNode != NULL){
								int current_packet_int = sshsNodeGetInt(state->fileInputConfigNode, "PacketContainerInterval");
								current_packet_int -= (int)((current_packet_int/100)*CHANGE_FACTOR);
								sshsNodePutInt(state->fileInputConfigNode, "PacketContainerInterval", current_packet_int);
							}
							state->bitmap = al_load_bitmap("modules/infofilter/skin/info_caer_320x240_slow_motion.png");
#else
							caerLog(CAER_LOG_WARNING, "InfoThread",
								"Not in file input mode! Impossible to Slow Motion the stream.");
#endif
							break;
						case 4:
							start = clock();
							pressed = true;
#ifdef ENABLE_FILE_INPUT
							if(state->fileInputConfigNode != NULL){
								int current_packet_int = sshsNodeGetInt(state->fileInputConfigNode, "PacketContainerInterval");
								current_packet_int += (int)((current_packet_int/100)*CHANGE_FACTOR);
								sshsNodePutInt(state->fileInputConfigNode, "PacketContainerInterval", current_packet_int);
							}
							state->bitmap = al_load_bitmap("modules/infofilter/skin/info_caer_320x240_forward.png");
#else
							caerLog(CAER_LOG_WARNING, "InfoThread",
								"Not in file input mode! Impossible to Fast Forward the stream.");
#endif
							caerLog(CAER_LOG_INFO, "InfoThread", "Forward");
							break;
						case 5:
							start = clock();
							pressed = true;
							state->bitmap = al_load_bitmap("modules/infofilter/skin/info_caer_320x240_rewind_start.png");
							caerLog(CAER_LOG_INFO, "InfoThread", "Rewind from start");
							break;
						case 6:
							start = clock();
							state->bitmap = al_load_bitmap("modules/infofilter/skin/info_caer_320x240_rec.png");
							caerLog(CAER_LOG_INFO, "InfoThread", "Start Recording");
							break;
						case 7:
							start = clock();
							pressed = true;
							state->bitmap = al_load_bitmap("modules/infofilter/skin/info_caer_320x240_stop.png");
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
		al_draw_text(state->font, al_map_rgb(0,0,0), BITMAP_SIZE_X/2, (BITMAP_SIZE_Y-(size*2)),ALLEGRO_ALIGN_CENTRE, state->txt_string);

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

	if(state->fileInputConfigNode == NULL){
		atomic_store(&state->fileInputID, fileInputID);
		if(atomic_load(&state->fileInputID) != NULL){
			state->fileInputConfigNode = caerMainloopGetSourceNode(U16T(atomic_load(&state->fileInputID)));
		}
	}

	if(container == NULL){
		//nothing to do
		return;
	}

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

}

static void caerInfoFilterExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.

	INFilterState state = moduleData->moduleState;

	// free display stuff is done in the thread exit
	atomic_store(&state->running, false);

	// wait for thread to finish
	pthread_join(state->eventChecker, NULL);

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


