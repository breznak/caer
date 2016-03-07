#include "visualizer_allegro.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "modules/statistics/statistics.h"
#include "ext/portable_time.h"
#include <allegro5/allegro.h>

#define TEXT_SPACING 20 // in pixels

#define SYSTEM_TIMEOUT 10 // in seconds

struct visualizer_allegro_state {
	ALLEGRO_DISPLAY *displayEvents;
	ALLEGRO_DISPLAY *displayFrames;
    ALLEGRO_BITMAP *bb;
    ALLEGRO_BITMAP *bbframes;
	uint32_t  *eventRenderer;
	int16_t eventRendererSizeX;
	int16_t eventRendererSizeY;
	size_t eventRendererSlowDown;
	struct caer_statistics_state eventStatistics;
	uint16_t  *frameRenderer;
	int32_t frameRendererSizeX;
	int32_t frameRendererSizeY;
	int32_t frameRendererPositionX;
	int32_t frameRendererPositionY;
	enum caer_frame_event_color_channels frameChannels;
	struct caer_statistics_state frameStatistics;
	int16_t subsampleRendering;
	int16_t subsampleCount;
};

typedef struct visualizer_allegro_state *visualizerAllegroState;

static bool caerVisualizerAllegroInit(caerModuleData moduleData);
static void caerVisualizerAllegroRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerVisualizerAllegroExit(caerModuleData moduleData);
static bool allocateEventRenderer(visualizerAllegroState state, int16_t sourceID);
static bool allocateFrameRenderer(visualizerAllegroState state, int16_t sourceID);

ALLEGRO_BITMAP *create_memory_bitmap(int w, int h);

static struct caer_module_functions caerVisualizerFunctions = { .moduleInit = &caerVisualizerAllegroInit, .moduleRun =
	&caerVisualizerAllegroRun, .moduleConfig =
NULL, .moduleExit = &caerVisualizerAllegroExit };

void caerVisualizerAllegro(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "VisualizerAllegro");

	caerModuleSM(&caerVisualizerFunctions, moduleData, sizeof(struct visualizer_allegro_state), 2, polarity, frame);
}

static bool caerVisualizerAllegroInit(caerModuleData moduleData) {
	visualizerAllegroState state = moduleData->moduleState;

   
	// Configuration.
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "showEvents", true);
#ifdef DVS128
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "showFrames", false);
#else
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "showFrames", true);
#endif

	sshsNodePutShortIfAbsent(moduleData->moduleNode, "subsampleRendering", 1);

	state->subsampleRendering = sshsNodeGetShort(moduleData->moduleNode, "subsampleRendering");
	state->subsampleCount = 1;


	if (!caerStatisticsStringInit(&state->eventStatistics)) {
		return (false);
	}

	state->eventStatistics.divisionFactor = 1000;

	if (!caerStatisticsStringInit(&state->frameStatistics)) {
		return (false);
	}

	state->frameStatistics.divisionFactor = 1;

    if(!al_init()) {
        fprintf(stderr, "failed to initialize allegro!\n");
        return -1;
    } 

    state->displayEvents = al_create_display(VISUALIZER_ALLEGRO_SCREEN_WIDTH,VISUALIZER_ALLEGRO_SCREEN_HEIGHT);
    if( state->displayEvents == NULL) {
        fprintf(stderr, "failed to create display!\n");
        return -1;
    }
    
    state->bb = al_get_backbuffer(state->displayEvents);
    al_clear_to_color(al_map_rgb(0,0,0));
    al_flip_display();  
    
    state->displayFrames = al_create_display(VISUALIZER_ALLEGRO_SCREEN_WIDTH,VISUALIZER_ALLEGRO_SCREEN_HEIGHT);
    if( state->displayFrames == NULL) {
        fprintf(stderr, "failed to create display!\n");
        return -1;
    }
    
    state->bbframes = al_get_backbuffer(state->displayFrames);
    al_clear_to_color(al_map_rgb(0,0,0));
    al_flip_display();  
    
	return (true);
}

static void caerVisualizerAllegroExit(caerModuleData moduleData) {
	visualizerAllegroState state = moduleData->moduleState;


	// Ensure render maps are freed.
	if (state->eventRenderer != NULL) {
		free(state->eventRenderer);
		state->eventRenderer = NULL;
	}

	if (state->frameRenderer != NULL) {
		free(state->frameRenderer);
		state->frameRenderer = NULL;
	}

	// Statistics text.
	caerStatisticsStringExit(&state->eventStatistics);
	caerStatisticsStringExit(&state->frameStatistics);
}

static void caerVisualizerAllegroRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	visualizerAllegroState state = moduleData->moduleState;

	// Subsampling, only render every Nth packet.
	if (state->subsampleCount >= state->subsampleRendering) {
		// Polarity events to render.
		caerPolarityEventPacket polarity = va_arg(args, caerPolarityEventPacket);
		bool renderPolarity = sshsNodeGetBool(moduleData->moduleNode, "showEvents");

		// Frames to render.
		caerFrameEventPacket frame = va_arg(args, caerFrameEventPacket);
		bool renderFrame = sshsNodeGetBool(moduleData->moduleNode, "showFrames");

		// Update polarity event rendering map.
		if (renderPolarity && polarity != NULL) {
			// If the event renderer is not allocated yet, do it.
			if (state->eventRenderer == NULL) {
				if (!allocateEventRenderer(state, caerEventPacketHeaderGetEventSource(&polarity->packetHeader))) {
					// Failed to allocate memory, nothing to do.
					caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
						"Failed to allocate memory for eventRenderer.");
					return;
				}
			}

			if (state->subsampleRendering > 1) {
				memset(state->eventRenderer, 0,
					(size_t) (state->eventRendererSizeX * state->eventRendererSizeY) * sizeof(uint32_t));
			}

			CAER_POLARITY_ITERATOR_VALID_START(polarity)
				if (caerPolarityEventGetPolarity(caerPolarityIteratorElement)) {
					// Green.
					state->eventRenderer[(caerPolarityEventGetY(caerPolarityIteratorElement) * state->eventRendererSizeX)
						+ caerPolarityEventGetX(caerPolarityIteratorElement)] = (U32T(0x00FFUL << 8));
				}
				else {
					// Red.
					state->eventRenderer[(caerPolarityEventGetY(caerPolarityIteratorElement) * state->eventRendererSizeX)
						+ caerPolarityEventGetX(caerPolarityIteratorElement)] = (U32T(0x00FFUL << 0));
				}CAER_POLARITY_ITERATOR_VALID_END

			// Accumulate events over four polarity packets.
			if (state->subsampleRendering <= 1) {
				if (state->eventRendererSlowDown++ == 4) {
					state->eventRendererSlowDown = 0;

					memset(state->eventRenderer, 0,
						(size_t) (state->eventRendererSizeX * state->eventRendererSizeY) * sizeof(uint32_t));
				}
			}
		}

		// Select latest frame to render.
		if (renderFrame && frame != NULL) {
			// If the event renderer is not allocated yet, do it.
			if (state->frameRenderer == NULL) {
				if (!allocateFrameRenderer(state, caerEventPacketHeaderGetEventSource(&frame->packetHeader))) {
					// Failed to allocate memory, nothing to do.
					caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
						"Failed to allocate memory for frameRenderer.");
					return;
				}
			}

			caerFrameEvent currFrameEvent;

			for (int32_t i = caerEventPacketHeaderGetEventNumber(&frame->packetHeader) - 1; i >= 0; i--) {
				currFrameEvent = caerFrameEventPacketGetEvent(frame, i);

				// Only operate on the last valid frame.
				if (caerFrameEventIsValid(currFrameEvent)) {
					// Copy the frame content to the permanent frameRenderer.
					// Use frame sizes to correctly support small ROI frames.
					state->frameRendererSizeX = caerFrameEventGetLengthX(currFrameEvent);
					state->frameRendererSizeY = caerFrameEventGetLengthY(currFrameEvent);
					state->frameRendererPositionX = caerFrameEventGetPositionX(currFrameEvent);
					state->frameRendererPositionY = caerFrameEventGetPositionY(currFrameEvent);
					state->frameChannels = caerFrameEventGetChannelNumber(currFrameEvent);

					memcpy(state->frameRenderer, caerFrameEventGetPixelArrayUnsafe(currFrameEvent),
						((size_t) state->frameRendererSizeX * (size_t) state->frameRendererSizeY
							* (size_t) state->frameChannels * sizeof(uint16_t)));

					break;
				}
			}
		}

		// Detect if nothing happened for a long time.
		struct timespec currentTime;
		portable_clock_gettime_monotonic(&currentTime);

		uint64_t diffNanoTimeEvents = (uint64_t) (((int64_t) (currentTime.tv_sec
			- state->eventStatistics.lastTime.tv_sec) * 1000000000LL)
			+ (int64_t) (currentTime.tv_nsec - state->eventStatistics.lastTime.tv_nsec));
		bool noEventsTimeout = (diffNanoTimeEvents >= U64T(SYSTEM_TIMEOUT * 1000000000LL));

		uint64_t diffNanoTimeFrames = (uint64_t) (((int64_t) (currentTime.tv_sec
			- state->frameStatistics.lastTime.tv_sec) * 1000000000LL)
			+ (int64_t) (currentTime.tv_nsec - state->frameStatistics.lastTime.tv_nsec));
		bool noFramesTimeout = (diffNanoTimeFrames >= U64T(SYSTEM_TIMEOUT * 1000000000LL));

        // TO DO SEPARATE THREAD
        //
		// All rendering calls at the end.
		// Only execute if something actually changed (packets not null).
		if ((renderPolarity && (polarity != NULL || noEventsTimeout))
			|| (renderFrame && (frame != NULL || noFramesTimeout))) {

            //set backbuffer as target
            al_clear_to_color(al_map_rgb(0,0,0));

			// Render polarity events.
			if (renderPolarity) {
				// Write statistics text.
				caerStatisticsStringUpdate((caerEventPacketHeader) polarity, &state->eventStatistics);
                //lock bitmap
                al_set_target_bitmap(state->bb); 
                ALLEGRO_LOCKED_REGION * lock;
                lock = al_lock_bitmap(state->bb, ALLEGRO_PIXEL_FORMAT_ABGR_8888_LE, ALLEGRO_LOCK_WRITEONLY);
                if(lock != NULL){
                    uint8_t *data = lock->data;
                    uint8_t *eventRR = state->eventRenderer;
                    for (int y = 0; y < state->eventRendererSizeY; y++) {
                        memcpy(data, eventRR, state->eventRendererSizeX * sizeof(uint32_t));
                        
                        data += lock->pitch;
                        eventRR += state->eventRendererSizeX * sizeof(uint32_t);
                    }
                //unlock                    
                al_unlock_bitmap(state->bb); 
                // update display
                al_flip_display();       
                }
			}

			// Render latest frame.
			if (renderFrame) {
				// Write statistics text.
				caerStatisticsStringUpdate((caerEventPacketHeader) frame, &state->frameStatistics);

                //lock bitmap
                al_set_target_bitmap(state->bbframes); 
                ALLEGRO_LOCKED_REGION * lock;
                lock = al_lock_bitmap(state->bbframes, ALLEGRO_PIXEL_FORMAT_ABGR_8888_LE, ALLEGRO_LOCK_WRITEONLY);
                if(lock != NULL){
                    uint8_t *data = lock->data;
                    uint8_t *eventRR = state->frameRenderer;
                    for (int y = 0; y < state->eventRendererSizeY; y++) {
                        memcpy(data, eventRR, state->frameRendererSizeX * sizeof(uint16_t));
                        
                        data += lock->pitch;
                        eventRR += state->frameRendererSizeX * sizeof(uint16_t);
                    }
                //unlock                    
                al_unlock_bitmap(state->bbframes);  
                // update display
                al_flip_display();      
                }

				switch (state->frameChannels) {
					case GRAYSCALE:
						break;

					case RGB:
						break;

					case RGBA:
						break;
				}
			}

		}

		state->subsampleCount = 1;
	}
	
	
	else {
		state->subsampleCount++;
	}
	
	
}

static bool allocateEventRenderer(visualizerAllegroState state, int16_t sourceID) {
	// Get size information from source.
	sshsNode sourceInfoNode = caerMainloopGetSourceInfo((uint16_t) sourceID);
	int16_t sizeX = sshsNodeGetShort(sourceInfoNode, "dvsSizeX");
	int16_t sizeY = sshsNodeGetShort(sourceInfoNode, "dvsSizeY");

	state->eventRenderer = calloc((size_t) (sizeX * sizeY), sizeof(uint32_t));
	if (state->eventRenderer == NULL) {
		return (false); // Failure.
	}

	// Assign maximum sizes for event renderer.
	state->eventRendererSizeX = sizeX;
	state->eventRendererSizeY = sizeY;

	return (true);
}

static bool allocateFrameRenderer(visualizerAllegroState state, int16_t sourceID) {
	// Get size information from source.
	sshsNode sourceInfoNode = caerMainloopGetSourceInfo((uint16_t) sourceID);
	int16_t sizeX = sshsNodeGetShort(sourceInfoNode, "apsSizeX");
	int16_t sizeY = sshsNodeGetShort(sourceInfoNode, "apsSizeY");

	state->frameRenderer = calloc((size_t) (sizeX * sizeY * MAX_CHANNELS), sizeof(uint16_t));
	if (state->frameRenderer == NULL) {
		return (false); // Failure.
	}

	// Assign maximum sizes and defaults for frame renderer.
	state->frameRendererSizeX = sizeX;
	state->frameRendererSizeY = sizeY;
	state->frameRendererPositionX = 0;
	state->frameRendererPositionY = 0;
	state->frameChannels = MAX_CHANNELS;

	return (true);
}
