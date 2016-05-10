#include "accumulatefilter.h"
#include "base/mainloop.h"
#include "base/module.h"

// define state/internal vars of the filter
struct AccFilter_state {
        // time packet
	int64_t close; //close time for current packet
        caerPolarityEventPacket *curr;
        caerPolarityEventPacket *next; //backlog packet
	int32_t deltaT;
        // 2D buffer
	int16_t buff2dMaxX;
	int16_t buff2dMaxY;
	int64_t **buff2D;
        polarity_t mode;
        // 1D buffer        
        int32_t buff1dMax;
        int64_t *buff1D;
};

typedef struct AccFilter_state *AccFilterState;

// required methods
static bool caerAccumulateFilterInit(caerModuleData moduleData);
static void caerAccumulateFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerAccumulateFilterConfig(caerModuleData moduleData);
static void caerAccumulateFilterExit(caerModuleData moduleData);
//helpers
static bool allocate2DBuffer(AccFilterState state, int16_t sizeX, int16_t sizeY);
static void transform1D(int64_t* vector1D, uint16_t x, uint16_t y, bool p);
static void processEvent(AccFilterState state, caerPolarityEvent evt, caerPolarityEventPacket packet);

// map functions to filter handlers
static struct caer_module_functions caerAccumulateFilterFunctions = { 
	.moduleInit = &caerAccumulateFilterInit, 
	.moduleRun = &caerAccumulateFilterRun, 
	.moduleConfig = &caerAccumulateFilterConfig, 
	.moduleExit = &caerAccumulateFilterExit };

// this is called from mainloop
void caerAccumulateFilter(uint16_t moduleID, caerPolarityEventPacket polarity) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "AccFilter");
	caerModuleSM(&caerAccumulateFilterFunctions, moduleData, sizeof(struct AccFilter_state), 1, polarity);
}

static bool caerAccumulateFilterInit(caerModuleData moduleData) {
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "deltaT", 1000);
        sshsNodePutIntIfAbsent(moduleData->moduleNode, "buff1dMax", 1000); //depends on transform() function

	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

        caerAccumulateFilterConfig(moduleData);
	// Nothing that can fail here.
	return (true);
}

static void caerAccumulateFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerPolarityEventPacket polarity = va_arg(args, caerPolarityEventPacket);

	// Only process packets with content.
	if (polarity == NULL) {
		return;
	}

	AccFilterState state = moduleData->moduleState;

        // update camera dimensions
        if(state->buff2dMaxX == -1 || state->buff2dMaxY == -1) { //FIXME this should be in Init() if I know "dvsSizeX" at the time
          // Get size information from source.
          uint16_t sourceID = caerEventPacketHeaderGetEventSource(&polarity->packetHeader);
          sshsNode sourceInfoNode = caerMainloopGetSourceInfo((uint16_t) sourceID);
          state->buff2dMaxX = sshsNodeGetShort(sourceInfoNode, "dvsSizeX");
          state->buff2dMaxY = sshsNodeGetShort(sourceInfoNode, "dvsSizeY");
          free(sourceInfoNode); sourceInfoNode = NULL;
          // allocate 2D buffer
          if (state->buff2D == NULL && (state->buff2dMaxX != -1 || state->buff2dMaxY != -1)) {
                if (!allocate2DBuffer(state, state->buff2dMaxX, state->buff2dMaxY)) {
                        // Failed to allocate memory, nothing to do.
                        caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for buff2D.");
                        return;
                }
          }
        }

        CAER_POLARITY_ITERATOR_VALID_START(state->next) //process all evts from previous next
                processEvent(state, caerPolarityIteratorElement, state->next);
                caerPolarityEventInvalidate(caerPolarityIteratorElement, state->next);
        CAER_POLARITY_ITERATOR_VALID_END

	CAER_POLARITY_ITERATOR_VALID_START(polarity)
		processEvent(state, caerPolarityIteratorElement, polarity);
	CAER_POLARITY_ITERATOR_VALID_END
}

static void caerAccumulateFilterConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	AccFilterState state = moduleData->moduleState;

        state->close = -1;
	state->deltaT = sshsNodeGetInt(moduleData->moduleNode, "deltaT");
        state->buff2dMaxX = -1; // -1 means will be set to camera dimensions automatically
        state->buff2dMaxY = -1;
        state->buff1dMax = sshsNodeGetInt(moduleData->moduleNode, "buff1dMax"); 
        state->mode = POLARITY_ON; //TODO add config node for this

        // If the 2D buff is not allocated yet, do it.
        // ... must be done in run() as sizes are dymanically set from packet header //FIXME can I get camera dims already here?, would be nicer

      // Allocate 1D buff if needed
      if (state->buff1D == NULL) {
        state->buff1D = calloc((size_t) state->buff1dMax, sizeof(int64_t *));
        if (state->buff1D == NULL) {
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for buff1D.");
                        return;
        }
      }

      //FIXME how to create my own caerPolarityEventPacket? and add an event to it?; WHAT should the constants be?
      state->curr = caerPolarityEventPacketAllocate(I32T(5000), I16T(1), I32T(0));
      state->next = caerPolarityEventPacketAllocate(I32T(5000), I16T(1), I32T(0));


}

static void caerAccumulateFilterExit(caerModuleData moduleData) {
	AccFilterState state = moduleData->moduleState;
	// Ensure 2D buff is freed.
	if (state->buff2D != NULL) {
		free(state->buff2D[0]);
		free(state->buff2D);
		state->buff2D = NULL;
	}
        // free 1D buff
        if (state->buff1D != NULL) {
          free(state->buff1D);
          state->buff1D = NULL;
        }
}

// helper method to initialize 2D array with dimX,dimY sizes of the camera
static bool allocate2DBuffer(AccFilterState state, int16_t sizeX, int16_t sizeY) {
	// Initialize double-indirection contiguous 2D array, so that array[x][y]
	// is possible, see http://c-faq.com/aryptr/dynmuldimary.html for info. //FIXME rather use a library for this?!
	state->buff2D = calloc((size_t) sizeX, sizeof(int64_t *));
	if (state->buff2D == NULL) {
		return (false); // Failure.
	}

	state->buff2D[0] = calloc((size_t) (sizeX * sizeY), sizeof(int64_t));
	if (state->buff2D[0] == NULL) {
		free(state->buff2D);
		state->buff2D = NULL;

		return (false); // Failure.
	}

	for (size_t i = 1; i < (size_t) sizeX; i++) {
		state->buff2D[i] = state->buff2D[0] + (i * (size_t) sizeY);
	}

	// Assign max ranges for arrays (0 to MAX-1).
	state->buff2dMaxX = (sizeX - 1);
	state->buff2dMaxY = (sizeY - 1);
	return (true);
}

// transform (2D) events to 1D vector, keeping Euclidian similarity
static void transform1D(int64_t* vector1D, uint16_t x, uint16_t y, bool p) {
  //TODO implement transformation(s)
  vector1D[501] = p?1:0;
}

// process single event
static void processEvent(AccFilterState state, caerPolarityEvent evt, caerPolarityEventPacket packet) {
                // Get values on which to operate.
                int64_t ts = caerPolarityEventGetTimestamp64(evt, packet);
                uint16_t x = caerPolarityEventGetX(evt);
                uint16_t y = caerPolarityEventGetY(evt);
                bool p = caerPolarityEventGetPolarity(evt);

                // update curr packet's close time
                if(state->close == -1) {
                  state->close = ts + state->deltaT;
                }
                // (in)validate evts in polarity packet; not in [start..close]
                if(ts < state->close - state->deltaT) { // < T
                  caerPolarityEventInvalidate(evt, packet); // this should not occur!
                } else if (ts > state->close) { // > T
                  //next.addEvent(); //FIXME how to add event to a packet?
                  caerPolarityEventInvalidate(evt, packet);
                } else { // ok
		  //curr.addEvent(); //FIXME
		}

                // example write to 2D buffer
                if(state->mode == POLARITY_ON && p) { state->buff2D[x][y]=1; }
                else if(state->mode == POLARITY_OFF && !p) { state->buff2D[x][y]=1; }
                else if(state->mode == POLARITY_REPLACE) { state->buff2D[x][y]= p?1:0; }
                else { state->buff2D[x][y]=1; } // BOTH

                // example write to 1D buffer
                transform1D(&state->buff1D, x, y, p);
}
