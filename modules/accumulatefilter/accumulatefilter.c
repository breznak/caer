#include "accumulatefilter.h"
#include "base/mainloop.h"
#include "base/module.h"

// define state/internal vars of the filter
struct AccFilter_state {
        // time packet
	int64_t close; //close time for current packet
        caerPolarityEventPacket *curr;
        caerPolarityEventPacket *next; //backlog packet
	int32_t deltaT; // how often a clock sync is generated (new packet released),in ms
        int lastTsOverflow_; // helper keeping prev tsOverflow for comparison if those changed 
        // 2D buffer
	simple2DBufferByte buff2D;
        polarity_t mode; // config parameter
        // 1D buffer        
        int32_t buff1dMax;
        int64_t *buff1D;
        // etc
        bool initialized = false; // the filter is fully initialized (after 1st run() call)
        bool release = false; // the new packet should be released (either clock or other reasons)
};

typedef struct AccFilter_state *AccFilterState;

// required methods
static bool caerAccumulateFilterInit(caerModuleData moduleData);
static void caerAccumulateFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerAccumulateFilterConfig(caerModuleData moduleData);
static void caerAccumulateFilterExit(caerModuleData moduleData);
//helpers
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
        sshsNodePutStringIfAbsent(moduleData->moduleNode, "mode", "both");


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
        if(!state->initialized) { //FIXME this should be in Init() if I know packet header at the time
          // Get size information from source.
          uint16_t sourceID = caerEventPacketHeaderGetEventSource(&polarity->packetHeader);
          sshsNode sourceInfoNode = caerMainloopGetSourceInfo((uint16_t) sourceID);
          // allocate 2D buffer
          int16_t sizeX = sshsNodeGetShort(sourceInfoNode, "dvsSizeX");
          int16_t sizeY = sshsNodeGetShort(sourceInfoNode, "dvsSizeY");
          state->buff2D = simple2DBufferInitByte((size_t) sizeX, (size_t) sizeY);
          // allocate new helper event packets
	  int maxEvtsSize = 50000; //FIXME must be large enough, we want to send packet ourselves on correct clock signal, not when the packet is full (which happens automatically)
      	  int source = moduleData->moduleID; // this module created this new packet
      	  int tsOverflow = caerEventPacketHeaderGetEventTSOverflow(&polarity->packetHeader);
          state->lastTsOverflow_ = tsOverflow;
      	  state->curr = caerPolarityEventPacketAllocate(I32T(maxEvtsSize), I16T(source), I32T(tsOverflow)); //TODO use growing event packets when implemented (no need for maxEvtsSize)
      	  state->next = caerPolarityEventPacketAllocate(I32T(maxEvtsSize), I16T(source), I32T(tsOverflow));
          // initialization finished
          state->initialized = true;
        }

        // check if timestamp overflowed (we must release the packet, as all new timestamps are invalid)
        int tsOverflow = caerEventPacketHeaderGetEventTSOverflow(&polarity->packetHeader);
        if (state->lastTsOverflow_ != tsOverflow) {
 	  state->release = true; 
	  state->lastTsOverflow_ = tsOverflow;
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
        state->buff1dMax = sshsNodeGetInt(moduleData->moduleNode, "buff1dMax");
        char *str = sshNodeGetString(moduleData->moduleNode, "mode"); 
        // parse mode:
        if (caerStrEquals(str, "on")) {
		 state->mode = POLARITY_ON;
        } else if (caerStrEquals(str, "off")) {
                 state->mode = POLARITY_OFF;
        } else if (caerStrEquals(str, "both")) {
                 state->mode = POLARITY_BOTH;
        } else if (caerStrEquals(str, "replace")) {
                 state->mode = POLARITY_REPLACE;
        } else {
                caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
			"Invalid AccumulateFilter.mode! Use one of on, off, both, replace. ");
        	state->mode = POLARITY_BOTH;
	}

      // Allocate 1D buff if needed
      if (state->buff1D == NULL) {
        state->buff1D = calloc((size_t) state->buff1dMax, sizeof(int64_t *));
        if (state->buff1D == NULL) {
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for buff1D.");
                        return;
        }
      }
}

static void caerAccumulateFilterExit(caerModuleData moduleData) {
	AccFilterState state = moduleData->moduleState;
	// Ensure 2D buff is freed.
        simple2DBufferFreeByte(state->buff2D);
        // free 1D buff
        if (state->buff1D != NULL) {
          free(state->buff1D);
          state->buff1D = NULL;
        }
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
