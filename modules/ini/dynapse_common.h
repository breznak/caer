#ifndef DYNAPSE_COMMON_H_
#define DYNAPSE_COMMON_H_

#include "main.h"
#include "base/mainloop.h"
#include "base/module.h"

#include <limits.h>

#ifdef HAVE_PTHREADS
#include "ext/c11threads_posix.h"
#endif

#include <math.h>
#include <stdatomic.h>
#include <sys/types.h>
#include <libcaer/devices/dynapse.h>

struct gen_spike_state {
	atomic_bool doStim;
	atomic_int_fast32_t stim_type;
	atomic_int_fast32_t stim_avr;				// Hertz [1/s]
	atomic_int_fast32_t stim_std;				//
	atomic_int_fast32_t stim_duration;
	atomic_bool repeat;
	atomic_bool teaching;
	atomic_bool sendTeachingStimuli;
	atomic_bool sendInhibitoryStimuli;
	atomic_bool setCam;
	atomic_bool setCamSingle;
	atomic_bool clearCam;
	atomic_bool clearAllCam;
	atomic_bool doStimPrimitiveBias;
	atomic_bool doStimPrimitiveCam;
	atomic_bool loadDefaultBiases;
	atomic_bool done;
	atomic_bool started;
	thrd_t spikeGenThread;
	atomic_bool running;
	/*address spike*/
	atomic_int_fast32_t core_d;
	atomic_int_fast32_t address;
	atomic_int_fast32_t core_s;
	atomic_int_fast32_t chip_id;
	atomic_int_fast32_t dx;
	atomic_int_fast32_t dy;
	atomic_bool sx;
	atomic_bool sy;
	/* ETF */
	// stimulation Thread ETF
	atomic_bool ETFstarted;
	atomic_bool ETFdone;
	atomic_int_fast32_t ETFchip_id;		// the chip that will be measured [0,4,8,12]
	atomic_int_fast32_t ETFduration;	// total stimulation duration
	atomic_int_fast32_t ETFphase_num;	// stimulation phase number
	atomic_bool ETFrepeat;
	int ETFstepnum;
};

struct caer_input_dynapse_state {
	caerDeviceHandle deviceState;
	sshsNode eventSourceConfigNode;
	struct gen_spike_state genSpikeState;
};

typedef struct caer_input_dynapse_state *caerInputDynapseState;

bool caerInputDYNAPSEInit(caerModuleData moduleData);
void caerInputDYNAPSEExit(caerModuleData moduleData);
void caerInputDYNAPSERun(caerModuleData moduleData, size_t argsNumber, va_list args);
const char *chipIDToName(int16_t chipID, bool withEndSlash);

void caerDynapseSetBias(caerInputDynapseState state, uint32_t chipId, uint32_t coreId, const char *biasName_t,
	uint8_t coarseValue, uint16_t fineValue, const char *lowHigh, const char *npBias);
uint32_t generatesBitsCoarseFineBiasSetting(sshsNode node, const char *biasName, uint8_t coarseValue,
	uint16_t fineValue, const char *hlbias, const char *currentLevel, const char *sex,
	bool enabled, int chipid);
bool setCamContent(caerInputDynapseState state, int16_t chipId, bool ei, bool fs, int16_t address, int8_t source_core,
	int8_t coreId, int16_t row, int16_t column);
bool caerGenSpikeInit(caerModuleData moduleData);
void caerDynapseSetBias(caerInputDynapseState state, uint32_t chipId, uint32_t coreId, const char *biasName_t,
	uint8_t coarseValue, uint16_t fineValue, const char *lowHigh, const char *npBias);

#endif /* DYNAPSE_COMMON_H_ */
