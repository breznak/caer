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
	atomic_bool done;
	atomic_bool started;
	thrd_t spikeGenThread;
	atomic_bool running;
};



struct caer_input_dynapse_state {
	caerDeviceHandle deviceState;
	struct gen_spike_state genSpikeState;
};

typedef struct caer_input_dynapse_state *caerInputDynapseState;

bool caerInputDYNAPSEInit(caerModuleData moduleData, uint16_t deviceType);
void caerInputDYNAPSEExit(caerModuleData moduleData);
void caerInputDYNAPSERun(caerModuleData moduleData, size_t argsNumber, va_list args);

#endif /* DYNAPSE_COMMON_H_ */
