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
	bool doStim;
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
};

struct caer_input_dynapse_state {
	caerDeviceHandle deviceState;
	sshsNode eventSourceConfigNode;
	struct gen_spike_state genSpikeState;
};

typedef struct caer_input_dynapse_state *caerInputDynapseState;

bool caerInputDYNAPSEInit(caerModuleData moduleData, uint16_t deviceType);
void caerInputDYNAPSEExit(caerModuleData moduleData);
void caerInputDYNAPSERun(caerModuleData moduleData, size_t argsNumber, va_list args);
const char *chipIDToName(int16_t chipID, bool withEndSlash);

static uint32_t convertBias(const char *biasName, const char* lowhi, const char*cl, const char*sex, uint8_t enal,
	uint16_t fineValue, uint8_t coarseValue, uint8_t special);
static uint32_t generateCoarseFineBias(sshsNode biasNode);
static void systemConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void usbConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void spikeConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void sramConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void camConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void usbConfigSend(sshsNode node, caerModuleData moduleData);
static void biasConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void updateLowPowerBiases(caerModuleData moduleData, int chipid);
static void updateSilentBiases(caerModuleData moduleData, int chipid);
static char *int2bin(int a);
static bool EnableStimuliGen(caerModuleData moduleData);
static bool DisableStimuliGen(caerModuleData moduleData, int16_t eventSourceID);
void caerDynapseSetBias(caerInputDynapseState state, uint32_t chipId, uint32_t coreId, const char *biasName_t,
		uint8_t coarseValue, uint16_t fineValue, const char *lowHigh, const char *npBias);
uint32_t generatesBitsCoarseFineBiasSetting(sshsNode node, const char *biasName,
	uint8_t coarseValue, uint16_t fineValue, const char *hlbias, const char *currentLevel, const char *sex,
	bool enabled, int chipid);

#endif /* DYNAPSE_COMMON_H_ */
