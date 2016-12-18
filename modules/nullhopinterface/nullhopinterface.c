/* NullHop Zynq Interface cAER module
 *  Author: federico.corradi@inilabs.com
 */
#include "main.h"
#include <libcaer/events/frame.h>

#include "nullhopinterface.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "wrapper.h"
#include <sys/types.h>
#include <sys/wait.h>

struct nullhopwrapper_state {
	double detThreshold;
	struct MyClass* cpp_class; //pointer to cpp_class_object
};

typedef struct nullhopwrapper_state *nullhopwrapperState;

static bool caerNullHopWrapperInit(caerModuleData moduleData);
static void caerNullHopWrapperRun(caerModuleData moduleData, size_t argsNumber,
		va_list args);
static void caerNullHopWrapperExit(caerModuleData moduleData);

static struct caer_module_functions caerNullHopWrapperFunctions = {
		.moduleInit = &caerNullHopWrapperInit, .moduleRun =
				&caerNullHopWrapperRun, .moduleConfig =
		NULL, .moduleExit = &caerNullHopWrapperExit };

const char * caerNullHopWrapper(uint16_t moduleID,
		int * imagestreamer, bool * haveimg, int* result) {

	caerModuleData moduleData = caerMainloopFindModule(moduleID,
			"caerNullHopWrapper", CAER_MODULE_PROCESSOR);
	caerModuleSM(&caerNullHopWrapperFunctions, moduleData,
			sizeof(struct nullhopwrapper_state), 3, imagestreamer, haveimg, result);

	return (NULL);
}

static bool caerNullHopWrapperInit(caerModuleData moduleData) {

	nullhopwrapperState state = moduleData->moduleState;
	sshsNodePutDoubleIfAbsent(moduleData->moduleNode, "detThreshold", 0.5);
	state->detThreshold = sshsNodeGetDouble(moduleData->moduleNode,
			"detThreshold");

	//Initializing nullhop network..
	state->cpp_class = newzs_driver("modules/nullhopinterface/nets/roshamboNet_v3.nhp");

	return (true);
}

static void caerNullHopWrapperExit(caerModuleData moduleData) {
	nullhopwrapperState state = moduleData->moduleState;

	//zs_driverMonitor_closeThread(state->cpp_class); // join
	//deleteMyClass(state->cpp_class); //free memory block
}

static void caerNullHopWrapperRun(caerModuleData moduleData, size_t argsNumber,
		va_list args) {
	UNUSED_ARGUMENT(argsNumber);
	int * imagestreamer_hists = va_arg(args, int*);
	bool * haveimg = va_arg(args, bool*);
	int * result = va_arg(args, int*);

	if (imagestreamer_hists == NULL) {
		return;
	}

	nullhopwrapperState state = moduleData->moduleState;

	//update module state
	state->detThreshold = sshsNodeGetDouble(moduleData->moduleNode,
			"detThreshold");

	if(haveimg[0] == true){

		result[0] = zs_driver_classify_image(state->cpp_class, imagestreamer_hists) + 1;
	}

	return;
}
