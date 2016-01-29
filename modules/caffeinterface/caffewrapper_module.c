/* Caffe Interface cAER module
*  Author: federico.corradi@inilabs.com
*/

#include "base/mainloop.h"
#include "base/module.h"
#include "wrapper.h"

struct caffewrapper_state {
	uint32_t *integertest;
	char * file_to_classify;
	struct MyClass* cpp_class; //pointer to cpp_class_object
};

typedef struct caffewrapper_state *caffewrapperState;

static bool caerCaffeWrapperInit(caerModuleData moduleData);
static void caerCaffeWrapperRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerCaffeWrapperExit(caerModuleData moduleData);

static struct caer_module_functions caerCaffeWrapperFunctions = { .moduleInit = &caerCaffeWrapperInit, .moduleRun =
	&caerCaffeWrapperRun, .moduleConfig =
NULL, .moduleExit = &caerCaffeWrapperExit };

const char * caerCaffeWrapper(uint16_t moduleID, char ** file_string) {

	caerModuleData moduleData = caerMainloopFindModule(moduleID, "caerCaffeWrapper");
	caerModuleSM(&caerCaffeWrapperFunctions, moduleData, sizeof(struct caffewrapper_state), 1, file_string);

	return;
}

static bool caerCaffeWrapperInit(caerModuleData moduleData) {
	caffewrapperState state = moduleData->moduleState;
	//Initializing caffe network..
	state->cpp_class = newMyClass();
	MyClass_init_network(state->cpp_class);

	return (true);
}

static void caerCaffeWrapperExit(caerModuleData moduleData) {
	caffewrapperState state = moduleData->moduleState;

        deleteMyClass(state->cpp_class); //free memory block
	return (true);
}

static void caerCaffeWrapperRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);
	caffewrapperState state = moduleData->moduleState;
	char ** file_string = va_arg(args, char **);
	//run prediction if we generated a valid image
	if(file_string != NULL){
        	MyClass_file_set(state->cpp_class, *file_string);
	}
	return;
}
