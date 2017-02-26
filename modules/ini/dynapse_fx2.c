#include "dynapse_common.h"
#include "dynapse_fx2.h"

static bool caerInputDYNAPSEFX2Init(caerModuleData moduleData);

static struct caer_module_functions caerInputDYNAPSEFX2Functions = { .moduleInit = &caerInputDYNAPSEFX2Init, .moduleRun =
	&caerInputDYNAPSERun, .moduleConfig = NULL, .moduleExit = &caerInputDYNAPSEExit };

caerEventPacketContainer caerInputDYNAPSEFX2(uint16_t moduleID) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "DYNAPSEFX2", CAER_MODULE_INPUT);
	if (moduleData == NULL) {
		return (NULL);
	}

	caerEventPacketContainer result = NULL;

	caerModuleSM(&caerInputDYNAPSEFX2Functions, moduleData, sizeof(struct caer_input_dynapse_state), 1, &result);

	return (result);
}

static bool caerInputDYNAPSEFX2Init(caerModuleData moduleData) {
	return (caerInputDYNAPSEInit(moduleData));
}
