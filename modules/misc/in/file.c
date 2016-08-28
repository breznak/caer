#include "file.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "input_common.h"
#include <sys/types.h>
#include <fcntl.h>

static bool caerInputFileInit(caerModuleData moduleData);

static struct caer_module_functions caerInputFileFunctions = { .moduleInit = &caerInputFileInit, .moduleRun =
	&caerInputCommonRun, .moduleConfig = NULL, .moduleExit = &caerInputCommonExit };

caerEventPacketContainer caerInputFile(uint16_t moduleID) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "FileInput", CAER_MODULE_INPUT);
	if (moduleData == NULL) {
		return (NULL);
	}

	caerEventPacketContainer result = NULL;

	caerModuleSM(&caerInputFileFunctions, moduleData, CAER_INPUT_COMMON_STATE_STRUCT_SIZE, 1, &result);

	return (result);
}

static bool caerInputFileInit(caerModuleData moduleData) {
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "filePath", "");

	char *filePath = sshsNodeGetString(moduleData->moduleNode, "filePath");

	if (caerStrEquals(filePath, "")) {
		free(filePath);

		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
			"No input file given, please specify the 'filePath' parameter.");
		return (false);
	}

	int fileFd = open(filePath, O_RDONLY);
	if (fileFd < 0) {
		caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString,
			"Could not open input file '%s' for reading. Error: %d.", filePath, errno);
		free(filePath);

		return (false);
	}

	caerLog(CAER_LOG_INFO, moduleData->moduleSubSystemString, "Opened input file '%s' successfully for reading.",
		filePath);
	free(filePath);

	if (!caerInputCommonInit(moduleData, fileFd, false, false)) {
		close(fileFd);

		return (false);
	}

	return (true);
}
