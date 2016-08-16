/*
 * module.c
 *
 *  Created on: Dec 14, 2013
 *      Author: chtekk
 */

#include "module.h"

static void caerModuleShutdownListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);

void caerModuleSM(caerModuleFunctions moduleFunctions, caerModuleData moduleData, size_t memSize, size_t argsNumber,
	...) {
	va_list args;
	va_start(args, argsNumber);
	caerModuleSMv(moduleFunctions, moduleData, memSize, argsNumber, args);
	va_end(args);
}

void caerModuleSMv(caerModuleFunctions moduleFunctions, caerModuleData moduleData, size_t memSize, size_t argsNumber,
	va_list args) {
	bool running = atomic_load_explicit(&moduleData->running, memory_order_relaxed);

	if (moduleData->moduleStatus == RUNNING && running) {
		if (atomic_load_explicit(&moduleData->configUpdate, memory_order_relaxed) != 0) {
			if (moduleFunctions->moduleConfig != NULL) {
				// Call config function, which will have to reset configUpdate.
				moduleFunctions->moduleConfig(moduleData);
			}
		}

		if (moduleFunctions->moduleRun != NULL) {
			moduleFunctions->moduleRun(moduleData, argsNumber, args);
		}

		if (atomic_load_explicit(&moduleData->doReset, memory_order_relaxed) != 0) {
			if (moduleFunctions->moduleReset != NULL) {
				// Call reset function. 'doReset' variable reset is done here.
				uint32_t resetCallSourceID = U32T(atomic_exchange(&moduleData->doReset, 0));
				moduleFunctions->moduleReset(moduleData, U16T(resetCallSourceID));
			}
		}
	}
	else if (moduleData->moduleStatus == STOPPED && running) {
		if (memSize != 0) {
			moduleData->moduleState = calloc(1, memSize);
			if (moduleData->moduleState == NULL) {
				return;
			}
		}
		else {
			// memSize is zero, so moduleState must be NULL.
			moduleData->moduleState = NULL;
		}

		if (moduleFunctions->moduleInit != NULL) {
			if (!moduleFunctions->moduleInit(moduleData)) {
				free(moduleData->moduleState);
				moduleData->moduleState = NULL;

				return;
			}
		}

		moduleData->moduleStatus = RUNNING;
	}
	else if (moduleData->moduleStatus == RUNNING && !running) {
		moduleData->moduleStatus = STOPPED;

		if (moduleFunctions->moduleExit != NULL) {
			moduleFunctions->moduleExit(moduleData);
		}

		free(moduleData->moduleState);
		moduleData->moduleState = NULL;
	}
}

caerModuleData caerModuleInitialize(uint16_t moduleID, const char *moduleShortName, sshsNode mainloopNode) {
	// Generate short module name with ID, reused in all error messages and later code.
	size_t nameLength = (size_t) snprintf(NULL, 0, "%" PRIu16 "-%s", moduleID, moduleShortName);
	char nameString[nameLength + 1];
	snprintf(nameString, nameLength + 1, "%" PRIu16 "-%s", moduleID, moduleShortName);

	// Allocate memory for the module.
	caerModuleData moduleData = calloc(1, sizeof(struct caer_module_data));
	if (moduleData == NULL) {
		caerLog(CAER_LOG_ALERT, nameString, "Failed to allocate memory for module. Error: %d.", errno);
		return (NULL);
	}

	// Set module ID for later identification (hash-table key).
	moduleData->moduleID = moduleID;

	// Put module into startup state. 'running' flag is updated later based on user startup wishes.
	moduleData->moduleStatus = STOPPED;

	// Determine SSHS module node. Use short name for better human recognition.
	char sshsString[nameLength + 2];
	strncpy(sshsString, nameString, nameLength);
	sshsString[nameLength] = '/';
	sshsString[nameLength + 1] = '\0';

	// Initialize configuration.
	moduleData->moduleNode = sshsGetRelativeNode(mainloopNode, sshsString);
	if (moduleData->moduleNode == NULL) {
		free(moduleData);

		caerLog(CAER_LOG_ALERT, nameString, "Failed to allocate configuration node for module.");
		return (NULL);
	}

	// Setup default full log string name.
	moduleData->moduleSubSystemString = malloc(nameLength + 1);
	if (moduleData->moduleSubSystemString == NULL) {
		free(moduleData);

		caerLog(CAER_LOG_ALERT, nameString, "Failed to allocate subsystem string for module.");
		return (NULL);
	}

	strncpy(moduleData->moduleSubSystemString, nameString, nameLength);
	moduleData->moduleSubSystemString[nameLength] = '\0';

	// Initialize shutdown controls. By default modules always run.
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "runAtStartup", true); // Allow for users to disable a module at start.
	bool runModule = sshsNodeGetBool(moduleData->moduleNode, "runAtStartup");

	atomic_store_explicit(&moduleData->running, runModule, memory_order_relaxed);
	sshsNodePutBool(moduleData->moduleNode, "running", runModule);
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleShutdownListener);

	atomic_thread_fence(memory_order_release);

	return (moduleData);
}

void caerModuleDestroy(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleShutdownListener);

	// Deallocate module memory. Module state has already been destroyed.
	free(moduleData->moduleSubSystemString);
	free(moduleData);
}

bool caerModuleSetSubSystemString(caerModuleData moduleData, const char *subSystemString) {
	// Allocate new memory for new string.
	size_t subSystemStringLenght = strlen(subSystemString);

	char *newSubSystemString = malloc(subSystemStringLenght + 1);
	if (newSubSystemString == NULL) {
		// Failed to allocate memory. Log this and don't use the new string.
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
			"Failed to allocate new sub-system string for module.");
		return (false);
	}

	// Copy new string into allocated memory.
	strncpy(newSubSystemString, subSystemString, subSystemStringLenght);
	newSubSystemString[subSystemStringLenght] = '\0';

	// Switch new string with old string and free old memory.
	free(moduleData->moduleSubSystemString);
	moduleData->moduleSubSystemString = newSubSystemString;

	return (true);
}

void caerModuleConfigUpdateReset(caerModuleData moduleData) {
	atomic_store(&moduleData->configUpdate, 0);
}

void caerModuleConfigDefaultListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);
	UNUSED_ARGUMENT(changeKey);
	UNUSED_ARGUMENT(changeType);
	UNUSED_ARGUMENT(changeValue);

	caerModuleData data = userData;

	// Simply set the config update flag to 1 on any attribute change.
	if (event == ATTRIBUTE_MODIFIED) {
		atomic_store(&data->configUpdate, 1);
	}
}

static void caerModuleShutdownListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	caerModuleData data = userData;

	if (event == ATTRIBUTE_MODIFIED && changeType == BOOL && caerStrEquals(changeKey, "running")) {
		// Running changed, let's see.
		if (changeValue.boolean == true) {
			atomic_store(&data->running, true);
		}
		else {
			atomic_store(&data->running, false);
		}
	}
}
