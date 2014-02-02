/*
 * module.c
 *
 *  Created on: Dec 14, 2013
 *      Author: chtekk
 */

#include "module.h"
#include <pthread.h> // For pthread_exit(), since this all happens inside threads.

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
	uintptr_t running = atomic_ops_uint_load(&moduleData->running, ATOMIC_OPS_FENCE_NONE);

	if (moduleData->moduleStatus == RUNNING && running == 1) {
		if (atomic_ops_uint_load(&moduleData->configUpdate, ATOMIC_OPS_FENCE_NONE) != 0) {
			if (moduleFunctions->moduleConfig != NULL) {
				// Call config function, which will have to reset configUpdate.
				moduleFunctions->moduleConfig(moduleData);
			}
		}

		moduleFunctions->moduleRun(moduleData, argsNumber, args);
	}
	else if (moduleData->moduleStatus == STOPPED && running == 1) {
		moduleData->moduleState = calloc(1, memSize);
		if (moduleData->moduleState == NULL && memSize != 0) {
			return;
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
	else if (moduleData->moduleStatus == RUNNING && running == 0) {
		moduleData->moduleStatus = STOPPED;

		if (moduleFunctions->moduleExit != NULL) {
			moduleFunctions->moduleExit(moduleData);
		}

		free(moduleData->moduleState);
		moduleData->moduleState = NULL;
	}
}

caerModuleData caerModuleInitialize(uint16_t moduleID, const char *moduleShortName, sshsNode mainloopNode) {
	// Allocate memory for the module.
	caerModuleData moduleData = calloc(1, sizeof(struct caer_module_data));
	if (moduleData == NULL) {
		caerLog(LOG_CRITICAL, "Failed to allocate memory for module %" PRIu16 "-%s. Error: %s (%d).", moduleID,
			moduleShortName, caerLogStrerror(errno), errno);
		pthread_exit(NULL);
	}

	// Set module ID for later identification (hash-table key).
	moduleData->moduleID = moduleID;

	// Put module into startup state.
	moduleData->moduleStatus = STOPPED;
	atomic_ops_uint_store(&moduleData->running, 1, ATOMIC_OPS_FENCE_FULL);

	// Determine SSHS module node. Use short name for better human recognition.
	size_t printLength = (size_t) snprintf(NULL, 0, "%" PRIu16 "-%s/", moduleID, moduleShortName);
	char modString[printLength + 1];
	snprintf(modString, printLength + 1, "%" PRIu16 "-%s/", moduleID, moduleShortName);

	// Initialize configuration, shutdown hooks.
	moduleData->moduleNode = sshsGetRelativeNode(mainloopNode, modString);
	if (moduleData->moduleNode == NULL) {
		caerLog(LOG_CRITICAL, "Failed to allocate configuration node for module '%s'.", modString);
		pthread_exit(NULL);
	}

	sshsNodePutBool(moduleData->moduleNode, "shutdown", false); // Always reset to false.
	sshsNodeAddAttrListener(moduleData->moduleNode, moduleData, &caerModuleShutdownListener);

	return (moduleData);
}

void caerModuleDestroy(caerModuleData moduleData) {
	// Deallocate module memory. Module state has already been destroyed.
	free(moduleData);
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
		atomic_ops_uint_store(&data->configUpdate, 1, ATOMIC_OPS_FENCE_NONE);
	}
}

static void caerModuleShutdownListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	caerModuleData data = userData;

	if (event == ATTRIBUTE_MODIFIED && changeType == BOOL && strcmp(changeKey, "shutdown") == 0) {
		// Shutdown changed, let's see.
		if (changeValue.boolean == true) {
			atomic_ops_uint_store(&data->running, 0, ATOMIC_OPS_FENCE_NONE);
		}
		else {
			atomic_ops_uint_store(&data->running, 1, ATOMIC_OPS_FENCE_NONE);
		}
	}
}
