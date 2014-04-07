#include "file.h"
#include "base/mainloop.h"
#include "base/module.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pwd.h>
#include <time.h>

struct file_state {
	int fileDescriptor;
	bool validOnly;
	bool excludeHeader;
	size_t maxBytesPerPacket;
	struct iovec *sgioMemory;
};

typedef struct file_state *fileState;

static bool caerOutputFileInit(caerModuleData moduleData);
static void caerOutputFileRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerOutputFileConfig(caerModuleData moduleData);
static void caerOutputFileExit(caerModuleData moduleData);

static struct caer_module_functions caerOutputFileFunctions = { .moduleInit = &caerOutputFileInit, .moduleRun =
	&caerOutputFileRun, .moduleConfig = &caerOutputFileConfig, .moduleExit = &caerOutputFileExit };

void caerOutputFile(uint16_t moduleID, size_t outputTypesNumber, ...) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "FileOutput");

	va_list args;
	va_start(args, outputTypesNumber);
	caerModuleSMv(&caerOutputFileFunctions, moduleData, sizeof(struct file_state), outputTypesNumber, args);
	va_end(args);
}

static char *getUserHomeDirectory(void);
static char *getFullFilePath(const char *directory, const char *prefix);
static void caerOutputFileConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);

// Remember to free strings returned by this.
static char *getUserHomeDirectory(void) {
	// First check the environment for $HOME.
	char *homeVar = getenv("HOME");

	if (homeVar != NULL) {
		char *retVar = strdup(homeVar);
		if (retVar == NULL) {
			caerLog(LOG_CRITICAL, "Unable to allocate memory for user home directory path.");
			return (NULL);
		}

		return (retVar);
	}

	// Else try to get it from the user data storage.
	struct passwd userPasswd;
	struct passwd *userPasswdPtr;
	char userPasswdBuf[2048];

	if (getpwuid_r(getuid(), &userPasswd, userPasswdBuf, sizeof(userPasswdBuf), &userPasswdPtr) == 0) {
		// Success!
		char *retVar = strdup(userPasswd.pw_dir);
		if (retVar == NULL) {
			caerLog(LOG_CRITICAL, "Unable to allocate memory for user home directory path.");
			return (NULL);
		}

		return (retVar);
	}

	// Else just return /tmp as a place to write to.
	char *retVar = strdup("/tmp");
	if (retVar == NULL) {
		caerLog(LOG_CRITICAL, "Unable to allocate memory for user home directory path.");
		return (NULL);
	}

	return (retVar);
}

static char *getFullFilePath(const char *directory, const char *prefix) {
	// First get time suffix string.
	time_t currentTimeEpoch = time(NULL);

	struct tm currentTime;
	localtime_r(&currentTimeEpoch, &currentTime);

	// Following time format uses exactly 19 characters (5 separators,
	// 4 year, 2 month, 2 day, 2 hours, 2 minutes, 2 seconds).
	size_t currentTimeStringLength = 19;
	char currentTimeString[currentTimeStringLength + 1]; // + 1 for terminating NUL byte.
	strftime(currentTimeString, currentTimeStringLength + 1, "%Y-%m-%d_%H:%M:%S", &currentTime);

	if (strcmp(prefix, "") == 0) {
		// If the prefix is the empty string, use a minimal one.
		prefix = DEFAULT_PREFIX;
	}

	// Assemble together: directory/prefix-time.aer2
	size_t filePathLength = strlen(directory) + strlen(prefix) + currentTimeStringLength + 8;
	// 1 for the directory/prefix separating slash, 1 for prefix-time separating
	// dash, 5 for file extension, 1 for terminating NUL byte = +8.

	char *filePath = malloc(filePathLength);
	if (filePath == NULL) {
		caerLog(LOG_CRITICAL, "Unable to allocate memory for full file path.");
		return (NULL);
	}

	snprintf(filePath, filePathLength, "%s/%s-%s.aer2", directory, prefix, currentTimeString);

	return (filePath);
}

static bool caerOutputFileInit(caerModuleData moduleData) {
	fileState state = moduleData->moduleState;

	// First, always create all needed setting nodes, set their default values
	// and add their listeners.
	char *userHomeDir = getUserHomeDirectory();
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "directory", userHomeDir);
	free(userHomeDir);

	sshsNodePutStringIfAbsent(moduleData->moduleNode, "prefix", DEFAULT_PREFIX);

	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "validEventsOnly", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "excludeHeader", false);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "maxBytesPerPacket", 0);

	// Install default listener to signal configuration updates asynchronously.
	sshsNodeAddAttrListener(moduleData->moduleNode, moduleData, &caerOutputFileConfigListener);

	// Generate current file name and open it.
	char *directory = sshsNodeGetString(moduleData->moduleNode, "directory");
	char *prefix = sshsNodeGetString(moduleData->moduleNode, "prefix");
	char *filePath = getFullFilePath(directory, prefix);
	free(directory);
	free(prefix);

	state->fileDescriptor = open(filePath, O_WRONLY | O_CREAT, S_IWUSR | S_IRUSR | S_IRGRP);
	if (state->fileDescriptor < 0) {
		caerLog(LOG_CRITICAL, "Could not create or open output file '%s' for writing. Error: %s (%d).", filePath,
			caerLogStrerror(errno), errno);
		free(filePath);

		return (false);
	}

	caerLog(LOG_DEBUG, "Opened output file '%s' successfully for writing.", filePath);
	free(filePath);

	// Set valid events flag, and allocate memory for scatter/gather IO for it.
	state->validOnly = sshsNodeGetBool(moduleData->moduleNode, "validEventsOnly");
	state->excludeHeader = sshsNodeGetBool(moduleData->moduleNode, "excludeHeader");
	state->maxBytesPerPacket = sshsNodeGetInt(moduleData->moduleNode, "maxBytesPerPacket");

	if (state->validOnly) {
		state->sgioMemory = calloc(IOVEC_SIZE, sizeof(struct iovec));
		if (state->sgioMemory == NULL) {
			caerLog(LOG_ALERT, "Impossible to allocate memory for scatter/gather IO, using memory copy method.");
		}
		else {
			caerLog(LOG_INFO, "Using scatter/gather IO for outputting valid events only.");
		}
	}
	else {
		state->sgioMemory = NULL;
	}

	return (true);
}

static void caerOutputFileRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	fileState state = moduleData->moduleState;

	// For each output argument, write it to the file.
	// Each type has a header first thing, that gives us the length, so we can
	// cast it to that and use this information to correctly interpret it.
	for (size_t i = 0; i < argsNumber; i++) {
		caerEventPacketHeader packetHeader = va_arg(args, caerEventPacketHeader);

		// Only work if there is any content.
		if (packetHeader != NULL) {
			if ((state->validOnly && caerEventPacketHeaderGetEventValid(packetHeader) > 0)
				|| (!state->validOnly && caerEventPacketHeaderGetEventNumber(packetHeader) > 0)) {
				caerOutputCommonSend(packetHeader, state->fileDescriptor, state->sgioMemory, state->validOnly,
					state->excludeHeader, state->maxBytesPerPacket);
			}
		}
	}
}

static void caerOutputFileConfig(caerModuleData moduleData) {
	fileState state = moduleData->moduleState;

	// Get the current value to examine by atomic exchange, since we don't
	// want there to be any possible store between a load/store pair.
	uintptr_t configUpdate = atomic_ops_uint_swap(&moduleData->configUpdate, 0, ATOMIC_OPS_FENCE_NONE);

	if (configUpdate & (0x01 << 0)) {
		// validOnly flag changed.
		bool validOnlyFlag = sshsNodeGetBool(moduleData->moduleNode, "validEventsOnly");

		// Only react if the actual state differs from the wanted one.
		if (state->validOnly != validOnlyFlag) {
			// If we want it, turn it on.
			if (validOnlyFlag) {
				state->validOnly = true;

				state->sgioMemory = calloc(IOVEC_SIZE, sizeof(struct iovec));
				if (state->sgioMemory == NULL) {
					caerLog(LOG_ALERT,
						"Impossible to allocate memory for scatter/gather IO, using memory copy method.");
				}
				else {
					caerLog(LOG_INFO, "Using scatter/gather IO for outputting valid events only.");
				}
			}
			else {
				// Else disable it.
				state->validOnly = false;

				free(state->sgioMemory);
				state->sgioMemory = NULL;
			}
		}
	}

	if (configUpdate & (0x01 << 2)) {
		state->excludeHeader = sshsNodeGetBool(moduleData->moduleNode, "excludeHeader");
		state->maxBytesPerPacket = sshsNodeGetInt(moduleData->moduleNode, "maxBytesPerPacket");
	}

	if (configUpdate & (0x01 << 1)) {
		// Filename related settings changed.
		// Generate new file name and open it.
		char *directory = sshsNodeGetString(moduleData->moduleNode, "directory");
		char *prefix = sshsNodeGetString(moduleData->moduleNode, "prefix");
		char *filePath = getFullFilePath(directory, prefix);
		free(directory);
		free(prefix);

		int newFileDescriptor = open(filePath, O_WRONLY | O_CREAT, S_IWUSR | S_IRUSR | S_IRGRP);
		if (newFileDescriptor < 0) {
			caerLog(LOG_CRITICAL, "Could not create or open output file '%s' for writing. Error: %s (%d).", filePath,
				caerLogStrerror(errno), errno);
			free(filePath);

			return;
		}

		caerLog(LOG_DEBUG, "Opened output file '%s' successfully for writing.", filePath);
		free(filePath);

		// New fd ready and opened, close old and set new.
		close(state->fileDescriptor);
		state->fileDescriptor = newFileDescriptor;
	}
}

static void caerOutputFileExit(caerModuleData moduleData) {
	fileState state = moduleData->moduleState;

	// Close open file.
	close(state->fileDescriptor);

	// Make sure to free scatter/gather IO memory.
	free(state->sgioMemory);
	state->sgioMemory = NULL;
}

static void caerOutputFileConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);
	UNUSED_ARGUMENT(changeValue);

	caerModuleData data = userData;

	// Distinguish changes to the validOnly flag or to the filename, by setting
	// configUpdate appropriately like a bit-field.
	if (event == ATTRIBUTE_MODIFIED) {
		if (changeType == BOOL && strcmp(changeKey, "validEventsOnly") == 0) {
			atomic_ops_uint_or(&data->configUpdate, (0x01 << 0), ATOMIC_OPS_FENCE_NONE);
		}

		if (changeType == STRING && (strcmp(changeKey, "directory") == 0 || strcmp(changeKey, "prefix") == 0)) {
			atomic_ops_uint_or(&data->configUpdate, (0x01 << 1), ATOMIC_OPS_FENCE_NONE);
		}

		if ((changeType == BOOL && strcmp(changeKey, "excludeHeader") == 0)
			|| (changeType == INT && strcmp(changeKey, "maxBytesPerPacket") == 0)) {
			atomic_ops_uint_or(&data->configUpdate, (0x01 << 2), ATOMIC_OPS_FENCE_NONE);
		}
	}
}
