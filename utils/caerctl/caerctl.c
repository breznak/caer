#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include "ext/libuv.h"
#include "ext/nets.h"
#include "ext/sshs/sshs.h"
#include "base/config_server.h"
#include "utils/ext/libuvline/libuvline.h"

#ifdef __APPLE__
#include <sys/syslimits.h>
#endif

static void handleInputLine(const char *buf, size_t bufLength);
static void handleCommandCompletion(const char *buf, size_t bufLength, libuvTTYCompletions autoComplete);

static void actionCompletion(const char *buf, size_t bufLength, libuvTTYCompletions autoComplete,
	const char *partialActionString, size_t partialActionStringLength);
static void nodeCompletion(const char *buf, size_t bufLength, libuvTTYCompletions autoComplete, uint8_t actionCode,
	const char *partialNodeString, size_t partialNodeStringLength);
static void keyCompletion(const char *buf, size_t bufLength, libuvTTYCompletions autoComplete, uint8_t actionCode,
	const char *nodeString, size_t nodeStringLength, const char *partialKeyString, size_t partialKeyStringLength);
static void typeCompletion(const char *buf, size_t bufLength, libuvTTYCompletions autoComplete, uint8_t actionCode,
	const char *nodeString, size_t nodeStringLength, const char *keyString, size_t keyStringLength,
	const char *partialTypeString, size_t partialTypeStringLength);
static void valueCompletion(const char *buf, size_t bufLength, libuvTTYCompletions autoComplete, uint8_t actionCode,
	const char *nodeString, size_t nodeStringLength, const char *keyString, size_t keyStringLength,
	const char *typeString, size_t typeStringLength, const char *partialValueString, size_t partialValueStringLength);
static void addCompletionSuffix(libuvTTYCompletions autoComplete, const char *buf, size_t completionPoint,
	const char *suffix,
	bool endSpace, bool endSlash);

static const struct {
	const char *name;
	size_t nameLen;
	uint8_t code;
} actions[] = { { "node_exists", 11, CAER_CONFIG_NODE_EXISTS }, { "attr_exists", 11, CAER_CONFIG_ATTR_EXISTS }, { "get",
	3, CAER_CONFIG_GET }, { "put", 3, CAER_CONFIG_PUT } };
static const size_t actionsLength = sizeof(actions) / sizeof(actions[0]);

static uv_os_sock_t sockFd = -1;

int main(int argc, char *argv[]) {
	// First of all, parse the IP:Port we need to connect to.
	// Those are for now also the only two parameters permitted.
	// If none passed, attempt to connect to default IP:Port.
	const char *ipAddress = "127.0.0.1";
	uint16_t portNumber = 4040;

	if (argc != 1 && argc != 3) {
		fprintf(stderr, "Incorrect argument number. Either pass none for default IP:Port"
			"combination of 127.0.0.1:4040, or pass the IP followed by the Port.\n");
		return (EXIT_FAILURE);
	}

	// If explicitly passed, parse arguments.
	if (argc == 3) {
		ipAddress = argv[1];
		sscanf(argv[2], "%" SCNu16, &portNumber);
	}

	// Get history file path relative to home directory.
	char homeDir[PATH_MAX];
	size_t homeDirLength = PATH_MAX;

	int retVal = uv_os_homedir(homeDir, &homeDirLength);
	UV_RET_CHECK_STDERR(retVal, "uv_os_homedir", return (EXIT_FAILURE));

	strcat(homeDir, "/.caerctl_history");

	// Generate address to connect to.
	struct sockaddr_in configServerAddress;

	retVal = uv_ip4_addr(ipAddress, portNumber, &configServerAddress);
	UV_RET_CHECK_STDERR(retVal, "uv_ip4_addr", return (EXIT_FAILURE));

	// Connect to the remote cAER config server.
	sockFd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (sockFd < 0) {
		fprintf(stderr, "Failed to create TCP socket.\n");
		return (EXIT_FAILURE);
	}

	if (connect(sockFd, (struct sockaddr *) &configServerAddress, sizeof(struct sockaddr_in)) < 0) {
		fprintf(stderr, "Failed to connect to remote config server.\n");

		close(sockFd);
		return (EXIT_FAILURE);
	}

	// Create a shell prompt with the IP:Port displayed.
	size_t shellPromptLength = (size_t) snprintf(NULL, 0, "cAER @ %s:%" PRIu16, ipAddress, portNumber);
	char shellPrompt[shellPromptLength + 1]; // +1 for terminating NUL byte.
	snprintf(shellPrompt, shellPromptLength + 1, "cAER @ %s:%" PRIu16, ipAddress, portNumber);

	// Main event loop for connection handling.
	uv_loop_t caerctlLoop;
	retVal = uv_loop_init(&caerctlLoop);
	UV_RET_CHECK_STDERR(retVal, "uv_loop_init", close(sockFd); return (EXIT_FAILURE));

	// TTY handling.
	struct libuv_tty_struct caerctlTTY;
	retVal = libuvTTYInit(&caerctlLoop, &caerctlTTY, shellPrompt, &handleInputLine);
	UV_RET_CHECK_STDERR(retVal, "libuvTTYInit", goto loopCleanup);

	libuvTTYHistorySetFile(&caerctlTTY, homeDir);

	retVal = libuvTTYAutoCompleteSetCallback(&caerctlTTY, &handleCommandCompletion);
	UV_RET_CHECK_STDERR(retVal, "libuvTTYAutoCompleteSetCallback", goto ttyCleanup);

	// Run event loop.
	retVal = uv_run(&caerctlLoop, UV_RUN_DEFAULT);
	UV_RET_CHECK_STDERR(retVal, "uv_run", goto ttyCleanup);

	// Cleanup event loop and memory.
	ttyCleanup: {
		libuvTTYClose(&caerctlTTY);
	}

	loopCleanup: {
		bool errorCleanup = (retVal < 0);

		// Cleanup all remaining handles and run until all callbacks are done.
		retVal = libuvCloseLoopHandles(&caerctlLoop);
		UV_RET_CHECK_STDERR(retVal, "libuvCloseLoopHandles",);

		retVal = uv_loop_close(&caerctlLoop);
		UV_RET_CHECK_STDERR(retVal, "uv_loop_close",);

		close(sockFd);

		if (errorCleanup) {
			return (EXIT_FAILURE);
		}
		else {
			return (EXIT_SUCCESS);
		}
	}
}

static inline void setExtraLen(uint8_t *buf, uint16_t extraLen) {
	*((uint16_t *) (buf + 2)) = htole16(extraLen);
}

static inline void setNodeLen(uint8_t *buf, uint16_t nodeLen) {
	*((uint16_t *) (buf + 4)) = htole16(nodeLen);
}

static inline void setKeyLen(uint8_t *buf, uint16_t keyLen) {
	*((uint16_t *) (buf + 6)) = htole16(keyLen);
}

static inline void setValueLen(uint8_t *buf, uint16_t valueLen) {
	*((uint16_t *) (buf + 8)) = htole16(valueLen);
}

#define MAX_CMD_PARTS 5

#define CMD_PART_ACTION 0
#define CMD_PART_NODE 1
#define CMD_PART_KEY 2
#define CMD_PART_TYPE 3
#define CMD_PART_VALUE 4

static void handleInputLine(const char *buf, size_t bufLength) {
	// First let's split up the command into its constituents.
	char *commandParts[MAX_CMD_PARTS + 1] = { NULL };

	// Create a copy of buf, so that strtok_r() can modify it.
	char bufCopy[bufLength + 1];
	strcpy(bufCopy, buf);

	// Split string into usable parts.
	size_t idx = 0;
	char *tokenSavePtr = NULL, *nextCmdPart = NULL, *currCmdPart = bufCopy;
	while ((nextCmdPart = strtok_r(currCmdPart, " ", &tokenSavePtr)) != NULL) {
		if (idx < MAX_CMD_PARTS) {
			commandParts[idx] = nextCmdPart;
		}
		else {
			// Abort, too many parts.
			fprintf(stderr, "Error: command is made up of too many parts.\n");
			return;
		}

		idx++;
		currCmdPart = NULL;
	}

	// Check that we got something.
	if (commandParts[CMD_PART_ACTION] == NULL) {
		fprintf(stderr, "Error: empty command.\n");
		return;
	}

	// Let's get the action code first thing.
	uint8_t actionCode = UINT8_MAX;

	for (size_t i = 0; i < actionsLength; i++) {
		if (strcmp(commandParts[CMD_PART_ACTION], actions[i].name) == 0) {
			actionCode = actions[i].code;
		}
	}

	// Control message format: 1 byte ACTION, 1 byte TYPE, 2 bytes EXTRA_LEN,
	// 2 bytes NODE_LEN, 2 bytes KEY_LEN, 2 bytes VALUE_LEN, then up to 4086
	// bytes split between EXTRA, NODE, KEY, VALUE (with 4 bytes for NUL).
	// Basically: (EXTRA_LEN + NODE_LEN + KEY_LEN + VALUE_LEN) <= 4086.
	// EXTRA, NODE, KEY, VALUE have to be NUL terminated, and their length
	// must include the NUL termination byte.
	// This results in a maximum message size of 4096 bytes (4KB).
	uint8_t dataBuffer[CAER_CONFIG_SERVER_BUFFER_SIZE];
	size_t dataBufferLength = 0;

	// Now that we know what we want to do, let's decode the command line.
	switch (actionCode) {
		case CAER_CONFIG_NODE_EXISTS: {
			// Check parameters needed for operation.
			if (commandParts[CMD_PART_NODE] == NULL) {
				fprintf(stderr, "Error: missing node parameter.\n");
				return;
			}
			if (commandParts[CMD_PART_NODE + 1] != NULL) {
				fprintf(stderr, "Error: too many parameters for command.\n");
				return;
			}

			size_t nodeLength = strlen(commandParts[CMD_PART_NODE]) + 1; // +1 for terminating NUL byte.

			dataBuffer[0] = actionCode;
			dataBuffer[1] = 0; // UNUSED.
			setExtraLen(dataBuffer, 0); // UNUSED.
			setNodeLen(dataBuffer, (uint16_t) nodeLength);
			setKeyLen(dataBuffer, 0); // UNUSED.
			setValueLen(dataBuffer, 0); // UNUSED.

			memcpy(dataBuffer + CAER_CONFIG_SERVER_HEADER_SIZE, commandParts[CMD_PART_NODE], nodeLength);

			dataBufferLength = CAER_CONFIG_SERVER_HEADER_SIZE + nodeLength;

			break;
		}

		case CAER_CONFIG_ATTR_EXISTS:
		case CAER_CONFIG_GET: {
			// Check parameters needed for operation.
			if (commandParts[CMD_PART_NODE] == NULL) {
				fprintf(stderr, "Error: missing node parameter.\n");
				return;
			}
			if (commandParts[CMD_PART_KEY] == NULL) {
				fprintf(stderr, "Error: missing key parameter.\n");
				return;
			}
			if (commandParts[CMD_PART_TYPE] == NULL) {
				fprintf(stderr, "Error: missing type parameter.\n");
				return;
			}
			if (commandParts[CMD_PART_TYPE + 1] != NULL) {
				fprintf(stderr, "Error: too many parameters for command.\n");
				return;
			}

			size_t nodeLength = strlen(commandParts[CMD_PART_NODE]) + 1; // +1 for terminating NUL byte.
			size_t keyLength = strlen(commandParts[CMD_PART_KEY]) + 1; // +1 for terminating NUL byte.

			enum sshs_node_attr_value_type type = sshsHelperStringToTypeConverter(commandParts[CMD_PART_TYPE]);
			if (type == SSHS_UNKNOWN) {
				fprintf(stderr, "Error: invalid type parameter.\n");
				return;
			}

			dataBuffer[0] = actionCode;
			dataBuffer[1] = (uint8_t) type;
			setExtraLen(dataBuffer, 0); // UNUSED.
			setNodeLen(dataBuffer, (uint16_t) nodeLength);
			setKeyLen(dataBuffer, (uint16_t) keyLength);
			setValueLen(dataBuffer, 0); // UNUSED.

			memcpy(dataBuffer + CAER_CONFIG_SERVER_HEADER_SIZE, commandParts[CMD_PART_NODE], nodeLength);
			memcpy(dataBuffer + CAER_CONFIG_SERVER_HEADER_SIZE + nodeLength, commandParts[CMD_PART_KEY], keyLength);

			dataBufferLength = CAER_CONFIG_SERVER_HEADER_SIZE + nodeLength + keyLength;

			break;
		}

		case CAER_CONFIG_PUT: {
			// Check parameters needed for operation.
			if (commandParts[CMD_PART_NODE] == NULL) {
				fprintf(stderr, "Error: missing node parameter.\n");
				return;
			}
			if (commandParts[CMD_PART_KEY] == NULL) {
				fprintf(stderr, "Error: missing key parameter.\n");
				return;
			}
			if (commandParts[CMD_PART_TYPE] == NULL) {
				fprintf(stderr, "Error: missing type parameter.\n");
				return;
			}
			if (commandParts[CMD_PART_VALUE] == NULL) {
				fprintf(stderr, "Error: missing value parameter.\n");
				return;
			}
			if (commandParts[CMD_PART_VALUE + 1] != NULL) {
				fprintf(stderr, "Error: too many parameters for command.\n");
				return;
			}

			size_t nodeLength = strlen(commandParts[CMD_PART_NODE]) + 1; // +1 for terminating NUL byte.
			size_t keyLength = strlen(commandParts[CMD_PART_KEY]) + 1; // +1 for terminating NUL byte.
			size_t valueLength = strlen(commandParts[CMD_PART_VALUE]) + 1; // +1 for terminating NUL byte.

			enum sshs_node_attr_value_type type = sshsHelperStringToTypeConverter(commandParts[CMD_PART_TYPE]);
			if (type == SSHS_UNKNOWN) {
				fprintf(stderr, "Error: invalid type parameter.\n");
				return;
			}

			dataBuffer[0] = actionCode;
			dataBuffer[1] = (uint8_t) type;
			setExtraLen(dataBuffer, 0); // UNUSED.
			setNodeLen(dataBuffer, (uint16_t) nodeLength);
			setKeyLen(dataBuffer, (uint16_t) keyLength);
			setValueLen(dataBuffer, (uint16_t) valueLength);

			memcpy(dataBuffer + CAER_CONFIG_SERVER_HEADER_SIZE, commandParts[CMD_PART_NODE], nodeLength);
			memcpy(dataBuffer + CAER_CONFIG_SERVER_HEADER_SIZE + nodeLength, commandParts[CMD_PART_KEY], keyLength);
			memcpy(dataBuffer + CAER_CONFIG_SERVER_HEADER_SIZE + nodeLength + keyLength, commandParts[CMD_PART_VALUE],
				valueLength);

			dataBufferLength = CAER_CONFIG_SERVER_HEADER_SIZE + nodeLength + keyLength + valueLength;

			break;
		}

		default:
			fprintf(stderr, "Error: unknown command.\n");
			return;
	}

	// Send formatted command to configuration server.
	if (!sendUntilDone(sockFd, dataBuffer, dataBufferLength)) {
		fprintf(stderr, "Error: unable to send data to config server (%d).\n", errno);
		return;
	}

	// The response from the server follows a simplified version of the request
	// protocol. A byte for ACTION, a byte for TYPE, 2 bytes for MSG_LEN and then
	// up to 4092 bytes of MSG, for a maximum total of 4096 bytes again.
	// MSG must be NUL terminated, and the NUL byte shall be part of the length.
	if (!recvUntilDone(sockFd, dataBuffer, 4)) {
		fprintf(stderr, "Error: unable to receive data from config server (%d).\n", errno);
		return;
	}

	// Decode response header fields (all in little-endian).
	uint8_t action = dataBuffer[0];
	uint8_t type = dataBuffer[1];
	uint16_t msgLength = le16toh(*(uint16_t * )(dataBuffer + 2));

	// Total length to get for response.
	if (!recvUntilDone(sockFd, dataBuffer + 4, msgLength)) {
		fprintf(stderr, "Error: unable to receive data from config server (%d).\n", errno);
		return;
	}

	// Convert action back to a string.
	const char *actionString = NULL;

	// Detect error response.
	if (action == CAER_CONFIG_ERROR) {
		actionString = "error";
	}
	else {
		for (size_t i = 0; i < actionsLength; i++) {
			if (actions[i].code == action) {
				actionString = actions[i].name;
			}
		}
	}

	// Display results.
	fprintf(stdout, "Result: action=%s, type=%s, msgLength=%" PRIu16 ", msg='%s'.\n", actionString,
		sshsHelperTypeToStringConverter(type), msgLength, dataBuffer + 4);
}

static void handleCommandCompletion(const char *buf, size_t bufLength, libuvTTYCompletions autoComplete) {
	// First let's split up the command into its constituents.
	char *commandParts[MAX_CMD_PARTS + 1] = { NULL };

	// Create a copy of buf, so that strtok_r() can modify it.
	char bufCopy[bufLength + 1];
	strcpy(bufCopy, buf);

	// Split string into usable parts.
	size_t idx = 0;
	char *tokenSavePtr = NULL, *nextCmdPart = NULL, *currCmdPart = bufCopy;
	while ((nextCmdPart = strtok_r(currCmdPart, " ", &tokenSavePtr)) != NULL) {
		if (idx < MAX_CMD_PARTS) {
			commandParts[idx] = nextCmdPart;
		}
		else {
			// Abort, too many parts.
			return;
		}

		idx++;
		currCmdPart = NULL;
	}

	// Also calculate number of commands already present in line (word-depth).
	// This is actually much more useful to understand where we are and what to do.
	size_t commandDepth = idx;

	if (commandDepth > 0 && bufLength > 1 && buf[bufLength - 1] != ' ') {
		// If commands are present, ensure they have been "confirmed" by at least
		// one terminating spacing character. Else don't calculate the last command.
		commandDepth--;
	}

	// Check that we got something.
	if (commandDepth == 0) {
		// Always start off with a command/action.
		size_t cmdActionLength = 0;
		if (commandParts[CMD_PART_ACTION] != NULL) {
			cmdActionLength = strlen(commandParts[CMD_PART_ACTION]);
		}

		actionCompletion(buf, bufLength, autoComplete, commandParts[CMD_PART_ACTION], cmdActionLength);

		return;
	}

	// Let's get the action code first thing.
	uint8_t actionCode = UINT8_MAX;

	for (size_t i = 0; i < actionsLength; i++) {
		if (strcmp(commandParts[CMD_PART_ACTION], actions[i].name) == 0) {
			actionCode = actions[i].code;
		}
	}

	switch (actionCode) {
		case CAER_CONFIG_NODE_EXISTS:
			if (commandDepth == 1) {
				size_t cmdNodeLength = 0;
				if (commandParts[CMD_PART_NODE] != NULL) {
					cmdNodeLength = strlen(commandParts[CMD_PART_NODE]);
				}

				nodeCompletion(buf, bufLength, autoComplete, actionCode, commandParts[CMD_PART_NODE], cmdNodeLength);
			}

			break;

		case CAER_CONFIG_ATTR_EXISTS:
		case CAER_CONFIG_GET:
			if (commandDepth == 1) {
				size_t cmdNodeLength = 0;
				if (commandParts[CMD_PART_NODE] != NULL) {
					cmdNodeLength = strlen(commandParts[CMD_PART_NODE]);
				}

				nodeCompletion(buf, bufLength, autoComplete, actionCode, commandParts[CMD_PART_NODE], cmdNodeLength);
			}
			if (commandDepth == 2) {
				size_t cmdKeyLength = 0;
				if (commandParts[CMD_PART_KEY] != NULL) {
					cmdKeyLength = strlen(commandParts[CMD_PART_KEY]);
				}

				keyCompletion(buf, bufLength, autoComplete, actionCode, commandParts[CMD_PART_NODE],
					strlen(commandParts[CMD_PART_NODE]), commandParts[CMD_PART_KEY], cmdKeyLength);
			}
			if (commandDepth == 3) {
				size_t cmdTypeLength = 0;
				if (commandParts[CMD_PART_TYPE] != NULL) {
					cmdTypeLength = strlen(commandParts[CMD_PART_TYPE]);
				}

				typeCompletion(buf, bufLength, autoComplete, actionCode, commandParts[CMD_PART_NODE],
					strlen(commandParts[CMD_PART_NODE]), commandParts[CMD_PART_KEY], strlen(commandParts[CMD_PART_KEY]),
					commandParts[CMD_PART_TYPE], cmdTypeLength);
			}

			break;

		case CAER_CONFIG_PUT:
			if (commandDepth == 1) {
				size_t cmdNodeLength = 0;
				if (commandParts[CMD_PART_NODE] != NULL) {
					cmdNodeLength = strlen(commandParts[CMD_PART_NODE]);
				}

				nodeCompletion(buf, bufLength, autoComplete, actionCode, commandParts[CMD_PART_NODE], cmdNodeLength);
			}
			if (commandDepth == 2) {
				size_t cmdKeyLength = 0;
				if (commandParts[CMD_PART_KEY] != NULL) {
					cmdKeyLength = strlen(commandParts[CMD_PART_KEY]);
				}

				keyCompletion(buf, bufLength, autoComplete, actionCode, commandParts[CMD_PART_NODE],
					strlen(commandParts[CMD_PART_NODE]), commandParts[CMD_PART_KEY], cmdKeyLength);
			}
			if (commandDepth == 3) {
				size_t cmdTypeLength = 0;
				if (commandParts[CMD_PART_TYPE] != NULL) {
					cmdTypeLength = strlen(commandParts[CMD_PART_TYPE]);
				}

				typeCompletion(buf, bufLength, autoComplete, actionCode, commandParts[CMD_PART_NODE],
					strlen(commandParts[CMD_PART_NODE]), commandParts[CMD_PART_KEY], strlen(commandParts[CMD_PART_KEY]),
					commandParts[CMD_PART_TYPE], cmdTypeLength);
			}
			if (commandDepth == 4) {
				size_t cmdValueLength = 0;
				if (commandParts[CMD_PART_VALUE] != NULL) {
					cmdValueLength = strlen(commandParts[CMD_PART_VALUE]);
				}

				valueCompletion(buf, bufLength, autoComplete, actionCode, commandParts[CMD_PART_NODE],
					strlen(commandParts[CMD_PART_NODE]), commandParts[CMD_PART_KEY], strlen(commandParts[CMD_PART_KEY]),
					commandParts[CMD_PART_TYPE], strlen(commandParts[CMD_PART_TYPE]), commandParts[CMD_PART_VALUE],
					cmdValueLength);
			}

			break;
	}
}

static void actionCompletion(const char *buf, size_t bufLength, libuvTTYCompletions autoComplete,
	const char *partialActionString, size_t partialActionStringLength) {
	UNUSED_ARGUMENT(buf);
	UNUSED_ARGUMENT(bufLength);

	// Always start off with a command.
	for (size_t i = 0; i < actionsLength; i++) {
		if (strncmp(actions[i].name, partialActionString, partialActionStringLength) == 0) {
			addCompletionSuffix(autoComplete, "", 0, actions[i].name, true, false);
		}
	}

	// Add quit and exit too.
	if (strncmp("exit", partialActionString, partialActionStringLength) == 0) {
		addCompletionSuffix(autoComplete, "", 0, "exit", true, false);
	}
	if (strncmp("quit", partialActionString, partialActionStringLength) == 0) {
		addCompletionSuffix(autoComplete, "", 0, "quit", true, false);
	}
}

static void nodeCompletion(const char *buf, size_t bufLength, libuvTTYCompletions autoComplete, uint8_t actionCode,
	const char *partialNodeString, size_t partialNodeStringLength) {
	UNUSED_ARGUMENT(actionCode);

	// If partialNodeString is still empty, the first thing is to complete the root.
	if (partialNodeStringLength == 0) {
		addCompletionSuffix(autoComplete, buf, bufLength, "/", false, false);
		return;
	}

	// Get all the children of the last fully defined node (/ or /../../).
	char *lastNode = strrchr(partialNodeString, '/');
	if (lastNode == NULL) {
		// No / found, invalid, cannot auto-complete.
		return;
	}

	size_t lastNodeLength = (size_t) (lastNode - partialNodeString) + 1;

	uint8_t dataBuffer[CAER_CONFIG_SERVER_BUFFER_SIZE];

	// Send request for all children names.
	dataBuffer[0] = CAER_CONFIG_GET_CHILDREN;
	dataBuffer[1] = 0; // UNUSED.
	setExtraLen(dataBuffer, 0); // UNUSED.
	setNodeLen(dataBuffer, (uint16_t) (lastNodeLength + 1)); // +1 for terminating NUL byte.
	setKeyLen(dataBuffer, 0); // UNUSED.
	setValueLen(dataBuffer, 0); // UNUSED.

	memcpy(dataBuffer + CAER_CONFIG_SERVER_HEADER_SIZE, partialNodeString, lastNodeLength);
	dataBuffer[CAER_CONFIG_SERVER_HEADER_SIZE + lastNodeLength] = '\0';

	if (!sendUntilDone(sockFd, dataBuffer, CAER_CONFIG_SERVER_HEADER_SIZE + lastNodeLength + 1)) {
		// Failed to contact remote host, no auto-completion!
		return;
	}

	if (!recvUntilDone(sockFd, dataBuffer, 4)) {
		// Failed to contact remote host, no auto-completion!
		return;
	}

	// Decode response header fields (all in little-endian).
	uint8_t action = dataBuffer[0];
	uint8_t type = dataBuffer[1];
	uint16_t msgLength = le16toh(*(uint16_t * )(dataBuffer + 2));

	// Total length to get for response.
	if (!recvUntilDone(sockFd, dataBuffer + 4, msgLength)) {
		// Failed to contact remote host, no auto-completion!
		return;
	}

	if (action == CAER_CONFIG_ERROR || type != SSHS_STRING) {
		// Invalid request made, no auto-completion.
		return;
	}

	// At this point we made a valid request and got back a full response.
	for (size_t i = 0; i < msgLength; i++) {
		if (strncasecmp((const char *) dataBuffer + 4 + i, lastNode + 1, strlen(lastNode + 1)) == 0) {
			addCompletionSuffix(autoComplete, buf, bufLength - strlen(lastNode + 1), (const char *) dataBuffer + 4 + i,
			false, true);
		}

		// Jump to the NUL character after this string.
		i += strlen((const char *) dataBuffer + 4 + i);
	}
}

static void keyCompletion(const char *buf, size_t bufLength, libuvTTYCompletions autoComplete, uint8_t actionCode,
	const char *nodeString, size_t nodeStringLength, const char *partialKeyString, size_t partialKeyStringLength) {
	UNUSED_ARGUMENT(actionCode);

	uint8_t dataBuffer[CAER_CONFIG_SERVER_BUFFER_SIZE];

	// Send request for all attribute names for this node.
	dataBuffer[0] = CAER_CONFIG_GET_ATTRIBUTES;
	dataBuffer[1] = 0; // UNUSED.
	setExtraLen(dataBuffer, 0); // UNUSED.
	setNodeLen(dataBuffer, (uint16_t) (nodeStringLength + 1)); // +1 for terminating NUL byte.
	setKeyLen(dataBuffer, 0); // UNUSED.
	setValueLen(dataBuffer, 0); // UNUSED.

	memcpy(dataBuffer + CAER_CONFIG_SERVER_HEADER_SIZE, nodeString, nodeStringLength);
	dataBuffer[CAER_CONFIG_SERVER_HEADER_SIZE + nodeStringLength] = '\0';

	if (!sendUntilDone(sockFd, dataBuffer, CAER_CONFIG_SERVER_HEADER_SIZE + nodeStringLength + 1)) {
		// Failed to contact remote host, no auto-completion!
		return;
	}

	if (!recvUntilDone(sockFd, dataBuffer, 4)) {
		// Failed to contact remote host, no auto-completion!
		return;
	}

	// Decode response header fields (all in little-endian).
	uint8_t action = dataBuffer[0];
	uint8_t type = dataBuffer[1];
	uint16_t msgLength = le16toh(*(uint16_t * )(dataBuffer + 2));

	// Total length to get for response.
	if (!recvUntilDone(sockFd, dataBuffer + 4, msgLength)) {
		// Failed to contact remote host, no auto-completion!
		return;
	}

	if (action == CAER_CONFIG_ERROR || type != SSHS_STRING) {
		// Invalid request made, no auto-completion.
		return;
	}

	// At this point we made a valid request and got back a full response.
	for (size_t i = 0; i < msgLength; i++) {
		if (strncasecmp((const char *) dataBuffer + 4 + i, partialKeyString, partialKeyStringLength) == 0) {
			addCompletionSuffix(autoComplete, buf, bufLength - partialKeyStringLength,
				(const char *) dataBuffer + 4 + i,
				true, false);
		}

		// Jump to the NUL character after this string.
		i += strlen((const char *) dataBuffer + 4 + i);
	}
}

static void typeCompletion(const char *buf, size_t bufLength, libuvTTYCompletions autoComplete, uint8_t actionCode,
	const char *nodeString, size_t nodeStringLength, const char *keyString, size_t keyStringLength,
	const char *partialTypeString, size_t partialTypeStringLength) {
	UNUSED_ARGUMENT(actionCode);

	uint8_t dataBuffer[CAER_CONFIG_SERVER_BUFFER_SIZE];

	// Send request for all type names for this key on this node.
	dataBuffer[0] = CAER_CONFIG_GET_TYPES;
	dataBuffer[1] = 0; // UNUSED.
	setExtraLen(dataBuffer, 0); // UNUSED.
	setNodeLen(dataBuffer, (uint16_t) (nodeStringLength + 1)); // +1 for terminating NUL byte.
	setKeyLen(dataBuffer, (uint16_t) (keyStringLength + 1)); // +1 for terminating NUL byte.
	setValueLen(dataBuffer, 0); // UNUSED.

	memcpy(dataBuffer + CAER_CONFIG_SERVER_HEADER_SIZE, nodeString, nodeStringLength);
	dataBuffer[CAER_CONFIG_SERVER_HEADER_SIZE + nodeStringLength] = '\0';

	memcpy(dataBuffer + CAER_CONFIG_SERVER_HEADER_SIZE + nodeStringLength + 1, keyString, keyStringLength);
	dataBuffer[CAER_CONFIG_SERVER_HEADER_SIZE + nodeStringLength + 1 + keyStringLength] = '\0';

	if (!sendUntilDone(sockFd, dataBuffer,
		CAER_CONFIG_SERVER_HEADER_SIZE + nodeStringLength + 1 + keyStringLength + 1)) {
		// Failed to contact remote host, no auto-completion!
		return;
	}

	if (!recvUntilDone(sockFd, dataBuffer, 4)) {
		// Failed to contact remote host, no auto-completion!
		return;
	}

	// Decode response header fields (all in little-endian).
	uint8_t action = dataBuffer[0];
	uint8_t type = dataBuffer[1];
	uint16_t msgLength = le16toh(*(uint16_t * )(dataBuffer + 2));

	// Total length to get for response.
	if (!recvUntilDone(sockFd, dataBuffer + 4, msgLength)) {
		// Failed to contact remote host, no auto-completion!
		return;
	}

	if (action == CAER_CONFIG_ERROR || type != SSHS_STRING) {
		// Invalid request made, no auto-completion.
		return;
	}

	// At this point we made a valid request and got back a full response.
	for (size_t i = 0; i < msgLength; i++) {
		if (strncasecmp((const char *) dataBuffer + 4 + i, partialTypeString, partialTypeStringLength) == 0) {
			addCompletionSuffix(autoComplete, buf, bufLength - partialTypeStringLength,
				(const char *) dataBuffer + 4 + i, true, false);
		}

		// Jump to the NUL character after this string.
		i += strlen((const char *) dataBuffer + 4 + i);
	}
}

static void valueCompletion(const char *buf, size_t bufLength, libuvTTYCompletions autoComplete, uint8_t actionCode,
	const char *nodeString, size_t nodeStringLength, const char *keyString, size_t keyStringLength,
	const char *typeString, size_t typeStringLength, const char *partialValueString, size_t partialValueStringLength) {
	UNUSED_ARGUMENT(actionCode);
	UNUSED_ARGUMENT(typeStringLength);

	enum sshs_node_attr_value_type type = sshsHelperStringToTypeConverter(typeString);
	if (type == SSHS_UNKNOWN) {
		// Invalid type, no auto-completion.
		return;
	}

	if (partialValueStringLength != 0) {
		// If there already is content, we can't do any auto-completion here, as
		// we have no idea about what a valid value would be to complete ...
		// Unless this is a boolean, then we can propose true/false strings.
		if (type == SSHS_BOOL) {
			if (strncmp("true", partialValueString, partialValueStringLength) == 0) {
				addCompletionSuffix(autoComplete, buf, bufLength - partialValueStringLength, "true", false, false);
			}
			if (strncmp("false", partialValueString, partialValueStringLength) == 0) {
				addCompletionSuffix(autoComplete, buf, bufLength - partialValueStringLength, "false", false, false);
			}
		}

		return;
	}

	uint8_t dataBuffer[CAER_CONFIG_SERVER_BUFFER_SIZE];

	// Send request for the current value, so we can auto-complete with it as default.
	dataBuffer[0] = CAER_CONFIG_GET;
	dataBuffer[1] = (uint8_t) type;
	setExtraLen(dataBuffer, 0); // UNUSED.
	setNodeLen(dataBuffer, (uint16_t) (nodeStringLength + 1)); // +1 for terminating NUL byte.
	setKeyLen(dataBuffer, (uint16_t) (keyStringLength + 1)); // +1 for terminating NUL byte.
	setValueLen(dataBuffer, 0); // UNUSED.

	memcpy(dataBuffer + CAER_CONFIG_SERVER_HEADER_SIZE, nodeString, nodeStringLength);
	dataBuffer[CAER_CONFIG_SERVER_HEADER_SIZE + nodeStringLength] = '\0';

	memcpy(dataBuffer + CAER_CONFIG_SERVER_HEADER_SIZE + nodeStringLength + 1, keyString, keyStringLength);
	dataBuffer[CAER_CONFIG_SERVER_HEADER_SIZE + nodeStringLength + 1 + keyStringLength] = '\0';

	if (!sendUntilDone(sockFd, dataBuffer,
		CAER_CONFIG_SERVER_HEADER_SIZE + nodeStringLength + 1 + keyStringLength + 1)) {
		// Failed to contact remote host, no auto-completion!
		return;
	}

	if (!recvUntilDone(sockFd, dataBuffer, 4)) {
		// Failed to contact remote host, no auto-completion!
		return;
	}

	// Decode response header fields (all in little-endian).
	uint8_t action = dataBuffer[0];
	uint16_t msgLength = le16toh(*(uint16_t * )(dataBuffer + 2));

	// Total length to get for response.
	if (!recvUntilDone(sockFd, dataBuffer + 4, msgLength)) {
		// Failed to contact remote host, no auto-completion!
		return;
	}

	if (action == CAER_CONFIG_ERROR) {
		// Invalid request made, no auto-completion.
		return;
	}

	// At this point we made a valid request and got back a full response.
	// We can just use it directly and paste it in as completion.
	addCompletionSuffix(autoComplete, buf, bufLength, (const char *) dataBuffer + 4, false, false);

	// If this is a boolean value, we can also add the inverse as a second completion.
	if (type == SSHS_BOOL) {
		if (strcmp((const char *) dataBuffer + 4, "true") == 0) {
			addCompletionSuffix(autoComplete, buf, bufLength, "false", false, false);
		}
		else {
			addCompletionSuffix(autoComplete, buf, bufLength, "true", false, false);
		}
	}
}

static void addCompletionSuffix(libuvTTYCompletions autoComplete, const char *buf, size_t completionPoint,
	const char *suffix,
	bool endSpace, bool endSlash) {
	char concat[2048];

	if (endSpace) {
		if (endSlash) {
			snprintf(concat, 2048, "%.*s%s/ ", (int) completionPoint, buf, suffix);
		}
		else {
			snprintf(concat, 2048, "%.*s%s ", (int) completionPoint, buf, suffix);
		}
	}
	else {
		if (endSlash) {
			snprintf(concat, 2048, "%.*s%s/", (int) completionPoint, buf, suffix);
		}
		else {
			snprintf(concat, 2048, "%.*s%s", (int) completionPoint, buf, suffix);
		}
	}

	libuvTTYAutoCompleteAddCompletion(autoComplete, concat);
}
