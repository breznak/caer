/*
 * caerctl.c
 *
 *  Created on: Jan 5, 2014
 *      Author: chtekk
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <strings.h>
#include <assert.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pwd.h>
#include "ext/libuv.h"
#include "ext/sshs/sshs.h"
#include "ext/portable_misc.h"
#include "base/config_server.h"
#include "utils/ext/linenoise/linenoise.h"

#define UV_RET_CHECK_CC(RET_VAL, FUNC_NAME, CLEANUP_ACTIONS) \
	if (RET_VAL < 0) { \
		fprintf(stderr, FUNC_NAME " failed, error %d (%s).\n", \
			RET_VAL, uv_err_name(RET_VAL)); \
		CLEANUP_ACTIONS; \
	}

static void ttyAlloc(uv_handle_t *tty, size_t suggestedSize, uv_buf_t *buf);
static void ttyRead(uv_stream_t *tty, ssize_t sizeRead, const uv_buf_t *buf);
static void tcpConnect(uv_connect_t *clientConnect, int status);
static void tcpAlloc(uv_handle_t *client, size_t suggestedSize, uv_buf_t *buf);
static void tcpRead(uv_stream_t *client, ssize_t sizeRead, const uv_buf_t *buf);

static void handleRequest(uv_connect_t *client, int status);
static void handleInputLine(const char *buf, size_t bufLength);
static void handleCommandCompletion(const char *buf, linenoiseCompletions *lc);
static void actionCompletion(const char *buf, size_t bufLength, linenoiseCompletions *lc,
	const char *partialActionString, size_t partialActionStringLength);
static void nodeCompletion(const char *buf, size_t bufLength, linenoiseCompletions *lc, uint8_t actionCode,
	const char *partialNodeString, size_t partialNodeStringLength);
static void keyCompletion(const char *buf, size_t bufLength, linenoiseCompletions *lc, uint8_t actionCode,
	const char *nodeString, size_t nodeStringLength, const char *partialKeyString, size_t partialKeyStringLength);
static void typeCompletion(const char *buf, size_t bufLength, linenoiseCompletions *lc, uint8_t actionCode,
	const char *nodeString, size_t nodeStringLength, const char *keyString, size_t keyStringLength,
	const char *partialTypeString, size_t partialTypeStringLength);
static void valueCompletion(const char *buf, size_t bufLength, linenoiseCompletions *lc, uint8_t actionCode,
	const char *nodeString, size_t nodeStringLength, const char *keyString, size_t keyStringLength,
	const char *typeString, size_t typeStringLength, const char *partialValueString, size_t partialValueStringLength);
static void linenoiseAddCompletionSuffix(linenoiseCompletions *lc, const char *buf, size_t completionPoint,
	const char *suffix, bool endSpace, bool endSlash);
static char *getUserHomeDirectory(void);

static const struct {
	const char *name;
	size_t nameLen;
	uint8_t code;
} actions[] = { { "node_exists", 11, CAER_CONFIG_NODE_EXISTS }, { "attr_exists", 11, CAER_CONFIG_ATTR_EXISTS }, { "get",
	3, CAER_CONFIG_GET }, { "put", 3, CAER_CONFIG_PUT } };
static const size_t actionsLength = sizeof(actions) / sizeof(actions[0]);

static uv_stream_t *ttyIn = NULL;
static uv_stream_t *ttyOut = NULL;
static uv_stream_t *tcpClient = NULL;
static char *shellPromptStr = NULL;

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

	// Generate address to connect to.
	struct sockaddr_in configServerAddress;

	int retVal = uv_ip4_addr(ipAddress, portNumber, &configServerAddress);
	UV_RET_CHECK_CC(retVal, "uv_ip4_addr", return (EXIT_FAILURE));

	// Create a shell prompt with the IP:Port displayed.
	size_t shellPromptLength = (size_t) snprintf(NULL, 0, "cAER @ %s:%" PRIu16 " >> ", ipAddress, portNumber);
	char shellPrompt[shellPromptLength + 1]; // +1 for terminating NUL byte.
	snprintf(shellPrompt, shellPromptLength + 1, "cAER @ %s:%" PRIu16 " >> ", ipAddress, portNumber);
	shellPromptStr = shellPrompt;

	// Main event loop for connection handling.
	uv_loop_t caerctlLoop;
	retVal = uv_loop_init(&caerctlLoop);
	UV_RET_CHECK_CC(retVal, "uv_loop_init", return (EXIT_FAILURE));

	// Initialize stdin TTY.
	uv_tty_t caerctlTTYInput;
	uv_tty_init(&caerctlLoop, &caerctlTTYInput, STDIN_FILENO, true);
	UV_RET_CHECK_CC(retVal, "uv_tty_init(STDIN)", goto loopCleanup);
	ttyIn = (uv_stream_t *) &caerctlTTYInput;
	caerctlTTYInput.data = NULL;

	// Switch stdin TTY to RAW mode, so we get every character and can build
	// up auto-completion and console like input.
	retVal = uv_tty_set_mode(&caerctlTTYInput, UV_TTY_MODE_RAW);
	UV_RET_CHECK_CC(retVal, "uv_tty_set_mode", goto ttyInCleanup);

	// Initialize stdout TTY.
	uv_tty_t caerctlTTYOutput;
	uv_tty_init(&caerctlLoop, &caerctlTTYOutput, STDOUT_FILENO, false);
	UV_RET_CHECK_CC(retVal, "uv_tty_init(STDOUT)", goto ttyInCleanup);
	ttyOut = (uv_stream_t *) &caerctlTTYOutput;
	caerctlTTYOutput.data = NULL;

	// Open a TCP client socket for configuration handling.
	uv_tcp_t caerctlTCPClient;
	retVal = uv_tcp_init(&caerctlLoop, &caerctlTCPClient);
	UV_RET_CHECK_CC(retVal, "uv_tcp_init", goto ttyOutCleanup);
	tcpClient = (uv_stream_t *) &caerctlTCPClient;
	caerctlTCPClient.data = NULL;

	// Start reading on TTYs, as they are ready now.
	retVal = uv_read_start(ttyIn, &ttyAlloc, &ttyRead);
	UV_RET_CHECK_CC(retVal, "uv_read_start(tty)", goto tcpCleanup);

	// Connect to the remote TCP server.
	uv_connect_t caerctlTCPConnect;
	//retVal = uv_tcp_connect(&caerctlTCPConnect, &caerctlTCPClient, (struct sockaddr *) &configServerAddress,&tcpConnect);
	UV_RET_CHECK_CC(retVal, "uv_tcp_connect", goto tcpCleanup);

	// Run event loop.
	retVal = uv_run(&caerctlLoop, UV_RUN_DEFAULT);
	UV_RET_CHECK_CC(retVal, "uv_run", goto tcpCleanup);

	// Cleanup event loop and memory.
	tcpCleanup: {
		uv_close((uv_handle_t *) &caerctlTCPClient, NULL);
	}

	ttyOutCleanup: {
		uv_close((uv_handle_t *) &caerctlTTYOutput, NULL);
	}

	ttyInCleanup: {
		uv_tty_reset_mode();

		uv_close((uv_handle_t *) &caerctlTTYInput, NULL);
	}

	loopCleanup: {
		bool errorCleanup = (retVal < 0);

		// Cleanup all remaining handles and run until all callbacks are done.
		retVal = libuvCloseLoopHandles(&caerctlLoop);
		UV_RET_CHECK_CC(retVal, "libuvCloseLoopHandles",);

		retVal = uv_loop_close(&caerctlLoop);
		UV_RET_CHECK_CC(retVal, "uv_loop_close",);

		if (errorCleanup) {
			return (EXIT_FAILURE);
		}
		else {
			return (EXIT_SUCCESS);
		}
	}
}

static void ttyAlloc(uv_handle_t *tty, size_t suggestedSize, uv_buf_t *buf) {
	UNUSED_ARGUMENT(suggestedSize);

	// We use one buffer per connection, with a fixed maximum size, and
	// re-use it until we have read a full message.
	if (tty->data == NULL) {
		tty->data = simpleBufferInit(4096);

		if (tty->data == NULL) {
			// Allocation failure!
			buf->base = NULL;
			buf->len = 0;

			fprintf(stderr, "Failed to allocate memory for client buffer.");
			return;
		}
	}

	simpleBuffer dataBuf = tty->data;

	buf->base = (char *) (dataBuf->buffer + dataBuf->bufferUsedSize);
	buf->len = dataBuf->bufferSize - dataBuf->bufferUsedSize;
}

#define ESCAPE "\x1B"

static void ttyRead(uv_stream_t *tty, ssize_t sizeRead, const uv_buf_t *buf) {
	fprintf(stderr, "Read STDIN.\n");

	if (sizeRead < 0) {
		fprintf(stderr, "STDIN closed: %s.\n", uv_err_name((int) sizeRead));
		uv_read_stop(tty);
		return;
	}

	fprintf(stderr, "Input length: %zu.\n", sizeRead);

	if (sizeRead == 1) {
		// Got char.
		fprintf(stderr, "Char is: (%X).\n", buf->base[0]);
	}
	else if (sizeRead == 3) {
		// Arrow keys.
		if (memcmp(buf->base, ESCAPE "[A", 3) == 0) {
			fprintf(stderr, "Got UP key.\n");
		}
		else if (memcmp(buf->base, ESCAPE "[B", 3) == 0) {
			fprintf(stderr, "Got DOWN key.\n");
		}
		else if (memcmp(buf->base, ESCAPE "[D", 3) == 0) {
			fprintf(stderr, "Got LEFT key.\n");
		}
		else if (memcmp(buf->base, ESCAPE "[C", 3) == 0) {
			fprintf(stderr, "Got RIGHT key.\n");
		}
		else {
			fprintf(stderr, "Got unknown sequence: (%X) (%X) (%X).\n", buf->base[0], buf->base[1], buf->base[2]);
		}
	}
	else {
		fprintf(stderr, "Got unknown sequence: ");
		for (size_t i = 0; i < (size_t) sizeRead; i++) {
			fprintf(stderr, "(%X) ", buf->base[i]);
		}
		fprintf(stderr, ".\n");
	}
}

static void tcpConnect(uv_connect_t *clientConnect, int status) {
	assert(clientConnect->handle == tcpClient);

	UV_RET_CHECK_CC(status, "Connection", uv_stop(clientConnect->handle->loop); return);

	int retVal = uv_read_start(clientConnect->handle, &tcpAlloc, &tcpRead);
	UV_RET_CHECK_CC(retVal, "uv_read_start(tcp)", uv_stop(clientConnect->handle->loop); return);
}

static void tcpAlloc(uv_handle_t *client, size_t suggestedSize, uv_buf_t *buf) {
	UNUSED_ARGUMENT(suggestedSize);

	// We use one buffer per connection, with a fixed maximum size, and
	// re-use it until we have read a full message.
	if (client->data == NULL) {
		client->data = simpleBufferInit(4096);

		if (client->data == NULL) {
			// Allocation failure!
			buf->base = NULL;
			buf->len = 0;

			fprintf(stderr, "Failed to allocate memory for client buffer.");
			return;
		}
	}

	simpleBuffer dataBuf = client->data;

	buf->base = (char *) (dataBuf->buffer + dataBuf->bufferUsedSize);
	buf->len = dataBuf->bufferSize - dataBuf->bufferUsedSize;
}

static void tcpRead(uv_stream_t *client, ssize_t sizeRead, const uv_buf_t *buf) {

}

static void handleRequest(uv_connect_t *client, int status) {
	UV_RET_CHECK_CC(status, "Connection", return);

	// Set our own command completion function.
	linenoiseSetCompletionCallback(&handleCommandCompletion);

	// Generate command history file path (in user home).
	char commandHistoryFilePath[1024];

	char *userHomeDir = getUserHomeDirectory();
	if (userHomeDir == NULL) {
		fprintf(stderr, "Failed to determine user's home directory.\n");
		return;
	}

	snprintf(commandHistoryFilePath, 1024, "%s/.caerctl_history", userHomeDir);
	free(userHomeDir);

	// Load command history file.
	linenoiseHistoryLoad(commandHistoryFilePath);

	while (true) {
		// Display prompt and read input (NOTE: remember to free input after use!).
		char *inputLine = linenoise(client->data);

		// Check for EOF first.
		if (inputLine == NULL) {
			// Exit loop.
			break;
		}

		// Add input to command history.
		linenoiseHistoryAdd(inputLine);

		// Then, after having added to history, check for termination commands.
		if (strncmp(inputLine, "quit", 4) == 0 || strncmp(inputLine, "exit", 4) == 0) {
			// Exit loop, free memory.
			free(inputLine);
			break;
		}

		// Try to generate a request, if there's any content.
		size_t inputLineLength = strlen(inputLine);

		if (inputLineLength > 0) {
			handleInputLine(inputLine, inputLineLength);
		}

		// Free input after use.
		free(inputLine);
	}

	// Save command history file.
	linenoiseHistorySave(commandHistoryFilePath);
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
	uint8_t dataBuffer[4096];
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

			memcpy(dataBuffer + 10, commandParts[CMD_PART_NODE], nodeLength);

			dataBufferLength = 10 + nodeLength;

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

			memcpy(dataBuffer + 10, commandParts[CMD_PART_NODE], nodeLength);
			memcpy(dataBuffer + 10 + nodeLength, commandParts[CMD_PART_KEY], keyLength);

			dataBufferLength = 10 + nodeLength + keyLength;

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

			memcpy(dataBuffer + 10, commandParts[CMD_PART_NODE], nodeLength);
			memcpy(dataBuffer + 10 + nodeLength, commandParts[CMD_PART_KEY], keyLength);
			memcpy(dataBuffer + 10 + nodeLength + keyLength, commandParts[CMD_PART_VALUE], valueLength);

			dataBufferLength = 10 + nodeLength + keyLength + valueLength;

			break;
		}

		default:
			fprintf(stderr, "Error: unknown command.\n");
			return;
	}

	// Send formatted command to configuration server.
//	if (!sendUntilDone(sockFd, dataBuffer, dataBufferLength)) {
//		fprintf(stderr, "Error: unable to send data to config server (%d).\n", errno);
//		return;
//	}

	// The response from the server follows a simplified version of the request
	// protocol. A byte for ACTION, a byte for TYPE, 2 bytes for MSG_LEN and then
	// up to 4092 bytes of MSG, for a maximum total of 4096 bytes again.
	// MSG must be NUL terminated, and the NUL byte shall be part of the length.
//	if (!recvUntilDone(sockFd, dataBuffer, 4)) {
//		fprintf(stderr, "Error: unable to receive data from config server (%d).\n", errno);
//		return;
//	}

	// Decode response header fields (all in little-endian).
	uint8_t action = dataBuffer[0];
	uint8_t type = dataBuffer[1];
	uint16_t msgLength = le16toh(*(uint16_t * )(dataBuffer + 2));

	// Total length to get for response.
//	if (!recvUntilDone(sockFd, dataBuffer + 4, msgLength)) {
//		fprintf(stderr, "Error: unable to receive data from config server (%d).\n", errno);
//		return;
//	}

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
	printf("Result: action=%s, type=%s, msgLength=%" PRIu16 ", msg='%s'.\n", actionString,
		sshsHelperTypeToStringConverter(type), msgLength, dataBuffer + 4);
}

static void handleCommandCompletion(const char *buf, linenoiseCompletions *lc) {
	size_t bufLength = strlen(buf);

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

		actionCompletion(buf, bufLength, lc, commandParts[CMD_PART_ACTION], cmdActionLength);

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

				nodeCompletion(buf, bufLength, lc, actionCode, commandParts[CMD_PART_NODE], cmdNodeLength);
			}

			break;

		case CAER_CONFIG_ATTR_EXISTS:
		case CAER_CONFIG_GET:
			if (commandDepth == 1) {
				size_t cmdNodeLength = 0;
				if (commandParts[CMD_PART_NODE] != NULL) {
					cmdNodeLength = strlen(commandParts[CMD_PART_NODE]);
				}

				nodeCompletion(buf, bufLength, lc, actionCode, commandParts[CMD_PART_NODE], cmdNodeLength);
			}
			if (commandDepth == 2) {
				size_t cmdKeyLength = 0;
				if (commandParts[CMD_PART_KEY] != NULL) {
					cmdKeyLength = strlen(commandParts[CMD_PART_KEY]);
				}

				keyCompletion(buf, bufLength, lc, actionCode, commandParts[CMD_PART_NODE],
					strlen(commandParts[CMD_PART_NODE]), commandParts[CMD_PART_KEY], cmdKeyLength);
			}
			if (commandDepth == 3) {
				size_t cmdTypeLength = 0;
				if (commandParts[CMD_PART_TYPE] != NULL) {
					cmdTypeLength = strlen(commandParts[CMD_PART_TYPE]);
				}

				typeCompletion(buf, bufLength, lc, actionCode, commandParts[CMD_PART_NODE],
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

				nodeCompletion(buf, bufLength, lc, actionCode, commandParts[CMD_PART_NODE], cmdNodeLength);
			}
			if (commandDepth == 2) {
				size_t cmdKeyLength = 0;
				if (commandParts[CMD_PART_KEY] != NULL) {
					cmdKeyLength = strlen(commandParts[CMD_PART_KEY]);
				}

				keyCompletion(buf, bufLength, lc, actionCode, commandParts[CMD_PART_NODE],
					strlen(commandParts[CMD_PART_NODE]), commandParts[CMD_PART_KEY], cmdKeyLength);
			}
			if (commandDepth == 3) {
				size_t cmdTypeLength = 0;
				if (commandParts[CMD_PART_TYPE] != NULL) {
					cmdTypeLength = strlen(commandParts[CMD_PART_TYPE]);
				}

				typeCompletion(buf, bufLength, lc, actionCode, commandParts[CMD_PART_NODE],
					strlen(commandParts[CMD_PART_NODE]), commandParts[CMD_PART_KEY], strlen(commandParts[CMD_PART_KEY]),
					commandParts[CMD_PART_TYPE], cmdTypeLength);
			}
			if (commandDepth == 4) {
				size_t cmdValueLength = 0;
				if (commandParts[CMD_PART_VALUE] != NULL) {
					cmdValueLength = strlen(commandParts[CMD_PART_VALUE]);
				}

				valueCompletion(buf, bufLength, lc, actionCode, commandParts[CMD_PART_NODE],
					strlen(commandParts[CMD_PART_NODE]), commandParts[CMD_PART_KEY], strlen(commandParts[CMD_PART_KEY]),
					commandParts[CMD_PART_TYPE], strlen(commandParts[CMD_PART_TYPE]), commandParts[CMD_PART_VALUE],
					cmdValueLength);
			}

			break;
	}
}

static void actionCompletion(const char *buf, size_t bufLength, linenoiseCompletions *lc,
	const char *partialActionString, size_t partialActionStringLength) {
	UNUSED_ARGUMENT(buf);
	UNUSED_ARGUMENT(bufLength);

	// Always start off with a command.
	for (size_t i = 0; i < actionsLength; i++) {
		if (strncmp(actions[i].name, partialActionString, partialActionStringLength) == 0) {
			linenoiseAddCompletionSuffix(lc, "", 0, actions[i].name, true, false);
		}
	}

	// Add quit and exit too.
	if (strncmp("exit", partialActionString, partialActionStringLength) == 0) {
		linenoiseAddCompletionSuffix(lc, "", 0, "exit", true, false);
	}
	if (strncmp("quit", partialActionString, partialActionStringLength) == 0) {
		linenoiseAddCompletionSuffix(lc, "", 0, "quit", true, false);
	}
}

static void nodeCompletion(const char *buf, size_t bufLength, linenoiseCompletions *lc, uint8_t actionCode,
	const char *partialNodeString, size_t partialNodeStringLength) {
	UNUSED_ARGUMENT(actionCode);

	// If partialNodeString is still empty, the first thing is to complete the root.
	if (partialNodeStringLength == 0) {
		linenoiseAddCompletionSuffix(lc, buf, bufLength, "/", false, false);
		return;
	}

	// Get all the children of the last fully defined node (/ or /../../).
	char *lastNode = strrchr(partialNodeString, '/');
	if (lastNode == NULL) {
		// No / found, invalid, cannot auto-complete.
		return;
	}

	size_t lastNodeLength = (size_t) (lastNode - partialNodeString) + 1;

	uint8_t dataBuffer[1024];

	// Send request for all children names.
	dataBuffer[0] = CAER_CONFIG_GET_CHILDREN;
	dataBuffer[1] = 0; // UNUSED.
	setExtraLen(dataBuffer, 0); // UNUSED.
	setNodeLen(dataBuffer, (uint16_t) (lastNodeLength + 1)); // +1 for terminating NUL byte.
	setKeyLen(dataBuffer, 0); // UNUSED.
	setValueLen(dataBuffer, 0); // UNUSED.

	memcpy(dataBuffer + 10, partialNodeString, lastNodeLength);
	dataBuffer[10 + lastNodeLength] = '\0';

//	if (!sendUntilDone(sockFd, dataBuffer, 10 + lastNodeLength + 1)) {
//		// Failed to contact remote host, no auto-completion!
//		return;
//	}
//
//	if (!recvUntilDone(sockFd, dataBuffer, 4)) {
//		// Failed to contact remote host, no auto-completion!
//		return;
//	}

	// Decode response header fields (all in little-endian).
	uint8_t action = dataBuffer[0];
	uint8_t type = dataBuffer[1];
	uint16_t msgLength = le16toh(*(uint16_t * )(dataBuffer + 2));

	// Total length to get for response.
//	if (!recvUntilDone(sockFd, dataBuffer + 4, msgLength)) {
//		// Failed to contact remote host, no auto-completion!
//		return;
//	}

	if (action == CAER_CONFIG_ERROR || type != SSHS_STRING) {
		// Invalid request made, no auto-completion.
		return;
	}

	// At this point we made a valid request and got back a full response.
	for (size_t i = 0; i < msgLength; i++) {
		if (strncasecmp((const char *) dataBuffer + 4 + i, lastNode + 1, strlen(lastNode + 1)) == 0) {
			linenoiseAddCompletionSuffix(lc, buf, bufLength - strlen(lastNode + 1), (const char *) dataBuffer + 4 + i,
			false, true);
		}

		// Jump to the NUL character after this string.
		i += strlen((const char *) dataBuffer + 4 + i);
	}
}

static void keyCompletion(const char *buf, size_t bufLength, linenoiseCompletions *lc, uint8_t actionCode,
	const char *nodeString, size_t nodeStringLength, const char *partialKeyString, size_t partialKeyStringLength) {
	UNUSED_ARGUMENT(actionCode);

	uint8_t dataBuffer[1024];

	// Send request for all attribute names for this node.
	dataBuffer[0] = CAER_CONFIG_GET_ATTRIBUTES;
	dataBuffer[1] = 0; // UNUSED.
	setExtraLen(dataBuffer, 0); // UNUSED.
	setNodeLen(dataBuffer, (uint16_t) (nodeStringLength + 1)); // +1 for terminating NUL byte.
	setKeyLen(dataBuffer, 0); // UNUSED.
	setValueLen(dataBuffer, 0); // UNUSED.

	memcpy(dataBuffer + 10, nodeString, nodeStringLength);
	dataBuffer[10 + nodeStringLength] = '\0';

//	if (!sendUntilDone(sockFd, dataBuffer, 10 + nodeStringLength + 1)) {
//		// Failed to contact remote host, no auto-completion!
//		return;
//	}
//
//	if (!recvUntilDone(sockFd, dataBuffer, 4)) {
//		// Failed to contact remote host, no auto-completion!
//		return;
//	}

	// Decode response header fields (all in little-endian).
	uint8_t action = dataBuffer[0];
	uint8_t type = dataBuffer[1];
	uint16_t msgLength = le16toh(*(uint16_t * )(dataBuffer + 2));

	// Total length to get for response.
//	if (!recvUntilDone(sockFd, dataBuffer + 4, msgLength)) {
//		// Failed to contact remote host, no auto-completion!
//		return;
//	}

	if (action == CAER_CONFIG_ERROR || type != SSHS_STRING) {
		// Invalid request made, no auto-completion.
		return;
	}

	// At this point we made a valid request and got back a full response.
	for (size_t i = 0; i < msgLength; i++) {
		if (strncasecmp((const char *) dataBuffer + 4 + i, partialKeyString, partialKeyStringLength) == 0) {
			linenoiseAddCompletionSuffix(lc, buf, bufLength - partialKeyStringLength, (const char *) dataBuffer + 4 + i,
			true, false);
		}

		// Jump to the NUL character after this string.
		i += strlen((const char *) dataBuffer + 4 + i);
	}
}

static void typeCompletion(const char *buf, size_t bufLength, linenoiseCompletions *lc, uint8_t actionCode,
	const char *nodeString, size_t nodeStringLength, const char *keyString, size_t keyStringLength,
	const char *partialTypeString, size_t partialTypeStringLength) {
	UNUSED_ARGUMENT(actionCode);

	uint8_t dataBuffer[1024];

	// Send request for all type names for this key on this node.
	dataBuffer[0] = CAER_CONFIG_GET_TYPES;
	dataBuffer[1] = 0; // UNUSED.
	setExtraLen(dataBuffer, 0); // UNUSED.
	setNodeLen(dataBuffer, (uint16_t) (nodeStringLength + 1)); // +1 for terminating NUL byte.
	setKeyLen(dataBuffer, (uint16_t) (keyStringLength + 1)); // +1 for terminating NUL byte.
	setValueLen(dataBuffer, 0); // UNUSED.

	memcpy(dataBuffer + 10, nodeString, nodeStringLength);
	dataBuffer[10 + nodeStringLength] = '\0';

	memcpy(dataBuffer + 10 + nodeStringLength + 1, keyString, keyStringLength);
	dataBuffer[10 + nodeStringLength + 1 + keyStringLength] = '\0';

//	if (!sendUntilDone(sockFd, dataBuffer, 10 + nodeStringLength + 1 + keyStringLength + 1)) {
//		// Failed to contact remote host, no auto-completion!
//		return;
//	}
//
//	if (!recvUntilDone(sockFd, dataBuffer, 4)) {
//		// Failed to contact remote host, no auto-completion!
//		return;
//	}

	// Decode response header fields (all in little-endian).
	uint8_t action = dataBuffer[0];
	uint8_t type = dataBuffer[1];
	uint16_t msgLength = le16toh(*(uint16_t * )(dataBuffer + 2));

	// Total length to get for response.
//	if (!recvUntilDone(sockFd, dataBuffer + 4, msgLength)) {
//		// Failed to contact remote host, no auto-completion!
//		return;
//	}

	if (action == CAER_CONFIG_ERROR || type != SSHS_STRING) {
		// Invalid request made, no auto-completion.
		return;
	}

	// At this point we made a valid request and got back a full response.
	for (size_t i = 0; i < msgLength; i++) {
		if (strncasecmp((const char *) dataBuffer + 4 + i, partialTypeString, partialTypeStringLength) == 0) {
			linenoiseAddCompletionSuffix(lc, buf, bufLength - partialTypeStringLength,
				(const char *) dataBuffer + 4 + i, true, false);
		}

		// Jump to the NUL character after this string.
		i += strlen((const char *) dataBuffer + 4 + i);
	}
}

static void valueCompletion(const char *buf, size_t bufLength, linenoiseCompletions *lc, uint8_t actionCode,
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
				linenoiseAddCompletionSuffix(lc, buf, bufLength - partialValueStringLength, "true", false, false);
			}
			if (strncmp("false", partialValueString, partialValueStringLength) == 0) {
				linenoiseAddCompletionSuffix(lc, buf, bufLength - partialValueStringLength, "false", false, false);
			}
		}

		return;
	}

	uint8_t dataBuffer[1024];

	// Send request for the current value, so we can auto-complete with it as default.
	dataBuffer[0] = CAER_CONFIG_GET;
	dataBuffer[1] = (uint8_t) type;
	setExtraLen(dataBuffer, 0); // UNUSED.
	setNodeLen(dataBuffer, (uint16_t) (nodeStringLength + 1)); // +1 for terminating NUL byte.
	setKeyLen(dataBuffer, (uint16_t) (keyStringLength + 1)); // +1 for terminating NUL byte.
	setValueLen(dataBuffer, 0); // UNUSED.

	memcpy(dataBuffer + 10, nodeString, nodeStringLength);
	dataBuffer[10 + nodeStringLength] = '\0';

	memcpy(dataBuffer + 10 + nodeStringLength + 1, keyString, keyStringLength);
	dataBuffer[10 + nodeStringLength + 1 + keyStringLength] = '\0';

//	if (!sendUntilDone(sockFd, dataBuffer, 10 + nodeStringLength + 1 + keyStringLength + 1)) {
//		// Failed to contact remote host, no auto-completion!
//		return;
//	}
//
//	if (!recvUntilDone(sockFd, dataBuffer, 4)) {
//		// Failed to contact remote host, no auto-completion!
//		return;
//	}

	// Decode response header fields (all in little-endian).
	uint8_t action = dataBuffer[0];
	uint16_t msgLength = le16toh(*(uint16_t * )(dataBuffer + 2));

	// Total length to get for response.
//	if (!recvUntilDone(sockFd, dataBuffer + 4, msgLength)) {
//		// Failed to contact remote host, no auto-completion!
//		return;
//	}

	if (action == CAER_CONFIG_ERROR) {
		// Invalid request made, no auto-completion.
		return;
	}

	// At this point we made a valid request and got back a full response.
	// We can just use it directly and paste it in as completion.
	linenoiseAddCompletionSuffix(lc, buf, bufLength, (const char *) dataBuffer + 4, false, false);

	// If this is a boolean value, we can also add the inverse as a second completion.
	if (type == SSHS_BOOL) {
		if (strcmp((const char *) dataBuffer + 4, "true") == 0) {
			linenoiseAddCompletionSuffix(lc, buf, bufLength, "false", false, false);
		}
		else {
			linenoiseAddCompletionSuffix(lc, buf, bufLength, "true", false, false);
		}
	}
}

static void linenoiseAddCompletionSuffix(linenoiseCompletions *lc, const char *buf, size_t completionPoint,
	const char *suffix, bool endSpace, bool endSlash) {
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

	linenoiseAddCompletion(lc, concat);
}

// Remember to free strings returned by this.
static char *getUserHomeDirectory(void) {
	char *homeDir = NULL;

	// First check the environment for $HOME.
	char *homeVar = getenv("HOME");

	if (homeVar != NULL) {
		homeDir = strdup(homeVar);
	}

	// Else try to get it from the user data storage.
	if (homeDir == NULL) {
		struct passwd userPasswd;
		struct passwd *userPasswdPtr;
		char userPasswdBuf[2048];

		if (getpwuid_r(getuid(), &userPasswd, userPasswdBuf, sizeof(userPasswdBuf), &userPasswdPtr) == 0) {
			homeDir = strdup(userPasswd.pw_dir);
		}
	}

	if (homeDir == NULL) {
		// Else just return /tmp as a place to write to.
		homeDir = strdup("/tmp");
	}

	// Check if anything worked.
	if (homeDir == NULL) {
		return (NULL);
	}

	char *realHomeDir = portable_realpath(homeDir);
	if (realHomeDir == NULL) {
		free(homeDir);

		return (NULL);
	}

	free(homeDir);

	return (realHomeDir);
}
