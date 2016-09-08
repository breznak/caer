#include "config_server.h"
#include <stdatomic.h>
#include "ext/libuv.h"

#ifdef HAVE_PTHREADS
#include "ext/c11threads_posix.h"
#endif

#define CONFIG_SERVER_NAME "Config Server"
#define UV_RET_CHECK_CS(RET_VAL, FUNC_NAME, CLEANUP_ACTIONS) UV_RET_CHECK(RET_VAL, CONFIG_SERVER_NAME, FUNC_NAME, CLEANUP_ACTIONS)

static struct {
	atomic_bool running;
	uv_async_t asyncShutdown;
	thrd_t thread;
} configServerThread;

static int caerConfigServerRunner(void *inPtr);
static void caerConfigServerHandleRequest(uv_stream_t *client, uint8_t action, uint8_t type, const uint8_t *extra,
	size_t extraLength, const uint8_t *node, size_t nodeLength, const uint8_t *key, size_t keyLength,
	const uint8_t *value, size_t valueLength);

void caerConfigServerStart(void) {
	// Start the thread.
	if ((errno = thrd_create(&configServerThread.thread, &caerConfigServerRunner, NULL)) == thrd_success) {
		// Successfully started thread.
		caerLog(CAER_LOG_DEBUG, CONFIG_SERVER_NAME, "Thread created successfully.");
	}
	else {
		// Failed to create thread.
		caerLog(CAER_LOG_EMERGENCY, CONFIG_SERVER_NAME, "Failed to create thread. Error: %d.", errno);
		exit(EXIT_FAILURE);
	}
}

void caerConfigServerStop(void) {
	// Only execute if the server thread was actually started.
	if (!atomic_load(&configServerThread.running)) {
		return;
	}

	// Disable the configuration server thread.
	uv_async_send(&configServerThread.asyncShutdown);

	// Then wait on it to finish.
	if ((errno = thrd_join(configServerThread.thread, NULL)) == thrd_success) {
		// Successfully joined thread.
		caerLog(CAER_LOG_DEBUG, CONFIG_SERVER_NAME, "Thread terminated successfully.");
	}
	else {
		// Failed to join thread.
		caerLog(CAER_LOG_EMERGENCY, CONFIG_SERVER_NAME, "Failed to terminate thread. Error: %d.", errno);
		exit(EXIT_FAILURE);
	}
}

// Control message format: 1 byte ACTION, 1 byte TYPE, 2 bytes EXTRA_LEN,
// 2 bytes NODE_LEN, 2 bytes KEY_LEN, 2 bytes VALUE_LEN, then up to 4086
// bytes split between EXTRA, NODE, KEY, VALUE (with 4 bytes for NUL).
// Basically: (EXTRA_LEN + NODE_LEN + KEY_LEN + VALUE_LEN) <= 4086.
// EXTRA, NODE, KEY, VALUE have to be NUL terminated, and their length
// must include the NUL termination byte.
// This results in a maximum message size of 4096 bytes (4KB).
#define CONFIG_SERVER_BUFFER_SIZE 4096
#define CONFIG_HEADER_LENGTH 10

static void configServerConnection(uv_stream_t *server, int status);
static void configServerAlloc(uv_handle_t *client, size_t suggestedSize, uv_buf_t *buf);
static void configServerRead(uv_stream_t *client, ssize_t nread, const uv_buf_t *buf);
static void configServerShutdown(uv_shutdown_t *clientShutdown, int status);
static void configServerWrite(uv_write_t *clientWrite, int status);
static void configServerAsyncShutdown(uv_async_t *asyncShutdown);

static void configServerConnection(uv_stream_t *server, int status) {
	UV_RET_CHECK_CS(status, "Connection", return);

	uv_tcp_t *tcpClient = calloc(1, sizeof(*tcpClient));
	if (tcpClient == NULL) {
		caerLog(CAER_LOG_ERROR, CONFIG_SERVER_NAME, "Failed to allocate memory for new client.");
		return;
	}

	int retVal;

	retVal = uv_tcp_init(server->loop, tcpClient);
	UV_RET_CHECK_CS(retVal, "uv_tcp_init", free(tcpClient));

	retVal = uv_accept(server, (uv_stream_t *) tcpClient);
	UV_RET_CHECK_CS(retVal, "uv_accept", uv_close((uv_handle_t *) tcpClient, &libuvCloseFree));

	retVal = uv_read_start((uv_stream_t *) tcpClient, &configServerAlloc, &configServerRead);
	UV_RET_CHECK_CS(retVal, "uv_read_start", uv_close((uv_handle_t *) tcpClient, &libuvCloseFree));
}

static void configServerAlloc(uv_handle_t *client, size_t suggestedSize, uv_buf_t *buf) {
	UNUSED_ARGUMENT(suggestedSize);

	// We use one buffer per connection, with a fixed maximum size, and
	// re-use it until we have read a full message.
	if (client->data == NULL) {
		client->data = simpleBufferInit(CONFIG_SERVER_BUFFER_SIZE);

		if (client->data == NULL) {
			// Allocation failure!
			buf->base = NULL;
			buf->len = 0;

			caerLog(CAER_LOG_ERROR, CONFIG_SERVER_NAME, "Failed to allocate memory for client buffer.");
			return;
		}
	}

	simpleBuffer dataBuf = client->data;

	buf->base = (char *) (dataBuf->buffer + dataBuf->bufferUsedSize);
	buf->len = dataBuf->bufferSize - dataBuf->bufferUsedSize;
}

static void configServerRead(uv_stream_t *client, ssize_t sizeRead, const uv_buf_t *buf) {
	UNUSED_ARGUMENT(buf); // Use our own buffer directly.

	// sizeRead < 0: Error or EndOfFile (EOF).
	if (sizeRead < 0) {
		if (sizeRead == UV_EOF) {
			caerLog(CAER_LOG_INFO, CONFIG_SERVER_NAME, "Client %d closed connection.", client->accepted_fd);
		}
		else {
			caerLog(CAER_LOG_ERROR, CONFIG_SERVER_NAME, "Read failed, error %ld (%s).", sizeRead,
				uv_err_name((int) sizeRead));
		}

		// Close connection.
		uv_shutdown_t *clientShutdown = calloc(1, sizeof(*clientShutdown));
		if (clientShutdown == NULL) {
			caerLog(CAER_LOG_ERROR, CONFIG_SERVER_NAME, "Failed to allocate memory for client shutdown.");

			// Hard close.
			uv_close((uv_handle_t *) client, &libuvCloseFree);
			return;
		}

		int retVal = uv_shutdown(clientShutdown, client, &configServerShutdown);
		UV_RET_CHECK_CS(retVal, "uv_shutdown",
			free(clientShutdown); uv_close((uv_handle_t *) client, &libuvCloseFree));
	}

	// sizeRead == 0: EAGAIN, do nothing.

	// sizeRead > 0: received data.
	if (sizeRead > 0) {
		simpleBuffer dataBuf = client->data;

		// Update main client buffer with just read data.
		dataBuf->bufferUsedSize += (size_t) sizeRead;

		// If we have enough data, we start parsing the lengths.
		// The main header is 10 bytes.
		if (dataBuf->bufferUsedSize >= CONFIG_HEADER_LENGTH) {
			// Decode length header fields (all in little-endian).
			uint16_t extraLength = le16toh(*(uint16_t * )(dataBuf->buffer + 2));
			uint16_t nodeLength = le16toh(*(uint16_t * )(dataBuf->buffer + 4));
			uint16_t keyLength = le16toh(*(uint16_t * )(dataBuf->buffer + 6));
			uint16_t valueLength = le16toh(*(uint16_t * )(dataBuf->buffer + 8));

			// Total length to get for command.
			size_t readLength = (size_t) (extraLength + nodeLength + keyLength + valueLength);

			if (dataBuf->bufferUsedSize >= (CONFIG_HEADER_LENGTH + readLength)) {
				// Decode remaining header fields.
				uint8_t action = dataBuf->buffer[0];
				uint8_t type = dataBuf->buffer[1];

				// Now we have everything. The header fields are already
				// fully decoded: handle request (and send back data eventually).
				caerConfigServerHandleRequest(client, action, type, dataBuf->buffer + CONFIG_HEADER_LENGTH, extraLength,
					dataBuf->buffer + CONFIG_HEADER_LENGTH + extraLength, nodeLength,
					dataBuf->buffer + CONFIG_HEADER_LENGTH + extraLength + nodeLength, keyLength,
					dataBuf->buffer + CONFIG_HEADER_LENGTH + extraLength + nodeLength + keyLength, valueLength);

				// Reset buffer for next request.
				dataBuf->bufferUsedSize = 0;
			}
		}
	}
}

static void configServerShutdown(uv_shutdown_t *clientShutdown, int status) {
	libuvCloseFree((uv_handle_t *) clientShutdown);

	uv_close((uv_handle_t *) clientShutdown->handle, &libuvCloseFree);

	UV_RET_CHECK_CS(status, "AfterShutdown", return);
}


static void configServerWrite(uv_write_t *clientWrite, int status) {
	libuvCloseFree((uv_handle_t *) clientWrite);

	UV_RET_CHECK_CS(status, "AfterWrite", return);
}

static void configServerAsyncShutdown(uv_async_t *asyncShutdown) {
	uv_stop(asyncShutdown->loop);
}

static int caerConfigServerRunner(void *inPtr) {
	UNUSED_ARGUMENT(inPtr);

	// Set thread name.
	thrd_set_name("ConfigServer");

	// Get the right configuration node first.
	sshsNode serverNode = sshsGetNode(sshsGetGlobal(), "/server/");

	// Ensure default values are present.
	sshsNodePutStringIfAbsent(serverNode, "ipAddress", "127.0.0.1");
	sshsNodePutIntIfAbsent(serverNode, "portNumber", 4040);
	sshsNodePutShortIfAbsent(serverNode, "backlogSize", 5);

	int retVal;
	bool eventLoopInitialized = false;
	bool tcpServerInitialized = false;

	// Main event loop for handling connections.
	uv_loop_t configServerLoop;
	retVal = uv_loop_init(&configServerLoop);
	UV_RET_CHECK_CS(retVal, "uv_loop_init", goto loopCleanup);
	eventLoopInitialized = true;

	// Initialize async callback to stop the event loop on termination.
	retVal = uv_async_init(&configServerLoop, &configServerThread.asyncShutdown, &configServerAsyncShutdown);
	UV_RET_CHECK_CS(retVal, "uv_async_init", goto loopCleanup);
	atomic_store(&configServerThread.running, true);

	// Open a TCP server socket for configuration handling.
	// TCP chosen for reliability, which is more important here than speed.
	uv_tcp_t configServerTCP;
	retVal = uv_tcp_init(&configServerLoop, &configServerTCP);
	UV_RET_CHECK_CS(retVal, "uv_tcp_init", goto loopCleanup);
	tcpServerInitialized = true;

	// Generate address.
	struct sockaddr_in configServerAddress;

	char *ipAddress = sshsNodeGetString(serverNode, "ipAddress");
	retVal = uv_ip4_addr(ipAddress, sshsNodeGetInt(serverNode, "portNumber"), &configServerAddress);
	UV_RET_CHECK_CS(retVal, "uv_ip4_addr", free(ipAddress); goto loopCleanup);
	free(ipAddress);

	// Bind socket to above address.
	retVal = uv_tcp_bind(&configServerTCP, (struct sockaddr *) &configServerAddress, 0);
	UV_RET_CHECK_CS(retVal, "uv_tcp_bind", goto loopCleanup);

	// Listen to new connections on the socket.
	retVal = uv_listen((uv_stream_t *) &configServerTCP, sshsNodeGetShort(serverNode, "backlogSize"),
		&configServerConnection);
	UV_RET_CHECK_CS(retVal, "uv_listen", goto loopCleanup);

	// Run event loop.
	retVal = uv_run(&configServerLoop, UV_RUN_DEFAULT);
	UV_RET_CHECK_CS(retVal, "uv_run", goto loopCleanup);

	// Cleanup event loop and memory.
	loopCleanup: {
		bool errorCleanup = (retVal < 0);

		if (tcpServerInitialized) {
			uv_close((uv_handle_t *) &configServerTCP, NULL);
		}

		if (atomic_load(&configServerThread.running)) {
			uv_close((uv_handle_t *) &configServerThread.asyncShutdown, NULL);
		}

		// Cleanup all remaining handles and run until all callbacks are done.
		retVal = libuvCloseLoopHandles(&configServerLoop);
		UV_RET_CHECK_CS(retVal, "libuvCloseLoopHandles",);

		if (eventLoopInitialized) {
			retVal = uv_loop_close(&configServerLoop);
			UV_RET_CHECK_CS(retVal, "uv_loop_close",);
		}

		// Mark the configuration server thread as stopped.
		atomic_store(&configServerThread.running, false);

		if (errorCleanup) {
			return (EXIT_FAILURE);
		}
		else {
			return (EXIT_SUCCESS);
		}
	}
}

// The response from the server follows a simplified version of the request
// protocol. A byte for ACTION, a byte for TYPE, 2 bytes for MSG_LEN and then
// up to 4092 bytes of MSG, for a maximum total of 4096 bytes again.
// MSG must be NUL terminated, and the NUL byte shall be part of the length.
static inline void setMsgLen(uint8_t *buf, uint16_t msgLen) {
	*((uint16_t *) (buf + 2)) = htole16(msgLen);
}

static inline void caerConfigSendError(uv_stream_t *client, const char *errorMsg) {
	size_t errorMsgLength = strlen(errorMsg);
	size_t responseLength = 4 + errorMsgLength + 1; // +1 for terminating NUL byte.

	libuvWriteBuf response = libuvWriteBufInit(responseLength);
	if (response == NULL) {
		caerLog(CAER_LOG_ERROR, CONFIG_SERVER_NAME, "Failed to allocate memory for client error response.");
		return;
	}

	response->dataBuf[0] = CAER_CONFIG_ERROR;
	response->dataBuf[1] = SSHS_STRING;
	setMsgLen(response->dataBuf, (uint16_t) (errorMsgLength + 1));
	memcpy(response->dataBuf + 4, errorMsg, errorMsgLength);
	response->dataBuf[4 + errorMsgLength] = '\0';

	uv_write_t *clientWrite = calloc(1, sizeof(*clientWrite));
	if (clientWrite == NULL) {
		free(response);

		caerLog(CAER_LOG_ERROR, CONFIG_SERVER_NAME, "Failed to allocate memory for client error write.");
		return;
	}

	clientWrite->data = response;

	int retVal = uv_write(clientWrite, client, &response->buf, 1, &configServerWrite);
	UV_RET_CHECK_CS(retVal, "uv_write", free(response); free(clientWrite));

	caerLog(CAER_LOG_DEBUG, "Config Server", "Sent back error message '%s' to client.", errorMsg);
}

static inline void caerConfigSendResponse(uv_stream_t *client, uint8_t action, uint8_t type, const uint8_t *msg,
	size_t msgLength) {
	size_t responseLength = 4 + msgLength;

	libuvWriteBuf response = libuvWriteBufInit(responseLength);
	if (response == NULL) {
		caerLog(CAER_LOG_ERROR, CONFIG_SERVER_NAME, "Failed to allocate memory for client response.");
		return;
	}

	response->dataBuf[0] = action;
	response->dataBuf[1] = type;
	setMsgLen(response->dataBuf, (uint16_t) msgLength);
	memcpy(response->dataBuf + 4, msg, msgLength);
	// Msg must already be NUL terminated!

	uv_write_t *clientWrite = calloc(1, sizeof(*clientWrite));
	if (clientWrite == NULL) {
		free(response);

		caerLog(CAER_LOG_ERROR, CONFIG_SERVER_NAME, "Failed to allocate memory for client write.");
		return;
	}

	clientWrite->data = response;

	int retVal = uv_write(clientWrite, client, &response->buf, 1, &configServerWrite);
	UV_RET_CHECK_CS(retVal, "uv_write", free(response); free(clientWrite));

	caerLog(CAER_LOG_DEBUG, "Config Server",
		"Sent back message to client: action=%" PRIu8 ", type=%" PRIu8 ", msgLength=%zu.", action, type, msgLength);
}

static void caerConfigServerHandleRequest(uv_stream_t *client, uint8_t action, uint8_t type, const uint8_t *extra,
	size_t extraLength, const uint8_t *node, size_t nodeLength, const uint8_t *key, size_t keyLength,
	const uint8_t *value, size_t valueLength) {
	UNUSED_ARGUMENT(extra);

	caerLog(CAER_LOG_DEBUG, "Config Server",
		"Handling request: action=%" PRIu8 ", type=%" PRIu8 ", extraLength=%zu, nodeLength=%zu, keyLength=%zu, valueLength=%zu.",
		action, type, extraLength, nodeLength, keyLength, valueLength);

	// Interpretation of data is up to each action individually.
	sshs configStore = sshsGetGlobal();

	switch (action) {
		case CAER_CONFIG_NODE_EXISTS: {
			// We only need the node name here. Type is not used (ignored)!
			bool result = sshsExistsNode(configStore, (const char *) node);

			// Send back result to client. Format is the same as incoming data.
			const uint8_t *sendResult = (const uint8_t *) ((result) ? ("true") : ("false"));
			size_t sendResultLength = (result) ? (5) : (6);
			caerConfigSendResponse(client, CAER_CONFIG_NODE_EXISTS, SSHS_BOOL, sendResult, sendResultLength);

			break;
		}

		case CAER_CONFIG_ATTR_EXISTS: {
			bool nodeExists = sshsExistsNode(configStore, (const char *) node);

			// Only allow operations on existing nodes, this is for remote
			// control, so we only manipulate what's already there!
			if (!nodeExists) {
				// Send back error message to client.
				caerConfigSendError(client, "Node doesn't exist. Operations are only allowed on existing data.");

				break;
			}

			// This cannot fail, since we know the node exists from above.
			sshsNode wantedNode = sshsGetNode(configStore, (const char *) node);

			// Check if attribute exists.
			bool result = sshsNodeAttributeExists(wantedNode, (const char *) key, type);

			// Send back result to client. Format is the same as incoming data.
			const uint8_t *sendResult = (const uint8_t *) ((result) ? ("true") : ("false"));
			size_t sendResultLength = (result) ? (5) : (6);
			caerConfigSendResponse(client, CAER_CONFIG_ATTR_EXISTS, SSHS_BOOL, sendResult, sendResultLength);

			break;
		}

		case CAER_CONFIG_GET: {
			bool nodeExists = sshsExistsNode(configStore, (const char *) node);

			// Only allow operations on existing nodes, this is for remote
			// control, so we only manipulate what's already there!
			if (!nodeExists) {
				// Send back error message to client.
				caerConfigSendError(client, "Node doesn't exist. Operations are only allowed on existing data.");

				break;
			}

			// This cannot fail, since we know the node exists from above.
			sshsNode wantedNode = sshsGetNode(configStore, (const char *) node);

			// Check if attribute exists. Only allow operations on existing attributes!
			bool attrExists = sshsNodeAttributeExists(wantedNode, (const char *) key, type);

			if (!attrExists) {
				// Send back error message to client.
				caerConfigSendError(client,
					"Attribute of given type doesn't exist. Operations are only allowed on existing data.");

				break;
			}

			union sshs_node_attr_value result = sshsNodeGetAttribute(wantedNode, (const char *) key, type);

			char *resultStr = sshsHelperValueToStringConverter(type, result);

			if (resultStr == NULL) {
				// Send back error message to client.
				caerConfigSendError(client, "Failed to allocate memory for value string.");
			}
			else {
				caerConfigSendResponse(client, CAER_CONFIG_GET, type, (const uint8_t *) resultStr,
					strlen(resultStr) + 1);

				free(resultStr);
			}

			// If this is a string, we must remember to free the original result.str
			// too, since it will also be a copy of the string coming from SSHS.
			if (type == SSHS_STRING) {
				free(result.string);
			}

			break;
		}

		case CAER_CONFIG_PUT: {
			bool nodeExists = sshsExistsNode(configStore, (const char *) node);

			// Only allow operations on existing nodes, this is for remote
			// control, so we only manipulate what's already there!
			if (!nodeExists) {
				// Send back error message to client.
				caerConfigSendError(client, "Node doesn't exist. Operations are only allowed on existing data.");

				break;
			}

			// This cannot fail, since we know the node exists from above.
			sshsNode wantedNode = sshsGetNode(configStore, (const char *) node);

			// Check if attribute exists. Only allow operations on existing attributes!
			bool attrExists = sshsNodeAttributeExists(wantedNode, (const char *) key, type);

			if (!attrExists) {
				// Send back error message to client.
				caerConfigSendError(client,
					"Attribute of given type doesn't exist. Operations are only allowed on existing data.");

				break;
			}

			// Put given value into config node. Node, attr and type are already verified.
			const char *typeStr = sshsHelperTypeToStringConverter(type);
			if (!sshsNodeStringToNodeConverter(wantedNode, (const char *) key, typeStr, (const char *) value)) {
				// Send back error message to client.
				caerConfigSendError(client, "Impossible to convert value according to type.");

				break;
			}

			// Send back confirmation to the client.
			caerConfigSendResponse(client, CAER_CONFIG_PUT, SSHS_BOOL, (const uint8_t *) "true", 5);

			break;
		}

		case CAER_CONFIG_GET_CHILDREN: {
			bool nodeExists = sshsExistsNode(configStore, (const char *) node);

			// Only allow operations on existing nodes, this is for remote
			// control, so we only manipulate what's already there!
			if (!nodeExists) {
				// Send back error message to client.
				caerConfigSendError(client, "Node doesn't exist. Operations are only allowed on existing data.");

				break;
			}

			// This cannot fail, since we know the node exists from above.
			sshsNode wantedNode = sshsGetNode(configStore, (const char *) node);

			// Get the names of all the child nodes and return them.
			size_t numNames;
			const char **childNames = sshsNodeGetChildNames(wantedNode, &numNames);

			// No children at all, return empty.
			if (childNames == NULL) {
				// Send back error message to client.
				caerConfigSendError(client, "Node has no children.");

				break;
			}

			// We need to return a big string with all of the child names,
			// separated by a NUL character.
			size_t namesLength = 0;

			for (size_t i = 0; i < numNames; i++) {
				namesLength += strlen(childNames[i]) + 1; // +1 for terminating NUL byte.
			}

			// Allocate a buffer for the names and copy them over.
			char namesBuffer[namesLength];

			for (size_t i = 0, acc = 0; i < numNames; i++) {
				size_t len = strlen(childNames[i]) + 1;
				memcpy(namesBuffer + acc, childNames[i], len);
				acc += len;
			}

			caerConfigSendResponse(client, CAER_CONFIG_GET_CHILDREN, SSHS_STRING, (const uint8_t *) namesBuffer,
				namesLength);

			break;
		}

		case CAER_CONFIG_GET_ATTRIBUTES: {
			bool nodeExists = sshsExistsNode(configStore, (const char *) node);

			// Only allow operations on existing nodes, this is for remote
			// control, so we only manipulate what's already there!
			if (!nodeExists) {
				// Send back error message to client.
				caerConfigSendError(client, "Node doesn't exist. Operations are only allowed on existing data.");

				break;
			}

			// This cannot fail, since we know the node exists from above.
			sshsNode wantedNode = sshsGetNode(configStore, (const char *) node);

			// Get the keys of all the attributes and return them.
			size_t numKeys;
			const char **attrKeys = sshsNodeGetAttributeKeys(wantedNode, &numKeys);

			// No attributes at all, return empty.
			if (attrKeys == NULL) {
				// Send back error message to client.
				caerConfigSendError(client, "Node has no attributes.");

				break;
			}

			// We need to return a big string with all of the attribute keys,
			// separated by a NUL character.
			size_t keysLength = 0;

			for (size_t i = 0; i < numKeys; i++) {
				keysLength += strlen(attrKeys[i]) + 1; // +1 for terminating NUL byte.
			}

			// Allocate a buffer for the keys and copy them over.
			char keysBuffer[keysLength];

			for (size_t i = 0, acc = 0; i < numKeys; i++) {
				size_t len = strlen(attrKeys[i]) + 1;
				memcpy(keysBuffer + acc, attrKeys[i], len);
				acc += len;
			}

			caerConfigSendResponse(client, CAER_CONFIG_GET_ATTRIBUTES, SSHS_STRING, (const uint8_t *) keysBuffer,
				keysLength);

			break;
		}

		case CAER_CONFIG_GET_TYPES: {
			bool nodeExists = sshsExistsNode(configStore, (const char *) node);

			// Only allow operations on existing nodes, this is for remote
			// control, so we only manipulate what's already there!
			if (!nodeExists) {
				// Send back error message to client.
				caerConfigSendError(client, "Node doesn't exist. Operations are only allowed on existing data.");

				break;
			}

			// This cannot fail, since we know the node exists from above.
			sshsNode wantedNode = sshsGetNode(configStore, (const char *) node);

			// Check if any keys match the given one and return its types.
			size_t numTypes;
			enum sshs_node_attr_value_type *attrTypes = sshsNodeGetAttributeTypes(wantedNode, (const char *) key,
				&numTypes);

			// No attributes for specified key, return empty.
			if (attrTypes == NULL) {
				// Send back error message to client.
				caerConfigSendError(client, "Node has no attributes with specified key.");

				break;
			}

			// We need to return a big string with all of the attribute types,
			// separated by a NUL character.
			size_t typesLength = 0;

			for (size_t i = 0; i < numTypes; i++) {
				const char *typeString = sshsHelperTypeToStringConverter(attrTypes[i]);
				typesLength += strlen(typeString) + 1; // +1 for terminating NUL byte.
			}

			// Allocate a buffer for the types and copy them over.
			char typesBuffer[typesLength];

			for (size_t i = 0, acc = 0; i < numTypes; i++) {
				const char *typeString = sshsHelperTypeToStringConverter(attrTypes[i]);
				size_t len = strlen(typeString) + 1;
				memcpy(typesBuffer + acc, typeString, len);
				acc += len;
			}

			caerConfigSendResponse(client, CAER_CONFIG_GET_TYPES, SSHS_STRING, (const uint8_t *) typesBuffer,
				typesLength);

			break;
		}

		default:
			// Unknown action, send error back to client.
			caerConfigSendError(client, "Unknown action.");

			break;
	}
}
