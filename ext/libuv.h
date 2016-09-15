#ifndef EXT_LIBUV_H_
#define EXT_LIBUV_H_

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <uv.h>
#include "ext/buffers.h"

#define UV_RET_CHECK(RET_VAL, SUBSYS_NAME, FUNC_NAME, CLEANUP_ACTIONS) \
	if (RET_VAL < 0) { \
		caerLog(CAER_LOG_ERROR, SUBSYS_NAME, FUNC_NAME " failed, error %d (%s).", \
			RET_VAL, uv_err_name(RET_VAL)); \
		CLEANUP_ACTIONS; \
	}

static inline bool simpleBufferFileWrite(uv_loop_t *loop, uv_file file, int64_t fileOffset, simpleBuffer buffer) {
	if (buffer->bufferUsedSize > buffer->bufferSize) {
		// Using more memory than available, this can't work!
		return (false);
	}

	if (buffer->bufferPosition >= buffer->bufferUsedSize) {
		// Position is after any valid data, this can't work!
		return (false);
	}

	int retVal;
	uv_fs_t fileWrite;
	uv_buf_t writeBuffer;

	size_t curWritten = 0;
	size_t bytesToWrite = buffer->bufferUsedSize - buffer->bufferPosition;

	while (curWritten < bytesToWrite) {
		writeBuffer.base = (char *) buffer->buffer + buffer->bufferPosition + curWritten;
		writeBuffer.len = bytesToWrite - curWritten;

		retVal = uv_fs_write(loop, &fileWrite, file, &writeBuffer, 1, fileOffset + (int64_t) curWritten, NULL);
		if (retVal < 0) {
			errno = retVal;
			break;
		}

		if (fileWrite.result < 0) {
			// Error.
			errno = (int) fileWrite.result;
			break;
		}
		else if (fileWrite.result == 0) {
			// Nothing was written, but also no errors, so we try again.
			continue;
		}

		curWritten += (size_t) fileWrite.result;
	}

	uv_fs_req_cleanup(&fileWrite);

	return (curWritten == bytesToWrite);
}

static inline bool simpleBufferFileRead(uv_loop_t *loop, uv_file file, int64_t fileOffset, simpleBuffer buffer) {
	if (buffer->bufferPosition >= buffer->bufferSize) {
		// Position is after maximum capacity, this can't work!
		return (false);
	}

	int retVal;
	uv_fs_t fileRead;
	uv_buf_t readBuffer;

	size_t curRead = 0;
	size_t bytesToRead = buffer->bufferSize - buffer->bufferPosition;

	while (curRead < bytesToRead) {
		readBuffer.base = (char *) buffer->buffer + buffer->bufferPosition + curRead;
		readBuffer.len = bytesToRead - curRead;

		retVal = uv_fs_read(loop, &fileRead, file, &readBuffer, 1, fileOffset + (int64_t) curRead, NULL);
		if (retVal < 0) {
			errno = retVal;
			break;
		}

		if (fileRead.result < 0) {
			// Error.
			errno = (int) fileRead.result;
			break;
		}
		else if (fileRead.result == 0) {
			// End of File reached.
			errno = 0;
			break;
		}

		curRead += (size_t) fileRead.result;
	}

	uv_fs_req_cleanup(&fileRead);

	if (curRead == bytesToRead) {
		// Actual data, update UsedSize.
		buffer->bufferUsedSize = buffer->bufferPosition + (size_t) curRead;
		return (true);
	}
	else {
		return (false);
	}
}

struct libuvWriteBufStruct {
	uv_buf_t buf;
	void *data; // Allow arbitrary data to be attached to buffer. Must be on heap.
	uint8_t dataBuf[];
};

typedef struct libuvWriteBufStruct *libuvWriteBuf;

static inline libuvWriteBuf libuvWriteBufInit(size_t size) {
	libuvWriteBuf writeBuf = calloc(1, sizeof(*writeBuf) + size);
	if (writeBuf == NULL) {
		return (NULL);
	}

	writeBuf->buf.base = (char *) writeBuf->dataBuf;
	writeBuf->buf.len = size;

	return (writeBuf);
}

static inline void libuvCloseFree(uv_handle_t *handle) {
	free(handle->data);
	free(handle);
}

static inline void libuvCloseLoopWalk(uv_handle_t *handle, void *arg) {
	(void) (arg); // UNUSED.

	if (!uv_is_closing(handle)) {
		uv_close(handle, &libuvCloseFree);
	}
}

static inline int libuvCloseLoopHandles(uv_loop_t *loop) {
	uv_walk(loop, &libuvCloseLoopWalk, NULL);

	return (uv_run(loop, UV_RUN_DEFAULT));
}

static inline void libuvWriteFree(uv_write_t *writeRequest, int status) {
	(void) (status); // UNUSED.

	libuvWriteBuf buf = writeRequest->data;
	free(buf->data);
	free(buf);

	free(writeRequest);
}

// buffer has to be dynamically allocated (on heap). On success, will get free'd
// automatically. On failure, buffer won't be touched.
static inline int libuvWrite(uv_stream_t *dest, libuvWriteBuf buffer) {
	uv_write_t *writeRequest = calloc(1, sizeof(*writeRequest));
	if (writeRequest == NULL) {
		return (UV_ENOMEM);
	}

	writeRequest->data = buffer;

	int retVal = uv_write(writeRequest, dest, &buffer->buf, 1, &libuvWriteFree);
	if (retVal < 0) {
		free(writeRequest);
	}

	return (retVal);
}

#endif /* EXT_LIBUV_H_ */
