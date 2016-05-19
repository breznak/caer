#ifndef BUFFERS_H_
#define BUFFERS_H_

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "nets.h"

struct simple_buffer {
	/// Current position inside buffer.
	size_t bufferPosition;
	/// Size of data currently inside buffer, in bytes.
	size_t bufferUsedSize;
	/// Size of buffer, in bytes.
	size_t bufferSize;
	/// Buffer for R/W to file descriptor (buffered I/O).
	uint8_t buffer[];
};

typedef struct simple_buffer *simpleBuffer;

static inline simpleBuffer simpleBufferInit(size_t size) {
	// Allocate new buffer.
	simpleBuffer newBuffer = malloc(sizeof(*newBuffer) + (size * sizeof(uint8_t)));
	if (newBuffer == NULL) {
		return (NULL);
	}

	// Update new buffer size information.
	newBuffer->bufferSize = size;
	newBuffer->bufferUsedSize = 0;
	newBuffer->bufferPosition = 0;

	return (newBuffer);
}

static inline bool simpleBufferWrite(int fd, simpleBuffer buffer) {
	if (buffer->bufferUsedSize > buffer->bufferSize) {
		// Using more memory than available, this can't work!
		return (false);
	}

	if (buffer->bufferPosition > buffer->bufferUsedSize) {
		// Position is after any valid data, this can't work!
		return (false);
	}

	return (writeUntilDone(fd, buffer->buffer + buffer->bufferPosition, buffer->bufferUsedSize - buffer->bufferPosition));
}

static inline bool simpleBufferRead(int fd, simpleBuffer buffer) {
	if (buffer->bufferPosition > buffer->bufferSize) {
		// Position is after any valid data, this can't work!
		return (false);
	}

	// Try to fill whole buffer.
	ssize_t result = readUntilDone(fd, buffer->buffer + buffer->bufferPosition,
		buffer->bufferSize - buffer->bufferPosition);

	if (result < 0) {
		// Error.
		return (false);
	}
	else if (result == 0) {
		// End of File reached.
		errno = 0;
		return (false);
	}
	else {
		// Actual data, update UsedSize.
		buffer->bufferUsedSize = buffer->bufferPosition + (size_t) result;
		return (true);
	}
}

#endif /* BUFFERS_H_ */
