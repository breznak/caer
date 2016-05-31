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

// Initialize double-indirection contiguous 2D array, so that array[x][y]
// is possible, see http://c-faq.com/aryptr/dynmuldimary.html for info.
#define buffers_define_2d_typed(TYPE, NAME) \
\
	struct simple_2d_buffer_##TYPE { \
	size_t sizeX; \
	size_t sizeY; \
	TYPE *buffer2d[]; \
}; \
\
typedef struct simple_2d_buffer_##TYPE *simple2DBuffer##NAME; \
\
static inline simple2DBuffer##NAME simple2DBufferInit##NAME(size_t sizeX, size_t sizeY) { \
	simple2DBuffer##NAME buffer2d = malloc(sizeof(*buffer2d) + (sizeX * sizeof(TYPE *))); \
	if (buffer2d == NULL) { \
		return (NULL); \
	} \
\
	buffer2d->buffer2d[0] = calloc(sizeX * sizeY, sizeof(TYPE)); \
	if (buffer2d->buffer2d[0] == NULL) { \
		free(buffer2d); \
		return (NULL); \
	} \
\
	for (size_t i = 1; i < sizeX; i++) { \
		buffer2d->buffer2d[i] = buffer2d->buffer2d[0] + (i * sizeY); \
	} \
\
	buffer2d->sizeX = sizeX; \
	buffer2d->sizeY = sizeY; \
\
	return (buffer2d); \
} \
\
static inline void simple2DBufferFree##NAME(simple2DBuffer##NAME buffer2d) { \
	if (buffer2d != NULL) { \
		free(buffer2d->buffer2d[0]); \
		free(buffer2d); \
	} \
}

buffers_define_2d_typed(int8_t, Byte)
buffers_define_2d_typed(int16_t, Short)
buffers_define_2d_typed(int32_t, Int)
buffers_define_2d_typed(int64_t, Long)
buffers_define_2d_typed(float, Float)
buffers_define_2d_typed(double, Double)

#endif /* BUFFERS_H_ */
