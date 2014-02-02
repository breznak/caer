/*
 * ringbuffer.c
 *
 *  Created on: Dec 10, 2013
 *      Author: llongi
 */

#include "ringbuffer.h"
#include "ext/atomic_ops/atomic_ops.h"

struct ring_buffer {
	CACHELINE_ALONE(size_t, putPos)
	;
	CACHELINE_ALONE(size_t, getPos)
	;
	CACHELINE_ALONE(size_t, size)
	;
	atomic_ops_ptr elements[];
};

RingBuffer ringBufferInit(size_t size) {
	// Force multiple of two size for performance.
	if (size == 0 || (size & (size - 1)) != 0) {
		return (NULL);
	}

	RingBuffer rBuf = NULL;
	if (posix_memalign((void **)&rBuf, CACHELINE_SIZE, sizeof(struct ring_buffer) + (size * sizeof(atomic_ops_ptr))) != 0) {
		return (NULL);
	}

	// Initialize counter variables.
	rBuf->putPos = 0;
	rBuf->getPos = 0;
	rBuf->size = size;

	// Initialize pointers.
	for (size_t i = 0; i < size; i++) {
		atomic_ops_ptr_store(&rBuf->elements[i], NULL, ATOMIC_OPS_FENCE_NONE);
	}

	atomic_ops_fence(ATOMIC_OPS_FENCE_RELEASE);

	return (rBuf);
}

void ringBufferFree(RingBuffer rBuf) {
	free(rBuf);
}

bool ringBufferPut(RingBuffer rBuf, void *elem) {
	if (elem == NULL) {
		// NULL elements are disallowed (used as place-holders).
		// Critical error, should never happen -> exit!
		exit(EXIT_FAILURE);
	}

	void *curr = atomic_ops_ptr_load(&rBuf->elements[rBuf->putPos], ATOMIC_OPS_FENCE_ACQUIRE);

	// If the place where we want to put the new element is NULL, it's still
	// free and we can use it.
	if (curr == NULL) {
		atomic_ops_ptr_store(&rBuf->elements[rBuf->putPos], elem, ATOMIC_OPS_FENCE_RELEASE);

		// Increase local put pointer.
		rBuf->putPos = ((rBuf->putPos + 1) & (rBuf->size - 1));

		return (true);
	}

	// Else, buffer is full.
	return (false);
}

void *ringBufferGet(RingBuffer rBuf) {
	void *curr = atomic_ops_ptr_load(&rBuf->elements[rBuf->getPos], ATOMIC_OPS_FENCE_ACQUIRE);

	// If the place where we want to get an element from is not NULL, there
	// is valid content there, which we return, and reset the place to NULL.
	if (curr != NULL) {
		atomic_ops_ptr_store(&rBuf->elements[rBuf->getPos], NULL, ATOMIC_OPS_FENCE_RELEASE);

		// Increase local get pointer.
		rBuf->getPos = ((rBuf->getPos + 1) & (rBuf->size - 1));

		return (curr);
	}

	// Else, buffer is empty.
	return (NULL);
}

void *ringBufferLook(RingBuffer rBuf) {
	void *curr = atomic_ops_ptr_load(&rBuf->elements[rBuf->getPos], ATOMIC_OPS_FENCE_ACQUIRE);

	// If the place where we want to get an element from is not NULL, there
	// is valid content there, which we return, without removing it from the
	// ring buffer.
	if (curr != NULL) {
		return (curr);
	}

	// Else, buffer is empty.
	return (NULL);
}
