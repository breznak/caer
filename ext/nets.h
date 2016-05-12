/*
 * nets.h
 *
 *  Created on: Jan 19, 2014
 *      Author: chtekk
 */

#ifndef NETS_H_
#define NETS_H_

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <fcntl.h>

static inline bool socketBlockingMode(int fd, bool blocking) {
	if (fd < 0) {
		return (false);
	}

	int currFlags = fcntl(fd, F_GETFL, 0);
	if (currFlags < 0) {
		return (false);
	}

	if (blocking) {
		currFlags &= ~O_NONBLOCK;
	}
	else {
		currFlags |= O_NONBLOCK;
	}

	return (fcntl(fd, F_SETFL, currFlags) == 0);
}

static inline bool socketReuseAddr(int fd, bool reuseAddr) {
	if (fd < 0) {
		return (false);
	}

	int val = 0;

	if (reuseAddr) {
		val = 1;
	}

	return (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(int)) == 0);
}

// Write toWrite bytes to the socket sock from buffer.
// Return true on success, false on failure.
static inline bool sendUntilDone(int sock, void *buffer, size_t toWrite) {
	uint8_t *dataBuffer = buffer;
	size_t curWritten = 0;

	while (curWritten < toWrite) {
		ssize_t sendResult = send(sock, dataBuffer + curWritten, toWrite - curWritten, 0);
		if (sendResult <= 0) {
			return (false);
		}

		curWritten += (size_t) sendResult;
	}

	return (true);
}

// Read toRead bytes from the socket sock into buffer.
// Return true on success, false on failure.
static inline bool recvUntilDone(int sock, void *buffer, size_t toRead) {
	uint8_t *dataBuffer = buffer;
	size_t curRead = 0;

	while (curRead < toRead) {
		ssize_t recvResult = recv(sock, dataBuffer + curRead, toRead - curRead, 0);
		if (recvResult <= 0) {
			return (false);
		}

		curRead += (size_t) recvResult;
	}

	return (true);
}

// Write toWrite bytes to the file descriptor fd from buffer.
// Return true on success, false on failure.
static inline bool writeUntilDone(int fd, void *buffer, size_t toWrite) {
	uint8_t *dataBuffer = buffer;
	size_t curWritten = 0;

	while (curWritten < toWrite) {
		ssize_t writeResult = write(fd, dataBuffer + curWritten, toWrite - curWritten);
		if (writeResult <= 0) {
			return (false);
		}

		curWritten += (size_t) writeResult;
	}

	return (true);
}

// Read toRead bytes from the file descriptor fd into buffer.
// Return true on success, false on failure.
static inline bool readUntilDone(int fd, void *buffer, size_t toRead) {
	uint8_t *dataBuffer = buffer;
	size_t curRead = 0;

	while (curRead < toRead) {
		ssize_t readResult = read(fd, dataBuffer + curRead, toRead - curRead);
		if (readResult <= 0) {
			return (false);
		}

		curRead += (size_t) readResult;
	}

	return (true);
}

#endif /* NETS_H_ */
