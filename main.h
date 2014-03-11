/*
 * main.h
 *
 *  Created on: Oct 8, 2013
 *      Author: llongi
 */

#ifndef MAIN_H_
#define MAIN_H_

// Common includes, useful for everyone.
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <errno.h>
#include <endian.h>
#include "base/log.h"
#include "ext/sshs/sshs.h"
#include "ext/atomic_ops/atomic_ops.h"

// Common macros, useful for everyone.
#define U8T(X) ((uint8_t) (X))
#define U16T(X) ((uint16_t) (X))
#define U32T(X) ((uint32_t) (X))
#define U64T(X) ((uint64_t) (X))
#define MASK_NUMBITS32(X) U32T(U32T(U32T(1) << X) - 1)
#define MASK_NUMBITS64(X) U64T(U64T(U64T(1) << X) - 1)
#define SWAP_VAR(type, x, y) { type tmpv; tmpv = (x); (x) = (y); (y) = tmpv; }

static inline bool str_equals(const char *s1, const char *s2) {
	if (s1 == NULL || s2 == NULL) {
		return (false);
	}

	if (strcmp(s1, s2) == 0) {
		return (true);
	}

	return (false);
}

#endif /* MAIN_H_ */
