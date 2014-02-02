/**
 * This file is part of the atomic_ops project.
 *
 * For the full copyright and license information, please view the COPYING
 * file that was distributed with this source code.
 *
 * @copyright  (c) the atomic_ops project
 * @author     Luca Longinotti <chtekk@longitekk.com>
 * @license    BSD 2-clause
 * @version    $Id: atomic_ops.h 1098 2012-07-30 17:56:14Z llongi $
 */

#ifndef ATOMIC_OPS_H
#define ATOMIC_OPS_H 1

/*
 * Includes and defines
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

// Alignment specification support (with defines for cache line alignment)
#undef ATTR_ALIGNED
#undef CACHELINE_ALIGNED
#undef CACHELINE_ALONE

#if !defined(CACHELINE_SIZE)
	#define CACHELINE_SIZE 64 // Default (big enough for almost all processors)
	// Must be power of two!
#endif

#if defined(__GNUC__)
	#define ATTR_ALIGNED(x) __attribute__ ((__aligned__ (x)))
#else
	#define ATTR_ALIGNED(x)
#endif

#define CACHELINE_ALIGNED ATTR_ALIGNED(CACHELINE_SIZE)
#define CACHELINE_ALONE(t, v) t v CACHELINE_ALIGNED; uint8_t PAD_##v[CACHELINE_SIZE - (sizeof(t) & (CACHELINE_SIZE - 1))]

// Forced inlining support
#undef ATTR_ALWAYSINLINE

#if defined(__GNUC__)
	#define ATTR_ALWAYSINLINE __attribute__ ((__always_inline__))
#else
	#define ATTR_ALWAYSINLINE
#endif

// Suppress unused argument warnings, if needed
#define UNUSED_ARGUMENT(arg) (void)(arg)

/*
 * Type Definitions
 */

typedef struct { volatile  intptr_t v; } atomic_ops_int  ATTR_ALIGNED(sizeof( intptr_t));
typedef struct { volatile uintptr_t v; } atomic_ops_uint ATTR_ALIGNED(sizeof(uintptr_t));
#define ATOMIC_OPS_INT_INIT(X)  { (( intptr_t)(X)) }
#define ATOMIC_OPS_UINT_INIT(X) { ((uintptr_t)(X)) }

typedef struct { void * volatile v; } atomic_ops_ptr ATTR_ALIGNED(sizeof(void *));
#define ATOMIC_OPS_PTR_INIT(X) { ((void *)(X)) }

typedef struct { atomic_ops_ptr p; } atomic_ops_flagptr ATTR_ALIGNED(sizeof(void *));
#define ATOMIC_OPS_FLAGPTR_INIT(P, F) { (ATOMIC_OPS_PTR_INIT((void *)(((uintptr_t)(P)) | ((uintptr_t)(F))))) }

typedef enum {
	ATOMIC_OPS_FENCE_NONE    = (1 << 0), // Compiler barrier (don't let the compiler reorder)
	ATOMIC_OPS_FENCE_ACQUIRE = (1 << 1), // Acquire barrier (nothing from after is reordered before)
	ATOMIC_OPS_FENCE_RELEASE = (1 << 2), // Release barrier (nothing from before is reordered after)
	ATOMIC_OPS_FENCE_FULL    = (1 << 3), // Full barrier (nothing moves around, at all)
	ATOMIC_OPS_FENCE_READ    = (1 << 4), // Read barrier (order reads, like full but only wrt reads)
	ATOMIC_OPS_FENCE_WRITE   = (1 << 5), // Write barrier (order writes, like full but only wrt writes)
} ATOMIC_OPS_FENCE;

/*
 * Functions
 */

static inline intptr_t atomic_ops_int_load(const atomic_ops_int *atomic, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline void atomic_ops_int_store(atomic_ops_int *atomic, intptr_t val, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline void atomic_ops_int_not(atomic_ops_int *atomic, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline void atomic_ops_int_and(atomic_ops_int *atomic, intptr_t val, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline void atomic_ops_int_or(atomic_ops_int *atomic, intptr_t val, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline void atomic_ops_int_xor(atomic_ops_int *atomic, intptr_t val, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline void atomic_ops_int_add(atomic_ops_int *atomic, intptr_t val, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline void atomic_ops_int_inc(atomic_ops_int *atomic, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline void atomic_ops_int_dec(atomic_ops_int *atomic, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline intptr_t atomic_ops_int_fetch_and_add(atomic_ops_int *atomic, intptr_t val, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline intptr_t atomic_ops_int_fetch_and_inc(atomic_ops_int *atomic, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline intptr_t atomic_ops_int_fetch_and_dec(atomic_ops_int *atomic, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline intptr_t atomic_ops_int_casr(atomic_ops_int *atomic, intptr_t oldval, intptr_t newval, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline bool atomic_ops_int_cas(atomic_ops_int *atomic, intptr_t oldval, intptr_t newval, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline intptr_t atomic_ops_int_swap(atomic_ops_int *atomic, intptr_t val, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;

static inline uintptr_t atomic_ops_uint_load(const atomic_ops_uint *atomic, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline void atomic_ops_uint_store(atomic_ops_uint *atomic, uintptr_t val, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline void atomic_ops_uint_not(atomic_ops_uint *atomic, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline void atomic_ops_uint_and(atomic_ops_uint *atomic, uintptr_t val, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline void atomic_ops_uint_or(atomic_ops_uint *atomic, uintptr_t val, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline void atomic_ops_uint_xor(atomic_ops_uint *atomic, uintptr_t val, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline void atomic_ops_uint_add(atomic_ops_uint *atomic, uintptr_t val, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline void atomic_ops_uint_inc(atomic_ops_uint *atomic, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline void atomic_ops_uint_dec(atomic_ops_uint *atomic, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline uintptr_t atomic_ops_uint_fetch_and_add(atomic_ops_uint *atomic, uintptr_t val, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline uintptr_t atomic_ops_uint_fetch_and_inc(atomic_ops_uint *atomic, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline uintptr_t atomic_ops_uint_fetch_and_dec(atomic_ops_uint *atomic, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline uintptr_t atomic_ops_uint_casr(atomic_ops_uint *atomic, uintptr_t oldval, uintptr_t newval, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline bool atomic_ops_uint_cas(atomic_ops_uint *atomic, uintptr_t oldval, uintptr_t newval, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline uintptr_t atomic_ops_uint_swap(atomic_ops_uint *atomic, uintptr_t val, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;

static inline void * atomic_ops_ptr_load(const atomic_ops_ptr *atomic, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline void atomic_ops_ptr_store(atomic_ops_ptr *atomic, void *val, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline void * atomic_ops_ptr_casr(atomic_ops_ptr *atomic, void *oldval, void *newval, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline bool atomic_ops_ptr_cas(atomic_ops_ptr *atomic, void *oldval, void *newval, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline void * atomic_ops_ptr_swap(atomic_ops_ptr *atomic, void *val, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;

static inline void atomic_ops_fence(ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;

static inline void atomic_ops_pause(void) ATTR_ALWAYSINLINE;

/*
 * Flag-Pointer Functions
 */

static inline void * atomic_ops_flagptr_load(const atomic_ops_flagptr *atomic, bool *flag, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline void * atomic_ops_flagptr_load_full(const atomic_ops_flagptr *atomic, bool *flag, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline void atomic_ops_flagptr_store(atomic_ops_flagptr *atomic, void *newptr, bool newflag, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline void * atomic_ops_flagptr_casr(atomic_ops_flagptr *atomic, bool *flag, void *oldptr, bool oldflag, void *newptr, bool newflag, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline bool atomic_ops_flagptr_cas(atomic_ops_flagptr *atomic, void *oldptr, bool oldflag, void *newptr, bool newflag, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;
static inline void * atomic_ops_flagptr_swap(atomic_ops_flagptr *atomic, bool *flag, void *newptr, bool newflag, ATOMIC_OPS_FENCE fence) ATTR_ALWAYSINLINE;

/*
 * Implementations
 */

#include "atomic_ops/emulation.h"

#if defined(__GNUC__)
	#if defined(i386) || defined(__i386) || defined(__i386__) || defined(__i486__) || defined(__i586__) \
	|| defined(__i686__) || defined(_M_IX86) || defined(_X86_) || defined(__X86__) || defined(__I86__) \
	|| defined(__THW_INTEL__) || defined(__INTEL__) || defined(__amd64) || defined(__amd64__) \
	|| defined(__x86_64) || defined(__x86_64__) || defined(_M_X64)
		#include "atomic_ops/gcc/x86-64.h"
	#elif defined(__sparc) || defined(__sparc__) || defined(__sparcv9)
		#include "atomic_ops/gcc/sparcv9.h"
	#elif defined(__ia64) || defined(__ia64__) || defined(_IA64) || defined(__IA64) || defined(__IA64__) \
	|| defined(_M_IA64)
		#include "atomic_ops/gcc/ia64.h"
	#elif defined(__powerpc) || defined(__powerpc__) || defined(__POWERPC__)  || defined(__ppc__) \
	|| defined(__PPC__) || defined(_ARCH_PPC) || defined(_M_PPC)
		#include "atomic_ops/gcc/ppc.h"
	#elif defined(__arm) || defined(__arm__) || defined(__TARGET_ARCH_ARM) || defined(_ARM)
		#include "atomic_ops/gcc/armv7.h"
	#else
		#include "atomic_ops/gcc/sync_intrinsics.h"
	#endif
#else
	#error Compiler not supported.
#endif

#include "atomic_ops/flagptr.h"

#endif /* ATOMIC_OPS_H */
