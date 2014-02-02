/**
 * This file is part of the atomic_ops project.
 *
 * For the full copyright and license information, please view the COPYING
 * file that was distributed with this source code.
 *
 * @copyright  (c) the atomic_ops project
 * @author     Luca Longinotti <chtekk@longitekk.com>
 * @license    BSD 2-clause
 * @version    $Id: x86-64.h 1101 2012-08-02 23:07:05Z llongi $
 */

/*
 * GCC X86 and X86-64 implementation for recent systems.
 * Anything older than a Pentium 4 is not directly supported, as there is no
 * support for load barriers and full barriers there.
 * If you need this to work on Pentium 3, use the following:
 *
 *		uintptr_t v = 0;
 *		__asm__ __volatile__ ("lock; or"ATOMIC_OPS_SS" $0, %0" : "+m" (v) :: "memory");
 *
 *	as a full, as well as read, barrier.
 *	Even older systems also lack write barriers, so you'd have to use
 *	the above there too.
 */

// ATOMIC_OPS_SS (Size Suffix): appended to asm ops to specify operand length
#if UINTPTR_MAX == UINT32_MAX
	#define ATOMIC_OPS_SS "l"
#elif UINTPTR_MAX == UINT64_MAX
	#define ATOMIC_OPS_SS "q"
#else
	#error uintptr_t is not a 32 or 64 bit type. Only 32/64 bit systems are supported for x86-64.
#endif

#define GEN_atomic_ops_load(TYPE, MNEMONIC) \
static inline TYPE atomic_ops_##MNEMONIC##_load(const atomic_ops_##MNEMONIC *atomic, ATOMIC_OPS_FENCE fence) {		\
	if (fence == ATOMIC_OPS_FENCE_RELEASE || fence == ATOMIC_OPS_FENCE_FULL || fence == ATOMIC_OPS_FENCE_WRITE) {	\
		atomic_ops_fence(ATOMIC_OPS_FENCE_FULL); /* Prevent #StoreLoad reordering */								\
	}																												\
																													\
	TYPE val;																										\
	__asm__ __volatile__ ("mov"ATOMIC_OPS_SS" %1, %0"																\
						: "=r" (val)																				\
						: "m" (atomic->v)																			\
						: "memory");																				\
	return (val);																									\
}

GEN_atomic_ops_load(intptr_t,  int)
GEN_atomic_ops_load(uintptr_t, uint)
GEN_atomic_ops_load(void *,    ptr)

#define GEN_atomic_ops_store(TYPE, MNEMONIC) \
static inline void atomic_ops_##MNEMONIC##_store(atomic_ops_##MNEMONIC *atomic, TYPE val, ATOMIC_OPS_FENCE fence) {	\
	__asm__ __volatile__ ("mov"ATOMIC_OPS_SS" %1, %0"																\
						: "=m" (atomic->v)																			\
						: "ir" (val)																				\
						: "memory");																				\
																													\
	if (fence == ATOMIC_OPS_FENCE_ACQUIRE || fence == ATOMIC_OPS_FENCE_FULL || fence == ATOMIC_OPS_FENCE_READ) {	\
		atomic_ops_fence(ATOMIC_OPS_FENCE_FULL); /* Prevent #StoreLoad reordering */								\
	}																												\
}

GEN_atomic_ops_store(intptr_t,  int)
GEN_atomic_ops_store(uintptr_t, uint)
GEN_atomic_ops_store(void *,    ptr)

#define GEN_atomic_ops_mem_val(OPNAME, TYPE, MNEMONIC) \
static inline void atomic_ops_##MNEMONIC##_##OPNAME(atomic_ops_##MNEMONIC *atomic, TYPE val, ATOMIC_OPS_FENCE fence) {	\
	UNUSED_ARGUMENT(fence);																								\
																														\
	__asm__ __volatile__ ("lock; "#OPNAME ATOMIC_OPS_SS" %1,%0"															\
						: "+m" (atomic->v)																				\
						: "ir" (val)																					\
						: "memory");																					\
}

GEN_atomic_ops_mem_val(and, intptr_t,  int)
GEN_atomic_ops_mem_val(and, uintptr_t, uint)
GEN_atomic_ops_mem_val(or,  intptr_t,  int)
GEN_atomic_ops_mem_val(or,  uintptr_t, uint)
GEN_atomic_ops_mem_val(xor, intptr_t,  int)
GEN_atomic_ops_mem_val(xor, uintptr_t, uint)
GEN_atomic_ops_mem_val(add, intptr_t,  int)
GEN_atomic_ops_mem_val(add, uintptr_t, uint)

#define GEN_atomic_ops_mem(OPNAME, MNEMONIC) \
static inline void atomic_ops_##MNEMONIC##_##OPNAME(atomic_ops_##MNEMONIC *atomic, ATOMIC_OPS_FENCE fence) {	\
	UNUSED_ARGUMENT(fence);																						\
																												\
	__asm__ __volatile__ ("lock; "#OPNAME ATOMIC_OPS_SS" %0"													\
						: "+m" (atomic->v)																		\
						: /* no additional input operands */													\
						: "memory");																			\
}

GEN_atomic_ops_mem(not, int)
GEN_atomic_ops_mem(not, uint)
GEN_atomic_ops_mem(inc, int)
GEN_atomic_ops_mem(inc, uint)
GEN_atomic_ops_mem(dec, int)
GEN_atomic_ops_mem(dec, uint)

#define GEN_atomic_ops_fetch_and_add(TYPE, MNEMONIC) \
static inline TYPE atomic_ops_##MNEMONIC##_fetch_and_add(atomic_ops_##MNEMONIC *atomic, TYPE val, ATOMIC_OPS_FENCE fence) {	\
	UNUSED_ARGUMENT(fence);																									\
																															\
	TYPE result;																											\
	__asm__ __volatile__ ("lock; xadd"ATOMIC_OPS_SS" %0,%1"																	\
						: "=r" (result), "+m" (atomic->v)																	\
						: "0" (val)																							\
						: "memory");																						\
	return (result);																										\
}

GEN_atomic_ops_fetch_and_add(intptr_t,  int)
GEN_atomic_ops_fetch_and_add(uintptr_t, uint)

// EMULATED
EMU_GEN_atomic_ops_fetch_and_incdec_by_faa(inc, intptr_t,  int,  1)
EMU_GEN_atomic_ops_fetch_and_incdec_by_faa(inc, uintptr_t, uint, 1)
EMU_GEN_atomic_ops_fetch_and_incdec_by_faa(dec, intptr_t,  int,  -1)
EMU_GEN_atomic_ops_fetch_and_incdec_by_faa(dec, uintptr_t, uint, -1)

#define GEN_atomic_ops_casr(TYPE, MNEMONIC) \
static inline TYPE atomic_ops_##MNEMONIC##_casr(atomic_ops_##MNEMONIC *atomic, TYPE oldval, TYPE newval, ATOMIC_OPS_FENCE fence) {	\
	UNUSED_ARGUMENT(fence);																											\
																																	\
	TYPE result;																													\
	__asm__ __volatile__ ("lock; cmpxchg"ATOMIC_OPS_SS" %3,%1"																		\
						: "=a" (result), "+m" (atomic->v)																			\
						: "0" (oldval), "q" (newval)																				\
						: "memory");																								\
	return (result);																												\
}

GEN_atomic_ops_casr(intptr_t,  int)
GEN_atomic_ops_casr(uintptr_t, uint)
GEN_atomic_ops_casr(void *,    ptr)

#define GEN_atomic_ops_cas(TYPE, MNEMONIC) \
static inline bool atomic_ops_##MNEMONIC##_cas(atomic_ops_##MNEMONIC *atomic, TYPE oldval, TYPE newval, ATOMIC_OPS_FENCE fence) {	\
	UNUSED_ARGUMENT(fence);																											\
																																	\
	bool result;																													\
	__asm__ __volatile__ ("lock; cmpxchg"ATOMIC_OPS_SS" %3,%1; setz %0"																\
						: "=a" (result), "+m" (atomic->v)																			\
						: "0" (oldval), "q" (newval)																				\
						: "memory");																								\
	return (result);																												\
}

GEN_atomic_ops_cas(intptr_t,  int)
GEN_atomic_ops_cas(uintptr_t, uint)
GEN_atomic_ops_cas(void *,    ptr)

#define GEN_atomic_ops_swap(TYPE, MNEMONIC) \
static inline TYPE atomic_ops_##MNEMONIC##_swap(atomic_ops_##MNEMONIC *atomic, TYPE val, ATOMIC_OPS_FENCE fence) {	\
	UNUSED_ARGUMENT(fence);																							\
																													\
	TYPE result;																									\
	__asm__ __volatile__ ("lock; xchg"ATOMIC_OPS_SS" %0,%1"															\
						: "=r" (result), "+m" (atomic->v)															\
						: "0" (val)																					\
						: "memory");																				\
	return (result);																								\
}

GEN_atomic_ops_swap(intptr_t,  int)
GEN_atomic_ops_swap(uintptr_t, uint)
GEN_atomic_ops_swap(void *,    ptr)

static inline void atomic_ops_fence(ATOMIC_OPS_FENCE fence) {
	__asm__ __volatile__ ("" ::: "memory");

	if (fence == ATOMIC_OPS_FENCE_ACQUIRE) {
		// Loads always have acquire semantics on x86
		atomic_ops_uint v = ATOMIC_OPS_UINT_INIT(0);
		uintptr_t val;

		val = atomic_ops_uint_load(&v, ATOMIC_OPS_FENCE_ACQUIRE);
		UNUSED_ARGUMENT(val);
	}

	if (fence == ATOMIC_OPS_FENCE_RELEASE) {
		// Stores always have release semantics on x86
		atomic_ops_uint v;
		uintptr_t val = 0;

		atomic_ops_uint_store(&v, val, ATOMIC_OPS_FENCE_RELEASE);
	}

	if (fence == ATOMIC_OPS_FENCE_FULL) {
		__asm__ __volatile__ ("mfence" ::: "memory");
	}

	if (fence == ATOMIC_OPS_FENCE_READ) {
		__asm__ __volatile__ ("lfence" ::: "memory");
	}

	if (fence == ATOMIC_OPS_FENCE_WRITE) {
		__asm__ __volatile__ ("sfence" ::: "memory");
	}
}

static inline void atomic_ops_pause(void) {
	__asm__ __volatile__ ("pause" ::: "memory");
}
