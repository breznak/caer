/**
 * This file is part of the atomic_ops project.
 *
 * For the full copyright and license information, please view the COPYING
 * file that was distributed with this source code.
 *
 * @copyright  (c) the atomic_ops project
 * @author     Luca Longinotti <chtekk@longitekk.com>
 * @license    BSD 2-clause
 * @version    $Id: sparcv9.h 1103 2012-08-16 18:04:02Z llongi $
 */

// ATOMIC_OPS_SS (Size Suffix): appended to asm ops to specify operand length
#if UINTPTR_MAX == UINT32_MAX
	#define ATOMIC_OPS_SS ""
	#define ATOMIC_OPS_SS_SIGNED "sw"
	#define ATOMIC_OPS_SS_UNSIGNED "uw"
#elif UINTPTR_MAX == UINT64_MAX
	#define ATOMIC_OPS_SS "x"
	#define ATOMIC_OPS_SS_SIGNED "x"
	#define ATOMIC_OPS_SS_UNSIGNED "x"
#else
	#error uintptr_t is not a 32 or 64 bit type. Only 32/64 bit systems are supported for SPARCv9.
#endif

static inline void atomic_ops_emu_entry_fence(ATOMIC_OPS_FENCE fence) {
	// Since we emulate using CAS, there is a LOAD before the CAS, which is what we have to consider for the fences
	if (fence == ATOMIC_OPS_FENCE_RELEASE || fence == ATOMIC_OPS_FENCE_FULL) {
		__asm__ __volatile__ ("membar #LoadLoad | #StoreLoad" ::: "memory");
	}
	if (fence == ATOMIC_OPS_FENCE_READ) {
		__asm__ __volatile__ ("membar #LoadLoad" ::: "memory");
	}
	if (fence == ATOMIC_OPS_FENCE_WRITE) {
		__asm__ __volatile__ ("membar #StoreLoad" ::: "memory");
	}
}

static inline void atomic_ops_emu_exit_fence(ATOMIC_OPS_FENCE fence) {
	if (fence == ATOMIC_OPS_FENCE_ACQUIRE || fence == ATOMIC_OPS_FENCE_FULL) {
		__asm__ __volatile__ ("membar #LoadLoad | #LoadStore" ::: "memory");
	}
	if (fence == ATOMIC_OPS_FENCE_READ) {
		__asm__ __volatile__ ("membar #LoadLoad" ::: "memory");
	}
	if (fence == ATOMIC_OPS_FENCE_WRITE) {
		__asm__ __volatile__ ("membar #LoadStore" ::: "memory");
	}
}

#define GEN_atomic_ops_load(TYPE, MNEMONIC, OPS_SS) \
static inline TYPE atomic_ops_##MNEMONIC##_load(const atomic_ops_##MNEMONIC *atomic, ATOMIC_OPS_FENCE fence) {	\
	if (fence == ATOMIC_OPS_FENCE_RELEASE || fence == ATOMIC_OPS_FENCE_FULL) {									\
		__asm__ __volatile__ ("membar #LoadLoad | #StoreLoad" ::: "memory");									\
	}																											\
	if (fence == ATOMIC_OPS_FENCE_READ) {																		\
		__asm__ __volatile__ ("membar #LoadLoad" ::: "memory");													\
	}																											\
	if (fence == ATOMIC_OPS_FENCE_WRITE) {																		\
		__asm__ __volatile__ ("membar #StoreLoad" ::: "memory");												\
	}																											\
																												\
	TYPE val;																									\
	__asm__ __volatile__ ("ld"OPS_SS" [%1], %0"																	\
						: "=&r" (val)																			\
						: "r" (&atomic->v)																		\
						: "memory");																			\
																												\
	if (fence == ATOMIC_OPS_FENCE_ACQUIRE || fence == ATOMIC_OPS_FENCE_FULL) {									\
		__asm__ __volatile__ ("membar #LoadLoad | #LoadStore" ::: "memory");									\
	}																											\
	if (fence == ATOMIC_OPS_FENCE_READ) {																		\
		__asm__ __volatile__ ("membar #LoadLoad" ::: "memory");													\
	}																											\
	if (fence == ATOMIC_OPS_FENCE_WRITE) {																		\
		__asm__ __volatile__ ("membar #LoadStore" ::: "memory");												\
	}																											\
																												\
	return (val);																								\
}

GEN_atomic_ops_load(intptr_t,  int,  ATOMIC_OPS_SS_SIGNED)
GEN_atomic_ops_load(uintptr_t, uint, ATOMIC_OPS_SS_UNSIGNED)
GEN_atomic_ops_load(void *,    ptr,  ATOMIC_OPS_SS_UNSIGNED)

#define GEN_atomic_ops_store(TYPE, MNEMONIC, OPS_SS) \
static inline void atomic_ops_##MNEMONIC##_store(atomic_ops_##MNEMONIC *atomic, TYPE val, ATOMIC_OPS_FENCE fence) {	\
	if (fence == ATOMIC_OPS_FENCE_RELEASE || fence == ATOMIC_OPS_FENCE_FULL) {										\
		__asm__ __volatile__ ("membar #LoadStore | #StoreStore" ::: "memory");										\
	}																												\
	if (fence == ATOMIC_OPS_FENCE_READ) {																			\
		__asm__ __volatile__ ("membar #LoadStore" ::: "memory");													\
	}																												\
	if (fence == ATOMIC_OPS_FENCE_WRITE) {																			\
		__asm__ __volatile__ ("membar #StoreStore" ::: "memory");													\
	}																												\
																													\
	__asm__ __volatile__ ("st"OPS_SS" %0, [%1]"																		\
						: /* no output operands */																	\
						: "r" (val), "r" (&atomic->v)																\
						: "memory");																				\
																													\
	if (fence == ATOMIC_OPS_FENCE_ACQUIRE || fence == ATOMIC_OPS_FENCE_FULL) {										\
		__asm__ __volatile__ ("membar #StoreLoad | #StoreStore" ::: "memory");										\
	}																												\
	if (fence == ATOMIC_OPS_FENCE_READ) {																			\
		__asm__ __volatile__ ("membar #StoreLoad" ::: "memory");													\
	}																												\
	if (fence == ATOMIC_OPS_FENCE_WRITE) {																			\
		__asm__ __volatile__ ("membar #StoreStore" ::: "memory");													\
	}																												\
}

GEN_atomic_ops_store(intptr_t,  int,  ATOMIC_OPS_SS_SIGNED)
GEN_atomic_ops_store(uintptr_t, uint, ATOMIC_OPS_SS_UNSIGNED)
GEN_atomic_ops_store(void *,    ptr,  ATOMIC_OPS_SS_UNSIGNED)

// EMULATED
EMU_GEN_atomic_ops_not_by_cas(intptr_t,  int)
EMU_GEN_atomic_ops_not_by_cas(uintptr_t, uint)
EMU_GEN_atomic_ops_andorxoradd_by_cas(and, intptr_t,  int,  &)
EMU_GEN_atomic_ops_andorxoradd_by_cas(and, uintptr_t, uint, &)
EMU_GEN_atomic_ops_andorxoradd_by_cas(or,  intptr_t,  int,  |)
EMU_GEN_atomic_ops_andorxoradd_by_cas(or,  uintptr_t, uint, |)
EMU_GEN_atomic_ops_andorxoradd_by_cas(xor, intptr_t,  int,  ^)
EMU_GEN_atomic_ops_andorxoradd_by_cas(xor, uintptr_t, uint, ^)
EMU_GEN_atomic_ops_andorxoradd_by_cas(add, intptr_t,  int,  +)
EMU_GEN_atomic_ops_andorxoradd_by_cas(add, uintptr_t, uint, +)
EMU_GEN_atomic_ops_incdec_by_add(inc, intptr_t,  int,  1)
EMU_GEN_atomic_ops_incdec_by_add(inc, uintptr_t, uint, 1)
EMU_GEN_atomic_ops_incdec_by_add(dec, intptr_t,  int,  -1)
EMU_GEN_atomic_ops_incdec_by_add(dec, uintptr_t, uint, -1)
EMU_GEN_atomic_ops_fetch_and_add_by_cas(intptr_t,  int)
EMU_GEN_atomic_ops_fetch_and_add_by_cas(uintptr_t, uint)
EMU_GEN_atomic_ops_fetch_and_incdec_by_faa(inc, intptr_t,  int,  1)
EMU_GEN_atomic_ops_fetch_and_incdec_by_faa(inc, uintptr_t, uint, 1)
EMU_GEN_atomic_ops_fetch_and_incdec_by_faa(dec, intptr_t,  int,  -1)
EMU_GEN_atomic_ops_fetch_and_incdec_by_faa(dec, uintptr_t, uint, -1)

#define GEN_atomic_ops_casr(TYPE, MNEMONIC) \
static inline TYPE atomic_ops_##MNEMONIC##_casr(atomic_ops_##MNEMONIC *atomic, TYPE oldval, TYPE newval, ATOMIC_OPS_FENCE fence) {	\
	if (fence == ATOMIC_OPS_FENCE_RELEASE || fence == ATOMIC_OPS_FENCE_FULL) {														\
		__asm__ __volatile__ ("membar #LoadStore | #StoreStore" ::: "memory");														\
	}																																\
	if (fence == ATOMIC_OPS_FENCE_READ) {																							\
		__asm__ __volatile__ ("membar #LoadStore" ::: "memory");																	\
	}																																\
	if (fence == ATOMIC_OPS_FENCE_WRITE) {																							\
		__asm__ __volatile__ ("membar #StoreStore" ::: "memory");																	\
	}																																\
																																	\
	TYPE result;																													\
	__asm__ __volatile__ ("cas"ATOMIC_OPS_SS" [%3], %1, %0"																			\
						: "=&r" (result), 																							\
						: "r" (oldval), "0" (newval), "r" (&atomic->v)																\
						: "memory");																								\
																																	\
	if (fence == ATOMIC_OPS_FENCE_ACQUIRE || fence == ATOMIC_OPS_FENCE_FULL) {														\
		__asm__ __volatile__ ("membar #LoadLoad | #LoadStore" ::: "memory");														\
	}																																\
	if (fence == ATOMIC_OPS_FENCE_READ) {																							\
		__asm__ __volatile__ ("membar #LoadLoad" ::: "memory");																		\
	}																																\
	if (fence == ATOMIC_OPS_FENCE_WRITE) {																							\
		__asm__ __volatile__ ("membar #LoadStore" ::: "memory");																	\
	}																																\
																																	\
	return (result);																												\
}

GEN_atomic_ops_casr(intptr_t,  int)
GEN_atomic_ops_casr(uintptr_t, uint)
GEN_atomic_ops_casr(void *,    ptr)

// EMULATED
EMU_GEN_atomic_ops_cas_by_casr(intptr_t,  int)
EMU_GEN_atomic_ops_cas_by_casr(uintptr_t, uint)
EMU_GEN_atomic_ops_cas_by_casr(void *,    ptr)

#if UINTPTR_MAX == UINT64_MAX

// EMULATED (because swap is 32 bit only!)
EMU_GEN_atomic_ops_swap_by_cas(intptr_t,  int)
EMU_GEN_atomic_ops_swap_by_cas(uintptr_t, uint)
EMU_GEN_atomic_ops_swap_by_cas(void *,    ptr)

#else

#define GEN_atomic_ops_swap(TYPE, MNEMONIC) \
static inline TYPE atomic_ops_##MNEMONIC##_swap(atomic_ops_##MNEMONIC *atomic, TYPE val, ATOMIC_OPS_FENCE fence) {	\
	if (fence == ATOMIC_OPS_FENCE_RELEASE || fence == ATOMIC_OPS_FENCE_FULL) {										\
		__asm__ __volatile__ ("membar #LoadStore | #StoreStore" ::: "memory");										\
	}																												\
	if (fence == ATOMIC_OPS_FENCE_READ) {																			\
		__asm__ __volatile__ ("membar #LoadStore" ::: "memory");													\
	}																												\
	if (fence == ATOMIC_OPS_FENCE_WRITE) {																			\
		__asm__ __volatile__ ("membar #StoreStore" ::: "memory");													\
	}																												\
																													\
	TYPE result;																									\
	__asm__ __volatile__ ("swap [%2], %0"																			\
						: "=&r" (result), 																			\
						: "0" (val), "r" (&atomic->v)																\
						: "memory");																				\
																													\
	if (fence == ATOMIC_OPS_FENCE_ACQUIRE || fence == ATOMIC_OPS_FENCE_FULL) {										\
		__asm__ __volatile__ ("membar #LoadLoad | #LoadStore" ::: "memory");										\
	}																												\
	if (fence == ATOMIC_OPS_FENCE_READ) {																			\
		__asm__ __volatile__ ("membar #LoadLoad" ::: "memory");														\
	}																												\
	if (fence == ATOMIC_OPS_FENCE_WRITE) {																			\
		__asm__ __volatile__ ("membar #LoadStore" ::: "memory");													\
	}																												\
																													\
	return (result);																								\
}

GEN_atomic_ops_swap(intptr_t,  int)
GEN_atomic_ops_swap(uintptr_t, uint)
GEN_atomic_ops_swap(void *,    ptr)

#endif

static inline void atomic_ops_fence(ATOMIC_OPS_FENCE fence) {
	__asm__ __volatile__ ("" ::: "memory");

	if (fence == ATOMIC_OPS_FENCE_ACQUIRE) {
		// Loads have efficient acquire semantics, especially on SPARC-TSO
		atomic_ops_uint v = ATOMIC_OPS_UINT_INIT(0);
		uintptr_t val;

		val = atomic_ops_uint_load(&v, ATOMIC_OPS_FENCE_ACQUIRE);
	}

	if (fence == ATOMIC_OPS_FENCE_RELEASE) {
		// Stores have efficient release semantics, especially on SPARC-TSO
		atomic_ops_uint v;
		uintptr_t val = 0;

		atomic_ops_uint_store(&v, val, ATOMIC_OPS_FENCE_RELEASE);
	}

	if (fence == ATOMIC_OPS_FENCE_FULL) {
		__asm__ __volatile__ ("membar #LoadLoad | #LoadStore | #StoreLoad | #StoreStore" ::: "memory");
	}

	if (fence == ATOMIC_OPS_FENCE_READ) {
		__asm__ __volatile__ ("membar #LoadLoad" ::: "memory");
	}

	if (fence == ATOMIC_OPS_FENCE_WRITE) {
		__asm__ __volatile__ ("membar #StoreStore" ::: "memory");
	}
}

static inline void atomic_ops_pause(void) {
	__asm__ __volatile__ ("" ::: "memory");
}
