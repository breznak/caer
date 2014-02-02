/**
 * This file is part of the atomic_ops project.
 *
 * For the full copyright and license information, please view the COPYING
 * file that was distributed with this source code.
 *
 * @copyright  (c) the atomic_ops project
 * @author     Luca Longinotti <chtekk@longitekk.com>
 * @license    BSD 2-clause
 * @version    $Id: sync_intrinsics.h 1100 2012-07-31 03:04:43Z llongi $
 */

/*
 * The GCC docs say: "In most cases, these builtins are considered a full barrier.
 * That is, no memory operand will be moved across the operation, either forward
 * or backward. Further, instructions will be issued as necessary to prevent the
 * processor from speculating loads across the operation and from queuing stores
 * after the operation."
 * It is absolutely not clear what "In most cases" means, neither is what barrier
 * means in this context. Empirical research suggests actual full memory barriers
 * are placed by the compiler to ensure the above semantics.
 */

static inline void atomic_ops_emu_entry_fence(ATOMIC_OPS_FENCE fence) {
	UNUSED_ARGUMENT(fence);
}

static inline void atomic_ops_emu_exit_fence(ATOMIC_OPS_FENCE fence) {
	UNUSED_ARGUMENT(fence);
}

#define GEN_atomic_ops_load(TYPE, MNEMONIC) \
static inline TYPE atomic_ops_##MNEMONIC##_load(const atomic_ops_##MNEMONIC *atomic, ATOMIC_OPS_FENCE fence) {	\
	if (fence == ATOMIC_OPS_FENCE_RELEASE || fence == ATOMIC_OPS_FENCE_FULL										\
	 || fence == ATOMIC_OPS_FENCE_READ    || fence == ATOMIC_OPS_FENCE_WRITE) {									\
		atomic_ops_fence(ATOMIC_OPS_FENCE_FULL);																\
	}																											\
																												\
	atomic_ops_fence(ATOMIC_OPS_FENCE_NONE);																	\
	TYPE val = atomic->v;																						\
	atomic_ops_fence(ATOMIC_OPS_FENCE_NONE);																	\
																												\
	if (fence == ATOMIC_OPS_FENCE_ACQUIRE || fence == ATOMIC_OPS_FENCE_FULL										\
	 || fence == ATOMIC_OPS_FENCE_READ    || fence == ATOMIC_OPS_FENCE_WRITE) {									\
		atomic_ops_fence(ATOMIC_OPS_FENCE_FULL);																\
	}																											\
																												\
	return (val);																								\
}

GEN_atomic_ops_load(intptr_t,  int)
GEN_atomic_ops_load(uintptr_t, uint)
GEN_atomic_ops_load(void *,    ptr)

#define GEN_atomic_ops_store(TYPE, MNEMONIC) \
static inline void atomic_ops_##MNEMONIC##_store(atomic_ops_##MNEMONIC *atomic, TYPE val, ATOMIC_OPS_FENCE fence) {	\
	if (fence == ATOMIC_OPS_FENCE_RELEASE || fence == ATOMIC_OPS_FENCE_FULL											\
	 || fence == ATOMIC_OPS_FENCE_READ    || fence == ATOMIC_OPS_FENCE_WRITE) {										\
		atomic_ops_fence(ATOMIC_OPS_FENCE_FULL);																	\
	}																												\
																													\
	atomic_ops_fence(ATOMIC_OPS_FENCE_NONE);																		\
	atomic->v = val;																								\
	atomic_ops_fence(ATOMIC_OPS_FENCE_NONE);																		\
																													\
	if (fence == ATOMIC_OPS_FENCE_ACQUIRE || fence == ATOMIC_OPS_FENCE_FULL											\
	 || fence == ATOMIC_OPS_FENCE_READ    || fence == ATOMIC_OPS_FENCE_WRITE) {										\
		atomic_ops_fence(ATOMIC_OPS_FENCE_FULL);																	\
	}																												\
}

GEN_atomic_ops_store(intptr_t,  int)
GEN_atomic_ops_store(uintptr_t, uint)
GEN_atomic_ops_store(void *,    ptr)

// EMULATED
EMU_GEN_atomic_ops_not_by_cas(intptr_t,  int)
EMU_GEN_atomic_ops_not_by_cas(uintptr_t, uint)
EMU_GEN_atomic_ops_andorxoradd_by_cas(and, intptr_t,  int,  &)
EMU_GEN_atomic_ops_andorxoradd_by_cas(and, uintptr_t, uint, &)
EMU_GEN_atomic_ops_andorxoradd_by_cas(or,  intptr_t,  int,  |)
EMU_GEN_atomic_ops_andorxoradd_by_cas(or,  uintptr_t, uint, |)
EMU_GEN_atomic_ops_andorxoradd_by_cas(xor, intptr_t,  int,  ^)
EMU_GEN_atomic_ops_andorxoradd_by_cas(xor, uintptr_t, uint, ^)
EMU_GEN_atomic_ops_add_by_faa(intptr_t,  int)
EMU_GEN_atomic_ops_add_by_faa(uintptr_t, uint)
EMU_GEN_atomic_ops_incdec_by_add(inc, intptr_t,  int,  1)
EMU_GEN_atomic_ops_incdec_by_add(inc, uintptr_t, uint, 1)
EMU_GEN_atomic_ops_incdec_by_add(dec, intptr_t,  int,  -1)
EMU_GEN_atomic_ops_incdec_by_add(dec, uintptr_t, uint, -1)

#define GEN_atomic_ops_fetch_and_add(TYPE, MNEMONIC) \
static inline TYPE atomic_ops_##MNEMONIC##_fetch_and_add(atomic_ops_##MNEMONIC *atomic, TYPE val, ATOMIC_OPS_FENCE fence) {	\
	UNUSED_ARGUMENT(fence);																									\
																															\
	return (__sync_fetch_and_add(&atomic->v, val, /* protected variables: */ &atomic->v));									\
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
	return (__sync_val_compare_and_swap(&atomic->v, oldval, newval, /* protected variables: */ &atomic->v));						\
}

GEN_atomic_ops_casr(intptr_t,  int)
GEN_atomic_ops_casr(uintptr_t, uint)
GEN_atomic_ops_casr(void *,    ptr)

#define GEN_atomic_ops_cas(TYPE, MNEMONIC) \
static inline bool atomic_ops_##MNEMONIC##_cas(atomic_ops_##MNEMONIC *atomic, TYPE oldval, TYPE newval, ATOMIC_OPS_FENCE fence) {	\
	UNUSED_ARGUMENT(fence);																											\
																																	\
	return (__sync_bool_compare_and_swap(&atomic->v, oldval, newval, /* protected variables: */ &atomic->v));						\
}

GEN_atomic_ops_cas(intptr_t,  int)
GEN_atomic_ops_cas(uintptr_t, uint)
GEN_atomic_ops_cas(void *,    ptr)

// EMULATED
EMU_GEN_atomic_ops_swap_by_cas(intptr_t,  int)
EMU_GEN_atomic_ops_swap_by_cas(uintptr_t, uint)
EMU_GEN_atomic_ops_swap_by_cas(void *,    ptr)

static inline void atomic_ops_fence(ATOMIC_OPS_FENCE fence) {
	__asm__ __volatile__ ("" ::: "memory");

	if (fence == ATOMIC_OPS_FENCE_ACQUIRE) {
		volatile intptr_t v = 0;

		__sync_lock_test_and_set(&v, 1);
	}

	if (fence == ATOMIC_OPS_FENCE_RELEASE) {
		volatile intptr_t v = 1;

		__sync_lock_release(&v);
	}

	if (fence == ATOMIC_OPS_FENCE_FULL || fence == ATOMIC_OPS_FENCE_READ || fence == ATOMIC_OPS_FENCE_WRITE) {
		__sync_synchronize();
	}
}

static inline void atomic_ops_pause(void) {
	__asm__ __volatile__ ("" ::: "memory");
}
