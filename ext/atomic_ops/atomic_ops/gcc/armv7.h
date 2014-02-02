/**
 * This file is part of the atomic_ops project.
 *
 * For the full copyright and license information, please view the COPYING
 * file that was distributed with this source code.
 *
 * @copyright  (c) the atomic_ops project
 * @author     Luca Longinotti <chtekk@longitekk.com>
 * @license    BSD 2-clause
 * @version    $Id: armv7.h 1104 2012-08-18 10:35:27Z llongi $
 */

#if UINTPTR_MAX != UINT32_MAX
	#error uintptr_t is not a 32 bit type. Only 32 bit systems are supported for ARMv7.
#endif

static inline void atomic_ops_emu_entry_fence(ATOMIC_OPS_FENCE fence) {
	if (fence == ATOMIC_OPS_FENCE_RELEASE || fence == ATOMIC_OPS_FENCE_FULL
	 || fence == ATOMIC_OPS_FENCE_READ    || fence == ATOMIC_OPS_FENCE_WRITE) {
		atomic_ops_fence(fence);
	}
}

static inline void atomic_ops_emu_exit_fence(ATOMIC_OPS_FENCE fence) {
	if (fence == ATOMIC_OPS_FENCE_ACQUIRE || fence == ATOMIC_OPS_FENCE_FULL
	 || fence == ATOMIC_OPS_FENCE_READ    || fence == ATOMIC_OPS_FENCE_WRITE) {
		atomic_ops_fence(fence);
	}
}

#define GEN_atomic_ops_ll(TYPE, MNEMONIC) \
static inline TYPE atomic_ops_##MNEMONIC##_ll(atomic_ops_##MNEMONIC *atomic) {	\
	TYPE val;																	\
	__asm__ __volatile__ ("ldrex %0, [%1]"										\
						: "=&r" (val)											\
						: "r" (&atomic->v)										\
						: "memory");											\
	return (val);																\
}

GEN_atomic_ops_ll(intptr_t,  int)
GEN_atomic_ops_ll(uintptr_t, uint)
GEN_atomic_ops_ll(void *,    ptr)

#define GEN_atomic_ops_sc(TYPE, MNEMONIC) \
static inline bool atomic_ops_##MNEMONIC##_sc(atomic_ops_##MNEMONIC *atomic, TYPE val) {	\
	bool res;																				\
	__asm__ __volatile__ ("strex %0, %1, [%2]"												\
						: "=&r" (res) 														\
						: "r" (val), "r" (&atomic->v)										\
						: "memory");														\
	return (!res);																			\
}

GEN_atomic_ops_sc(intptr_t,  int)
GEN_atomic_ops_sc(uintptr_t, uint)
GEN_atomic_ops_sc(void *,    ptr)

#define GEN_atomic_ops_load(TYPE, MNEMONIC) \
static inline TYPE atomic_ops_##MNEMONIC##_load(const atomic_ops_##MNEMONIC *atomic, ATOMIC_OPS_FENCE fence) {	\
	if (fence == ATOMIC_OPS_FENCE_RELEASE || fence == ATOMIC_OPS_FENCE_FULL										\
	 || fence == ATOMIC_OPS_FENCE_READ    || fence == ATOMIC_OPS_FENCE_WRITE) {									\
		atomic_ops_fence(fence);																				\
	}																											\
																												\
	TYPE val;																									\
	__asm__ __volatile__ ("ldr %0, [%1]"																		\
						: "=&r" (val)																			\
						: "r" (&atomic->v)																		\
						: "memory");																			\
																												\
	if (fence == ATOMIC_OPS_FENCE_ACQUIRE || fence == ATOMIC_OPS_FENCE_FULL										\
	 || fence == ATOMIC_OPS_FENCE_READ    || fence == ATOMIC_OPS_FENCE_WRITE) {									\
		atomic_ops_fence(fence);																				\
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
		atomic_ops_fence(fence);																					\
	}																												\
																													\
	__asm__ __volatile__ ("str %0, [%1]"																			\
						: /* no output operands */																	\
						: "r" (val), "r" (&atomic->v)																\
						: "memory");																				\
																													\
	if (fence == ATOMIC_OPS_FENCE_ACQUIRE || fence == ATOMIC_OPS_FENCE_FULL											\
	 || fence == ATOMIC_OPS_FENCE_READ    || fence == ATOMIC_OPS_FENCE_WRITE) {										\
		atomic_ops_fence(fence);																					\
	}																												\
}

GEN_atomic_ops_store(intptr_t,  int)
GEN_atomic_ops_store(uintptr_t, uint)
GEN_atomic_ops_store(void *,    ptr)

// EMULATED
EMU_GEN_atomic_ops_not_by_llsc(intptr_t,  int)
EMU_GEN_atomic_ops_not_by_llsc(uintptr_t, uint)
EMU_GEN_atomic_ops_andorxoradd_by_llsc(and, intptr_t,  int,  &)
EMU_GEN_atomic_ops_andorxoradd_by_llsc(and, uintptr_t, uint, &)
EMU_GEN_atomic_ops_andorxoradd_by_llsc(or,  intptr_t,  int,  |)
EMU_GEN_atomic_ops_andorxoradd_by_llsc(or,  uintptr_t, uint, |)
EMU_GEN_atomic_ops_andorxoradd_by_llsc(xor, intptr_t,  int,  ^)
EMU_GEN_atomic_ops_andorxoradd_by_llsc(xor, uintptr_t, uint, ^)
EMU_GEN_atomic_ops_andorxoradd_by_llsc(add, intptr_t,  int,  +)
EMU_GEN_atomic_ops_andorxoradd_by_llsc(add, uintptr_t, uint, +)
EMU_GEN_atomic_ops_incdec_by_add(inc, intptr_t,  int,  1)
EMU_GEN_atomic_ops_incdec_by_add(inc, uintptr_t, uint, 1)
EMU_GEN_atomic_ops_incdec_by_add(dec, intptr_t,  int,  -1)
EMU_GEN_atomic_ops_incdec_by_add(dec, uintptr_t, uint, -1)
EMU_GEN_atomic_ops_fetch_and_add_by_llsc(intptr_t,  int)
EMU_GEN_atomic_ops_fetch_and_add_by_llsc(uintptr_t, uint)
EMU_GEN_atomic_ops_fetch_and_incdec_by_faa(inc, intptr_t,  int,  1)
EMU_GEN_atomic_ops_fetch_and_incdec_by_faa(inc, uintptr_t, uint, 1)
EMU_GEN_atomic_ops_fetch_and_incdec_by_faa(dec, intptr_t,  int,  -1)
EMU_GEN_atomic_ops_fetch_and_incdec_by_faa(dec, uintptr_t, uint, -1)
EMU_GEN_atomic_ops_casr_by_llsc(intptr_t,  int)
EMU_GEN_atomic_ops_casr_by_llsc(uintptr_t, uint)
EMU_GEN_atomic_ops_casr_by_llsc(void *,    ptr)
EMU_GEN_atomic_ops_cas_by_llsc(intptr_t,  int)
EMU_GEN_atomic_ops_cas_by_llsc(uintptr_t, uint)
EMU_GEN_atomic_ops_cas_by_llsc(void *,    ptr)
EMU_GEN_atomic_ops_swap_by_llsc(intptr_t,  int)
EMU_GEN_atomic_ops_swap_by_llsc(uintptr_t, uint)
EMU_GEN_atomic_ops_swap_by_llsc(void *,    ptr)

#if defined(__ARM_ARCH_6__) && __ARM_ARCH_6__ == 1
static inline void atomic_ops_fence(ATOMIC_OPS_FENCE fence) {
	__asm__ __volatile__ ("" ::: "memory");

	if (fence != ATOMIC_OPS_FENCE_NONE) {
		uint32_t dest = 0;
		__asm__ __volatile__ ("mcr p15,0,%0,c7,c10,5" : "=&r" (dest) :: "memory");
	}
}
#else
static inline void atomic_ops_fence(ATOMIC_OPS_FENCE fence) {
	__asm__ __volatile__ ("" ::: "memory");

	if (fence == ATOMIC_OPS_FENCE_ACQUIRE || fence == ATOMIC_OPS_FENCE_RELEASE
	 || fence == ATOMIC_OPS_FENCE_FULL || fence == ATOMIC_OPS_FENCE_READ) {
		__asm__ __volatile__ ("dmb" ::: "memory");
	}

	if (fence == ATOMIC_OPS_FENCE_WRITE) {
		__asm__ __volatile__ ("dmb st" ::: "memory");
	}
}
#endif

static inline void atomic_ops_pause(void) {
	__asm__ __volatile__ ("" ::: "memory");
}
