/**
 * This file is part of the atomic_ops project.
 *
 * For the full copyright and license information, please view the COPYING
 * file that was distributed with this source code.
 *
 * @copyright  (c) the atomic_ops project
 * @author     Luca Longinotti <chtekk@longitekk.com>
 * @license    BSD 2-clause
 * @version    $Id: emulation.h 1100 2012-07-31 03:04:43Z llongi $
 */

// Alternative not implementations
#define EMU_GEN_atomic_ops_not_by_cas(TYPE, MNEMONIC) \
static inline void atomic_ops_##MNEMONIC##_not(atomic_ops_##MNEMONIC *atomic, ATOMIC_OPS_FENCE fence) {	\
	atomic_ops_emu_entry_fence(fence);																	\
																										\
	while (true) {																						\
		TYPE oldval = atomic_ops_##MNEMONIC##_load(atomic, ATOMIC_OPS_FENCE_NONE);						\
																										\
		if (atomic_ops_##MNEMONIC##_cas(atomic, oldval, (~oldval), ATOMIC_OPS_FENCE_NONE)) {			\
			atomic_ops_emu_exit_fence(fence);															\
			return;																						\
		}																								\
	}																									\
}

#define EMU_GEN_atomic_ops_not_by_llsc(TYPE, MNEMONIC) \
static inline void atomic_ops_##MNEMONIC##_not(atomic_ops_##MNEMONIC *atomic, ATOMIC_OPS_FENCE fence) {	\
	atomic_ops_emu_entry_fence(fence);																	\
																										\
	while (true) {																						\
		TYPE oldval = atomic_ops_##MNEMONIC##_ll(atomic);												\
																										\
		if (atomic_ops_##MNEMONIC##_sc(atomic, (~oldval))) {											\
			atomic_ops_emu_exit_fence(fence);															\
			return;																						\
		}																								\
	}																									\
}

// Alternative and/or/xor/add implementations
#define EMU_GEN_atomic_ops_andorxoradd_by_cas(FNAME, TYPE, MNEMONIC, OP) \
static inline void atomic_ops_##MNEMONIC##_##FNAME(atomic_ops_##MNEMONIC *atomic, TYPE val, ATOMIC_OPS_FENCE fence) {	\
	atomic_ops_emu_entry_fence(fence);																					\
																														\
	while (true) {																										\
		TYPE oldval = atomic_ops_##MNEMONIC##_load(atomic, ATOMIC_OPS_FENCE_NONE);										\
																														\
		if (atomic_ops_##MNEMONIC##_cas(atomic, oldval, (oldval OP val), ATOMIC_OPS_FENCE_NONE)) {						\
			atomic_ops_emu_exit_fence(fence);																			\
			return;																										\
		}																												\
	}																													\
}

#define EMU_GEN_atomic_ops_andorxoradd_by_llsc(FNAME, TYPE, MNEMONIC, OP) \
static inline void atomic_ops_##MNEMONIC##_##FNAME(atomic_ops_##MNEMONIC *atomic, TYPE val, ATOMIC_OPS_FENCE fence) {	\
	atomic_ops_emu_entry_fence(fence);																					\
																														\
	while (true) {																										\
		TYPE oldval = atomic_ops_##MNEMONIC##_ll(atomic);																\
																														\
		if (atomic_ops_##MNEMONIC##_sc(atomic, (oldval OP val))) {														\
			atomic_ops_emu_exit_fence(fence);																			\
			return;																										\
		}																												\
	}																													\
}

// Alternative add implementations
#define EMU_GEN_atomic_ops_add_by_faa(TYPE, MNEMONIC) \
static inline void atomic_ops_##MNEMONIC##_add(atomic_ops_##MNEMONIC *atomic, TYPE val, ATOMIC_OPS_FENCE fence) {	\
	atomic_ops_##MNEMONIC##_fetch_and_add(atomic, (TYPE) val, fence);												\
}

// Alternative inc/dec implementations
#define EMU_GEN_atomic_ops_incdec_by_add(FNAME, TYPE, MNEMONIC, VALUE) \
static inline void atomic_ops_##MNEMONIC##_##FNAME(atomic_ops_##MNEMONIC *atomic, ATOMIC_OPS_FENCE fence) {	\
	atomic_ops_##MNEMONIC##_add(atomic, (TYPE) VALUE, fence);												\
}

// Alternative FAA implementations
#define EMU_GEN_atomic_ops_fetch_and_add_by_cas(TYPE, MNEMONIC) \
static inline TYPE atomic_ops_##MNEMONIC##_fetch_and_add(atomic_ops_##MNEMONIC *atomic, TYPE val, ATOMIC_OPS_FENCE fence) {	\
	atomic_ops_emu_entry_fence(fence);																						\
																															\
	while (true) {																											\
		TYPE oldval = atomic_ops_##MNEMONIC##_load(atomic, ATOMIC_OPS_FENCE_NONE);											\
																															\
		if (atomic_ops_##MNEMONIC##_cas(atomic, oldval, (oldval + val), ATOMIC_OPS_FENCE_NONE)) {							\
			atomic_ops_emu_exit_fence(fence);																				\
			return (oldval);																								\
		}																													\
	}																														\
}

#define EMU_GEN_atomic_ops_fetch_and_add_by_llsc(TYPE, MNEMONIC) \
static inline TYPE atomic_ops_##MNEMONIC##_fetch_and_add(atomic_ops_##MNEMONIC *atomic, TYPE val, ATOMIC_OPS_FENCE fence) {	\
	atomic_ops_emu_entry_fence(fence);																						\
																															\
	while (true) {																											\
		TYPE oldval = atomic_ops_##MNEMONIC##_ll(atomic);																	\
																															\
		if (atomic_ops_##MNEMONIC##_sc(atomic, (oldval + val))) {															\
			atomic_ops_emu_exit_fence(fence);																				\
			return (oldval);																								\
		}																													\
	}																														\
}

// Alternative FAA_inc/FAA_dec implementations
#define EMU_GEN_atomic_ops_fetch_and_incdec_by_faa(FNAME, TYPE, MNEMONIC, VALUE) \
static inline TYPE atomic_ops_##MNEMONIC##_fetch_and_##FNAME(atomic_ops_##MNEMONIC *atomic, ATOMIC_OPS_FENCE fence) {	\
	return (atomic_ops_##MNEMONIC##_fetch_and_add(atomic, (TYPE) VALUE, fence));										\
}

// Alternative CAS-return implementations
#define EMU_GEN_atomic_ops_casr_by_llsc(TYPE, MNEMONIC) \
static inline TYPE atomic_ops_##MNEMONIC##_casr(atomic_ops_##MNEMONIC *atomic, TYPE oldval, TYPE newval, ATOMIC_OPS_FENCE fence) {	\
	atomic_ops_emu_entry_fence(fence);																								\
																																	\
	while (true) {																													\
		TYPE prev = atomic_ops_##MNEMONIC##_ll(atomic);																				\
																																	\
		if (prev != oldval) {																										\
			atomic_ops_emu_exit_fence(fence);																						\
			return (prev);																											\
		}																															\
																																	\
		if (atomic_ops_##MNEMONIC##_sc(atomic, newval)) {																			\
			atomic_ops_emu_exit_fence(fence);																						\
			return (oldval);																										\
		}																															\
	}																																\
}

// Alternative CAS-bool implementations
#define EMU_GEN_atomic_ops_cas_by_casr(TYPE, MNEMONIC) \
static inline bool atomic_ops_##MNEMONIC##_cas(atomic_ops_##MNEMONIC *atomic, TYPE oldval, TYPE newval, ATOMIC_OPS_FENCE fence) {	\
	return (atomic_ops_##MNEMONIC##_casr(atomic, oldval, newval, fence) == oldval);													\
}

#define EMU_GEN_atomic_ops_cas_by_llsc(TYPE, MNEMONIC) \
static inline bool atomic_ops_##MNEMONIC##_cas(atomic_ops_##MNEMONIC *atomic, TYPE oldval, TYPE newval, ATOMIC_OPS_FENCE fence) {	\
	atomic_ops_emu_entry_fence(fence);																								\
																																	\
	while (true) {																													\
		TYPE prev = atomic_ops_##MNEMONIC##_ll(atomic);																				\
																																	\
		if (prev != oldval) {																										\
			atomic_ops_emu_exit_fence(fence);																						\
			return (false);																											\
		}																															\
																																	\
		if (atomic_ops_##MNEMONIC##_sc(atomic, newval)) {																			\
			atomic_ops_emu_exit_fence(fence);																						\
			return (true);																											\
		}																															\
	}																																\
}

// Alternative SWAP implementations
#define EMU_GEN_atomic_ops_swap_by_cas(TYPE, MNEMONIC) \
static inline TYPE atomic_ops_##MNEMONIC##_swap(atomic_ops_##MNEMONIC *atomic, TYPE val, ATOMIC_OPS_FENCE fence) {	\
	atomic_ops_emu_entry_fence(fence);																				\
																													\
	while (true) {																									\
		TYPE oldval = atomic_ops_##MNEMONIC##_load(atomic, ATOMIC_OPS_FENCE_NONE);									\
																													\
		if (atomic_ops_##MNEMONIC##_cas(atomic, oldval, val, ATOMIC_OPS_FENCE_NONE)) {								\
			atomic_ops_emu_exit_fence(fence);																		\
			return (oldval);																						\
		}																											\
	}																												\
}

#define EMU_GEN_atomic_ops_swap_by_llsc(TYPE, MNEMONIC) \
static inline TYPE atomic_ops_##MNEMONIC##_swap(atomic_ops_##MNEMONIC *atomic, TYPE val, ATOMIC_OPS_FENCE fence) {	\
	atomic_ops_emu_entry_fence(fence);																				\
																													\
	while (true) {																									\
		TYPE oldval = atomic_ops_##MNEMONIC##_ll(atomic);															\
																													\
		if (atomic_ops_##MNEMONIC##_sc(atomic, val)) {																\
			atomic_ops_emu_exit_fence(fence);																		\
			return (oldval);																						\
		}																											\
	}																												\
}
