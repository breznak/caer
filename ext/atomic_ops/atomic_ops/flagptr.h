/**
 * This file is part of the atomic_ops project.
 *
 * For the full copyright and license information, please view the COPYING
 * file that was distributed with this source code.
 *
 * @copyright  (c) the atomic_ops project
 * @author     Luca Longinotti <chtekk@longitekk.com>
 * @license    BSD 2-clause
 * @version    $Id: flagptr.h 1079 2012-06-08 14:45:07Z llongi $
 */

/*
 * Flag-Pointer Implementation
 */

// Masks to access required bits in Flag-Pointers
#define ATOMIC_OPS_FLAGPTR_MASKPTR(X) ((void *)(((uintptr_t)(X)) & (((uintptr_t)-1) << 1)))
#define ATOMIC_OPS_FLAGPTR_MASKFLAG(X) ((bool)(((uintptr_t)(X)) & ((uintptr_t)1)))
#define ATOMIC_OPS_FLAGPTR_MAKEPTR(P, F) ((void *)(((uintptr_t)(P)) | ((uintptr_t)(F))))

static inline void * atomic_ops_flagptr_load(const atomic_ops_flagptr *atomic, bool *flag, ATOMIC_OPS_FENCE fence) {
	void *flagptr = atomic_ops_ptr_load(&atomic->p, fence);

	if (flag != NULL) {
		*flag = ATOMIC_OPS_FLAGPTR_MASKFLAG(flagptr);
	}

	return (ATOMIC_OPS_FLAGPTR_MASKPTR(flagptr));
}

static inline void * atomic_ops_flagptr_load_full(const atomic_ops_flagptr *atomic, bool *flag, ATOMIC_OPS_FENCE fence) {
	void *flagptr = atomic_ops_ptr_load(&atomic->p, fence);

	if (flag != NULL) {
		*flag = ATOMIC_OPS_FLAGPTR_MASKFLAG(flagptr);
	}

	return (flagptr);
}

static inline void atomic_ops_flagptr_store(atomic_ops_flagptr *atomic, void *newptr, bool newflag, ATOMIC_OPS_FENCE fence) {
	atomic_ops_ptr_store(&atomic->p, ATOMIC_OPS_FLAGPTR_MAKEPTR(newptr, newflag), fence);
}

static inline void * atomic_ops_flagptr_casr(atomic_ops_flagptr *atomic, bool *flag, void *oldptr, bool oldflag, void *newptr, bool newflag, ATOMIC_OPS_FENCE fence) {
	void *flagptr = atomic_ops_ptr_casr(&atomic->p, ATOMIC_OPS_FLAGPTR_MAKEPTR(oldptr, oldflag), ATOMIC_OPS_FLAGPTR_MAKEPTR(newptr, newflag), fence);

	if (flag != NULL) {
		*flag = ATOMIC_OPS_FLAGPTR_MASKFLAG(flagptr);
	}

	return (ATOMIC_OPS_FLAGPTR_MASKPTR(flagptr));
}

static inline bool atomic_ops_flagptr_cas(atomic_ops_flagptr *atomic, void *oldptr, bool oldflag, void *newptr, bool newflag, ATOMIC_OPS_FENCE fence) {
	return (atomic_ops_ptr_cas(&atomic->p, ATOMIC_OPS_FLAGPTR_MAKEPTR(oldptr, oldflag), ATOMIC_OPS_FLAGPTR_MAKEPTR(newptr, newflag), fence));
}

static inline void * atomic_ops_flagptr_swap(atomic_ops_flagptr *atomic, bool *flag, void *newptr, bool newflag, ATOMIC_OPS_FENCE fence) {
	void *flagptr = atomic_ops_ptr_swap(&atomic->p, ATOMIC_OPS_FLAGPTR_MAKEPTR(newptr, newflag), fence);

	if (flag != NULL) {
		*flag = ATOMIC_OPS_FLAGPTR_MASKFLAG(flagptr);
	}

	return (ATOMIC_OPS_FLAGPTR_MASKPTR(flagptr));
}
