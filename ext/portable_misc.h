#ifndef PORTABLE_MISC_H_
#define PORTABLE_MISC_H_

#include <stdlib.h>

/**
 * Fully resolve and clean up a (relative) file path.
 * What can be done depends on OS support.
 * Remember to free() the returned string after use!
 *
 * @param path a (relative) file path.
 * @return the absolute, clean file path.
 */
static inline char *portable_realpath(const char *path) {
#if defined(__linux__) || defined(__APPLE__)
	return (realpath(path, NULL));
#elif defined(_WIN32)
	return (_fullpath(NULL, path, _MAX_PATH));
#else
	#error "No portable realpath() found."
#endif
}

#endif
