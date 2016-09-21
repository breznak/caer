#ifndef LIBUVLINE_H_
#define LIBUVLINE_H_

#define LIBUV_SHELL_MAX_LINELENGTH 4096
#define LIBUV_SHELL_MAX_CMDINLENGTH 8
#define LIBUV_SHELL_MAX_COMPLETIONS 128

#include "ext/libuv.h"

typedef struct libuv_tty_completions_struct *libuvTTYCompletions;

struct libuv_tty_completions_struct {
	// Auto-completion support.
	void (*generateCompletions)(const char *buf, size_t bufLength, libuvTTYCompletions autoComplete);
	char *basedOnString;
	char completionConfirmChar;
	bool completionInProgress;
	size_t selectedCompletion;
	char *completions[LIBUV_SHELL_MAX_COMPLETIONS];
};

typedef struct libuv_tty_struct *libuvTTY;

struct libuv_tty_struct {
	// TTY I/O support.
	uv_tty_t ttyIn;
	char ttyCmdIn[LIBUV_SHELL_MAX_CMDINLENGTH];
	char *shellPrompt;
	char shellContent[LIBUV_SHELL_MAX_LINELENGTH];
	size_t shellContentIndex;
	void (*handleInputLine)(const char *buf, size_t bufLength);
	// Auto-completion support.
	libuvTTYCompletions autoComplete;
};

int libuvTTYInit(uv_loop_t *loop, libuvTTY tty, const char *shellPrompt,
	void (*handleInputLine)(const char *buf, size_t bufLength));
void libuvTTYClose(libuvTTY tty);

int libuvTTYAutoCompleteSetCallback(libuvTTY tty,
	void (*generateCompletions)(const char *buf, size_t bufLength, libuvTTYCompletions autoComplete));
void libuvTTYAutoCompleteSetConfirmChar(libuvTTY tty, char confirmChar);

void libuvTTYAutoCompleteClearCompletions(libuvTTYCompletions autoComplete);
void libuvTTYAutoCompleteAddCompletion(libuvTTYCompletions autoComplete, const char *completion);

#endif /* LIBUVLINE_H_ */
