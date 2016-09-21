#include "libuvline.h"

#define LIBUV_SHELL_ESCAPE "\x1B"
#define LIBUV_SHELL_BACKSPACE '\x08'
#define LIBUV_SHELL_DELETE '\x7F'
#define LIBUV_SHELL_ETX '\x03'
#define LIBUV_SHELL_EOT '\x04'
#define LIBUV_SHELL_NEWLINE '\x0A'
#define LIBUV_SHELL_CARRIAGERETURN '\x0D'
#define LIBUV_SHELL_TAB '\x09'

static void libuvTTYOnInputClose(uv_handle_t *handle);
static void libuvTTYAlloc(uv_handle_t *tty, size_t suggestedSize, uv_buf_t *buf);
static void libuvTTYRead(uv_stream_t *tty, ssize_t sizeRead, const uv_buf_t *buf);

static void libuvTTYAutoCompleteFree(libuvTTYCompletions autoComplete);
static void libuvTTYAutoCompleteUpdateCompletions(libuvTTYCompletions autoComplete, const char *currentString);

int libuvTTYInit(uv_loop_t *loop, libuvTTY tty, const char *shellPrompt,
	void (*handleInputLine)(const char *buf, size_t bufLength)) {
	int retVal;

	// Reset TTY memory.
	memset(tty, 0, sizeof(*tty));

	tty->handleInputLine = handleInputLine;

	// Generate shell prompt.
	size_t shellPromptLength = strlen(shellPrompt);
	tty->shellPrompt = malloc(shellPromptLength + 10); // For ESCAPE "[2K\r" " >> " and NUL character.
	if (tty->shellPrompt == NULL) {
		return (UV_ENOMEM);
	}

	// Erase current line, carriage return, shell prompt, separator, NUL character.
	size_t memIdx = 0;

	memcpy(tty->shellPrompt + memIdx, LIBUV_SHELL_ESCAPE "[2K\r", 5);
	memIdx += 5;

	memcpy(tty->shellPrompt + memIdx, shellPrompt, shellPromptLength);
	memIdx += shellPromptLength;

	memcpy(tty->shellPrompt + memIdx, " >> ", 4);
	memIdx += 4;

	tty->shellPrompt[memIdx] = '\0';

	// Initialize stdin TTY.
	retVal = uv_tty_init(loop, &tty->ttyIn, STDIN_FILENO, true);
	if (retVal < 0) {
		free(tty->shellPrompt);
		return (retVal);
	}

	// Associate TTY with its data.
	tty->ttyIn.data = tty;

	// Switch stdin TTY to RAW mode, so we get every character and can build
	// up auto-completion and console like input.
	retVal = uv_tty_set_mode(&tty->ttyIn, UV_TTY_MODE_RAW);
	if (retVal < 0) {
		uv_close((uv_handle_t *) &tty->ttyIn, &libuvTTYOnInputClose);
		return (retVal);
	}

	// Start reading on input TTY.
	retVal = uv_read_start((uv_stream_t *) &tty->ttyIn, &libuvTTYAlloc, &libuvTTYRead);
	if (retVal < 0) {
		uv_tty_reset_mode();
		uv_close((uv_handle_t *) &tty->ttyIn, &libuvTTYOnInputClose);
		return (retVal);
	}

	// Output shell prompt. Disable buffering on stdout.
	setvbuf(stdout, NULL, _IONBF, 0);
	fprintf(stdout, "%s", tty->shellPrompt);

	// Return success.
	return (0);
}

void libuvTTYClose(libuvTTY tty) {
	uv_tty_reset_mode();

	uv_close((uv_handle_t *) &tty->ttyIn, &libuvTTYOnInputClose);
}

static void libuvTTYOnInputClose(uv_handle_t *handle) {
	libuvTTY tty = handle->data;

	free(tty->shellPrompt);

	libuvTTYAutoCompleteFree(tty->autoComplete);
}

static void libuvTTYAlloc(uv_handle_t *tty, size_t suggestedSize, uv_buf_t *buf) {
	(void) (suggestedSize);

	// We use one fixed, small buffer in the main TTY. data structure.
	libuvTTY ttyData = tty->data;

	buf->base = ttyData->ttyCmdIn;
	buf->len = LIBUV_SHELL_MAX_CMDINLENGTH;
}

static void libuvTTYRead(uv_stream_t *tty, ssize_t sizeRead, const uv_buf_t *buf) {
	if (sizeRead < 0) {
		fprintf(stderr, "STDIN closed: %s.\n", uv_err_name((int) sizeRead));
		uv_read_stop(tty);
		return;
	}

	libuvTTY ttyData = tty->data;

	if (sizeRead == 1) {
		char c = buf->base[0];

		// Detect termination.
		if (c == LIBUV_SHELL_EOT || c == LIBUV_SHELL_ETX) {
			fprintf(stdout, "\n");
			uv_read_stop(tty);
			return;
		}

		// Detect newline -> go to newline and submit request.
		if (c == LIBUV_SHELL_NEWLINE || c == LIBUV_SHELL_CARRIAGERETURN) {
			fprintf(stdout, "\n");

			// Always support using 'exit' or 'quit' commands to stop the application.
			if (strncmp(ttyData->shellContent, "quit", 4) == 0 || strncmp(ttyData->shellContent, "exit", 4) == 0) {
				uv_read_stop(tty);
				return;
			}

			// Call input handler if there is any input.
			if (ttyData->shellContentIndex > 0) {
				ttyData->handleInputLine(ttyData->shellContent, ttyData->shellContentIndex);
			}

			// Reset line to empty.
			ttyData->shellContent[0] = '\0';
			ttyData->shellContentIndex = 0;

			// Manual change to string, reset auto-completion.
			libuvTTYAutoCompleteClearCompletions(ttyData->autoComplete);
		}
		else if (ttyData->autoComplete != NULL && c == LIBUV_SHELL_TAB) {
			// Auto-completion support.
			libuvTTYAutoCompleteUpdateCompletions(ttyData->autoComplete, ttyData->shellContent);

			// Select current completion.
			char *currCompletion = ttyData->autoComplete->completions[ttyData->autoComplete->selectedCompletion];

			if (currCompletion != NULL) {
				ttyData->autoComplete->completionInProgress = true;

				// Print current completion, if it exists.
				fprintf(stdout, "%s%s", ttyData->shellPrompt, currCompletion);

				return;
			}
		}
		else if (ttyData->autoComplete != NULL && c == ttyData->autoComplete->completionConfirmChar
			&& ttyData->autoComplete->completionInProgress) {
			ttyData->autoComplete->completionInProgress = false;

			// Confirmed hit! Update current string with new one.
			char *currCompletion = ttyData->autoComplete->completions[ttyData->autoComplete->selectedCompletion];
			size_t currCompletionLength = strlen(currCompletion);

			memcpy(ttyData->shellContent, currCompletion, currCompletionLength);
			ttyData->shellContentIndex = currCompletionLength;
			ttyData->shellContent[ttyData->shellContentIndex] = '\0';
		}
		else if (c == LIBUV_SHELL_BACKSPACE || c == LIBUV_SHELL_DELETE) {
			// Manual change to string, reset auto-completion.
			libuvTTYAutoCompleteClearCompletions(ttyData->autoComplete);

			if (ttyData->shellContentIndex == 0) {
				return;
			}
			ttyData->shellContentIndex--;
			ttyData->shellContent[ttyData->shellContentIndex] = '\0';
		}
		else {
			// Manual change to string, reset auto-completion.
			libuvTTYAutoCompleteClearCompletions(ttyData->autoComplete);

			// Got char. Add to current line.
			ttyData->shellContent[ttyData->shellContentIndex] = c;
			ttyData->shellContentIndex++;
			ttyData->shellContent[ttyData->shellContentIndex] = '\0';
		}

		// Write updated line to stdout.
		fprintf(stdout, "%s%s", ttyData->shellPrompt, ttyData->shellContent);
	}
	else if (sizeRead == 3) {
		// Arrow keys.
		// TODO: history support.
		if (memcmp(buf->base, LIBUV_SHELL_ESCAPE "[A", 3) == 0) {

		}
		else if (memcmp(buf->base, LIBUV_SHELL_ESCAPE "[B", 3) == 0) {

		}
		else if (memcmp(buf->base, LIBUV_SHELL_ESCAPE "[D", 3) == 0) {

		}
		else if (memcmp(buf->base, LIBUV_SHELL_ESCAPE "[C", 3) == 0) {

		}
		else {
			fprintf(stderr, "Got unknown sequence: (%X) (%X) (%X).\n", buf->base[0], buf->base[1], buf->base[2]);
		}
	}
	else {
		fprintf(stderr, "Got unknown sequence: ");
		for (size_t i = 0; i < (size_t) sizeRead; i++) {
			fprintf(stderr, "(%X) ", buf->base[i]);
		}
		fprintf(stderr, ".\n");
	}
}

int libuvTTYAutoCompleteSetCallback(libuvTTY tty,
	void (*generateCompletions)(const char *buf, size_t bufLength, libuvTTYCompletions autoComplete)) {
	// Initialize auto-complete support.
	if (generateCompletions != NULL) {
		tty->autoComplete = calloc(1, sizeof(*tty->autoComplete));
		if (tty->autoComplete == NULL) {
			return (UV_ENOMEM);
		}

		tty->autoComplete->generateCompletions = generateCompletions;
		libuvTTYAutoCompleteSetConfirmChar(tty, ' '); // Default confirmation character is SPACE.
	}
	else {
		// Disable auto-complete support.
		libuvTTYAutoCompleteFree(tty->autoComplete);
	}

	return (0);
}

void libuvTTYAutoCompleteSetConfirmChar(libuvTTY tty, char confirmChar) {
	if (tty->autoComplete != NULL) {
		tty->autoComplete->completionConfirmChar = confirmChar;
	}
}

static void libuvTTYAutoCompleteFree(libuvTTYCompletions autoComplete) {
	if (autoComplete == NULL) {
		return;
	}

	libuvTTYAutoCompleteClearCompletions(autoComplete);

	free(autoComplete);
}

void libuvTTYAutoCompleteClearCompletions(libuvTTYCompletions autoComplete) {
	if (autoComplete == NULL) {
		return;
	}

	for (size_t i = 0; i < LIBUV_SHELL_MAX_COMPLETIONS; i++) {
		free(autoComplete->completions[i]);
		autoComplete->completions[i] = NULL;
	}

	autoComplete->selectedCompletion = 0;
	autoComplete->completionInProgress = false;

	free(autoComplete->basedOnString);
	autoComplete->basedOnString = NULL;
}

void libuvTTYAutoCompleteAddCompletion(libuvTTYCompletions autoComplete, const char *completion) {
	if (autoComplete == NULL || completion == NULL) {
		return;
	}

	// Put copy of completion proposal into next free slot.
	for (size_t i = 0; i < LIBUV_SHELL_MAX_COMPLETIONS; i++) {
		if (autoComplete->completions[i] == NULL) {
			autoComplete->completions[i] = strdup(completion);
			break;
		}
	}
}

static void libuvTTYAutoCompleteUpdateCompletions(libuvTTYCompletions autoComplete, const char *currentString) {
	// If we never generated completions, or the we have new information on the completions,
	// we re-generate the list and reset the index to zero.
	if (autoComplete->basedOnString == NULL || strcasecmp(autoComplete->basedOnString, currentString)) {
		libuvTTYAutoCompleteClearCompletions(autoComplete);

		(*autoComplete->generateCompletions)(currentString, strlen(currentString), autoComplete);

		autoComplete->basedOnString = strdup(currentString);
	}
	else {
		// Same completion, just hit TAB again.
		autoComplete->selectedCompletion++;

		if (autoComplete->completions[autoComplete->selectedCompletion] == NULL) {
			autoComplete->selectedCompletion = 0; // Wrap around at end.
		}
	}
}
