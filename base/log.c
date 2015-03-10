#include "log.h"
#include <pthread.h>
#include <time.h>
#include <unistd.h>

static FILE *caerLogConsole = NULL;
static FILE *caerLogFile = NULL;
static atomic_ops_uint caerLogLevel = ATOMIC_OPS_UINT_INIT(0);

static void caerLogShutDownWriteBack(void);
static void caerLogSSHSLogger(const char *msg);
static void caerLogLevelListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);

void caerLogInit(void) {
	// Set stderr file output as standard console log output.
	caerLogConsole = stderr;

	sshsNode logNode = sshsGetNode(sshsGetGlobal(), "/logger/");

	// Ensure default log file and value are present.
	// The default path is a file named caer.log inside the program's CWD.
	const char *logFileName = "/caer.log";

	char *logFileDir = getcwd(NULL, 0);
	char *logFileDirClean = realpath(logFileDir, NULL);

	char *logFilePath = malloc(strlen(logFileDirClean) + strlen(logFileName) + 1); // +1 for terminating NUL byte.
	strcpy(logFilePath, logFileDirClean);
	strcat(logFilePath, logFileName);

	sshsNodePutStringIfAbsent(logNode, "logFile", logFilePath);

	free(logFilePath);
	free(logFileDirClean);
	free(logFileDir);

	sshsNodePutByteIfAbsent(logNode, "logLevel", LOG_NOTICE);

	// Try to open the specified file and error out if not possible.
	char *logFile = sshsNodeGetString(logNode, "logFile");
	caerLogFile = fopen(logFile, "a");

	if (caerLogFile == NULL) {
		// Must be able to open log file! _REQUIRED_
		fprintf(stderr, "Logger: Failed to open log file '%s'. Error: %s (%d).\n", logFile, caerLogStrerror(errno), errno);
		free(logFile);

		exit(EXIT_FAILURE);
	}

	free(logFile);

	// Make log file line-buffered, so that new messages get written right away.
	setlinebuf(caerLogFile);

	// Make sure log file gets flushed at exit time.
	atexit(&caerLogShutDownWriteBack);

	// Set global log level and install listener for its update.
	uint8_t logLevel = sshsNodeGetByte(logNode, "logLevel");
	atomic_ops_uint_store(&caerLogLevel, logLevel, ATOMIC_OPS_FENCE_RELEASE);

	sshsNodeAddAttrListener(logNode, NULL, &caerLogLevelListener);

	// From localtime_r() man-page: "According to POSIX.1-2004, localtime()
	// is required to behave as though tzset(3) was called, while
	// localtime_r() does not have this requirement."
	// So we make sure to call it here, to be portable.
	tzset();

	// Now that config is initialized (has to be!) and logging too, we can
	// set the SSHS logger to use our internal logger too.
	sshsSetGlobalErrorLogCallback(&caerLogSSHSLogger);

	// Log sub-system initialized fully and correctly, log this.
	caerLog(LOG_NOTICE, "Logger", "Initialization successful with log-level %" PRIu8 ".", logLevel);
}

void caerLogDisableConsole(void) {
	caerLogConsole = NULL;
	caerLog(LOG_DEBUG, "Logger", "Console logging disabled.");
}

void caerLog(uint8_t logLevel, const char *subSystem, const char *format, ...) {
	// Check that subSystem and format are defined correctly.
	if (subSystem == NULL || format == NULL) {
		caerLog(LOG_ERROR, "Logger", "Missing subSystem or format strings. Neither can be NULL.");
		return;
	}

	// Only log messages above the specified level.
	if (logLevel <= atomic_ops_uint_load(&caerLogLevel, ATOMIC_OPS_FENCE_NONE)) {
		// First prepend the time.
		time_t currentTimeEpoch = time(NULL);

		struct tm currentTime;
		localtime_r(&currentTimeEpoch, &currentTime);

		// Following time format uses exactly 19 characters (5 separators,
		// 4 year, 2 month, 2 day, 2 hours, 2 minutes, 2 seconds).
		size_t currentTimeStringLength = 19;
		char currentTimeString[currentTimeStringLength + 1]; // + 1 for terminating NUL byte.
		strftime(currentTimeString, currentTimeStringLength + 1, "%Y-%m-%d %H:%M:%S", &currentTime);

		// Prepend debug level as a string to format.
		const char *logLevelString;
		switch (logLevel) {
			case LOG_EMERGENCY:
				logLevelString = "EMERGENCY";
				break;

			case LOG_ALERT:
				logLevelString = "ALERT";
				break;

			case LOG_CRITICAL:
				logLevelString = "CRITICAL";
				break;

			case LOG_ERROR:
				logLevelString = "ERROR";
				break;

			case LOG_WARNING:
				logLevelString = "WARNING";
				break;

			case LOG_NOTICE:
				logLevelString = "NOTICE";
				break;

			case LOG_INFO:
				logLevelString = "INFO";
				break;

			case LOG_DEBUG:
				logLevelString = "DEBUG";
				break;

			default:
				logLevelString = "UNKNOWN";
				break;
		}

		// Copy all strings into one and ensure NUL termination.
		size_t logLength = (size_t) snprintf(NULL, 0, "%s: %s: %s: %s\n", currentTimeString, logLevelString, subSystem, format);
		char logString[logLength + 1];
		snprintf(logString, logLength + 1, "%s: %s: %s: %s\n", currentTimeString, logLevelString, subSystem, format);

		va_list argptr;

		if (caerLogConsole != NULL) {
			va_start(argptr, format);
			vfprintf(caerLogConsole, logString, argptr);
			va_end(argptr);
		}

		if (caerLogFile != NULL) {
			va_start(argptr, format);
			vfprintf(caerLogFile, logString, argptr);
			va_end(argptr);
		}
	}
}

// Guaranteed thread-safe strerror(), with no need to free the returned read-only string.
const char *caerLogStrerror(int errNum) {
	if (errNum < 0 || errNum >= sys_nerr || sys_errlist[errNum] == NULL) {
		return ("Unknown error");
	}
	else {
		return (sys_errlist[errNum]);
	}
}

static void caerLogShutDownWriteBack(void) {
	if (caerLogFile != NULL) {
		caerLog(LOG_DEBUG, "Logger", "Shutting down ...");

		// Ensure proper flushing and closing of the log file at shutdown.
		fflush(caerLogFile);
		fclose(caerLogFile);
	}
}

static void caerLogSSHSLogger(const char *msg) {
	caerLog(LOG_EMERGENCY, "SSHS", "%s", msg);
	// SSHS will exit automatically on critical errors.
}

static void caerLogLevelListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);
	UNUSED_ARGUMENT(userData);

	if (event == ATTRIBUTE_MODIFIED && changeType == BYTE && strcmp(changeKey, "logLevel") == 0) {
		// Update the global log level asynchronously.
		atomic_ops_uint_store(&caerLogLevel, changeValue.ubyte, ATOMIC_OPS_FENCE_RELEASE);
		caerLog(LOG_DEBUG, "Logger", "Log-level set to %" PRIu8 ".", changeValue.ubyte);
	}
}
