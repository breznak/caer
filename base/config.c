#include "config.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

/**
 * NOTE: cannot use caerLog() in configuration system, since it is initialized
 * _before_ the logging system, always!
 */

static char *caerConfigFilePath = NULL;

static void caerConfigShutDownWriteBack(void);

void caerConfigInit(const char *configFile, int argc, char *argv[]) {
	// If configFile is NULL, no config file will be accessed at all,
	// and neither will it be written back at shutdown.
	if (configFile != NULL) {
		// Let's try to open the file for reading, or create it.
		int configFileFd = open(configFile, O_RDONLY | O_CREAT, S_IWUSR | S_IRUSR | S_IRGRP);

		if (configFileFd >= 0) {
			// File opened for reading successfully.
			// This means it exists and we can access it, so let's remember
			// it for writing the config later at shutdown (if permitted).
			caerConfigFilePath = realpath(configFile, NULL);

			// Determine if there is actual content to parse first.
			struct stat configFileStat;
			fstat(configFileFd, &configFileStat);

			if (configFileStat.st_size > 0) {
				sshsNodeImportSubTreeFromXML(sshsGetNode(sshsGetGlobal(), "/"), configFileFd, true);
			}

			close(configFileFd);

			// Ensure configuration is written back at shutdown.
			atexit(&caerConfigShutDownWriteBack);
		}
		else {
			fprintf(stderr, "Could not create and/or read from the configuration file '%s'. Error: %s (%d).\n",
				configFile, caerLogStrerror(errno), errno);
			exit(EXIT_FAILURE);
		}
	}
	else {
		fprintf(stderr, "No configuration file defined, using default values for everything.\n");
	}

	// Override with command line arguments if requested.
	if (argc > 1) {
		// Format: -o node key type value (5 arguments). Equal to caerctl format.
		for (size_t i = 1; i < (size_t) argc; i += 5) {
			if ((i + 4) < (size_t) argc && strcmp(argv[i], "-o") == 0) {
				sshsNode node = sshsGetNode(sshsGetGlobal(), argv[i + 1]);
				if (node == NULL) {
					fprintf(stderr, "Node %s doesn't exist.\n", argv[i + 1]);
					continue;
				}

				if (!sshsNodeStringToNodeConverter(node, argv[i + 2], argv[i + 3], argv[i + 4])) {
					fprintf(stderr, "Failed to convert attribute %s of type %s with value %s.\n", argv[i + 2],
						argv[i + 3], argv[i + 4]);
				}
			}
		}
	}
}

static void caerConfigShutDownWriteBack(void) {
	if (caerConfigFilePath != NULL) {
		int configFileFd = open(caerConfigFilePath, O_WRONLY | O_TRUNC);

		if (configFileFd >= 0) {
			sshsNodeExportSubTreeToXML(sshsGetNode(sshsGetGlobal(), "/"), configFileFd);

			close(configFileFd);
		}
		else {
			fprintf(stderr, "Could not write to the configuration file '%s'. Error: %s (%d).\n", caerConfigFilePath,
				caerLogStrerror(errno), errno);
		}

		// realpath() allocated memory for this above.
		free(caerConfigFilePath);
	}
}
