/*
 * main.c
 *
 *  Created on: Oct 6, 2013
 *      Author: chtekk
 */

#include "main.h"
#include "base/config.h"
#include "base/config_server.h"
#include "base/mainloop.h"
#include "base/misc.h"
#include "modules/ini/davis_fx3.h"
#include "modules/statistics/statistics.h"

static bool mainloop_1(void);

static bool mainloop_1(void) {
	// Typed EventPackets contain events of a certain type.
	caerPolarityEventPacket davisfx3_polarity;
	caerInputDAViSFX3(1, &davisfx3_polarity, NULL, NULL, NULL);

	// Show statistics about event-rate.
	caerStatistics(2, (caerEventPacketHeader) davisfx3_polarity);

	return (true); // If false is returned, processing of this loop stops.
}

int main(int argc, char *argv[]) {
	// Initialize config storage from file, support command-line overrides.
	// If no init from file needed, pass NULL.
	caerConfigInit("caer-config.xml", argc, argv);

	// Initialize logging sub-system.
	caerLogInit();

	// Daemonize the application (run in background).
	//caerDaemonize();

	// Start the configuration server thread for run-time config changes.
	caerConfigServerStart();

	// Finally run the main event processing loops.
	struct caer_mainloop_definition mainLoops[1] = { { 1, &mainloop_1 } };
	caerMainloopRun(&mainLoops, 1);

	// After shutting down the mainloops, also shutdown the config server
	// thread if needed.
	caerConfigServerStop();

	return (EXIT_SUCCESS);
}
