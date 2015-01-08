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
#include "modules/ini/davis_fx2.h"
#include "modules/backgroundactivityfilter/backgroundactivityfilter.h"
#include "modules/statistics/statistics.h"
#include "modules/visualizer/visualizer.h"

static bool mainloop_1(void);
static bool mainloop_2(void);

static bool mainloop_1(void) {
	// Typed EventPackets contain events of a certain type.
	caerPolarityEventPacket davis_polarity;
	caerFrameEventPacket davis_frame;

	// Input modules grab data from outside sources (like devices, files, ...)
	// and put events into an event packet.
	caerInputDAVISFX2(1, &davis_polarity, &davis_frame, NULL, NULL);

	// Filters process event packets: for example to suppress certain events,
	// like with the Background Activity Filter, which suppresses events that
	// look to be uncorrelated with real scene changes (noise reduction).
	caerBackgroundActivityFilter(2, davis_polarity);

	// Filters can also extract information from event packets: for example
	// to show statistics about the current event-rate.
	caerStatistics(3, (caerEventPacketHeader) davis_frame, 1);

	// A small OpenGL visualizer exists to show what the output looks like.
	caerVisualizer(4, davis_polarity, davis_frame);

	return (true); // If false is returned, processing of this loop stops.
}

static bool mainloop_2(void) {
	// Typed EventPackets contain events of a certain type.
	caerPolarityEventPacket davis_polarity;
	caerFrameEventPacket davis_frame;

	// Input modules grab data from outside sources (like devices, files, ...)
	// and put events into an event packet.
	caerInputDAVISFX2(1, &davis_polarity, &davis_frame, NULL, NULL);

	// Filters process event packets: for example to suppress certain events,
	// like with the Background Activity Filter, which suppresses events that
	// look to be uncorrelated with real scene changes (noise reduction).
	caerBackgroundActivityFilter(2, davis_polarity);

	// A small OpenGL visualizer exists to show what the output looks like.
	//caerVisualizer(3, davis_polarity, davis_frame);

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
	struct caer_mainloop_definition mainLoops[2] = { { 1, &mainloop_1 }, { 2, &mainloop_2 } };
	caerMainloopRun(&mainLoops, 1);

	// After shutting down the mainloops, also shutdown the config server
	// thread if needed.
	caerConfigServerStop();

	return (EXIT_SUCCESS);
}
