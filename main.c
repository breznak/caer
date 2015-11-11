/*
 * main.c
 *
 *  Created on: Oct 6, 2013
 *      Author: chtekk
 */

#include "main.h"
#include "base/config.h"
#include "base/config_server.h"
#include "base/log.h"
#include "base/mainloop.h"
#include "base/misc.h"
#include "modules/ini/dvs128.h"
#include "modules/ini/davis_fx2.h"
#include "modules/ini/davis_fx3.h"
#include "modules/backgroundactivityfilter/backgroundactivityfilter.h"
#include "modules/statistics/statistics.h"
#include "modules/visualizer/visualizer.h"
#include "modules/misc/out/net_tcp_server.h"
#include "modules/misc/out/net_udp.h"

static bool mainloop_1(void);
static bool mainloop_2(void);

static bool mainloop_1(void) {
	// An eventPacketContainer bundles event packets of different types together,
	// to maintain time-coherence between the different events.
	caerEventPacketContainer container;

	// Input modules grab data from outside sources (like devices, files, ...)
	// and put events into an event packet.
#ifdef DVS128
	container = caerInputDVS128(1);
#endif
#ifdef DAVISFX2
	container = caerInputDAVISFX2(1);
#endif
#ifdef DAVISFX3
	container = caerInputDAVISFX3(1);
#endif

	// Typed EventPackets contain events of a certain type.
	caerSpecialEventPacket special = (caerSpecialEventPacket) caerEventPacketContainerGetEventPacket(container,
		SPECIAL_EVENT);
	caerPolarityEventPacket polarity = (caerPolarityEventPacket) caerEventPacketContainerGetEventPacket(container,
		POLARITY_EVENT);

#if defined(DAVISFX2) || defined(DAVISFX3)
	// Frame and IMU events exist only with DAVIS cameras.
	caerFrameEventPacket frame = (caerFrameEventPacket) caerEventPacketContainerGetEventPacket(container, FRAME_EVENT);
	caerIMU6EventPacket imu = (caerIMU6EventPacket) caerEventPacketContainerGetEventPacket(container, IMU6_EVENT);
#endif

	// Filters process event packets: for example to suppress certain events,
	// like with the Background Activity Filter, which suppresses events that
	// look to be uncorrelated with real scene changes (noise reduction).
	caerBackgroundActivityFilter(2, polarity);

	// Filters can also extract information from event packets: for example
	// to show statistics about the current event-rate.
	caerStatistics(3, (caerEventPacketHeader) polarity, 1000);

#ifdef ENABLE_VISUALIZER
	// A small OpenGL visualizer exists to show what the output looks like.
	#if defined(DAVISFX2) || defined(DAVISFX3)
		caerVisualizer(4, polarity, frame);
	#else
		caerVisualizer(4, polarity, NULL);
	#endif
#endif

#ifdef ENABLE_NET_STREAM
	// Send polarity packets out via TCP. This is the server mode!
	// External clients connect to cAER, and we send them the data.
	// WARNING: slow clients can dramatically slow this and the whole
	// processing pipeline down!
	caerOutputNetTCPServer(5, 1, polarity); // or (5, 2, polarity, frame) for polarity and frames

	// And also send them via UDP. This is fast, as it doesn't care what is on the other side.
	caerOutputNetUDP(6, 1, polarity); // or (6, 2, polarity, frame) for polarity and frames
#endif

	return (true); // If false is returned, processing of this loop stops.
}

static bool mainloop_2(void) {
	// An eventPacketContainer bundles event packets of different types together,
	// to maintain time-coherence between the different events.
	caerEventPacketContainer container;

	// Input modules grab data from outside sources (like devices, files, ...)
	// and put events into an event packet.
#ifdef DVS128
	container = caerInputDVS128(1);
#endif
#ifdef DAVISFX2
	container = caerInputDAVISFX2(1);
#endif
#ifdef DAVISFX3
	container = caerInputDAVISFX3(1);
#endif

	// Typed EventPackets contain events of a certain type.
	caerPolarityEventPacket polarity = (caerPolarityEventPacket) caerEventPacketContainerGetEventPacket(container,
		POLARITY_EVENT);

	// Filters process event packets: for example to suppress certain events,
	// like with the Background Activity Filter, which suppresses events that
	// look to be uncorrelated with real scene changes (noise reduction).
	caerBackgroundActivityFilter(2, polarity);

#ifdef ENABLE_NET_STREAM
	// Send polarity packets out via TCP.
	caerOutputNetTCPServer(3, 1, polarity);
#endif

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
	caerMainloopRun(&mainLoops, 1); // Only start Mainloop 1.

	// After shutting down the mainloops, also shutdown the config server
	// thread if needed.
	caerConfigServerStop();

	return (EXIT_SUCCESS);
}
