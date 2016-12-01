/*
 * main.c
 *
 *  Created on: Nov 2016
 *      Author: federico.corradi@inilabs.com
 *
 *  Compile & run:
 *  $ cd caer/
 *  $ rm -rf CMakeFiles CMakeCache.txt
 *  $ cmake -DCMAKE_BUILD_TYPE=Debug -DDYNAPSEFX2=1 -DENABLE_STATISTICS=1 -DENABLE_VISUALIZER=1 .
 *  $ make
 *  $ ./caer-bin
 */

#include "main.h"
#include "base/config.h"
#include "base/config_server.h"
#include "base/log.h"
#include "base/mainloop.h"
#include "base/misc.h"

// Devices support.
#ifdef DYNAPSEFX2
#include "modules/ini/dynapse_fx2.h"
#endif

#ifdef ENABLE_GEN_SPIKES
#include "modules/misc/in/gen_spikes.h"
#endif

// Input/Output support.
#ifdef ENABLE_FILE_INPUT
#include "modules/misc/in/file.h"
#endif
#ifdef ENABLE_NETWORK_INPUT
#include "modules/misc/in/net_tcp.h"
#include "modules/misc/in/unix_socket.h"
#endif

#ifdef ENABLE_FILE_OUTPUT
#include "modules/misc/out/file.h"
#endif
#ifdef ENABLE_NETWORK_OUTPUT
#include "modules/misc/out/net_tcp_server.h"
#include "modules/misc/out/net_tcp.h"
#include "modules/misc/out/net_udp.h"
#include "modules/misc/out/unix_socket_server.h"
#include "modules/misc/out/unix_socket.h"
#endif

#ifdef ENABLE_VISUALIZER
#include "modules/visualizer/visualizer.h"
#endif
// Common filters support.

static bool mainloop_1(void);

static bool mainloop_1(void) {

	// An eventPacketContainer bundles event packets of different types together,
	// to maintain time-coherence between the different events.
	caerEventPacketContainer container = NULL;
	caerSpikeEventPacket spike = NULL;
	caerSpecialEventPacket special = NULL;

	// Input modules grab data from outside sources (like devices, files, ...)
	// and put events into an event packet.

#ifdef DYNAPSEFX2
	container = caerInputDYNAPSEFX2(1);

	// We search for them by type here, because input modules may not have all or any of them.
	spike = (caerSpikeEventPacket) caerEventPacketContainerFindEventPacketByType(container, SPIKE_EVENT);
	special = (caerSpecialEventPacket) caerEventPacketContainerFindEventPacketByType(container, SPECIAL_EVENT);
#endif

#ifdef ENABLE_FILE_INPUT
	container = caerInputFile(10);
#endif
#ifdef ENABLE_NETWORK_INPUT
	container = caerInputNetTCP(11);
#endif
#if defined(ENABLE_FILE_INPUT) || defined(ENABLE_NETWORK_INPUT)
#ifdef ENABLE_VISUALIZER
	visualizerEventHandler = &caerInputVisualizerEventHandler;
#endif

	// We search for them by type here, because input modules may not have all or any of them.
	spike = (caerSpikeEventPacket) caerEventPacketContainerFindEventPacketByType(container, SPIKE_EVENT);
	special = (caerSpecialEventPacket) caerEventPacketContainerFindEventPacketByType(container, SPECIAL_EVENT);
#endif

	// Filters can also extract information from event packets: for example
	// to show statistics about the current event-rate.
#ifdef ENABLE_STATISTICS
	caerStatistics(3, (caerEventPacketHeader) spike, 1000);
#endif

#ifdef ENABLE_MEANRATEFILTER
	caerMeanRateFilter(4, spike);
#endif

	// A simple visualizer exists to show what the output looks like.
#ifdef ENABLE_VISUALIZER
	caerVisualizer(64, "Spike", &caerVisualizerRendererSpikeEvents, &caerVisualizerEventHandlerSpikeEvents, (caerEventPacketHeader) spike);
	caerVisualizer(65, "Frequency", &caerVisualizerRendererSpikeEventsFrequency,  &caerVisualizerEventHandlerSpikeEvents, (caerEventPacketHeader) spike);
#endif

#ifdef ENABLE_FILE_OUTPUT
	// Enable output to file (AEDAT 3.X format).
	caerOutputFile(7, 2, spike, special);
#endif

#ifdef ENABLE_NETWORK_OUTPUT
	// Send polarity packets out via TCP. This is the server mode!
	// External clients connect to cAER, and we send them the data.
	// WARNING: slow clients can dramatically slow this and the whole
	// processing pipeline down!
	caerOutputNetTCPServer(8, 2, spike, special);

	// And also send them via UDP. This is fast, as it doesn't care what is on the other side.
	caerOutputNetUDP(9, 2, spike, special);
#endif

	return (true); // If false is returned, processing of this loop stops.
}

int main(int argc, char **argv) {
	// Set thread name.
	thrd_set_name("Main");

	// Initialize config storage from file, support command-line overrides.
	// If no init from file needed, pass NULL.
	caerConfigInit("caer-config.xml", argc, argv);

	// Initialize logging sub-system.
	caerLogInit();

	// Daemonize the application (run in background, NOT AVAILABLE ON WINDOWS).
	// caerDaemonize();

	// Initialize visualizer framework (load fonts etc.).
#ifdef ENABLE_VISUALIZER
	caerVisualizerSystemInit();
#endif

	// Start the configuration server thread for run-time config changes.
	caerConfigServerStart();

	// Finally run the main event processing loops.
	struct caer_mainloop_definition mainLoops[1] = { { 1, &mainloop_1 } };
	caerMainloopRun(&mainLoops, 1); // Only start Mainloop 1.

	// After shutting down the mainloops, also shutdown the config server
	// thread if needed.
	caerConfigServerStop();

	return (EXIT_SUCCESS);
}
