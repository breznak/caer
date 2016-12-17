/*
 * main.c
 *
 *  Created on: Nov 2016
 *      Author: federico.corradi@inilabs.com
 *
 *  Compile & run:
 *  $ cd caer/
 *  $ rm -rf CMakeFiles CMakeCache.txt
 *  $ cmake -DCMAKE_BUILD_TYPE=Debug -DDYNAPSEFX2=1 -DENABLE_STATISTICS=1 -DENABLE_VISUALIZER=1 -DENABLE_MEANRATEFILTER=1 -DENABLE_FILE_OUTPUT=0 .
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
#include "modules/ini/dynapse_fx2.h"
#include "modules/ini/dynapse_common.h"

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

#ifdef ENABLE_MEANRATEFILTER
#include <libcaer/events/frame.h>
#endif
#ifdef ENABLE_LEARNINGFILTER
#include <libcaer/events/frame.h>
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

#ifdef DYNAPSEFX2 //should be 1 foe experiment
	container = caerInputDYNAPSEFX2(1);

	// We search for them by type here, because input modules may not have all or any of them.
	spike = (caerSpikeEventPacket) caerEventPacketContainerFindEventPacketByType(container, SPIKE_EVENT);
	special = (caerSpecialEventPacket) caerEventPacketContainerFindEventPacketByType(container, SPECIAL_EVENT);

#ifdef ENABLE_LEARNINGFILTER
	// create frame for displaying weight and synapse
	caerFrameEventPacket weightplot = NULL;
	caerFrameEventPacket synapseplot = NULL;
#endif

#endif

#ifdef ENABLE_FILE_INPUT //should be 0 for experiment
	container = caerInputFile(10);
	// We search for them by type here, because input modules may not have all or any of them.
	spike = (caerSpikeEventPacket) caerEventPacketContainerFindEventPacketByType(container, SPIKE_EVENT);
	special = (caerSpecialEventPacket) caerEventPacketContainerFindEventPacketByType(container, SPECIAL_EVENT);

#ifdef ENABLE_LEARNINGFILTER
	// create frame for displaying weight and synapse
	caerFrameEventPacket weightplot = NULL;
	caerFrameEventPacket synapseplot = NULL;
#endif

#endif

#ifdef ENABLE_NETWORK_INPUT
	container = caerInputNetTCP(11);
#endif

	// Filters can also extract information from event packets: for example
	// to show statistics about the current event-rate.
#ifdef ENABLE_STATISTICS
	caerStatistics(3, (caerEventPacketHeader) spike, 1000);
#endif

#ifdef ENABLE_MEANRATEFILTER
	// create frame for displaying frequencoes
	caerFrameEventPacket freqplot = NULL;
#ifdef DYNAPSEFX2
	caerMeanRateFilter(4, 1, spike, &freqplot);
#endif
#ifdef ENABLE_FILE_INPUT
	caerMeanRateFilter(4, 10, spike, &freqplot);
#endif
#endif

#ifdef ENABLE_LEARNINGFILTER
#ifdef DYNAPSEFX2
	caerLearningFilter(5, 1, spike, &weightplot, &synapseplot);
#endif
#endif

	// A simple visualizer exists to show what the output looks like.
#ifdef ENABLE_VISUALIZER
	caerVisualizer(64, "Spike", &caerVisualizerRendererSpikeEvents, NULL, (caerEventPacketHeader) spike);
#ifdef ENABLE_MEANRATEFILTER
	caerVisualizer(65, "Frequency", &caerVisualizerRendererFrameEvents, NULL, (caerEventPacketHeader) freqplot);
#endif
#ifdef ENABLE_LEARNINGFILTER
	caerVisualizer(66, "Weight", &caerVisualizerRendererFrameEvents, NULL, (caerEventPacketHeader) weightplot);
	caerVisualizer(67, "Synapse", &caerVisualizerRendererFrameEvents, NULL, (caerEventPacketHeader) synapseplot);
#endif
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

#ifdef ENABLE_MEANRATEFILTER
	free(freqplot);
#endif

	return (true); // If false is returned, processing of this loop stops.
}

int main(int argc, char **argv) {
	// Set thread name.
	thrd_set_name("Main");

	// Initialize config storage from file, support command-line overrides.
	// If no init from file needed, pass NULL.
//	caerConfigInit("caer-config.xml", argc, argv); //?

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
