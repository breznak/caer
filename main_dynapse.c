/*
 * main.c
 *
 *  Created on: Nov 2016
 *      Author: federico.corradi@inilabs.com
 *
 *  Compile & run:
 *  $ cd caer/
 *  $ rm -rf CMakeFiles CMakeCache.txt
 *  $ cmake -DDYNAPSEFX2=1 -DENABLE_DYNAPSEINIT=1 -DENABLE_STATISTICS=1 -DENABLE_VISUALIZER=1 .
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

#ifdef ENABLE_VISUALIZER
	caerVisualizerEventHandler visualizerEventHandler = NULL;
#endif

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

	// check dynapse output
#ifdef ENABLE_DYNAPSEINIT
	caerDynapseInit(2, spike);
#endif

	// Filters can also extract information from event packets: for example
	// to show statistics about the current event-rate.
#ifdef ENABLE_STATISTICS
	caerStatistics(3, (caerEventPacketHeader) spike, 1000);
#endif

	// A simple visualizer exists to show what the output looks like.
#ifdef ENABLE_VISUALIZER
	caerVisualizer(64, "Spike", &caerVisualizerRendererSpikeEvents, visualizerEventHandler, (caerEventPacketHeader) spike);
	//caerVisualizer(60, "Polarity", &caerVisualizerRendererPolarityEvents, visualizerEventHandler, (caerEventPacketHeader) polarity);
	//caerVisualizer(61, "Frame", &caerVisualizerRendererFrameEvents, visualizerEventHandler, (caerEventPacketHeader) frame);
	//caerVisualizer(62, "IMU6", &caerVisualizerRendererIMU6Events, visualizerEventHandler, (caerEventPacketHeader) imu);

	//caerVisualizerMulti(68, "PolarityAndFrame", &caerVisualizerMultiRendererPolarityAndFrameEvents, visualizerEventHandler, container);
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

/* input string managment */
char* getTagValue(char* a_tag_list, char* a_tag) {
	/* 'strtok' modifies the string. */
	char* tag_list_copy = malloc(strlen(a_tag_list) + 1);
	char* result = 0;
	char* s;

	strcpy(tag_list_copy, a_tag_list);

	s = strtok(tag_list_copy, "&");
	while (s) {
		char* equals_sign = strchr(s, '=');
		if (equals_sign) {
			*equals_sign = 0;
			if (0 == strcmp(s, a_tag)) {
				equals_sign++;
				result = malloc(strlen(equals_sign) + 1);
				strcpy(result, equals_sign);
			}
		}
		s = strtok(0, " ");
	}
	free(tag_list_copy);

	return result;
}
/* input string end*/

int main(int argc, char **argv) {
	// Set thread name.
	thrd_set_name("Main");

	/* decide what to initialize */
	/*bool skipuserpref = false;
	bool sram = true; // by default write the sram
	bool cam = true;  // by default write the cam

	if (argc == 3) {
		if (strlen(argv[1]) > 8 || strlen(argv[2]) > 8) {
			caerLog(CAER_LOG_ERROR, "Input arguments: ",
					"arguments to long, please select the right arguments.\n");
			caerLog(CAER_LOG_ERROR, "Input arguments: ",
					"Please specify sram=0 or 1 cam=0 or 1.  \n");
			caerLog(CAER_LOG_ERROR, "Input arguments: ",
					" 0 -> do not configure \n");
			caerLog(CAER_LOG_ERROR, "Input arguments: ",
					" 1 -> configure default content \n");
			return (EXIT_FAILURE);
		}
		char tag_list[20];
		strcat(tag_list, argv[1]);
		strcat(tag_list, "&");
		strcat(tag_list, argv[2]);

		char* sramconfig = getTagValue(argv[1], "sram");
		char* camconfig = getTagValue(argv[2], "cam");

		if(caerStrEquals(sramconfig,"1")){
			caerLog( CAER_LOG_ERROR,  "Input arguments: ", "The sram is ONE\n", sramconfig);
		 }else if(caerStrEquals(sramconfig,"0")){
			 caerLog( CAER_LOG_ERROR,  "Input arguments: ", "The sram is ZERO\n", sramconfig);
			 sram = false;
		 }else{
			 caerLog( CAER_LOG_ERROR, "Input arguments: ", "Sram option %s is invalid. Please select 1 or 0.\n", sramconfig);
			 return(EXIT_FAILURE);
		 }

		if(caerStrEquals(camconfig,"1")){
			caerLog( CAER_LOG_ERROR,  "Input arguments: ", "The sram is ONE\n", sramconfig);
		 }else if(caerStrEquals(camconfig,"0")){
			 caerLog( CAER_LOG_ERROR,  "Input arguments: ", "The sram is ZERO\n", sramconfig);
			 cam = false;
		 }else{
			 caerLog( CAER_LOG_ERROR, "Input arguments: ", "Sram option %s is invalid. Please select 1 or 0.\n", sramconfig);
			 return(EXIT_FAILURE);
		 }

	} else if (argc == 1) {
		caerLog(CAER_LOG_WARNING, "Input arguments: ", "none given.\n");
		caerLog(CAER_LOG_WARNING, "Input arguments: ",
				"Please specify sram=0 or 1 cam=0 or 1.  \n");
		caerLog(CAER_LOG_WARNING, "Input arguments: ",
				" 0 -> do not configure \n");
		caerLog(CAER_LOG_WARNING, "Input arguments: ",
				" 1 -> configure default content \n");
		caerLog(CAER_LOG_WARNING, "Initialization: ",
				"Proceeding with default configuration (ie: sram=1, cam=1).\n");
	} else if (argc <= 2) {
		caerLog(CAER_LOG_ERROR, "Input arguments: ",
				"Need exactly two input arguments.\n");
		return (EXIT_FAILURE);
	} else if (argc > 3) {
		caerLog(CAER_LOG_ERROR, "Input arguments: ",
				"Too many arguments supplied.\n");
		return (EXIT_FAILURE);
	} else {
		caerLog(CAER_LOG_WARNING, "Input arguments: ",
				"One argument expected.\n");
		caerLog(CAER_LOG_WARNING, "Initialization: ",
				"Proceeding with default configuration.\n");
		skipuserpref = true;
	}*/

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
