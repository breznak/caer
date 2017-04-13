/*
 * main.c
 *
 *  Created on: Jan 2017
 *      Author: federico.corradi@inilabs.com
 *
 * 	This example program shows how to connect a DVS128 or DAVISFX2/FX3 camera
 * 	to the DYNAP-SE Board using a software mapping
 *
 *  Compile & run:
 *
 *  rm -rf CMakeFiles CMakeCache.txt
 *  cp main_dynapse_dvs_software.c main.c
 *  cmake -DCMAKE_BUILD_TYPE=Debug -DDYNAPSEFX2=1 -DDVS128=1 -DENABLE_STATISTICS=1 \
 *  -DENABLE_VISUALIZER=1 -DENABLE_MEANRATEFILTER=1 -DENABLE_FILE_OUTPUT=1 -DENABLE_MONITORNEUFILTER=1 \
 *   -DENABLE_INFOFILTER=0 -DENABLE_DVSTODYNAPSE=1 .
 *   make -j 4
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

// DVS Device support.
#ifdef DVS128
#include "modules/ini/dvs128.h"
#endif
#ifdef DAVISFX2
#include "modules/ini/davis_fx2.h"
#endif
#ifdef DAVISFX3
#include "modules/ini/davis_fx3.h"
#endif

// Input/Output support.
#ifdef ENABLE_FILE_INPUT
#include "modules/misc/in/file.h"
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

#ifdef ENABLE_MEANRATEFILTER_DVS
#include <libcaer/events/frame.h>
#include "modules/meanratefilter_dvs/meanratefilter_dvs.h"
#endif

#ifdef ENABLE_MONITORNEUFILTER
#include "modules/monitorneufilter/monitorneufilter.h"
#endif

#ifdef ENABLE_INFOFILTER
#include "modules/infofilter/infofilter.h"
#endif

#ifdef ENABLE_BAFILTER
#include "modules/backgroundactivityfilter/backgroundactivityfilter.h"
#endif

#ifdef ENABLE_DVSTODYNAPSE
#include "modules/dvstodynapse/dvstodynapse.h"
#endif

#ifdef ENABLE_BAFILTER
#include "modules/backgroundactivityfilter/backgroundactivityfilter.h"
#endif

#ifdef ENABLE_MEDIANTRACKER
#include "modules/mediantracker/mediantracker.h"
#endif

#ifdef ENABLE_GESTURELEARNINGFILTER
#include <libcaer/events/frame.h>
#endif

#ifdef ENABLE_ROTATEFILTER
#include "modules/rotatefilter/rotatefilter.h"
#endif

#ifdef ENABLE_IMAGEGENERATOR
#include "modules/imagegenerator/imagegenerator.h"
#define CLASSIFYSIZE 64
#define DISPLAYIMGSIZE 64
#endif

#ifdef ENABLE_CAFFEINTERFACE
#define CAFFEVISUALIZERSIZE 1024
#include "modules/caffeinterface/wrapper.h"
#endif

#if defined(ENABLE_CAFFEINTERFACE) && defined(ENABLE_TRAINFROMCAFFE)
#include "modules/trainfromcaffe/trainfromcaffe.h"
#endif

static bool mainloop_1(void);

#ifdef ENABLE_GESTURELEARNINGFILTER
	// create frame for displaying weight and synapse
	caerFrameEventPacket weightplotG = NULL;
	caerFrameEventPacket synapseplotG = NULL;
#endif



static bool mainloop_1(void) {

	// An eventPacketContainer bundles event packets of different types together,
	// to maintain time-coherence between the different events.
	caerEventPacketContainer container = NULL;
	caerEventPacketContainer container_cam = NULL;
	caerSpikeEventPacket spike = NULL;
	caerSpecialEventPacket special = NULL;

	// Input modules grab data from outside sources (like devices, files, ...)
	// and put events into an event packet.

#ifdef DYNAPSEFX2 //should be 1 foe experiment
	container = caerInputDYNAPSEFX2(1);

	// We search for them by type here, because input modules may not have all or any of them.
	spike = (caerSpikeEventPacket) caerEventPacketContainerFindEventPacketByType(container, SPIKE_EVENT);
	special = (caerSpecialEventPacket) caerEventPacketContainerFindEventPacketByType(container, SPECIAL_EVENT);
#endif

#ifdef ENABLE_FILE_INPUT
	caerSpecialEventPacket special_cam = NULL;
	caerPolarityEventPacket polarity_cam = NULL;

	container_cam = caerInputFile(10);
	// We search for them by type here, because input modules may not have all or any of them.
	polarity_cam = (caerSpikeEventPacket) caerEventPacketContainerFindEventPacketByType(container_cam, POLARITY_EVENT);
	special_cam = (caerSpecialEventPacket) caerEventPacketContainerFindEventPacketByType(container_cam, SPECIAL_EVENT);
#endif

#ifdef DVS128
	container_cam = caerInputDVS128(10);

	caerSpecialEventPacket special_cam = NULL;
	caerPolarityEventPacket polarity_cam = NULL;

	// Typed EventPackets contain events of a certain type.
	special_cam = (caerSpecialEventPacket) caerEventPacketContainerGetEventPacket(container_cam, SPECIAL_EVENT);
	polarity_cam = (caerPolarityEventPacket) caerEventPacketContainerGetEventPacket(container_cam, POLARITY_EVENT);
#endif

#ifdef DAVISFX2
	container_cam = caerInputDAVISFX2(10);
#endif
#ifdef DAVISFX3
	container_cam = caerInputDAVISFX3(10);
#endif
#if defined(DAVISFX2) || defined(DAVISFX3)
	caerSpecialEventPacket special_cam = NULL;
	caerPolarityEventPacket polarity_cam = NULL;

	// Typed EventPackets contain events of a certain type.
	special_cam = (caerSpecialEventPacket) caerEventPacketContainerGetEventPacket(container_cam, SPECIAL_EVENT);
	polarity_cam = (caerPolarityEventPacket) caerEventPacketContainerGetEventPacket(container_cam, POLARITY_EVENT);

	// Frame and IMU events exist only with DAVIS cameras.
	caerFrameEventPacket frame_cam = NULL;
	caerIMU6EventPacket imu_cam = NULL;

	frame_cam = (caerFrameEventPacket) caerEventPacketContainerGetEventPacket(container_cam, FRAME_EVENT);
	imu_cam = (caerIMU6EventPacket) caerEventPacketContainerGetEventPacket(container_cam, IMU6_EVENT);
#endif

	// Filters process event packets: for example to suppress certain events,
	// like with the Background Activity Filter, which suppresses events that
	// look to be uncorrelated with real scene changes (noise reduction).
#ifdef ENABLE_BAFILTER
	caerBackgroundActivityFilter(12, polarity_cam);
#endif

#ifdef ENABLE_ROTATEFILTER
	caerRotateFilter(25, polarity_cam);
#endif

	// Filter that track one object by using the median position information
#ifdef ENABLE_MEDIANTRACKER
	caerFrameEventPacket medianFrame = NULL;
	caerMainloopFreeAfterLoop(&free, medianFrame);	// free memory after mainloop
	caerPoint4DEventPacket medianData  = caerMediantrackerFilter(13, polarity_cam, &medianFrame);
#endif

	// Filter that show the mean rate of events
#ifdef ENABLE_MEANRATEFILTER_DVS
	caerFrameEventPacket freqplot = NULL;
	caerMeanRateFilterDVS(15, polarity_cam, &freqplot);
#ifdef ENABLE_FILE_INPUT
	caerMeanRateFilterDVS(15, polarity_cam, &freqplot);
#endif
#endif

	// Fitler that maps polarity dvs events as spiking inputs of the dynapse processor
	// It is a software mapper
#ifdef ENABLE_DVSTODYNAPSE
#ifdef ENABLE_MEDIANTRACKER
	caerDvsToDynapse(5, spike, polarity_cam, medianData);
#else
	caerDvsToDynapse(5, spike, polarity_cam, NULL);
#endif
#endif

	// Filters can also extract information from event packets: for example
	// to show statistics about the current event-rate.
#ifdef ENABLE_STATISTICS
	caerStatistics(3, (caerEventPacketHeader) polarity_cam, 1000);
#endif

#ifdef ENABLE_IMAGEGENERATOR
	// it creates images by accumulating spikes
	int * classifyhist = calloc((int)CLASSIFYSIZE * CLASSIFYSIZE, sizeof(int));
	if (classifyhist == NULL) {
			return (false); // Failure.
	}else{
		caerMainloopFreeAfterLoop(&free, classifyhist);
	}
	bool *haveimage;
	haveimage = (bool*)malloc(1);
	if (haveimage == NULL) {
		return (false); // Failure.
	}else{
		caerMainloopFreeAfterLoop(&free, haveimage);
	}
	unsigned char ** frame_img_ptr = calloc(sizeof(unsigned char *), 1);
	// generate image and place it in classifyhist
	caerImageGenerator(20, polarity_cam, CLASSIFYSIZE, classifyhist, haveimage);
#ifdef ENABLE_VISUALIZER
   //put image into a frame packet containing a single frame
   caerFrameEventPacket imagegeneratorFrame = NULL;
   if(haveimage[0]){
	  caerImageGeneratorMakeFrame(20, classifyhist, &imagegeneratorFrame, CLASSIFYSIZE);
   }
#endif
#endif

	// this modules requires image generator
#if defined(ENABLE_CAFFEINTERFACE)
	// this wrapper let you interact with caffe framework
	// we now classify the latest image that has been generated by the imagegenerator filter
	// it also can create an image with activations values for different layers
	caerFrameEventPacket networkActivity = NULL; /* visualization of network activity */
	char * classification_results = (char *)calloc(1024, sizeof(char));/* classification_results: */
	if (classification_results == NULL) {
		caerLog(CAER_LOG_ERROR, __func__, "Failed to allocate classification_results.");
		return (false);
	}else{
		caerMainloopFreeAfterLoop(&free, classification_results);
	}
	int * classification_results_id;
	classification_results_id = (int*)malloc(1);	/* classification_results: */
	classification_results_id[0] = -1; // init
	if (classification_results_id == NULL) {
		caerLog(CAER_LOG_ERROR, __func__, "Failed to allocate classification_results_id.");
		return (false);
	}else{
		caerMainloopFreeAfterLoop(&free, classification_results_id);
	}
#ifdef ENABLE_IMAGEGENERATOR
	if(haveimage[0]) {
		caerCaffeWrapper(21, classifyhist, CLASSIFYSIZE, classification_results, classification_results_id,
			&networkActivity, CAFFEVISUALIZERSIZE);
	}
#endif
#endif

	// add classification results to the image generator frame
#ifdef ENABLE_IMAGEGENERATOR
#if defined(ENABLE_CAFFEINTERFACE)
	if(haveimage[0]){
		caerImageGeneratorAddText(23, classifyhist, &imagegeneratorFrame, CLASSIFYSIZE, classification_results);
	}
#endif
#endif

#ifdef ENABLE_MEANRATEFILTER
	// create frame for displaying frequencies
	caerFrameEventPacket freqplot = NULL;
	caerMeanRateFilter(4, spike, &freqplot);
#endif

#ifdef	ENABLE_TRAINFROMCAFFE
	caerFrameEventPacket group_a = NULL;
	caerFrameEventPacket group_b = NULL;
	caerFrameEventPacket group_c = NULL;
	caerFrameEventPacket group_d = NULL;
	if(classification_results_id[0] > -1){
		caerTrainingFromCaffeFilter(24, spike, classification_results_id[0]);
		caerTrainFromMakeFrame(24,&group_a, &group_b, &group_c, &group_d, 64);
	}
#endif

#ifdef ENABLE_GESTURELEARNINGFILTER
	caerGestureLearningFilter(11, 10, spike, &weightplotG, &synapseplotG);
#endif

#ifdef ENABLE_MONITORNEUFILTER
	caerMonitorNeuFilter(6, 1);
#endif

	// A simple visualizer exists to show what the output looks like.
#ifdef ENABLE_VISUALIZER
	caerVisualizer(64, "Spike", &caerVisualizerRendererSpikeEvents, caerVisualizerEventHandlerSpikeEvents, (caerEventPacketHeader) spike);
#ifdef ENABLE_MEANRATEFILTER
	if(freqplot != NULL){
		caerVisualizer(65, "Frequency", &caerVisualizerRendererFrameEvents, NULL, (caerEventPacketHeader) freqplot);
	}
#endif
	caerVisualizer(68, "UserSize", &caerVisualizerRendererSpikeEventsRaster, NULL, (caerEventPacketHeader) spike);
#if defined(DAVISFX2) || defined(DAVISFX3) || defined(DVS128) || defined(ENABLE_FILE_INPUT)
	caerVisualizer(66, "Polarity", &caerVisualizerRendererPolarityEvents, NULL, (caerEventPacketHeader) polarity_cam);
#endif
#ifdef ENABLE_GESTURELEARNINGFILTER
	caerVisualizer(69, "Weight", &caerVisualizerRendererFrameEvents, NULL, (caerEventPacketHeader) weightplotG);
	caerVisualizer(70, "Synapse", &caerVisualizerRendererFrameEvents, NULL, (caerEventPacketHeader) synapseplotG);
#endif
#ifdef ENABLE_MEANRATEFILTER_DVS
	if(freqplot != NULL){
		caerVisualizer(73, "MeanRateFrequency", &caerVisualizerRendererFrameEvents, NULL, (caerEventPacketHeader) freqplot);
	}
#endif
#ifdef	ENABLE_TRAINFROMCAFFE
	if(group_a != NULL){
		caerVisualizer(74, "Paper", &caerVisualizerRendererFrameEvents, NULL, (caerEventPacketHeader) group_a);
	}
	if(group_b != NULL){
		caerVisualizer(75, "Scissors", &caerVisualizerRendererFrameEvents, NULL, (caerEventPacketHeader) group_b);
	}
	if(group_c != NULL){
		caerVisualizer(76, "Rock", &caerVisualizerRendererFrameEvents, NULL, (caerEventPacketHeader) group_c);
	}
	if(group_d != NULL){
		caerVisualizer(77, "Background", &caerVisualizerRendererFrameEvents, NULL, (caerEventPacketHeader) group_d);
	}
#endif
#endif

#if defined(ENABLE_VISUALIZER) && defined (ENABLE_IMAGEGENERATOR)
	caerVisualizer(71, "ImageStreamerHist", &caerVisualizerRendererFrameEvents, NULL, (caerEventPacketHeader) imagegeneratorFrame);
#ifdef ENABLE_CAFFEINTERFACE
	//show the activations of the deep network
	caerVisualizer(72, "DeepNetworkActivations", &caerVisualizerRendererFrameEvents, NULL, (caerEventPacketHeader) networkActivity);
#endif
#endif


#if defined(ENABLE_MEDIANTRACKER) && defined (ENABLE_VISUALIZER)
	caerVisualizer(68, "ImageMedian", &caerVisualizerRendererFrameEvents, NULL, (caerEventPacketHeader) medianFrame);
#endif

	// Filter that adds buttons and timer for recording data
	// or playing a recording file. It implements fast forward and
	// slow motion buttons, as well as play again from start.
	// It is a very basic  interface (320x240)
#ifdef ENABLE_INFOFILTER
#if defined(ENABLE_FILE_INPUT) && defined(ENABLE_FILE_OUTPUT)
	caerInfoFilter(78, container, 10, 7);
#elif defined(ENABLE_FILE_INPUT) && !defined(ENABLE_FILE_OUTPUT)
	caerInfoFilter(78, container, 10, NULL);
#elif !defined(ENABLE_FILE_INPUT) && defined(ENABLE_FILE_OUTPUT)
	caerInfoFilter(78, container, NULL, 7);
#endif
#endif

#ifdef ENABLE_FILE_OUTPUT
	// Enable output to file (AEDAT 3.X format).
	caerOutputFile(7, 2, polarity_cam, special_cam);
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
