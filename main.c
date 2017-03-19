/*
 * main.c
 *
 *  Created on: Oct 6, 2013
 *      Author: chtekk
 *
 *  Compile & run:
 *  $ cd caer/
 *  $ rm -rf CMakeFiles CMakeCache.txt
 *  $ CC=clang-3.7 cmake [-DJAER_COMPAT_FORMAT=1 -DENABLE_VISUALIZER=1 -DENABLE_NET_STREAM=1] -DDAVISFX2 .
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

// Common filters support.
#ifdef ENABLE_BAFILTER
#include "modules/backgroundactivityfilter/backgroundactivityfilter.h"
#endif
#ifdef ENABLE_CAMERACALIBRATION
#include "modules/cameracalibration/cameracalibration.h"
#endif
#ifdef ENABLE_FRAMEENHANCER
#include "modules/frameenhancer/frameenhancer.h"
#endif
#ifdef ENABLE_STATISTICS
#include "modules/statistics/statistics.h"
#endif
#ifdef ENABLE_VISUALIZER
#include "modules/visualizer/visualizer.h"
#endif
#ifdef ENABLE_INFOFILTER
#include "modules/infofilter/infofilter.h"
#endif
#ifdef ENABLE_RECTANGULARTRACKER
#include "modules/rectangulartracker/rectangulartracker.h"
#endif
#ifdef ENABLE_MEDIANTRACKER
#include "modules/mediantracker/mediantracker.h"
#endif
#ifdef ENABLE_MEANRATEFILTER_DVS
#include <libcaer/events/frame.h>
#include "modules/meanratefilter_dvs/meanratefilter_dvs.h"
#endif
#ifdef ENABLE_ROTATEFILTER
#include "modules/rotatefilter/rotatefilter.h"
#endif
#ifdef ENABLE_ACTIVITYINDICATOR
#include "modules/activityindicator/activityindicator.h"
#endif
#ifdef ENABLE_OPENCVDISPLAY
#include "modules/opencvdisplay/opencvdisplay_module.h"
#endif
#ifdef ENABLE_OPENCVOPTICFLOW
#include "modules/opencvopticflow/opticflow.h"
#endif
#ifdef ENABLE_DENOISINGAUTOENCODER
#include "modules/denoisingautoencoder/denoisingautoencoder_module.h"
#endif
#ifdef ENABLE_PIXELMATRIX
#include "modules/pixelmatrix/pixelmatrix.h"
#endif

#ifdef ENABLE_IMAGEGENERATOR
#include "modules/imagegenerator/imagegenerator.h"
#define CLASSIFYSIZE 128
#define DISPLAYIMGSIZE 128
#endif
#ifdef ENABLE_CAFFEINTERFACE
#define CAFFEVISUALIZERSIZE 1024 
#include "modules/caffeinterface/wrapper.h"
#endif
#ifdef ENABLE_NULLHOPINTERFACE
#include "modules/nullhopinterface/nullhopinterface.h"
#endif

static bool mainloop_1(void);

static bool mainloop_1(void) {
	// An eventPacketContainer bundles event packets of different types together,
	// to maintain time-coherence between the different events.
	caerEventPacketContainer container = NULL;
	caerSpecialEventPacket special = NULL;
	caerPolarityEventPacket polarity = NULL;

#if (defined(ENABLE_FILE_INPUT) || defined(ENABLE_NETWORK_INPUT)) && (defined(DAVISFX2) || defined(DAVISFX3))
#error "Only one input mode can be enabled: please choose DAVISFX2/3 or ENABLE_FILE_INPUT"
#endif

#ifdef ENABLE_VISUALIZER
	caerVisualizerEventHandler visualizerEventHandler = NULL;
#endif

	// Input modules grab data from outside sources (like devices, files, ...)
	// and put events into an event packet.
#ifdef DVS128
	container = caerInputDVS128(1);

	// Typed EventPackets contain events of a certain type.
	special = (caerSpecialEventPacket) caerEventPacketContainerGetEventPacket(container, SPECIAL_EVENT);
	polarity = (caerPolarityEventPacket) caerEventPacketContainerGetEventPacket(container, POLARITY_EVENT);
#endif

#ifdef DAVISFX2
	container = caerInputDAVISFX2(1);
#endif
#ifdef DAVISFX3
	container = caerInputDAVISFX3(1);
#endif
#if defined(DAVISFX2) || defined(DAVISFX3)
	// Typed EventPackets contain events of a certain type.
	special = (caerSpecialEventPacket) caerEventPacketContainerGetEventPacket(container, SPECIAL_EVENT);
	polarity = (caerPolarityEventPacket) caerEventPacketContainerGetEventPacket(container, POLARITY_EVENT);

	// Frame and IMU events exist only with DAVIS cameras.
	caerFrameEventPacket frame = NULL;
	caerIMU6EventPacket imu = NULL;

	frame = (caerFrameEventPacket) caerEventPacketContainerGetEventPacket(container, FRAME_EVENT);
	imu = (caerIMU6EventPacket) caerEventPacketContainerGetEventPacket(container, IMU6_EVENT);
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

	// Typed EventPackets contain events of a certain type.
	// We search for them by type here, because input modules may not have all or any of them.
	special = (caerSpecialEventPacket) caerEventPacketContainerFindEventPacketByType(container, SPECIAL_EVENT);
	polarity = (caerPolarityEventPacket) caerEventPacketContainerFindEventPacketByType(container, POLARITY_EVENT);

	caerFrameEventPacket frame = NULL;
	caerIMU6EventPacket imu = NULL;

	frame = (caerFrameEventPacket) caerEventPacketContainerFindEventPacketByType(container, FRAME_EVENT);
	imu = (caerIMU6EventPacket) caerEventPacketContainerFindEventPacketByType(container, IMU6_EVENT);
#endif

	// Filter that adds buttons and timer for recording data
	// or playing a recording file. It implements fast forward and
	// slow motion buttons, as well as play again from start.
	// It is a very basic camera interface (320x240)
#ifdef ENABLE_INFOFILTER
#if defined(ENABLE_FILE_INPUT) && defined(ENABLE_FILE_OUTPUT)
	caerInfoFilter(78, container, 10, 7);
#elif defined(ENABLE_FILE_INPUT) && !defined(ENABLE_FILE_OUTPUT)
	caerInfoFilter(78, container, 10, NULL);
#elif !defined(ENABLE_FILE_INPUT) && defined(ENABLE_FILE_OUTPUT)
	caerInfoFilter(78, container, NULL, 7);
#endif
#endif

	// Filters process event packets: for example to suppress certain events,
	// like with the Background Activity Filter, which suppresses events that
	// look to be uncorrelated with real scene changes (noise reduction).
#ifdef ENABLE_BAFILTER
	caerBackgroundActivityFilter(2, polarity);
#endif

	// Filters can also extract information from event packets: for example
	// to show statistics about the current event-rate.
#ifdef ENABLE_STATISTICS
	caerStatistics(3, (caerEventPacketHeader) polarity, 1000);
#endif

	// Filter that rotate events in different ways
#ifdef ENABLE_ROTATEFILTER
	caerRotateFilter(16, polarity);
#endif

	// Filters that track multiple objects by using rectangular clusters
#ifdef ENABLE_RECTANGULARTRACKER
	caerFrameEventPacket rectangularFrame = NULL;
	caerRectangulartrackerFilter(12, polarity, &rectangularFrame);
#endif

	// Filter that track one object by using the median position information
#ifdef ENABLE_MEDIANTRACKER
	caerFrameEventPacket medianFrame = NULL;
	caerMediantrackerFilter(13, polarity, &medianFrame);
#endif

	// Show how crowed the areas is (Mensa project)
#ifdef ENABLE_ACTIVITYINDICATOR
	caerFrameEventPacket activityFrame = NULL;
	AResults rr = caerActivityIndicator(17, polarity, &activityFrame);
#endif

#if defined(ENABLE_OPENCVDISPLAY) && defined(ENABLE_ACTIVITYINDICATOR)
	caerFrameEventPacket frameRes = NULL;
	if(rr->activityValue != -1){
		frameRes = caerOpenCVDisplay(18, rr);
	}
#endif

	// Filter that show the mean rate of events
#ifdef ENABLE_MEANRATEFILTER_DVS
	caerFrameEventPacket freqplot = NULL;
	caerMeanRateFilterDVS(15, polarity, &freqplot);
#endif

	// Enable APS frame image enhancements.
#ifdef ENABLE_FRAMEENHANCER
	frame = caerFrameEnhancer(4, frame);
#endif

	// Enable image and event undistortion by using OpenCV camera calibration.
#ifdef ENABLE_CAMERACALIBRATION
	caerCameraCalibration(5, polarity, frame);
#endif

	//Enable camera pose estimation
#ifdef ENABLE_POSEESTIMATION
	caerPoseCalibration(6, polarity, frame);
#endif

	// A simple visualizer exists to show what the output looks like.
#ifdef ENABLE_VISUALIZER
	caerVisualizer(60, "Polarity", &caerVisualizerRendererPolarityEvents, visualizerEventHandler, (caerEventPacketHeader) polarity);
#if defined(DAVISFX2) || defined(DAVISFX3)
	caerVisualizer(61, "Frame", &caerVisualizerRendererFrameEvents, visualizerEventHandler, (caerEventPacketHeader) frame);
	caerVisualizer(62, "IMU6", &caerVisualizerRendererIMU6Events, visualizerEventHandler, (caerEventPacketHeader) imu);
#endif
#ifdef ENABLE_MEANRATEFILTER_DVS
	if(freqplot != NULL){
		caerVisualizer(70, "MeanRateFrequency", &caerVisualizerRendererFrameEvents, NULL, (caerEventPacketHeader) freqplot);
	}
#endif
#if defined(ENABLE_OPENCVDISPLAY) && defined(ENABLE_ACTIVITYINDICATOR)
	caerVisualizer(71, "FrameRes", &caerVisualizerRendererFrameEvents, visualizerEventHandler, (caerEventPacketHeader) frameRes);
#endif
	//caerVisualizerMulti(68, "PolarityAndFrame", &caerVisualizerMultiRendererPolarityAndFrameEvents, visualizerEventHandler, container);
#endif

#ifdef ENABLE_FILE_OUTPUT
	// Enable output to file (AEDAT 3.X format).
#ifdef DVS128
	caerOutputFile(7, 2, polarity, special);
#else
	caerOutputFile(7, 4, polarity, frame, imu, special);
#endif
#endif

#ifdef ENABLE_NETWORK_OUTPUT
	// Send polarity packets out via TCP. This is the server mode!
	// External clients connect to cAER, and we send them the data.
	// WARNING: slow clients can dramatically slow this and the whole
	// processing pipeline down!
#ifdef DVS128
	caerOutputNetTCPServer(8, 2, polarity, special);
#else
	caerOutputNetTCPServer(8, 4, polarity, frame, imu, special);
#endif

	// And also send them via UDP. This is fast, as it doesn't care what is on the other side.
#ifdef DVS128
	caerOutputNetUDP(9, 2, polarity, special);
#else
	caerOutputNetUDP(9, 4, polarity, frame, imu, special);
#endif
#endif

#ifdef ENABLE_IMAGEGENERATOR
	// it creates images by accumulating spikes
	int * classifyhist = calloc((int)CLASSIFYSIZE * CLASSIFYSIZE, sizeof(int));
	if (classifyhist == NULL) {
			return (false); // Failure.
	}
	bool *haveimage;
	haveimage = (bool*)malloc(1);
	unsigned char ** frame_img_ptr = calloc(sizeof(unsigned char *), 1);
	// generate image and place it in classifyhist
	caerImageGenerator(20, polarity, CLASSIFYSIZE, classifyhist, haveimage);
#ifdef ENABLE_VISUALIZER
   //put image into a frame packet containing a single frame
   caerFrameEventPacket imagegeneratorFrame = NULL;
   if(haveimage[0]){
	  caerImageGeneratorMakeFrame(20, classifyhist, &imagegeneratorFrame, CLASSIFYSIZE);
   }
#endif
#endif

	// this modules requires image generator
#if defined(ENABLE_CAFFEINTERFACE) || defined(ENABLE_NULLHOPINTERFACE)
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
        classification_results_id = (int*)malloc(1);    /* classification_results: */
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

#ifdef ENABLE_PIXELMATRIX
#if defined(ENABLE_CAFFEINTERFACE)
       if(haveimage[0]){
    	   caerPixelMatrixFilter(24, polarity, classification_results, classification_results_id);
       }
#endif
#endif

#ifdef ENABLE_NULLHOPINTERFACE
	caerNullHopWrapper(22, classifyhist, haveimage, classification_results);
#endif


#if defined(ENABLE_OPENCVOPTICFLOW) && defined(ENABLE_IMAGEGENERATOR) && defined(ENABLE_VISUALIZER)
	caerFrameEventPacket frameFlow = NULL;
	//caerFrameEventPacket frameFlow = caerOpticFlow(19, imagegeneratorFrame);
#endif


#if defined(ENABLE_DENOISINGAUTOENCODER) && defined(ENABLE_IMAGEGENERATOR) && defined(ENABLE_VISUALIZER)
	caerFrameEventPacket frameAutoEncoderFeatures = NULL;
	frameAutoEncoderFeatures = caerDenAutoEncoder(24, imagegeneratorFrame);
#endif


	// add classification results to the image generator frame
#ifdef ENABLE_IMAGEGENERATOR
#if defined(ENABLE_CAFFEINTERFACE)
	if(haveimage[0]){
		caerImageGeneratorAddText(23, classifyhist, &imagegeneratorFrame, CLASSIFYSIZE, classification_results);
	}
#endif
#endif

#if defined(ENABLE_VISUALIZER) && defined (ENABLE_IMAGEGENERATOR)
	caerVisualizer(65, "ImageStreamerHist", &caerVisualizerRendererFrameEvents, NULL, (caerEventPacketHeader) imagegeneratorFrame);
#ifdef ENABLE_CAFFEINTERFACE
	//show the activations of the deep network
	caerVisualizer(66, "DeepNetworkActivations", &caerVisualizerRendererFrameEvents, NULL, (caerEventPacketHeader) networkActivity);
#endif
#endif

#if defined(ENABLE_RECTANGULARTRACKER) && defined (ENABLE_VISUALIZER)
	caerVisualizer(67, "ImageClusters", &caerVisualizerRendererFrameEvents, NULL, (caerEventPacketHeader) rectangularFrame);
#endif

#if defined(ENABLE_MEDIANTRACKER) && defined (ENABLE_VISUALIZER)
	caerVisualizer(68, "ImageMedian", &caerVisualizerRendererFrameEvents, NULL, (caerEventPacketHeader) medianFrame);
#endif

#if defined(ENABLE_ACTIVITYINDICATOR) && defined (ENABLE_VISUALIZER)
	caerVisualizer(69, "ImageActivity", &caerVisualizerRendererFrameEvents, NULL, (caerEventPacketHeader) activityFrame);
#endif

#if defined(ENABLE_OPENCVOPTICFLOW) && defined(ENABLE_IMAGEGENERATOR)
	caerVisualizer(72, "OpticFlow", &caerVisualizerRendererFrameEvents, NULL, (caerEventPacketHeader) frameFlow);
#endif

#if defined(ENABLE_DENOISINGAUTOENCODER) && defined(ENABLE_IMAGEGENERATOR) && defined(ENABLE_VISUALIZER)
	caerVisualizer(73, "DenoiserAutoEncoder", &caerVisualizerRendererFrameEvents, NULL, (caerEventPacketHeader) frameAutoEncoderFeatures);
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
