/*
 * main.c
 *
 *  Created on: Dec, 2016
 *      Author: federico.corradi@inilabs.com
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
#ifdef ENABLE_STEREOCALIBRATION
#include "modules/stereocalibration/stereocalibration.h"
#endif

static bool mainloop_twocameras(void);

static bool mainloop_twocameras(void) {
	// An eventPacketContainer bundles event packets of different types together,
	// to maintain time-coherence between the different events.

#ifdef ENABLE_VISUALIZER
	caerVisualizerEventHandler visualizerEventHandler = NULL;
#endif

	caerEventPacketContainer container_cam0 = NULL;
	caerSpecialEventPacket special_cam0 = NULL;
	caerPolarityEventPacket polarity_cam0 = NULL;
	caerFrameEventPacket frame_cam0 = NULL;
	caerIMU6EventPacket imu_cam0 = NULL;

	caerEventPacketContainer container_cam1 = NULL;
	caerSpecialEventPacket special_cam1 = NULL;
	caerPolarityEventPacket polarity_cam1 = NULL;
	caerFrameEventPacket frame_cam1 = NULL;
	caerIMU6EventPacket imu_cam1 = NULL;

//common device supports
#ifdef DAVISFX2
	// Input modules grab data from outside sources (like devices, files, ...)
	// and put events into an event packet.
	container_cam0 = caerInputDAVISFX2(1);

	// Typed EventPackets contain events of a certain type.
	special_cam0 = (caerSpecialEventPacket) caerEventPacketContainerGetEventPacket(container_cam0, SPECIAL_EVENT);
	polarity_cam0 = (caerPolarityEventPacket) caerEventPacketContainerGetEventPacket(container_cam0, POLARITY_EVENT);
	frame_cam0 = (caerFrameEventPacket) caerEventPacketContainerGetEventPacket(container_cam0, FRAME_EVENT);
	imu_cam0 = (caerIMU6EventPacket) caerEventPacketContainerGetEventPacket(container_cam0, IMU6_EVENT);

	// Input modules grab data from outside sources (like devices, files, ...)
	// and put events into an event packet.
	container_cam1 = caerInputDAVISFX2(2);

	// Typed EventPackets contain events of a certain type.
	special_cam1 = (caerSpecialEventPacket) caerEventPacketContainerGetEventPacket(container_cam1, SPECIAL_EVENT);
	polarity_cam1 = (caerPolarityEventPacket) caerEventPacketContainerGetEventPacket(container_cam1, POLARITY_EVENT);
	frame_cam1 = (caerFrameEventPacket) caerEventPacketContainerGetEventPacket(container_cam1, FRAME_EVENT);
	imu_cam1 = (caerIMU6EventPacket) caerEventPacketContainerGetEventPacket(container_cam1, IMU6_EVENT);
#endif


#ifdef DAVISFX3
	// Input modules grab data from outside sources (like devices, files, ...)
	// and put events into an event packet.
	container_cam0 = caerInputDAVISFX3(1);

	// Typed EventPackets contain events of a certain type.
	special_cam0 = (caerSpecialEventPacket) caerEventPacketContainerGetEventPacket(container_cam0, SPECIAL_EVENT);
	polarity_cam0 = (caerPolarityEventPacket) caerEventPacketContainerGetEventPacket(container_cam0, POLARITY_EVENT);
	frame_cam0 = (caerFrameEventPacket) caerEventPacketContainerGetEventPacket(container_cam0, FRAME_EVENT);
	imu_cam0 = (caerIMU6EventPacket) caerEventPacketContainerGetEventPacket(container_cam0, IMU6_EVENT);

	// Input modules grab data from outside sources (like devices, files, ...)
	// and put events into an event packet.
	container_cam1 = caerInputDAVISFX3(2);

	// Typed EventPackets contain events of a certain type.
	special_cam1 = (caerSpecialEventPacket) caerEventPacketContainerGetEventPacket(container_cam1, SPECIAL_EVENT);
	polarity_cam1 = (caerPolarityEventPacket) caerEventPacketContainerGetEventPacket(container_cam1, POLARITY_EVENT);
	frame_cam1 = (caerFrameEventPacket) caerEventPacketContainerGetEventPacket(container_cam1, FRAME_EVENT);
	imu_cam1 = (caerIMU6EventPacket) caerEventPacketContainerGetEventPacket(container_cam1, IMU6_EVENT);
#endif

#ifdef ENABLE_FILE_INPUT
	container_cam0 = caerInputFile(10);

	// Typed EventPackets contain events of a certain type.
	// We search for them by type here, because input modules may not have all or any of them.
	special_cam0 = (caerSpecialEventPacket) caerEventPacketContainerFindEventPacketByType(container_cam0, SPECIAL_EVENT);
	polarity_cam0 = (caerPolarityEventPacket) caerEventPacketContainerFindEventPacketByType(container_cam0, POLARITY_EVENT);
	frame_cam0 = (caerFrameEventPacket) caerEventPacketContainerFindEventPacketByType(container_cam0, FRAME_EVENT);
	imu_cam0 = (caerIMU6EventPacket) caerEventPacketContainerFindEventPacketByType(container_cam0, IMU6_EVENT);

	container_cam1 = caerInputFile(100);

	// Typed EventPackets contain events of a certain type.
	// We search for them by type here, because input modules may not have all or any of them.
	special_cam1 = (caerSpecialEventPacket) caerEventPacketContainerFindEventPacketByType(container_cam1, SPECIAL_EVENT);
	polarity_cam1 = (caerPolarityEventPacket) caerEventPacketContainerFindEventPacketByType(container_cam1, POLARITY_EVENT);
	frame_cam1 = (caerFrameEventPacket) caerEventPacketContainerFindEventPacketByType(container_cam1, FRAME_EVENT);
	imu_cam1 = (caerIMU6EventPacket) caerEventPacketContainerFindEventPacketByType(container_cam1, IMU6_EVENT);
#endif

	// Filters process event packets: for example to suppress certain events,
	// like with the Background Activity Filter, which suppresses events that
	// look to be uncorrelated with real scene changes (noise reduction).
#ifdef ENABLE_BAFILTER
	caerBackgroundActivityFilter(3, polarity_cam0);
	caerBackgroundActivityFilter(33, polarity_cam1);
#endif

	// Enable APS frame image enhancements.
#ifdef ENABLE_FRAMEENHANCER
	frame_0 = caerFrameEnhancer(4, frame_cam0);
	frame_1 = caerFrameEnhancer(44, frame_cam1);
#endif

	// Enable synch of two data stream.
#ifdef ENABLE_DATASYNCH
	caerDataSynchFilter(1101, polarity_cam0, polarity_cam1, frame_cam0, frame_cam1,
			imu_cam0, imu_cam1, special_cam0, special_cam1);
#endif

	// Enable image and event undistortion, based on OpenCV stereo camera calibration.
#ifdef ENABLE_CAMERACALIBRATION
	caerCameraCalibration(5, polarity_cam0, frame_cam0);
	caerCameraCalibration(55, polarity_cam1, frame_cam1);
#endif

#ifdef ENABLE_STEREOCALIBRATION
	caerStereoCalibration(7, frame_cam0, frame_cam1);
#endif

	// A simple visualizer exists to show what the output looks like.
#ifdef ENABLE_VISUALIZER
	//caerVisualizerMulti(68, "PolarityAndFrame", &caerVisualizerMultiRendererPolarityAndFrameEvents, NULL, container_cam0);
	//caerVisualizerMulti(6688, "PolarityAndFrame", &caerVisualizerMultiRendererPolarityAndFrameEvents, NULL, container_cam1);
	caerVisualizer(68, "Frame", &caerVisualizerRendererFrameEvents, visualizerEventHandler, (caerEventPacketHeader) frame_cam0);
	caerVisualizer(6688, "Frame", &caerVisualizerRendererFrameEvents, visualizerEventHandler, (caerEventPacketHeader) frame_cam1);
	caerVisualizer(60, "Polarity", &caerVisualizerRendererPolarityEvents, visualizerEventHandler, (caerEventPacketHeader) polarity_cam0);
	caerVisualizer(6600, "Polarity", &caerVisualizerRendererPolarityEvents, visualizerEventHandler, (caerEventPacketHeader) polarity_cam1);
#if defined(ENABLE_VISUALIZER) && defined(ENABLE_IMAGEGENERATOR)
	caerVisualizer(64, "ImageStreamerFrameCam0", &caerVisualizerRendererFrameEvents, NULL, (caerEventPacketHeader) frame_cam0);
	caerVisualizer(6644, "ImageStreamerFrameCam0", &caerVisualizerRendererFrameEvents, NULL, (caerEventPacketHeader) frame_cam1);
#endif
#endif


#ifdef ENABLE_FILE_OUTPUT
	// Enable output to file (AER3.1 format).
	caerOutputFile(9, 4, polarity_cam0, frame_cam0, imu_cam0, special_cam0);
	caerOutputFile(99, 4, polarity_cam1, frame_cam1, imu_cam1, special_cam1);
#endif

	return (true); // If false is returned, processing of this loop stops.
}

int main(int argc, char **argv) {
	// Initialize config storage from file, support command-line overrides.
	// If no init from file needed, pass NULL.
	caerConfigInit("caer-config.xml", argc, argv);

	// Initialize logging sub-system.
	caerLogInit();

	// Initialize visualizer framework (load fonts etc.).
#ifdef ENABLE_VISUALIZER
	caerVisualizerSystemInit();
#endif

	// Daemonize the application (run in background).
	//caerDaemonize();

	// Start the configuration server thread for run-time config changes.
	caerConfigServerStart();

	// Finally run the main event processing loops.
	struct caer_mainloop_definition mainLoops[1] = { { 1, &mainloop_twocameras } };
	caerMainloopRun(&mainLoops, 1);

	// After shutting down the mainloops, also shutdown the config server
	// thread if needed.
	caerConfigServerStop();

	return (EXIT_SUCCESS);
}
