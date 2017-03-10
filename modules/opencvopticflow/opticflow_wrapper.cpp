#include "opticflow.hpp"
#include "opticflow_wrapper.h"

OpticFlow *OpticFlow_init(OpticFlowSettings settings) {
	try {
		return (new OpticFlow(settings));
	}
	catch (const std::exception& ex) {
		caerLog(CAER_LOG_ERROR, "OpticFlow_init()", "Failed with C++ exception: %s", ex.what());
		return (NULL);
	}
}

void OpticFlow_destroy(OpticFlow *opticFlowClass) {
	try {
		delete opticFlowClass;
	}
	catch (const std::exception& ex) {
		caerLog(CAER_LOG_ERROR, "OpticFlow_destroy()", "Failed with C++ exception: %s", ex.what());
	}
}

void OpticFlow_updateSettings(OpticFlow *opticFlowClass) {
	try {
		opticFlowClass->updateSettings();
	}
	catch (const std::exception& ex) {
		caerLog(CAER_LOG_ERROR, "OpticFlow_updateSettings()", "Failed with C++ exception: %s", ex.what());
	}
}

bool OpticFlow_doOpticFlow(OpticFlow *opticFlowClass, caerFrameEvent * frame, caerFrameEvent * frameInput, int sizeX, int sizeY) {
	try {
		return (opticFlowClass->doOpticFlow(frame, frameInput, sizeX, sizeY));
	}
	catch (const std::exception& ex) {
		caerLog(CAER_LOG_ERROR, "OpticFlow_findNewPoints()", "Failed with C++ exception: %s", ex.what());
		return (false);
	}
}

