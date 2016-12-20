#include "stereomatching.hpp"
#include <fstream>
#include <iostream>

StereoMatching::StereoMatching(StereoMatchingSettings settings) {
	this->settings = settings;

	updateSettings(this->settings);
}

void StereoMatching::updateSettings(StereoMatchingSettings settings) {
	this->settings = settings;

}


bool StereoMatching::loadCalibrationFile(StereoMatchingSettings settings) {


	return (true);
}
