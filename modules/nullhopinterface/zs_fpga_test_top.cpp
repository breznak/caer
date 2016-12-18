/*
 * zs_fpga_test_top.cpp
 *
 *  Created on: Nov 14, 2016
 *      Author: asa
 */

//#define FPGA_MODE
//#define ENABLE_LOG
//#define VERBOSITY_DEBUG
//#define ENABLE_RESULT_MONITOR

#include "zs_driver.cpp"
#include "npp_log_utilities.cpp"
#include <time.h>
#include <sys/time.h>
#include <ctime>
#include <chrono>
#include <string>
int main() {

	printf("\nStarting fpga testing...\n\n");
//test mapping:
//0 = faceNet
//1 = roshamboNet
	int test = 1;
	int num_frames = 1;

	std::string filename = "";

	std::vector<std::vector<int>> images;
	int num_row;
	int num_column;
	int num_channels;
	int test_num_events;
	int normalization_max = 255;
	switch (test) {
	case (0):
		filename = "faceNet.nhp";
		num_row = 36;
		num_column = 36;
		num_channels = 1;
		test_num_events = 2000;
		break;
	case (1):
		filename = "roshamboNet.nhp";
		num_row = 64;
		num_column = 64;
		num_channels = 1;
		test_num_events = 7000;
		break;
	default:
		printf("ERROR IN PROPERLY SETTING TEST PARAMETERS \n\n");
	}

	zs_driver driver(filename);

	// srand(time(NULL)); //seed
	srand(1);
	int dummy_pix = 0;

	printf("\nStarting data preparation...\n\n");

	// try n runs , 200,400,1000, 100000 etc..
	// initialize an image of all 0s
	for (unsigned int cc = 0; cc < num_frames; cc++) {
		std::vector<int> input_image;
		int zero_prob;
		// new image
		for (unsigned int i = 0; i < num_channels; ++i) {
			for (unsigned int j = 0; j < num_row; ++j) {
				for (unsigned int k = 0; k < num_column; ++k) {

					//zero_prob = (rand() % 10);
					//if(zero_prob>8){
					// input_image.push_back(abs(rand() % 256));
					//}
					//else{
					input_image.push_back(0);
				}

			}
		}

		//create event like image
		int num_events = 0;
		while (num_events < test_num_events) {
			int position = abs(rand() % input_image.size());
			input_image[position]++;
			num_events++;
		}

		//normalize it - it is not the actual normalization implemented in CAER - but we are not looking for generating "meaningful" images, just "computationally realistic"
		int max = std::max_element(input_image.begin(), input_image.end())[0];

		for (int entry_idx = 0; entry_idx < input_image.size(); entry_idx++) {
			input_image[entry_idx] =
					(input_image[entry_idx] * normalization_max) / max;
		}

		images.push_back(input_image);
	}

	printf("Data preparation done, starting run...\n\n");

	std::chrono::high_resolution_clock::time_point t1 =
			std::chrono::high_resolution_clock::now();

	for (unsigned int cc = 0; cc < num_frames; cc++) {

		driver.classify_image(images[cc].data());
	}

	std::chrono::high_resolution_clock::time_point t2 =
			std::chrono::high_resolution_clock::now();
	double duration = std::chrono::duration_cast < std::chrono::milliseconds
			> (t2 - t1).count();
	double duration_avg_ms = duration / (num_frames);

	printf("\nTime for hw classification, average over all frames: %f ms \n",
			duration_avg_ms);

	usleep(100000);
	return (0);
}
