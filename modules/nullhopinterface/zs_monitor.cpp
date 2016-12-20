#ifndef __zs_monitor__
#define __zs_monitor__

/*
 * zs_monitor.cpp
 *
 *  Created on: Nov 10, 2016
 *      Author: asa
 */
#include "npp_std_func_sw_pkg.cpp"
#include "zs_monitor.h"
#include <tuple>
#include <fstream>
#include <sstream>
#include <string>
#include <numeric>
#include <iterator>
#include <vector>
#include <algorithm>
#include <time.h>
#include <sys/time.h>
#include <ctime>
#include <chrono>
#include <string>

//#define ENABLE_RESULT_MONITOR
#ifdef ENABLE_RESULT_MONITOR
//#define RESULT_MONITOR_CHECK_LAYER_ACTIVATION_DISABLED
zs_monitor::zs_monitor(std::string filename) {

	if (filename.empty() == false) {
		// Read a .net file containing network description and prepares arrays in memory
		read_network_from_file(filename);
	} else {
		cnn_num_layers = 0;
	}

}

bool zs_monitor::read_network_from_file(std::string network_file_name) {
	FILE *l_net_file = fopen(network_file_name.c_str(), "r");

	if (l_net_file == NULL) {
		throw std::invalid_argument(
				"MONITOR: Failed attempt to read network file, impossible to proceed");
		return (false);
	} else {

		log_utilities::debug("Starting monitor network read");
		//Read number of layers
		cnn_num_layers = read_int_from_file(l_net_file);

		cnn_kernels.reserve(cnn_num_layers);

		monitor_activations.resize(cnn_num_layers + 1);

		//hw_activations.reserve(cnn_num_layers+1);

		log_utilities::debug("Monitor internal memory reserved");
		//Create layers to be used
		for (int layer_idx = 0; layer_idx < cnn_num_layers; layer_idx++) {
			zs_monitor_cnn_layer new_layer = zs_monitor_cnn_layer(l_net_file);
			cnn_kernels.push_back(new_layer);
		}
		log_utilities::debug("Monitor network read completed");
		fclose(l_net_file);
		return (true);
	}
}

void zs_monitor::classify_image(std::vector<uint64_t> l_image) {
	int row = 0;
	int column = 0;
	int channel = 0;

	log_utilities::debug("Classifying image on software...");

	monitor_activations[0] = image_1d_to_3d(l_image,
			cnn_kernels[0].num_input_rows, cnn_kernels[0].num_input_columns,
			cnn_kernels[0].num_input_channels);

	log_utilities::debug("Input image converted into 3d array");

	for (int layer_idx = 0; layer_idx < cnn_num_layers; layer_idx++) {
		log_utilities::debug("Computing layer %d", layer_idx);
		monitor_activations[layer_idx + 1] = compute_layer(
				monitor_activations[layer_idx], cnn_kernels[layer_idx]);

	}

	write_activations_to_file(monitor_activations);

	log_utilities::debug("Classification done");
}

std::vector<std::vector<std::vector<int64_t>>>zs_monitor::compute_layer(std::vector<std::vector<std::vector<int64_t>>> layer_input, zs_monitor_cnn_layer layer_parameters) {

	std::vector<std::vector<std::vector<int64_t>>> layer_result = compute_convolution(layer_input,layer_parameters);

	if (layer_parameters.pooling_enabled == 1) {
		layer_result = compute_pooling(layer_result);
	}

	return (layer_result);

}

//computes convolution and ReLu
std::vector<std::vector<std::vector<int64_t>>>zs_monitor::compute_convolution(std::vector<std::vector<std::vector<int64_t>>> layer_input, zs_monitor_cnn_layer layer_parameters) {
	//weights are indexed as [kernel_idx][channel][column][row]
	//image is indexed as [row][column][channel]
	std::vector<std::vector<std::vector<int64_t>>> output_image;
	std::vector<std::vector<std::vector<std::vector<int64_t>>> >all_kernels = layer_parameters.weights;
	std::vector<int64_t> all_biases = layer_parameters.biases;

	int padding = layer_parameters.padding*2;//*2 since we add a row of 0 on each side
	int num_input_rows =layer_parameters.num_input_rows;
	int num_input_columns =layer_parameters.num_input_columns;
	int num_input_channels =layer_parameters.num_input_channels;
	int kernel_side =layer_parameters.kernel_side;

	int output_num_row = num_input_rows - kernel_side + 1 + padding*2;
	int output_num_columns = num_input_columns - kernel_side + 1 + padding*2;
	int output_num_channels = layer_parameters.num_output_channels;
	int relu = layer_parameters.relu_enabled;

	//Output image initialization
	//log_utilities::debug("  r  e  lu = %d", relu );

	//Output image initialization
	log_utilities::debug("Output image sizing: %d %d %d ", output_num_row,output_num_columns,output_num_channels );
	output_image.resize(output_num_row);
	for (int row_idx = 0; row_idx < output_num_row; row_idx++) {
		output_image[row_idx].resize(output_num_columns);
		for (int column_idx = 0; column_idx < output_num_columns; column_idx++) {
			output_image[row_idx][column_idx].resize(output_num_channels);

		}
	}

	//Convolutions are compute moving a kernel around and the restarting. In this way we try to keep kernel's weights in CPU cache and optimize for speed
	for (int kernel_idx = 0; kernel_idx < output_num_channels; kernel_idx++) {
		std::vector<std::vector<std::vector<int64_t>>> kernel = all_kernels[kernel_idx];
		int64_t bias = all_biases[kernel_idx];
		//we compute first without considering padding
		//These two loops are for moving the kernel over the image
		for(int input_row = 0; input_row < output_num_row; input_row++ ) {
			for(int input_column = 0; input_column < output_num_columns; input_column++ ) {
				int64_t conv_result = 0;
				//Here we perform the actual computation
				for(int ker_row = 0; ker_row < kernel_side; ker_row++) {
					for(int ker_col = 0; ker_col < kernel_side; ker_col++) {
						for(int ker_ch = 0; ker_ch < num_input_channels; ker_ch++) {

							int64_t pixel = layer_input[input_row+ker_row][input_column+ker_col][ker_ch];

							int64_t weight = kernel[ker_ch][ker_row][ker_col];

							conv_result = conv_result + weight*pixel;
							int pixint = pixel;
							int weightint = weight;
							int convint = conv_result;
							//if (input_row == 4 && input_column == 3 && kernel_idx == 15) {
							//  log_utilities::debug("Pixel: %d Weight:%d accumulator: %d ", pixint,weightint,convint );
							//  }
						}
					}
				}

				conv_result = (conv_result + bias*std::pow(2,zs_parameters::MANTISSA_NUM_BITS))/std::pow(2,zs_parameters::MANTISSA_NUM_BITS);
				int biasint = bias;
				int convint = conv_result;
				// if (input_row == 4 && input_column == 3 && kernel_idx == 15) {
				//   log_utilities::debug("conv_result: %d bias: %d ", convint, biasint );
				//  throw "stop";
				//   }
				if (relu == 1) {
					if (conv_result < 0) {
						//log_utilities::debug("rectified");

						conv_result = 0;
					}
				}

				output_image[input_row + padding][input_column + padding][kernel_idx] = conv_result;

			}

		}

		//TODO following code needs a review, not reliable
		//Here we have done all conv for the kernel, but we still need to compute the most external ring of the image to consider padding

		//Top boarder calculation
		for(int input_row = 0; input_row < padding; input_row++ ) {
			for(int input_column = 0; input_column < num_input_columns - kernel_side + 1 + padding*2; input_column++ ) {
				int conv_result = 0;
				//Here we perform the actual computation
				for(int ker_row = 0; ker_row < kernel_side; ker_row++) {
					for(int ker_col = 0; ker_col < kernel_side; ker_col++) {
						for(int ker_ch = 0; ker_ch < num_input_channels; ker_ch++) {

							if ((input_row - padding+ker_row >= 0 ) && (input_column - padding +ker_col >= 0) &&(input_column-padding+ker_col < num_input_columns) ) {
								conv_result = conv_result + layer_input[input_row-padding+ker_row][input_column-padding+ker_col][ker_ch]*kernel[ker_ch][ker_col][ker_row];
							}
						}
					}
				}
				if (relu == 1) {
					if (conv_result < 0) {
						conv_result = 0;
					}
				}
				output_image[input_row][input_column][kernel_idx] = conv_result +bias;
			}
		}

		//bottom boarder calculation
		for(int input_row = num_input_rows - kernel_side + 1; input_row < padding*2 + num_input_rows - kernel_side + 1; input_row++ ) {
			for(int input_column = 0; input_column < num_input_columns - kernel_side + 1 + padding; input_column++ ) {
				int conv_result = 0;
				//Here we perform the actual computation
				for(int ker_row = 0; ker_row < kernel_side; ker_row++) {
					for(int ker_col = 0; ker_col < kernel_side; ker_col++) {
						for(int ker_ch = 0; ker_ch < num_input_channels; ker_ch++) {

							if ((input_row + ker_row < num_input_rows ) && (input_column + ker_col - padding >= 0) &&(input_column-padding+ker_col < num_input_columns)) {
								conv_result = conv_result + layer_input[input_row+ker_row][input_column-padding+ker_col][ker_ch]*kernel[ker_ch][ker_col][ker_row];
							}
						}
					}
				}
				if (relu == 1) {
					if (conv_result < 0) {
						conv_result = 0;
					}
				}
				output_image[input_row][input_column][kernel_idx] = conv_result +bias;
			}
		}

		//left boarder calculation
		for(int input_row = 0; input_row < num_input_rows - kernel_side + 1 + padding*2; input_row++ ) {
			for(int input_column = 0; input_column < padding; input_column++ ) {
				int conv_result = 0;
				//Here we perform the actual computation
				for(int ker_row = 0; ker_row < kernel_side; ker_row++) {
					for(int ker_col = 0; ker_col < kernel_side; ker_col++) {
						for(int ker_ch = 0; ker_ch < num_input_channels; ker_ch++) {

							if ((input_row-padding+ker_row >= 0 ) &&
									(input_row-padding+ker_row < num_input_rows ) &&
									(input_column + ker_col - padding >= 0) &&
									(input_column + ker_col - padding < num_input_columns)) {

								conv_result = conv_result + layer_input[input_row-padding+ker_row][input_column-padding+ker_col][ker_ch]*kernel[ker_ch][ker_col][ker_row];
							}
						}
					}
				}
				if (relu == 1) {
					if (conv_result < 0) {
						conv_result = 0;
					}
				}
				output_image[input_row][input_column][kernel_idx] = conv_result +bias;
			}
		}

		//right boarder calculation
		for(int input_row = 0; input_row < num_input_rows - kernel_side + 1 + padding*2; input_row++ ) {
			for(int input_column = num_input_columns; input_column < num_input_columns + padding; input_column++ ) {
				int conv_result = 0;
				//Here we perform the actual computation
				for(int ker_row = 0; ker_row < kernel_side; ker_row++) {
					for(int ker_col = 0; ker_col < kernel_side; ker_col++) {
						for(int ker_ch = 0; ker_ch < num_input_channels; ker_ch++) {

							if ((input_row-padding+ker_row >= 0 ) &&
									(input_row-padding+ker_row < num_input_rows ) &&
									(input_column + ker_col - padding >= 0) &&
									(input_column + ker_col - padding < num_input_columns)) {

								conv_result = conv_result + layer_input[input_row-padding+ker_row][input_column-padding+ker_col][ker_ch]*kernel[ker_ch][ker_col][ker_row];
							}
						}
					}
				}
				if (relu == 1) {
					if (conv_result < 0) {
						conv_result = 0;
					}
				}
				output_image[input_row][input_column][kernel_idx] = conv_result +bias;
			}
		}
	}

	log_utilities::debug("Convolution/ReLu layer done");
	return (output_image);
}

std::vector<std::vector<std::vector<int64_t>>>zs_monitor::compute_pooling(std::vector<std::vector<std::vector<int64_t>>> layer_input) {

	std::vector<std::vector<std::vector<int64_t>>> column_pooling;
	log_utilities::debug("Pooling on columns...");
	column_pooling.resize(layer_input.size());
	for (int row_idx = 0; row_idx < layer_input.size(); row_idx++) {
		column_pooling[row_idx].resize(layer_input[0].size()/2);

		for (int column_idx = 0; column_idx < layer_input[0].size()/2; column_idx++) {
			column_pooling[row_idx][column_idx].resize(layer_input[0][0].size());

			for (int channel_idx = 0; channel_idx < layer_input[0][0].size(); channel_idx++) {

				column_pooling[row_idx][column_idx][channel_idx] = std::max(layer_input[row_idx][column_idx*2][channel_idx],layer_input[row_idx][column_idx*2+1][channel_idx]);
			}
		}
	}

	std::vector<std::vector<std::vector<int64_t>>> row_pooling;
	log_utilities::debug("Pooling on rows...");
	row_pooling.resize(column_pooling.size()/2);
	for (int row_idx = 0; row_idx < column_pooling.size()/2; row_idx++) {
		row_pooling[row_idx].resize(column_pooling[0].size());

		for (int column_idx = 0; column_idx < column_pooling[0].size(); column_idx++) {
			row_pooling[row_idx][column_idx].resize(column_pooling[0][0].size());

			for (int channel_idx = 0; channel_idx < column_pooling[0][0].size(); channel_idx++) {

				row_pooling[row_idx][column_idx][channel_idx] = std::max(column_pooling[row_idx*2][column_idx][channel_idx],column_pooling[row_idx*2+1][column_idx][channel_idx]);
			}
		}
	}
	log_utilities::debug("Pooling done");
	return (row_pooling);
}

std::vector<std::vector<std::vector<int64_t>>>zs_monitor::image_1d_to_3d(std::vector<uint64_t> l_image, int num_rows, int num_columns, int num_channels) {

	std::vector<std::vector<std::vector<int64_t>>> new_image;
	int read_index = 0;
	int word_idx = 0;
	int16_t pixel = 0;
	log_utilities::debug("Converting image from 1d to 3d, expected size linear vector: %d, actual size linear vector: %d",num_rows* num_columns*num_channels/2,l_image.size());

	new_image.resize(num_rows);
	for (int row_idx = 0; row_idx < num_rows; row_idx++) {
		new_image[row_idx].resize(num_columns);
		for (int column_idx = 0; column_idx < num_columns; column_idx++) {
			new_image[row_idx][column_idx].resize(num_channels);
			for (int channel_idx = 0; channel_idx < num_channels; channel_idx++) {

				std::tie(pixel,read_index,word_idx) = get_next_word(l_image, read_index, word_idx);
				int64_t pixel64bit = pixel;
				//  log_utilities::debug("Pixel 16b: %d - Pixel 64b:%lld",pixel,pixel64bit );

				new_image[row_idx][column_idx][channel_idx] = pixel;

			}
		}

	}

	return (new_image);

}

void zs_monitor::write_activations_to_file(
		std::vector<std::vector<std::vector<std::vector<int64_t>>> >l_activations) {
#ifdef DUMP_ACTIVATIONS
	log_utilities::high("Dumping monitor's activations to file...");

	std::string dir_path = "./";
	for (int layer_idx = 1; layer_idx < l_activations.size(); layer_idx++) {
		std::stringstream string_generator;
		string_generator << dir_path << "layer_" << layer_idx << ".txt";
		std::string path = string_generator.str();
		std::ofstream activations_file;
		activations_file.open(path);
		if (activations_file.is_open()) {

			const int num_rows = l_activations[layer_idx].size();
			const int num_columns = l_activations[layer_idx][0].size();
			const int num_channels = l_activations[layer_idx][0][0].size();

			for (int channel_idx = 0; channel_idx < num_channels; channel_idx++) {
				for (int row_idx = 0; row_idx < num_rows; row_idx++) {
					for (int column_idx = 0; column_idx < num_columns; column_idx++) {
						float activation_as_float = l_activations[layer_idx][row_idx][column_idx][channel_idx];
						float rescaled_activation = activation_as_float/zs_parameters::MANTISSA_RESCALE_FACTOR;

						activations_file << rescaled_activation << std::endl;
					}
				}
			}
			activations_file.close();
		}
		else {
			log_utilities::error("Unable to open file");
		}
	}

	log_utilities::high("Dump completed");
#endif
}

int zs_monitor::get_monitor_classification() {
	int result_pos = cnn_num_layers;

	log_utilities::none("Final activations monitor (#%d): %lld %lld %lld %lld",
	        monitor_activations[result_pos][0][0].size(),
			monitor_activations[result_pos][0][0][0],
			monitor_activations[result_pos][0][0][1],
			monitor_activations[result_pos][0][0][2],
			monitor_activations[result_pos][0][0][3]);
	return (std::distance(monitor_activations[result_pos][0][0].begin(),
			std::max_element(monitor_activations[result_pos][0][0].begin(),
					monitor_activations[result_pos][0][0].end())));

}

//checks computation is correct
void zs_monitor::check_layer_activations(std::vector<uint64_t> activations,
		int layer_idx) {
#ifndef RESULT_MONITOR_CHECK_LAYER_ACTIVATION_DISABLED
	// hw_activations[layer_idx] = activations;
	bool error_found = false;
	std::vector < std::vector<std::vector<int64_t>>>sw_3dactivations = monitor_activations[layer_idx + 1];

	//These values refers to layer+1 because we are comparing the output of each layer
	const int monitor_activation_num_rows = sw_3dactivations.size();
	const int monitor_activation_num_columns = sw_3dactivations[0].size();
	const int monitor_activation_num_channels = sw_3dactivations[0][0].size();
	const int monitor_activation_size = monitor_activation_num_columns * monitor_activation_num_columns
	* monitor_activation_num_channels;

	log_utilities::debug(
			"check_layer_activations parameters - monitor_activation_num_rows:%d monitor_activation_num_columns:%d monitor_activation_num_channels:%d monitor_activation_size:%d",
			monitor_activation_num_rows, monitor_activation_num_columns, monitor_activation_num_channels,
			monitor_activation_size);

	//in hw currently compression is wired to relu
	if (cnn_kernels[layer_idx].relu_enabled == 1) {
		activations = remove_words_using_key(activations, zs_axi_bits::IDLE_MASK); //Last word is removed since it is t

		std::vector < std::vector<std::vector<int64_t>>>decompr_hw_image = decompress_sm_image(activations,
				cnn_kernels[layer_idx].num_output_rows, cnn_kernels[layer_idx].num_output_columns,
				cnn_kernels[layer_idx].num_output_channels, zs_parameters::SPARSITY_MAP_WORD_NUM_BITS);

		if ((decompr_hw_image.size() != monitor_activation_num_rows) || decompr_hw_image[0].size() != monitor_activation_num_columns
				|| decompr_hw_image[0][0].size() != monitor_activation_num_channels) {
			error_found = true;
			log_utilities::error(
					"***SOFTWARE ERROR DETECTED: hw and sw activation for layer %d have different sizes: %d-%d-%d vs %d-%d-%d",
					layer_idx, decompr_hw_image.size(), decompr_hw_image[0].size(), decompr_hw_image[0][0].size(),
					monitor_activation_num_rows, monitor_activation_num_columns, monitor_activation_num_channels);
		}

		for (int row_idx = 0; row_idx < monitor_activation_num_rows; row_idx++) {
			for (int column_idx = 0; column_idx < monitor_activation_num_columns; column_idx++) {
				for (int channel_idx = 0; channel_idx < monitor_activation_num_channels; channel_idx++) {
					/* log_utilities::debug(
					 "comparing pixels - sw: %d - hw: %d",sw_3dactivations[row_idx][column_idx][channel_idx],decompr_hw_image[row_idx][column_idx][channel_idx]);*/

					if (decompr_hw_image[row_idx][column_idx][channel_idx] != sw_3dactivations[row_idx][column_idx][channel_idx]) {
						error_found = true;
						log_utilities::error(
								"***ERROR DETECTED: Mismatch in output of layer %d - row %d - column %d - channel %d - HW value: %lld - SW value: %lld",
								layer_idx, row_idx, column_idx, channel_idx,
								(long long int) decompr_hw_image[row_idx][column_idx][channel_idx],
								(long long int) sw_3dactivations[row_idx][column_idx][channel_idx]);

					}

				}
			}
		}

	} else {
		//TODO this code is not verified
		//if activations are uncompressed
		int activ_idx = 0;
		int word_idx = 0;
		int hw_pixel = 0;

		if (monitor_activation_size != activations.size()) {
			error_found = true;
			log_utilities::error("***ERROR DETECTED: size of SW activations map is %d, size of HW output is %d:",
					monitor_activation_size, activations.size());
		}

		for (int row_idx = 0; row_idx < monitor_activation_num_rows; row_idx++) {
			for (int column_idx = 0; column_idx < monitor_activation_num_columns; column_idx++) {
				for (int channel_idx = 0; channel_idx < monitor_activation_num_channels; channel_idx++) {
					int monitor_pixel = sw_3dactivations[row_idx][column_idx][channel_idx];

					if (activ_idx != activations.size())
					std::tie(hw_pixel, activ_idx, word_idx) = get_next_word(activations, activ_idx, word_idx);
					else

					if (monitor_pixel != hw_pixel) {
						error_found = true;
						log_utilities::error(
								"***ERROR DETECTED: Mismatch in output of layer %d - row %d - column %d - channel %d - HW value: %lld - SW value: %lld",
								layer_idx, row_idx, column_idx, channel_idx, hw_pixel, monitor_pixel);

					}
				}
			}
		}
	}
	if (error_found == false)
	log_utilities::low("Layer %d output check as correct", layer_idx);
	else
	log_utilities::error("***Errors detected in layer %d", layer_idx);

	// return (error_found);
#endif
}

#else

//empty functions for disabled monitor mode
zs_monitor::zs_monitor(std::string filename) {
}
void zs_monitor::classify_image(std::vector<uint64_t> l_image) {
}
void zs_monitor::check_layer_activations(std::vector<uint64_t> activations, int layer_idx) {
}

int zs_monitor::get_monitor_classification() {

}



#endif

//empty functions for disabled monitor mode
zs_monitor::zs_monitor() {
}

#endif
