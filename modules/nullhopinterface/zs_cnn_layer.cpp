#ifndef __ZS_CNN_LAYER__
#define __ZS_CNN_LAYER__

#include "zs_cnn_layer.h"
#include "npp_log_utilities.h"
#include "npp_std_func_sw_pkg.cpp"

#include "zs_axi_formatter.h"
#include "iostream"
#include <stdio.h>
#include "string.h"
#include "cstdint"
#include "inttypes.h"
#include <unistd.h>

zs_cnn_layer::zs_cnn_layer(int l_layer_idx, FILE* l_net_file) {

	if (read_layer_from_file(l_net_file, l_layer_idx) == false) {
		throw "CNN layer not initialized properly";
	}

}

int zs_cnn_layer::get_uncompressed_input_image_num_pixels() {
	return (num_input_rows * num_input_columns * num_input_channels);
}

int zs_cnn_layer::get_pixels_per_row() {
	return (num_input_rows * num_input_channels);
}

int zs_cnn_layer::get_input_num_rows() {
	return (num_input_rows);
}

void zs_cnn_layer::set_layer_idx(int l_layer_idx) {
	layer_idx = l_layer_idx;
}
void zs_cnn_layer::set_compression_enabled(int l_compression_enabled) {
	compression_enabled = l_compression_enabled;
}
void zs_cnn_layer::set_kernel_size(int l_kernel_size) {
	kernel_side = l_kernel_size;
}
void zs_cnn_layer::set_num_input_channels(int l_num_input_channels) {
	num_input_channels = l_num_input_channels;
}
void zs_cnn_layer::set_num_input_columns(int l_num_input_columns) {
	num_input_columns = l_num_input_columns;
}
void zs_cnn_layer::set_num_input_rows(int l_num_input_rows) {
	num_input_rows = l_num_input_rows;
}
void zs_cnn_layer::set_num_output_channels(int l_num_output_channels) {
	num_output_channels = l_num_output_channels;
}
void zs_cnn_layer::set_pooling_enabled(int l_pooling_enabled) {
	pooling_enabled = l_pooling_enabled;
}
void zs_cnn_layer::set_relu_enabled(int l_relu_enabled) {
	relu_enabled = l_relu_enabled;
}
void zs_cnn_layer::set_padding(int l_padding) {
	padding = l_padding;
}
void zs_cnn_layer::set_num_weight(int l_num_weight) {
	num_weight = l_num_weight;
}
void zs_cnn_layer::set_num_biases(int l_num_biases) {
	num_biases = l_num_biases;
}

std::vector<std::vector<uint64_t>> zs_cnn_layer::get_config_array() {
	const int REG_TYPE = zs_parameters::REG_TYPE;
	std::vector<std::vector<uint64_t>> configs;

	for (int pass_idx = 0; pass_idx < num_pass; pass_idx++) {
		zs_axi_formatter axi_formatter = zs_axi_formatter();
		//axi_formatter.format takes: type,address,value
		log_utilities::debug("Starting preparation of config array for pass %d",
				pass_idx);

		/*     static bool check = false;
		 if (check == false) {
		 log_utilities::error("*********************\n%d - %d - %d", REG_TYPE, zs_address_space::config_pooling_enabled, pooling_enabled);
		 check = true;
		 }*/

		axi_formatter.append(REG_TYPE,
				zs_address_space::config_image_compression_enabled,
				compression_enabled);
		axi_formatter.append(REG_TYPE,
				zs_address_space::config_pre_sm_counter_max,
				pre_sm_counter_max);

		axi_formatter.append(REG_TYPE, zs_address_space::config_kernel_size,
				kernel_side);
		axi_formatter.append(REG_TYPE,
				zs_address_space::config_num_input_channels,
				num_input_channels);

		axi_formatter.append(REG_TYPE,
				zs_address_space::config_num_input_column, num_input_columns);
		axi_formatter.append(REG_TYPE, zs_address_space::config_num_input_rows,
				num_input_rows);

		axi_formatter.append(REG_TYPE,
				zs_address_space::config_num_output_channels,
				num_output_channels);
		axi_formatter.append(REG_TYPE, zs_address_space::config_pooling_enabled,
				pooling_enabled);

		axi_formatter.append(REG_TYPE, zs_address_space::config_relu_enabled,
				relu_enabled);
		axi_formatter.append(REG_TYPE,
				zs_address_space::config_contiguous_kernels,
				contiguous_kernels);

		axi_formatter.append(REG_TYPE,
				zs_address_space::config_num_macs_per_channel,
				macs_per_channel - 1); //The actual value in hw is from 0 to 7
		axi_formatter.append(REG_TYPE,
				zs_address_space::config_input_channel_decode_jump_mask,
				channel_decode_jump_mask);

		axi_formatter.append_new_word(REG_TYPE,
				zs_address_space::config_kernel_memory_write_complete_pulse, 0);
		axi_formatter.append_new_word(REG_TYPE,
				zs_address_space::config_kernel_memory_resetn_pulse, 0);

		axi_formatter.append(REG_TYPE,
				zs_address_space::config_row_column_offset, padding);

		if (pass_idx == 0) {
			axi_formatter.append(REG_TYPE,
					zs_address_space::config_image_in_memory, 0);
		} else {
			axi_formatter.append(REG_TYPE,
					zs_address_space::config_image_in_memory, 1);

		}
		std::vector<uint64_t> pass_config = axi_formatter.get_array();
		configs.push_back(pass_config);

	}
	log_utilities::debug("Config array setup completed");
	return (configs);
}

std::vector<std::vector<uint64_t>> zs_cnn_layer::get_weight_array(
		FILE* l_net_file) {

//Read kernels weight
	std::vector<std::vector<uint64_t>> weights;

	unsigned int pos_tmp;
	unsigned int x_ker;
	unsigned int y_ker;
	unsigned int input_ch_ker;
	unsigned int out_ch_ker;
	unsigned int effective_num_weights;
	unsigned int effective_channels_ratio;
	int num_weight_read_from_file = 0;
	const int total_num_weight_from_file = num_input_channels
			* num_output_channels * kernel_side * kernel_side;
	int l_weight;
	const int KER_TYPE = zs_parameters::KER_TYPE;
	const int REG_TYPE = zs_parameters::REG_TYPE;

	log_utilities::debug("Preparing weights array...");

	effective_num_weights = effective_num_input_channels
			* effective_num_output_channels * kernel_side * kernel_side;
	weights.reserve(num_pass);
	for (int pass_idx = 0; pass_idx < num_pass; pass_idx++) {
		zs_axi_formatter axi_formatter = zs_axi_formatter();

		for (int weight_idx = 0; weight_idx < effective_num_weights;
				weight_idx++) {
			pos_tmp = weight_idx;
			x_ker = pos_tmp % kernel_side; //x dimension of the kernel
			pos_tmp = pos_tmp / kernel_side;
			y_ker = pos_tmp % kernel_side; //y dimension of the kernel
			pos_tmp = pos_tmp / kernel_side;
			input_ch_ker = pos_tmp % effective_num_input_channels;
			pos_tmp = pos_tmp / effective_num_input_channels;
			out_ch_ker = pos_tmp + pass_idx * effective_num_output_channels;

			effective_channels_ratio = effective_num_output_channels
					/ num_output_channels;

			if (!(out_ch_ker % effective_channels_ratio)
					&& input_ch_ker < num_input_channels) {
				l_weight = read_int_from_file(l_net_file);
				num_weight_read_from_file++;
				axi_formatter.append(KER_TYPE, 0, l_weight);
			} else {
				axi_formatter.append(KER_TYPE, 0, 0);
			}
			if ((weight_idx + 1) % contiguous_kernels == 0)
				axi_formatter.flush_word();
		}
		axi_formatter.append_new_word(REG_TYPE,
				zs_address_space::config_kernel_memory_write_complete_pulse, 1);
		axi_formatter.append_new_word(REG_TYPE,
				zs_address_space::config_start_process_pulse, 1);

		weights.push_back(axi_formatter.get_array());

	}

	if (num_weight_read_from_file != total_num_weight_from_file) {
		log_utilities::error(
				"Wrong number of weight read from network file: %d,expected %d",
				num_weight_read_from_file, total_num_weight_from_file);
		throw "Wrong number of weight read from network file";
	}

	log_utilities::debug("Weights array ready");
	return (weights);

}

std::vector<std::vector<uint64_t>> zs_cnn_layer::get_biases_array(
		FILE* l_net_file) {

	std::vector<std::vector<uint64_t>> biases;
	int l_bias;
	const int BIAS_TYPE = zs_parameters::BIAS_TYPE;
	log_utilities::debug("Preparing biases array...");
	biases.reserve(num_pass);
	for (int pass_idx = 0; pass_idx < num_pass; pass_idx++) {

		zs_axi_formatter axi_formatter = zs_axi_formatter();
		for (int bias_idx = 0; bias_idx < bias_per_pass; bias_idx++) {

			if (bias_idx % macs_per_channel == 0) {
				l_bias = read_int_from_file(l_net_file);
				axi_formatter.append_new_word(BIAS_TYPE, bias_idx, l_bias);
			} else {
				axi_formatter.append_new_word(BIAS_TYPE, bias_idx, 0);
			}

		}

		std::vector<uint64_t> biases_pass_array = axi_formatter.get_array();

		if (biases_pass_array.size() != NUM_MACS) {
			log_utilities::error(
					"Bias preparation exception, bias size: %d, NUM_MACS: %d",
					biases_pass_array.size(), NUM_MACS);
			throw "Inconsistent number of bias to load";
		} else {
			log_utilities::debug(
					"Biases array layer %d - pass %d consistency check passed",
					layer_idx, pass_idx);
		}

		biases.push_back(biases_pass_array);
	}

	if (biases.size() != num_pass) {
		throw "Inconsistent number of bias to load once final array merged";
	} else {
		log_utilities::debug("Biases array layer %d consistency check passed",
				layer_idx);
	}

	log_utilities::debug("Biases array ready");
	return (biases);
}

//This function is mostly based on Hesham original driver
void zs_cnn_layer::set_derived_config() {

	int kernel_memories_required;
	int macs_per_channel_required;
	int kernel_side_square;
	int single_pass_output_channels;

	log_utilities::debug("Computing derived configuration parameters...");
	kernel_side_square = kernel_side * kernel_side;
	kernel_memories_required = kernel_side_square * num_input_channels / (4096)
			+ 1;

	if (num_output_channels > NUM_MACS) {
		int num_pass_rounded = num_output_channels / NUM_MACS;
		single_pass_output_channels = num_output_channels
				/ (num_pass_rounded + (num_output_channels % NUM_MACS ? 1 : 0));
		macs_per_channel_required = NUM_MACS / single_pass_output_channels;
	} else {
		macs_per_channel_required = NUM_MACS / num_output_channels;
	}

//With this IF we decide if the num of active macs in 1 pass is due to upper limit in terms of memory or for the upper limit
//in terms of num of macs
	if (macs_per_channel_required > kernel_memories_required) {
		macs_per_channel = macs_per_channel_required;
	} else {
		macs_per_channel = kernel_memories_required;
	}

//Only even num of macs per channel supported, so we round up
	if (macs_per_channel == 1)
		macs_per_channel = 1;
	else if (macs_per_channel <= 2)
		macs_per_channel = 2;
	else if (macs_per_channel <= 4)
		macs_per_channel = 4;
	else if (macs_per_channel <= 8)
		macs_per_channel = 8;
	else {
		fprintf(stderr, "Invalid macs_per_channel %d\n", macs_per_channel);
	}

	effective_num_output_channels = (NUM_MACS / macs_per_channel);
	num_pass = num_output_channels / effective_num_output_channels;

	int nearest_pow2_input_channels = 1;
	while (nearest_pow2_input_channels < num_input_channels) {
		nearest_pow2_input_channels *= 2;
	}

	if (macs_per_channel == 1) {
		contiguous_kernels = num_input_channels * kernel_side_square;
		channel_decode_jump_mask = nearest_pow2_input_channels - 1;
		effective_num_input_channels = num_input_channels;
	} else {
		int num_dummy_kernels;
		num_dummy_kernels = (nearest_pow2_input_channels - num_input_channels)
				* kernel_side_square;

		if (nearest_pow2_input_channels <= macs_per_channel) {
			contiguous_kernels = kernel_side_square;
			num_dummy_kernels +=
					(macs_per_channel - nearest_pow2_input_channels)
							* kernel_side_square;
		} else {
			contiguous_kernels =
					(nearest_pow2_input_channels / macs_per_channel)
							* kernel_side_square;
		}

		effective_num_input_channels = num_input_channels
				+ (num_dummy_kernels / kernel_side_square);
		channel_decode_jump_mask = (contiguous_kernels / kernel_side_square)
				- 1;

	}

	weight_per_pass = num_weight / num_pass;
	bias_per_pass = NUM_MACS;
	pre_sm_counter_max = (NUM_MACS
			/ (zs_parameters::NUM_MACS_PER_CLUSTER * macs_per_channel)) - 1;

	log_utilities::debug("macs_per_channel: %d", macs_per_channel);
	log_utilities::debug("effective_num_input_channels: %d",
			effective_num_input_channels);
	log_utilities::debug("effective_num_output_channels: %d",
			effective_num_output_channels);
	log_utilities::debug("contiguous_kernels: %d", contiguous_kernels);
	log_utilities::debug("channel_decode_jump_mask: %d",
			channel_decode_jump_mask);
	log_utilities::debug("weight_per_pass: %d", weight_per_pass);
	log_utilities::debug("bias_per_pass: %d", bias_per_pass);
	log_utilities::debug("num_pass: %d", num_pass);
	log_utilities::debug("pre_sm_counter_max: %d", pre_sm_counter_max);

	log_utilities::debug("Derived configuration parameters computation done");

}

void zs_cnn_layer::initialize_layer(int l_layer_idx, int l_compression_enabled,
		int l_kernel_size, int l_num_input_channels, int l_num_input_columns,
		int l_num_input_rows, int l_num_output_channels, int l_pooling_enabled,
		int l_relu_enabled, int l_padding, int l_num_weight, int l_num_biases) {

	set_layer_idx(l_layer_idx);
	set_compression_enabled(l_compression_enabled);
	set_kernel_size(l_kernel_size);
	set_num_input_channels(l_num_input_channels);
	set_num_input_columns(l_num_input_columns);
	set_num_input_rows(l_num_input_rows);

	set_num_output_channels(l_num_output_channels);
	set_pooling_enabled(l_pooling_enabled);
	set_relu_enabled(l_relu_enabled);
	set_padding(l_padding);
	set_num_weight(l_num_weight);
	set_num_biases(l_num_biases);
	set_derived_config();

}
void zs_cnn_layer::set_layer_config(FILE*l_net_file, int l_layer_idx) {

	int l_compression_enabled, l_kernel_size, l_num_input_channels,
			l_num_input_columns, l_num_input_rows, l_num_output_channels,
			l_pooling_enabled, l_relu_enabled, l_padding, l_bias,
			l_num_biases, l_num_weight;

	log_utilities::debug("Reading layer parameters...");

//Read layer config
	l_compression_enabled = read_int_from_file(l_net_file);
	log_utilities::debug("compression_enabled: %d", l_compression_enabled);
	l_kernel_size = read_int_from_file(l_net_file);
	log_utilities::debug("kernel_size: %d", l_kernel_size);
	l_num_input_channels = read_int_from_file(l_net_file);
	log_utilities::debug("num_input_channels: %d", l_num_input_channels);
	l_num_input_columns = read_int_from_file(l_net_file);
	log_utilities::debug("num_input_columns: %d", l_num_input_columns);
	l_num_input_rows = read_int_from_file(l_net_file);
	log_utilities::debug("num_input_rows: %d", l_num_input_rows);
	l_num_output_channels = read_int_from_file(l_net_file);
	log_utilities::debug("num_output_channels: %d", l_num_output_channels);
	l_pooling_enabled = read_int_from_file(l_net_file);
	log_utilities::debug("pooling_enabled: %d", l_pooling_enabled);
	l_relu_enabled = read_int_from_file(l_net_file);
	log_utilities::debug("relu_enabled: %d", l_relu_enabled);
	l_padding = read_int_from_file(l_net_file);
	log_utilities::debug("padding: %d", l_padding);

	l_num_weight = l_kernel_size * l_kernel_size * l_num_input_channels
			* l_num_output_channels;
	log_utilities::debug("num_weight: %d", l_num_weight);
	l_num_biases = l_num_output_channels;
	log_utilities::debug("num_biases: %d", l_num_biases);

	num_output_columns = (num_input_columns - kernel_side + 1 + padding * 2)
			/ (pooling_enabled + 1);

	num_output_rows = (num_input_rows - kernel_side + 1 + padding * 2)
			/ (pooling_enabled + 1);

//Layer initialization
	initialize_layer(l_layer_idx, l_compression_enabled, l_kernel_size,
			l_num_input_channels, l_num_input_columns, l_num_input_rows,
			l_num_output_channels, l_pooling_enabled, l_relu_enabled, l_padding,
			l_num_weight, l_num_biases);

	log_utilities::debug("Layer config setting completed");
}

bool zs_cnn_layer::read_layer_from_file(FILE* l_net_file, int l_layer_idx) {

	set_layer_config(l_net_file, l_layer_idx);

	std::vector<std::vector<uint64_t>> config_array = get_config_array();
	std::vector<std::vector<uint64_t>> weight_array = get_weight_array(
			l_net_file);
	std::vector<std::vector<uint64_t>> biases_array = get_biases_array(
			l_net_file);

	log_utilities::debug(
			"Merging config, weights and biases into loading-ready arrays...");
	load_array.resize(num_pass);
	for (int pass_idx = 0; pass_idx < num_pass; pass_idx++) {

		load_array[pass_idx].reserve(
				config_array[pass_idx].size() + biases_array[pass_idx].size()
						+ weight_array[pass_idx].size());
		load_array[pass_idx].insert(load_array[pass_idx].begin(),
				config_array[pass_idx].begin(), config_array[pass_idx].end()); //config is inserted at the begin
		load_array[pass_idx].insert(load_array[pass_idx].end(),
				biases_array[pass_idx].begin(), biases_array[pass_idx].end());
		load_array[pass_idx].insert(load_array[pass_idx].end(),
				weight_array[pass_idx].begin(), weight_array[pass_idx].end());

	}

	log_utilities::debug("Layer read from file completed");
	return (true);

}

int zs_cnn_layer::get_num_pass() {
	return (num_pass);
}

std::vector<uint64_t>* zs_cnn_layer::get_load_array(int pass_idx) {
	return (&load_array[pass_idx]);
}

#endif
