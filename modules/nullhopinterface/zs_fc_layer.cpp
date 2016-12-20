#ifndef __zs_fc_layer__
#define __zs_fc_layer__

#include "zs_fc_layer.h"
#include "npp_std_func_sw_pkg.cpp"
#include "npp_log_utilities.h"
#include "stdio.h"
#include "string.h"
#include "cstdint"
#include "inttypes.h"
#include <unistd.h>
#include <vector>

zs_fc_layer::zs_fc_layer(int l_layer_idx, FILE* l_net_file) {

    if (read_layer_from_file(l_net_file, l_layer_idx) == false) {
        throw "FC layer not initialized properly";
    }

}

int zs_fc_layer::get_input_num_rows() {
    return (num_input_rows);
}

void zs_fc_layer::set_layer_idx(int l_layer_idx) {
    layer_idx = l_layer_idx;
}
void zs_fc_layer::set_compression_enabled(int l_compression_enabled) {
    compression_enabled = l_compression_enabled;
}
void zs_fc_layer::set_kernel_size(int l_kernel_size) {
    kernel_side = l_kernel_size;
}
void zs_fc_layer::set_num_input_channels(int l_num_input_channels) {
    num_input_channels = l_num_input_channels;
}
void zs_fc_layer::set_num_input_columns(int l_num_input_columns) {
    num_input_columns = l_num_input_columns;
}
void zs_fc_layer::set_num_input_rows(int l_num_input_rows) {
    num_input_rows = l_num_input_rows;
}
void zs_fc_layer::set_num_output_channels(int l_num_output_channels) {
    num_output_channels = l_num_output_channels;
}
void zs_fc_layer::set_pooling_enabled(int l_pooling_enabled) {
    pooling_enabled = l_pooling_enabled;
}
void zs_fc_layer::set_relu_enabled(int l_relu_enabled) {
    relu_enabled = l_relu_enabled;
}
void zs_fc_layer::set_padding(int l_padding) {
    padding = l_padding;
}
void zs_fc_layer::set_num_weight(int l_num_weight) {
    num_weight = l_num_weight;
}
void zs_fc_layer::set_num_biases(int l_num_biases) {
    num_biases = l_num_biases;
}

std::vector<std::vector<int>> zs_fc_layer::read_weights(FILE* l_net_file) {
    //weights are indexed internally as [kernel_idx][channel][column][row]
    //weights are listed in the file as (fastest changing) column - row  - channel -kernel_idx (slower changing)
    std::vector<std::vector<std::vector<std::vector<int>>> >l_weights;

    l_weights.resize(num_output_channels);

    for(unsigned int kernel_idx = 0; kernel_idx < num_output_channels; kernel_idx++) {
        l_weights[kernel_idx].resize(num_input_channels);

        for(unsigned int input_channel_idx = 0; input_channel_idx < num_input_channels; input_channel_idx++) {
            l_weights[kernel_idx][input_channel_idx].resize(kernel_side);

            for(unsigned int row_idx = 0; row_idx < kernel_side; row_idx++) {

                l_weights[kernel_idx][input_channel_idx][row_idx].resize(kernel_side);

                for(unsigned int column_idx = 0; column_idx < kernel_side; column_idx++) {
                    int weight = read_int_from_file(l_net_file);

                    l_weights[kernel_idx][input_channel_idx][row_idx][column_idx] = weight;
                    //  log_utilities::debug("Weight 16b: %d - Weight 64b:%lld",weight,weight64bit );
                }
            }
        }
    }

    std::vector<std::vector<int>> linearized_weight;
    linearized_weight.resize(num_output_channels);
    for(unsigned int kernel_idx = 0; kernel_idx < num_output_channels; kernel_idx++) {
        for (int row_idx = 0; row_idx < kernel_side; row_idx++) {
            for (int column_idx = 0; column_idx < kernel_side; column_idx++) {
                for (int channel_idx = 0; channel_idx < num_input_channels; channel_idx++) {
                    linearized_weight[kernel_idx].push_back(l_weights[kernel_idx][channel_idx][row_idx][column_idx]);
                }
            }
        }
    }

    return (linearized_weight);
}

std::vector<int> zs_fc_layer::read_biases(FILE* l_net_file) {
    //biases are multiplied by MANTISSA_RESCALE_FACTOR in order to avoid to re shift FC results during live computation in zs_driver
    std::vector<int> l_biases;
    for (unsigned int kernel_idx = 0; kernel_idx < num_output_channels; kernel_idx++) {
        l_biases.push_back(read_int_from_file(l_net_file) * zs_parameters::MANTISSA_RESCALE_FACTOR);
    }
    return (l_biases);
}

void zs_fc_layer::initialize_layer(int l_layer_idx, int l_compression_enabled, int l_kernel_size,
        int l_num_input_channels, int l_num_input_columns, int l_num_input_rows,
        int l_num_output_channels, int l_pooling_enabled, int l_relu_enabled, int l_padding,
        int l_num_weight, int l_num_biases) {

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

}

void zs_fc_layer::set_layer_config(FILE*l_net_file, int l_layer_idx) {

    int l_compression_enabled, l_kernel_size, l_num_input_channels, l_num_input_columns,
            l_num_input_rows, l_num_output_channels, l_pooling_enabled, l_relu_enabled, l_padding,
            l_weight, l_bias, l_num_biases, l_num_weight;

    log_utilities::debug("Reading layer parameters...");

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

    l_num_weight = l_kernel_size * l_kernel_size * l_num_input_channels * l_num_output_channels;
    log_utilities::debug("num_weight: %d", l_num_weight);
    l_num_biases = l_num_output_channels;
    log_utilities::debug("num_biases: %d", l_num_biases);

//Layer initialization
    initialize_layer(l_layer_idx, l_compression_enabled, l_kernel_size, l_num_input_channels,
            l_num_input_columns, l_num_input_rows, l_num_output_channels, l_pooling_enabled,
            l_relu_enabled, l_padding, l_num_weight, l_num_biases);

    log_utilities::debug("Layer config setting completed");
}

bool zs_fc_layer::read_layer_from_file(FILE* l_net_file, int l_layer_idx) {

    set_layer_config(l_net_file, l_layer_idx);
    weights = read_weights(l_net_file);
    biases = read_biases(l_net_file);

    log_utilities::debug("Layer read from file completed");
    return (true);

}

#endif
