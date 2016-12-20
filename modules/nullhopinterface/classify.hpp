/*
 * zs_driver.h
 *
 *  Created on: Oct 10, 2016
 *      Author: asa
 */

#include "zs_backend_interface.h"
#include "zs_axi_formatter.h"
#include "zs_top_level_sw_pkg.cpp"
#include "zs_cnn_layer.h"
#include "zs_fc_layer.h"
#include "zs_monitor.h"
#include "stdio.h"
#include "iostream"
#include "string.h"
#include <vector>

class zs_driver {

public:
    zs_driver(std::string network_file_name);
    int classify_image(int* l_image);
    zs_backend_interface backend_if;
private:
    bool class_initialized;
    int total_num_processed_images;
    zs_axi_formatter pixel_formatter;
    zs_monitor monitor;
    int num_cnn_layers;
    int num_fc_layers;
    int total_num_layers;

    int first_layer_pixels_per_row;
    int first_layer_num_rows;
    int first_layer_num_pixels;
    int first_layer_num_axi_words;
    bool first_layer_pixels_per_row_odd;

    std::vector<uint64_t> first_layer_input;
    std::vector<int> first_layer_row_start_positions;
    std::vector<int> first_layer_row_start_positions_word_idx;

    std::vector<zs_cnn_layer> cnn_network;
    std::vector<zs_fc_layer> fc_network;

    void convert_input_image(int* l_image, int l_num_row, int l_total_num_pixel);
    std::vector<int64_t> compute_fc_layer(std::vector<int64_t> l_input, int layer_idx);
    std::vector<uint64_t> compute_cnn_layer(std::vector<uint64_t> l_input, int layer_idx,
            int pass_idx);
    void load_config_biases_kernels(int layer_idx, int pass_idx);
    void load_image(std::vector<uint64_t> l_input);
    bool read_network_from_file(std::string network_file_name);

    double time_accumulator;
    std::chrono::high_resolution_clock::time_point end_previous_frame;

};
