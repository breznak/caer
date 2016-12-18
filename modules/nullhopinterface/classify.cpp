#ifndef __ZS_DRIVER__
#define __ZS_DRIVER__

//#define SOFTWARE_ONLY_MODE

#include "classify.hpp"
#include "npp_log_utilities.h"
#include "npp_std_func_sw_pkg.cpp"
#include <functional>
#include <numeric>
#include <iterator>
#include <vector>
#include <algorithm>
#include <time.h>
#include <sys/time.h>
#include <ctime>
#include <chrono>
#include <string>
#include <exception>
#include <stdexcept>

zs_driver::zs_driver(std::string network_file_name) {
    num_fc_layers = 0;
    num_cnn_layers = 0;
    total_num_layers = 0;
    total_num_processed_images = 0;
    time_accumulator = 0;
    end_previous_frame = std::chrono::high_resolution_clock::now();
    log_utilities::none("Proceeding with network loading, network is: %s",
            network_file_name.c_str());

    if (network_file_name.empty() == false) {
        class_initialized = read_network_from_file(network_file_name); // Read a .net file containing network description and prepares arrays in memory
        monitor = zs_monitor(network_file_name);

        log_utilities::debug("Pre-loading config,biases and kernels for first layer...");
        load_config_biases_kernels(0, 0); // we start immediately to load config and weights for the first layer to save computational time
        log_utilities::debug("Pre-loading completed");
    } else {

        log_utilities::debug("No network file specified during driver initialization");
    }

}

int zs_driver::classify_image(int* l_image) {
    log_utilities::medium("*************************************\n\n");
    //log_utilities::none("Starting classification of image %d", total_num_processed_images);
    double duration;
    std::chrono::high_resolution_clock::time_point t_compression_end, t_fc_end, t_conv_end;

    std::chrono::high_resolution_clock::time_point t_computation_start =
            std::chrono::high_resolution_clock::now();

    int classification_result = -1;
    std::vector<uint64_t> next_layer_input;
    total_num_processed_images++;

    //Transform input image in ZS format and store it in class member first_layer_input
    //There is no return to avoid useless data movment and array initializations
    convert_input_image(l_image, first_layer_num_rows, first_layer_num_pixels);

    monitor.classify_image(first_layer_input);
#ifndef SOFTWARE_ONLY_MODE
    // First layer compute
    log_utilities::medium("Starting first layer computation on NHP...");

    try {
        next_layer_input = compute_cnn_layer(first_layer_input, 0, 0); // layer 0, pass 0

        monitor.check_layer_activations(next_layer_input, 0);

        for (int pass_idx = 1; pass_idx < cnn_network[0].get_num_pass(); pass_idx++) {
            log_utilities::medium("Starting layer 0 pass %d...", pass_idx);
            next_layer_input = compute_cnn_layer(next_layer_input, 0, pass_idx);
        }

        //Next CNN layers compute
        for (int layer_idx = 1; layer_idx < num_cnn_layers; layer_idx++) {

            for (int pass_idx = 0; pass_idx < cnn_network[0].get_num_pass(); pass_idx++) {
                log_utilities::medium("Starting layer %d pass %d...", layer_idx, pass_idx);
                next_layer_input = compute_cnn_layer(next_layer_input, layer_idx, pass_idx);
            }
            monitor.check_layer_activations(next_layer_input, layer_idx);
        }
    } catch (std::exception& e) {
        log_utilities::error(
                "**ERROR - Error in CNN computation, aborting image classification...");
        return (0);
    }
    log_utilities::medium("Convolutional layers completed, processing FC layers...");

    load_config_biases_kernels(0, 0); // we start already to load config and weigths for the next first layer so ZS will be ready to process the new image immediately

    //FC layers
    //TODO In next future CNN and FC layers will derive from same base class, so we can alternate FC and CNN layers
    //TODO In current release FC layers can be only at network's end

    //t_conv_end = std::chrono::high_resolution_clock::now();

    if (num_fc_layers > 0) {

        next_layer_input = remove_words_using_key(next_layer_input, zs_axi_bits::IDLE_MASK); //remove the termination signal from the fifo
        std::vector<int64_t> next_fc_input = decompress_sm_image_as_linear_vector(next_layer_input,
                zs_parameters::SPARSITY_MAP_WORD_NUM_BITS);

        //    t_compression_end = std::chrono::high_resolution_clock::now();

        for (int fc_layer_idx = 0; fc_layer_idx < num_fc_layers; fc_layer_idx++) {
            log_utilities::medium("Starting FC layer %d...", fc_layer_idx);
            next_fc_input = compute_fc_layer(next_fc_input, fc_layer_idx);

        }
        //We are not interested in probability distribution, so we return the position of the maximum that is the classification
        classification_result = std::distance(next_fc_input.begin(),
                std::max_element(next_fc_input.begin(), next_fc_input.end()));
        log_utilities::medium("Final activations: %lld %lld %lld %lld", next_fc_input[0],
                next_fc_input[1], next_fc_input[2], next_fc_input[3]);
    }

    t_fc_end = std::chrono::high_resolution_clock::now();

    //  duration = std::chrono::duration_cast < std::chrono::milliseconds
    //     > (t_conv_end - t_computation_start).count();
//
    // printf("Time CNN layers: %lf ms \n", duration);

    //  duration = std::chrono::duration_cast < std::chrono::milliseconds
//            > (t_compression_end - t_conv_end).count();

    //  printf("Time decompression layers: %lf ms \n", duration);
    // duration = std::chrono::duration_cast < std::chrono::milliseconds
    //     > (t_fc_end - t_compression_end).count();

    //  printf("Time FC layers: %lf ms \n", duration);

    duration = std::chrono::duration_cast < std::chrono::milliseconds > (t_fc_end - t_computation_start).count();
    //  printf("Total processing time: %lf ms \n", duration);
    double fps = (1.0f/(std::chrono::duration_cast < std::chrono::microseconds > (t_fc_end - end_previous_frame).count()))*1000.0f*1000.0f;
    end_previous_frame = t_fc_end;

    time_accumulator = time_accumulator + duration;
    double average_duration = time_accumulator / (total_num_processed_images + 1);


  //  printf("Average processing time: %lf ms - system FPS: %lf\n", average_duration,fps );


 //   if (total_num_processed_images % 200 == 0) {
   //     printf("\n\n****Checkpoint %d\n\n", total_num_processed_images);

    //}

#endif

#ifdef ENABLE_RESULT_MONITOR
    int monitor_classification = monitor.get_monitor_classification();
    if (monitor_classification != classification_result) {
        log_utilities::error("**ERROR: classification mismatch, NHP: %d, Monitor: %d",
                classification_result, monitor_classification);
    } else {
        log_utilities::none("Classification correct");
    }
#endif

#ifdef SOFTWARE_ONLY_MODE
    classification_result = monitor.get_monitor_classification();
#endif

//	printf("Time FC layers: %f ms \n", duration_avg_ms);
    std::string result_string;
    switch (classification_result) {
    case (0):
        result_string = "PAPER";
        break;
    case (1):
        result_string = "SCISSORS";
        break;
    case (2):
        result_string = "ROCK";
        break;
    case (3):
        result_string = "BACKGROUND";
        break;
    }
    log_utilities::none("Classification result: %d - %s", classification_result,
            result_string.c_str());
    log_utilities::high("Classification completed, pre-loading first layer data for next image...");

    log_utilities::high("Pre-load completed");
    return (classification_result); // added one because ZERO is NULL in the arduino code
}

//It assumes input is between 0 and 255 and needs to be normalized between 0 and 1
//The conversion is done in place directly on the class array to minimize data movement
//Code is optimized for computational speed rather than readability
void zs_driver::convert_input_image(int* l_image, int l_num_row, int l_total_num_pixel) {

    int input_pixel_idx, input_pixel_idx_incr;
    input_pixel_idx = 0;
    input_pixel_idx_incr = 1;

    log_utilities::debug("l_total_num_pixel %d", l_total_num_pixel);
    log_utilities::debug("axi_word_number %d", first_layer_num_axi_words);

    log_utilities::medium("Converting input image into internal format...");
    for (int output_pixel_idx = 0; output_pixel_idx < first_layer_num_axi_words;
            output_pixel_idx++) {

        first_layer_input[output_pixel_idx] = pixel_formatter.fast_2pixels_word_format(
                l_image[input_pixel_idx], l_image[input_pixel_idx_incr]);

        input_pixel_idx = input_pixel_idx + 2;
        input_pixel_idx_incr = input_pixel_idx_incr + 2;

    }

    log_utilities::debug("Checking for odd number of rows...");
    if (first_layer_pixels_per_row_odd == true) {
        log_utilities::debug("Odd row written");
        //input_pixel_idx incremented in last iteration of the for so already pointing to right place
        first_layer_input[first_layer_num_axi_words] = pixel_formatter.fast_1pixel_word_format(
                l_image[input_pixel_idx]);
    }
    first_layer_input[0] = pixel_formatter.set_new_row_flag(first_layer_input[0], 0); //for sure the first pixel starts a new row

    for (int row_idx = 0; row_idx < l_num_row - 1; row_idx++) {
        first_layer_input[first_layer_row_start_positions[row_idx]] =
                pixel_formatter.set_new_row_flag(
                        first_layer_input[first_layer_row_start_positions[row_idx]],
                        first_layer_row_start_positions_word_idx[row_idx]);

    }

    first_layer_input[first_layer_input.size() - 1] = pixel_formatter.format_word0((uint16_t) 1,
            (uint16_t) zs_parameters::REG_TYPE, (uint16_t) 1,
            (uint16_t) zs_address_space::config_image_load_done_pulse);

    log_utilities::debug("Conversion done.");
}

std::vector<uint64_t> zs_driver::compute_cnn_layer(std::vector<uint64_t> l_input, int layer_idx,
        int pass_idx) {

    if (layer_idx != 0 || pass_idx != 0) { //Data for first layer first pass are loaded at the end of previous pass
        load_config_biases_kernels(layer_idx, pass_idx);
    }
    if (pass_idx == 0) {
        load_image(l_input);
    } else {
        log_utilities::high("Multipass layer, no need to reload image");
    }
    return (backend_if.read());
}

//FC layers are currently computed in SW
//Notice that we are operating on both weight and pixel shifted left by MANTISSA_NUM_BITS, so the mult result is going to be shifted by 2 times this value
//In order to speedup the computation biases are read and shifted left by MANTISSA_NUM_BITS (thus realigned with pixels/weight mult result)
//and we just need to shift the result back after the computation
//TODO: Pooling currently not supported
std::vector<int64_t> zs_driver::compute_fc_layer(std::vector<int64_t> l_input, int layer_idx) {

    std::vector<int64_t> fc_output;
    int layer_num_output_channels = fc_network[layer_idx].num_output_channels;
    fc_output.resize(layer_num_output_channels);

    for (int kernel_idx = 0; kernel_idx < layer_num_output_channels; kernel_idx++) {

        fc_output[kernel_idx] = (inner_product(l_input.begin(), l_input.end(),
                fc_network[layer_idx].weights[kernel_idx].begin(),
                fc_network[layer_idx].biases[kernel_idx])) / zs_parameters::MANTISSA_RESCALE_FACTOR;
    }

    if (fc_network[layer_idx].relu_enabled == 1) {
        for (int kernel_idx = 0; kernel_idx < layer_num_output_channels; kernel_idx++) {
            if (fc_output[kernel_idx] < 0) {
                fc_output[kernel_idx] = 0;

            }
        }
    }

    return (fc_output);
}

void zs_driver::load_config_biases_kernels(int layer_idx, int pass_idx) {
    log_utilities::high("Starting loading of config, biases and kernels...");
    backend_if.write(cnn_network[layer_idx].get_load_array(pass_idx));
}

void zs_driver::load_image(std::vector<uint64_t> l_input) {
    log_utilities::high("Starting image load, number of words to send: %d", l_input.size());
    backend_if.write(&l_input);
}

bool zs_driver::read_network_from_file(std::string network_file_name) {
    log_utilities::low("Opening network file %s", network_file_name.c_str());
    FILE *l_net_file = fopen(network_file_name.c_str(), "r");

    if (l_net_file == NULL) {
        log_utilities::debug("File opening failed");
        throw std::invalid_argument("Failed attempt to read network file, impossible to proceed");

        return (false);
    } else {
        log_utilities::debug("File opened successfully");

        //Read number of layers
        total_num_layers = read_int_from_file(l_net_file);

        log_utilities::debug("Network structure initialized with %d layers", total_num_layers);

        //Create layers to be used
        for (int layer_idx = 0; layer_idx < total_num_layers; layer_idx++) {
            int layer_type = read_int_from_file(l_net_file);
            if (layer_type == 1) { //The layer has to run on accelerator
                zs_cnn_layer new_layer = zs_cnn_layer(layer_idx, l_net_file);
                cnn_network.push_back(new_layer);
                num_cnn_layers++;
            } else {
                zs_fc_layer new_layer = zs_fc_layer(layer_idx, l_net_file);
                fc_network.push_back(new_layer);
                num_fc_layers++;
            }

        }
        log_utilities::high("Number of layers - CNN: %d - FC: %d - Total: %d", num_cnn_layers,
                num_fc_layers, total_num_layers);
        log_utilities::medium(
                "All layer read, proceeding with first layer data structure preparation...");

        first_layer_num_pixels = cnn_network[0].get_uncompressed_input_image_num_pixels();
        first_layer_num_rows = cnn_network[0].get_input_num_rows();
        first_layer_pixels_per_row = cnn_network[0].get_pixels_per_row();
        first_layer_num_axi_words = first_layer_num_pixels >> 1;

        if (first_layer_pixels_per_row % 2 == 0)
            first_layer_pixels_per_row_odd = false;
        else
            first_layer_pixels_per_row_odd = true;

        first_layer_input.resize(
                first_layer_num_axi_words + (int) first_layer_pixels_per_row_odd + 1); //+1 for image load complete instruction

        for (int pixel_idx = 1; pixel_idx < first_layer_num_pixels; pixel_idx++) {
            if (pixel_idx % first_layer_pixels_per_row == 0) {
                first_layer_row_start_positions.push_back(pixel_idx >> 1);
                first_layer_row_start_positions_word_idx.push_back(pixel_idx % 2);
            }
        }

        log_utilities::debug("Preparation completed");
        fclose(l_net_file);
        log_utilities::debug("Network read from file done");
        return (true);
    }
}

#endif
