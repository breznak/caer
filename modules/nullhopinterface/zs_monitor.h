/*
 * zs_monitor.h
 *
 *  Created on: Nov 10, 2016
 *      Author: asa
 */

#ifndef ZS_MONITOR_H_
#define ZS_MONITOR_H_

#include "zs_cnn_layer.h"
#include "zs_monitor_cnn_layer.h"


class zs_monitor {
   public:
      zs_monitor(std::string filename);
      zs_monitor();
      void classify_image(std::vector<uint64_t> image);
      void check_layer_activations(std::vector<uint64_t> activations, int layer_idx);
      int get_monitor_classification();

   private:
      int cnn_num_layers;
      bool read_network_from_file(std::string network_file_name);
      std::vector<zs_monitor_cnn_layer> cnn_kernels;
      std::vector<std::vector<std::vector<std::vector<int64_t>>> >monitor_activations;
      void write_activations_to_file( std::vector<std::vector<std::vector<std::vector<int64_t>>> > l_activations);
      std::vector<std::vector<std::vector<int64_t>>>image_1d_to_3d(std::vector<uint64_t> l_image, int num_rows, int num_columns, int num_channels);

      std::vector<std::vector<std::vector<int64_t>>>compute_layer(std::vector<std::vector<std::vector<int64_t>>> layer_input, zs_monitor_cnn_layer layer_parameters);
      std::vector<std::vector<std::vector<int64_t>>>compute_convolution(std::vector<std::vector<std::vector<int64_t>>> layer_input, zs_monitor_cnn_layer layer_parameters);
      std::vector<std::vector<std::vector<int64_t>>>compute_pooling(std::vector<std::vector<std::vector<int64_t>>> layer_input);
};

#endif /* ZS_MONITOR_H_ */
