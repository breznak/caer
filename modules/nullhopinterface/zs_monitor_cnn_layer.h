/*
 * zs_monitor_cnn_layer.h
 *
 *  Created on: Nov 10, 2016
 *      Author: asa
 */

#ifndef ZS_MONITOR_CNN_LAYER_H_
#define ZS_MONITOR_CNN_LAYER_H_

#include <vector>
#include "stdio.h"
#include "string.h"
#include "cstdint"
#include "inttypes.h"
#include <unistd.h>

class zs_monitor_cnn_layer {

   public:
      zs_monitor_cnn_layer(FILE *l_net_file);
      int layer_type;
      int compression_enabled;
      int kernel_side;
      int num_input_channels;
      int num_input_columns;
      int num_input_rows;
      int num_output_channels;
      int pooling_enabled;
      int relu_enabled;
      int padding;

      int num_output_columns;
      int num_output_rows;

      std::vector<std::vector<std::vector<std::vector<int64_t>>> >weights;
      std::vector<int64_t> biases;

      private:
      void read_layer_from_file(FILE* l_net_file);
      void read_layer_config(FILE*l_net_file);
      std::vector<std::vector<std::vector<std::vector<int64_t>>> > read_weights(FILE* l_net_file);
      std::vector<int64_t> read_biases(FILE* l_net_file);
   };

#endif /* ZS_MONITOR_CNN_LAYER_H_ */
