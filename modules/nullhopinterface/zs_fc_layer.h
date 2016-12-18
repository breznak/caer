#ifndef __zs_fc_layer_h__
#define __zs_fc_layer_h__


#include "stdio.h"
#include "string.h"
#include "cstdint"
#include "inttypes.h"
#include <unistd.h>
#include <vector>

class zs_fc_layer {

   public:

      zs_fc_layer(int layer_idx, FILE* net_file);

      std::vector<std::vector<int>> weights;
      std::vector<int> biases;
      int num_output_channels;
      int pooling_enabled;
      int relu_enabled;
   private:
      int layer_idx;
      int compression_enabled;
      int kernel_side;
      int num_input_channels;
      int num_input_columns;
      int num_input_rows;


      int padding;
      int num_weight;
      int num_biases;

      void set_layer_idx(int l_layer_idx);
      void set_compression_enabled(int l_compression_enabled);
      void set_kernel_size(int l_kernel_size);
      void set_num_input_channels(int l_num_input_channels);
      void set_num_input_rows(int l_num_input_rows);
      void set_num_input_columns(int l_num_input_column);
      void set_num_output_channels(int l_num_output_channels);
      void set_pooling_enabled(int l_pooling_enabled);
      void set_relu_enabled(int l_relu_enabled);
      void set_padding(int l_padding);
      void set_num_weight(int l_num_weight);
      void set_num_biases(int l_num_biases);
      int get_input_num_rows();

      bool read_layer_from_file(FILE* l_net_file, int l_layer_idx);
      void set_layer_config(FILE*l_net_file, int l_layer_idx);
      void initialize_layer(int l_layer_idx, int l_compression_enabled, int l_kernel_size, int l_num_input_channels,
               int l_num_input_columns, int l_num_input_rows, int l_num_output_channels, int l_pooling_enabled,
               int l_relu_enabled, int l_padding, int l_num_weight, int l_num_biases);
      std::vector<int> read_biases(FILE* l_net_file);
      std::vector<std::vector<int>> read_weights(FILE* l_net_file);
};

#endif
