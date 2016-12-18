/*
 * zs_top_level_sw_pkg.cpp
 *
 *  Created on: Nov 1, 2016
 *      Author: asa
 */
#ifndef __ZS_TOP_LEVEL_SW_PKG__
#define __ZS_TOP_LEVEL_SW_PKG__
#include "stdio.h"
#include "string.h"
#include "cstdint"
#include "inttypes.h"
#include <cmath>

namespace zs_parameters {

   static const int MANTISSA_NUM_BITS = 8;
   static const int MANTISSA_RESCALE_FACTOR = std::pow(2, zs_parameters::MANTISSA_NUM_BITS);

   static const int NUM_MACS = 128;
   static const int NUM_MACS_PER_CLUSTER = 16;

   static const int REG_TYPE = 3;
   static const int KER_TYPE = 2;
   static const int IMG_TYPE = 1;
   static const int BIAS_TYPE = 0;

   static const int SPARSITY_MAP_WORD_NUM_BITS = 16;
}

namespace zs_address_space {

   static const int num_registers = 22;

   static const int config_image_compression_enabled = 0;
   static const int config_pre_sm_counter_max = 1;
   static const int config_kernel_size = 2;
   static const int config_num_input_channels = 3;
   static const int config_num_input_column = 4;
   static const int config_num_input_rows = 5;
   static const int config_num_output_channels = 6;
   static const int config_pooling_enabled = 7;
   static const int config_relu_enabled = 8;
   static const int config_contiguous_kernels = 9;
   static const int config_num_macs_per_channel = 10;
   static const int config_input_channel_decode_jump_mask = 11;
   static const int config_kernel_memory_write_complete_pulse = 12;
   static const int config_kernel_memory_resetn_pulse = 13;
   static const int config_image_load_done_pulse = 14;
   static const int config_layer_channel_offset = 16;
   static const int config_first_conv_layer = 17;
   static const int config_pixel_memory_loop_offset = 18;
   static const int config_start_process_pulse = 19;
   static const int config_image_in_memory = 20;
   static const int config_row_column_offset = 21;

   static const int config_image_start_new_row_instr = 1;

}

namespace zs_axi_bits {
   static const int ADDRESS_SIZE = 7;
   static const int TYPE_SIZE = 2;
   static const int VALID_SIZE = 2;
   static const int VALUE_SIZE = 16;
   static const int BURST_SIZE = 13;

   static const int SECOND_VALUE_SHIFT = 16;
   static const int TYPE_VALUE_SHIFT = 32;
   static const int FIRST_VALID_SHIFT = 34;
   static const int SECOND_VALID_SHIFT = 35;
   static const int FIRST_ADDR_SHIFT = 36;
   static const int SECOND_ADDR_SHIFT = 43;
   static const int BURST_VALUE_SHIFT = 50;
   static const int IDLE_SHIFT = 63;

//Input Masks
   static const uint64_t FIRST_VALUE_MASK = (uint64_t) (std::pow(2, VALUE_SIZE) - 1);
   static const uint64_t SECOND_VALUE_MASK = (uint64_t) ((uint64_t) (std::pow(2, VALUE_SIZE) - 1) << SECOND_VALUE_SHIFT);
   static const uint64_t TYPE_MASK = (uint64_t) ((uint64_t) (std::pow(2, TYPE_SIZE) - 1) << TYPE_VALUE_SHIFT);
   static const uint64_t VALID_MASK = (uint64_t) ((uint64_t) (std::pow(2, VALID_SIZE) - 1) << FIRST_VALID_SHIFT);
   static const uint64_t FIRST_VALID_MASK = (uint64_t) ((uint64_t) 1) << (FIRST_VALID_SHIFT);
   static const uint64_t SECOND_VALID_MASK = (uint64_t) ((uint64_t) 1) << (SECOND_VALID_SHIFT);
   static const uint64_t FIRST_ADDRESS_MASK = (uint64_t) ((uint64_t) (std::pow(2, ADDRESS_SIZE) - 1) << FIRST_ADDR_SHIFT);
   static const uint64_t SECOND_ADDRESS_MASK = (uint64_t) ((uint64_t) (std::pow(2, ADDRESS_SIZE) - 1) << SECOND_ADDR_SHIFT);
   static const uint64_t BURST_VALUE_MASK = (uint64_t) ((uint64_t) (std::pow(2, BURST_SIZE) - 1) << BURST_VALUE_SHIFT);

   static const uint64_t IDLE_MASK = (uint64_t) std::pow(2, IDLE_SHIFT);

   static const uint64_t ZS_ALL_INPUT_MASK = FIRST_VALUE_MASK | SECOND_VALUE_MASK | TYPE_MASK | VALID_MASK | FIRST_VALID_MASK
            | SECOND_VALID_MASK | FIRST_ADDRESS_MASK | SECOND_ADDRESS_MASK;

//static const uint64_t VALID_MASK = 2 ^ 34 + 2 ^ 35;
}

namespace axi_parameters {
   static const int AXI_WIDTH = 64;
   static const unsigned int AXI_TRANSFER_LENGTH_BYTES = 0x400;


   static const unsigned int AXI_TRANSFER_LENGTH_WORDS = axi_parameters::AXI_TRANSFER_LENGTH_BYTES
            / (axi_parameters::AXI_WIDTH / 8);
   static const uint64_t BURST_VALUE_SHIFTED = (uint64_t) ((uint64_t) AXI_TRANSFER_LENGTH_WORDS)
            << (zs_axi_bits::BURST_VALUE_SHIFT + 1);
   static const uint64_t BURST_VALUE_SHIFTED_VALID = BURST_VALUE_SHIFTED | (uint64_t) 1 << zs_axi_bits::BURST_VALUE_SHIFT;


}

#endif
