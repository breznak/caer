#ifndef __ZS_AXI_FORMATTER_H__
#define __ZS_AXI_FORMATTER_H__

#include "stdio.h"
#include <iostream>
#include <vector>
#include <cstdint>
#include "inttypes.h"
#include "string.h"


class zs_axi_formatter {
   private:

      std::vector<uint64_t> array;
      int entry_idx;
      uint64_t active_word;
      int word_idx;

      void initialize();
      uint64_t format_word_at_position(uint64_t l_old_word, int l_word_idx, int l_valid, int l_type, int l_address, int l_value);

   public:
      zs_axi_formatter();
      void append(int l_type, int l_address, int l_value);
      void append_new_word(int l_type, int l_address, int l_value);
      void flush_word();
      void append_empty();
      std::vector<uint64_t> get_array();
      // uint16_t int_to_short(int data);
       uint64_t fast_2pixels_word_format(int l_first_value, int l_second_value);
       uint64_t fast_1pixel_word_format(int l_first_value);
       uint64_t set_new_row_flag(uint64_t l_old_word, int slot);
       uint64_t format_word0(uint16_t l_short_value, uint16_t l_utype, uint16_t l_uvalid, uint16_t l_uaddress);
};

#endif
