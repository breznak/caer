#ifndef __ZS_AXI_FORMATTER__
#define __ZS_AXI_FORMATTER__

#include "zs_axi_formatter.h"
#include "zs_top_level_sw_pkg.cpp"
#include "npp_std_func_sw_pkg.cpp"
#include "stdio.h"
#include <iostream>
#include <vector>
#include <cstdint>
#include "inttypes.h"
#include "string.h"

zs_axi_formatter::zs_axi_formatter() {
   initialize();
}

void zs_axi_formatter::initialize() {
   word_idx = 0;
   entry_idx = 0;
   active_word = 0;
}

std::vector<uint64_t> zs_axi_formatter::get_array() {
   //We have to write the half word to the array before sending it out
   if (word_idx == 1) {
      array.push_back(active_word);
      active_word = 0;
      word_idx = 0;
   }

   return (array);
}

//d function for high performance
//-------------------------------------

 uint64_t zs_axi_formatter::fast_2pixels_word_format(int l_first_value, int l_second_value) {
   return ((uint64_t) int_to_short(l_first_value) | (uint64_t) int_to_short(l_second_value) << zs_axi_bits::SECOND_VALUE_SHIFT
         | (uint64_t) zs_parameters::IMG_TYPE << zs_axi_bits::TYPE_VALUE_SHIFT | (uint64_t) 1 << zs_axi_bits::FIRST_VALID_SHIFT
         | (uint64_t) 1 << zs_axi_bits::SECOND_VALID_SHIFT);
}

 uint64_t zs_axi_formatter::fast_1pixel_word_format(int l_first_value) {
   return ((uint64_t) int_to_short(l_first_value) | (uint64_t) zs_parameters::IMG_TYPE << zs_axi_bits::TYPE_VALUE_SHIFT
         | (uint64_t) 1 << zs_axi_bits::FIRST_VALID_SHIFT);
}

 uint64_t zs_axi_formatter::set_new_row_flag(uint64_t l_old_word, int slot) {
   if (slot == 0) {
      return (l_old_word | (uint64_t) zs_address_space::config_image_start_new_row_instr << zs_axi_bits::FIRST_ADDR_SHIFT);
   } else {
      return (l_old_word | (uint64_t) zs_address_space::config_image_start_new_row_instr << zs_axi_bits::SECOND_ADDR_SHIFT);
   }
}

//This function assumes the array already exist and need to be extended to insert the new data
//Is a low performance function to be used only in one-run cases and not for continuous runs (e.g. to be used for kernels and not for pixels)
void zs_axi_formatter::append(int l_type, int l_address, int l_value) {
   //if (word_idx == 0) // if word idx = 0, reset word
   //  active_word = 0;

   active_word = format_word_at_position(active_word, word_idx, 1, l_type, l_address, l_value);

   if (word_idx == 1) {
      array.push_back(active_word);
      active_word = 0;
      word_idx = 0;
   } else {
      word_idx = 1;
   }

}

void zs_axi_formatter::append_empty() {
   uint64_t new_word, old_word;
   old_word = (uint64_t) active_word * word_idx; //If we are at word idx == 0, in this way the word is reset. If word_idx == 1, is kept
   new_word = 0;

   if (word_idx == 0) {
      array.push_back(new_word);
   } else {
      array.push_back(active_word);
      array.push_back(new_word);
      active_word = 0;
      word_idx = 0;
   }

}

void zs_axi_formatter::flush_word() {

   if (word_idx == 1) {
      array.push_back(active_word);
      active_word = 0;
      word_idx = 0;
   }

}




void zs_axi_formatter::append_new_word(int l_type, int l_address, int l_value) {
   uint64_t new_word, old_word;
   old_word = (uint64_t) active_word * word_idx; //If we are at word idx == 0, in this way the word is reset. If word_idx == 1, is kept
   new_word = format_word_at_position(old_word, word_idx, 1, l_type, l_address, l_value);

   if (word_idx == 0) {
      array.push_back(new_word);
   } else {
      array.push_back(active_word);
      array.push_back(new_word);
      active_word = 0;
      word_idx = 0;
   }
}

uint64_t zs_axi_formatter::format_word_at_position(uint64_t l_old_word, int l_word_idx, int l_valid, int l_type, int l_address,
      int l_value) {
   uint64_t l_formatted_word = 0;
   uint16_t l_short_value = 0;
   uint16_t l_uvalid, l_utype, l_uaddress;
   l_uvalid = (uint16_t) l_valid;
   l_utype = (uint16_t) l_type;
   l_uaddress = (uint16_t) l_address;
   l_short_value = int_to_short(l_value);

   if (l_word_idx == 0) {
      l_formatted_word = format_word0(l_short_value, l_utype, l_uvalid, l_uaddress);
   } else {

      l_formatted_word = (uint64_t) l_old_word | (uint64_t) l_short_value << zs_axi_bits::SECOND_VALUE_SHIFT
            | (uint64_t) l_uvalid << zs_axi_bits::SECOND_VALID_SHIFT | (uint64_t) l_uaddress << zs_axi_bits::SECOND_ADDR_SHIFT;
   };

   /* static bool check = false;
    if (check == false) {
    log_utilities::error("*********************\n %llu", (unsigned long long int) l_formatted_word);
    std::bitset<64> bit_repr(l_formatted_word);
    log_utilities::error("%s", bit_repr.to_string().c_str());

    if (l_word_idx == 1)
    check = true;
    }*/

   return (l_formatted_word);

}

 uint64_t zs_axi_formatter::format_word0(uint16_t l_short_value, uint16_t l_utype, uint16_t l_uvalid, uint16_t l_uaddress) {

   return ((uint64_t) l_short_value | (uint64_t) l_utype << zs_axi_bits::TYPE_VALUE_SHIFT
         | (uint64_t) l_uvalid << zs_axi_bits::FIRST_VALID_SHIFT | (uint64_t) l_uaddress << zs_axi_bits::FIRST_ADDR_SHIFT);

}



#endif
