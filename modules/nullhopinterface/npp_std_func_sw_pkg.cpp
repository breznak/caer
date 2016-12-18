#ifndef __NPP_STD_FUNC_SW_PKG__
#define __NPP_STD_FUNC_SW_PKG__

#include "npp_log_utilities.h"
#include "zs_top_level_sw_pkg.cpp"

#include "stdio.h"
#include <tuple>

#include <vector>

//Read next integer in file and discard everything until new line
inline int read_int_from_file(FILE*file) {
    int value;
    fscanf(file, "%d%*[^\n]", &value);
    return (value);

}

inline uint16_t int_to_short(int data) {
    uint16_t newData;

    newData = (uint16_t) data;
    if (data < 0) {
        newData = (uint16_t) ~(data - 1);
        newData = (~newData) + 1;
    }
    return (newData);

}
//Increment indices in order to loop over 3d images in range 0-(MAX-1)
//Implemented as ifs since faster than series of divisions
inline std::tuple<int, int, int> update_3d_indices(int index0, int index1, int index2,
        int max_index0, int max_index1, int max_index2) {

    index0++;

    if (index0 == max_index0) {
        index0 = 0;
        index1++;
    }

    if (index1 == max_index1) {
        index1 = 0;
        index2++;
    }

    if (index2 == max_index2) {
        index2 = 0;
        log_utilities::debug("Overflowing index increase");
    }

    return (std::make_tuple(index0, index1, index2));
}

inline std::vector<uint64_t> remove_words_using_key(std::vector<uint64_t> array, uint64_t mask) {

    for (int entry_idx = 0; entry_idx < array.size(); entry_idx++) {

        if ((array[entry_idx] & mask) != 0) {
            array.erase(array.begin() + entry_idx);
            entry_idx--; //we decrease entry idx to avoid skipping one value when erasing

        }

    }
    return (array);
}

//Increment indices in order to loop over 4d images in range 0-(MAX-1)
//Implemented as ifs since faster than series of divisions
//Usually called as channel - column - row
inline std::tuple<int, int, int, int> update_4d_indices(int index0, int index1, int index2,
        int index3, int max_index0, int max_index1, int max_index2, int max_index3) {

    index0++;

    if (index0 == max_index0) {
        index0 = 0;
        index1++;
    }

    if (index1 == max_index1) {
        index1 = 0;
        index2++;
    }

    if (index2 == max_index2) {
        index2 = 0;
        index3++;
    }
    if (index3 == max_index3) {
        index3 = 0;
        // log_utilities::error("Overflowing index increase");
    }

    return (std::make_tuple(index0, index1, index2, index3));
}

//Not most efficient algorithm, but simpliest
inline int count_ones(uint64_t value) {
    unsigned int count = 0;
    while (value > 0) {           // until all bits are zero
        if ((value & 1) == 1)     // check lower bit
            count++;
        value >>= 1;              // shift bits, removing lower bit
    }
    return (count);
}

//Not most efficient algorithm, but simpliest
inline int count_zeros_until_first_one(uint64_t value) {
    unsigned int count = 0;
    while (value > 0) {           // until all bits are zero
        if ((value & 1) == 1)     // check lower bit
            return (count);
        count++;
        value >>= 1;              // shift bits, removing lower bit
    }
    return (count);
}

inline std::tuple<int16_t, int, int> get_next_word(std::vector<uint64_t> activations, int activ_idx,
        int word_idx) {

    uint64_t activ = activations[activ_idx];
    int16_t first_value = (int16_t) (activ & zs_axi_bits::FIRST_VALUE_MASK);
    int16_t second_value = (int16_t) ((activ & zs_axi_bits::SECOND_VALUE_MASK)
            >> zs_axi_bits::SECOND_VALUE_SHIFT);

    uint64_t first_valid = (uint64_t) activ & zs_axi_bits::FIRST_VALID_MASK;
    uint64_t second_valid = (uint64_t) activ & zs_axi_bits::SECOND_VALID_MASK;

    if (activ_idx < activations.size()) {
        if (word_idx == 0) {
            if (first_valid != 0) { //different from 0 rather than equal to 1 to save a shift

                return (std::make_tuple((int16_t) first_value, (int) activ_idx, (int) 1)); //word idx returned is 1 so we know we are going to read that word next time
            } else {
                return (get_next_word(activations, activ_idx + 1, 0)); //word invalid, move to next one
            }
        } else {
            if (second_valid != 0) { //different from 0 rather than equal to 1 to save a shift

                return (std::make_tuple((int16_t) second_value, (int) activ_idx + 1, 0));
            } else {
                return (get_next_word(activations, activ_idx + 1, 0));
            }
        }
    } else {
        return (std::make_tuple((int16_t) 0, (int) activations.size(), (int) 0));
    }
}

//This function return 64 bit data since we need to shift the value left by MANTISSA_NUM_BITS anyway.
inline std::vector<std::vector<std::vector<int64_t>>>decompress_sm_image(std::vector<uint64_t> input, int num_rows, int num_columns,
        int num_channels, int sm_length) {

    log_utilities::debug("Starting image decompression...");

    std::vector<std::vector<std::vector<int64_t>>> output_image;

    uint16_t current_sm =0;
    int in_word_idx = 0;
    int row = 0;
    int column = 0;
    int channel = 0;
    int sm_position = sm_length;
    int input_word_idx = 0; //the full get_next_word function updates also input_idx, but in this case we dont need it so we save it into an unused variable

    output_image.resize(num_rows);
    for (int row_idx = 0; row_idx < num_rows; row_idx++) {
        output_image[row_idx].resize(num_columns);
        for(int column_idx = 0; column_idx < num_columns; column_idx++) {
            output_image[row_idx][column_idx].resize(num_channels);
        }
    }

    log_utilities::debug("Image placeholder generated");
    log_utilities::debug("Total number of words: %d, num_rows: %d, num_columns: %d, num_channels: %d",input.size(),num_rows, num_columns,
            num_channels);

    uint64_t address;

    //  log_utilities::debug("input[0] %llu, input[1] %llu, input[2] %llu, input[3] %llu, input[4] %llu", input[0], input[1], input[2], input[3], input[4]);

    // log_utilities::debug("num_row %d, num_col %d, num_ch %d",num_rows , num_columns, num_channels);

    while (input_word_idx < input.size()) {
        int16_t next_word;
        std::tie(next_word, input_word_idx, in_word_idx) = get_next_word(input, input_word_idx, in_word_idx);

        if (in_word_idx == 1) { //Index is inverted since in_word_idx is pointing at NEXT position to be read, not current one
            address = input[input_word_idx] & (zs_axi_bits::FIRST_ADDRESS_MASK);
        } else {
            address = input[input_word_idx] & (zs_axi_bits::SECOND_ADDRESS_MASK);
        }

        // log_utilities::debug("next_word %d, input_word_idx %d, in_word_idx %d - row %d, col %d, ch %d", next_word , input_word_idx, in_word_idx, row , column, channel);

        if (sm_position == sm_length) { //the word is a sm

            //   log_utilities::debug("New sm detected (mode1) %d", next_word);
            current_sm = next_word;
            sm_position = 0;

            //check if new row set properly
            if (column == 0 && channel == 0) {

                if (address != 0) {
                    log_utilities::debug("New row start flag correctly matched");
                } else {
                    if (input_word_idx == input.size() -1 ) { //To avoid false error at last word due to zero fillers
                        log_utilities::error("**ERROR: Missing new row flag at row %d", row);
                        throw ".";
                    }
                }

            }

            if (current_sm == 0) { //new sm is 0

                for (int sm_idx = 0; sm_idx < sm_length; sm_idx++) {
                    std::tie(channel, column, row) = update_3d_indices(channel, column, row, num_channels,num_columns, num_rows);

                }
                sm_position = sm_length;
            }

        } else {

            //get coordinates
            for (int sm_idx = sm_position; sm_idx < sm_length; sm_idx++) {
                if ((current_sm & (1 << sm_idx)) != 0) {
                    current_sm = current_sm & (~(1 << sm_idx));
                    sm_position = sm_idx+1;
                    int64_t new_value = next_word;         //cast to int64_t
                    output_image[row][column][channel] = new_value;

                    if (new_value == 0) {
                        log_utilities::error("**ERROR: Zero pixel found in compressed image at position: %d %d %d ",row, column, channel);

                    }

                    if (address != 0) {
                        log_utilities::error("**ERROR: New row flag asserted on pixel instead of sm - coord: row: %d column: %d channel: %d", row, column, channel);

                    }

                    std::tie(channel, column, row) = update_3d_indices(channel, column, row, num_channels,num_columns, num_rows);

                    break;
                } else {
                    sm_position = sm_idx+1;
                    std::tie(channel, column, row) = update_3d_indices(channel, column, row, num_channels,num_columns, num_rows);

                    if (sm_position == sm_length) {
//
                        //   log_utilities::debug("New sm detected (mode2) %d", next_word);
                        current_sm = next_word;
                        sm_position = 0;

                        //check if new row set properly
                        if (column == 0 && channel == 0) {

                            if (address != 0) {
                                log_utilities::debug("New row start flag correctly matched");
                            } else {
                                if (input_word_idx == input.size() -1 ) { //To avoid false error at last word due to zero fillers
                                    log_utilities::error("**ERROR: Missing new row flag at row %d", row);

                                }
                            }

                        }

                        if (current_sm == 0) { //new sm is 0

                            for (int sm_idx = 0; sm_idx < sm_length; sm_idx++) {
                                std::tie(channel, column, row) = update_3d_indices(channel, column, row, num_channels,num_columns, num_rows);

                            }
                            sm_position = sm_length;
                        }

                    }
                }

            }

        }
    }

    log_utilities::debug("Decompression done");
    return(output_image);
}

//Values are reshifted into real value
inline std::vector<int64_t> decompress_sm_image_as_linear_vector(std::vector<uint64_t> input,
        int sm_length) {

    log_utilities::debug("Starting image decompression...");

    std::vector<int64_t> output_image;

    uint16_t current_sm = 0;
    int in_word_idx = 0;
    int sm_position = sm_length;
    int input_word_idx = 0; //the full get_next_word function updates also input_idx, but in this case we dont need it so we save it into an unused variable
    int input_size;
    input_size = input.size();
    do {
        int16_t next_word;
        std::tie(next_word, input_word_idx, in_word_idx) = get_next_word(input, input_word_idx,
                in_word_idx);

        if (sm_position == sm_length) { //the word is a sm

            current_sm = next_word;
            sm_position = 0;

            if (current_sm == 0) { //new sm is 0

                //It means there are zeros at the end of the input array so we dont need to update the SM and we have done
                if (input_word_idx == input_size) {
                    break;
                }

                for (int sm_idx = sm_position; sm_idx < sm_length; sm_idx++) {
                    output_image.push_back(0);
                }
                sm_position = sm_length;
            }

        } else {

            //get coordinates
            for (int sm_idx = sm_position; sm_idx < sm_length; sm_idx++) {
                if ((current_sm & (1 << sm_idx)) != 0) {
                    current_sm = current_sm & (~(1 << sm_idx));
                    sm_position = sm_idx + 1;
                    int64_t new_value = next_word;         //cast to int64_t
                    output_image.push_back(new_value);

                    break;         // TODO performance can be improved removing this break
                } else {
                    sm_position = sm_idx + 1;
                    output_image.push_back(0);

                    if (sm_position == sm_length) {

                        //log_utilities::debug("New sm detected (mode2) %d", next_word);
                        current_sm = next_word;
                        sm_position = 0;

                        if (current_sm == 0) { //new sm is 0
                            //It means there are zeros at the end of the input array so we dont need to update the SM and we have done
                            if (input_word_idx == input_size) {
                                break;
                            }
                            for (int sm_idx = sm_position; sm_idx < sm_length; sm_idx++) {
                                output_image.push_back(0);
                            }
                            sm_position = sm_length;
                        }
                    }
                }
            }
        }
    } while (input_word_idx < input_size);


    log_utilities::debug("Decompression done");
    return (output_image);
}

#endif

