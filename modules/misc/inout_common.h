#ifndef INPUT_OUTPUT_COMMON_H_
#define INPUT_OUTPUT_COMMON_H_

#include <stdatomic.h>
#include <unistd.h>

#define AEDAT3_NETWORK_HEADER_LENGTH 20
#define AEDAT3_NETWORK_MAGIC_NUMBER 0x1D378BC90B9A6658
#define AEDAT3_NETWORK_VERSION 0x01
#define AEDAT3_FILE_VERSION "3.1"

struct aedat3_network_header {
	int64_t magicNumber;
	int64_t sequenceNumber;
	int8_t versionNumber;
	int8_t formatNumber;
	int16_t sourceNumber;
}__attribute__((__packed__));

#endif /* INPUT_OUTPUT_COMMON_H_ */
