#ifndef __AXIDMA_CPP__
#define __AXIDMA_CPP__
#include "axidmalib.hpp"
#include <stdio.h>
#include <exception>
#include <stdexcept>
Axidma::Axidma(unsigned int axidma_addr_offset, unsigned int source_addr_offset,
		unsigned int destination_addr_offset) :
		MM2S_CONTROL_REGISTER(0x00), MM2S_STATUS_REGISTER(0x04), MM2S_START_ADDRESS(
				0x18), MM2S_LENGTH(0x28), S2MM_CONTROL_REGISTER(0x30), S2MM_STATUS_REGISTER(
				0x34), S2MM_DESTINATION_ADDRESS(0x48), S2MM_LENGTH(0x58), AXIDMA_ADDR_OFFSET(
				axidma_addr_offset), DESTINATION_ADDR_OFFSET(
				destination_addr_offset), SOURCE_ADDR_OFFSET(
				source_addr_offset), RUNNING(0x00000000), HALTED(0x00000001), IDLE(
				0x00000002), SGINCLD(0x00000008), DMAINTERR(0x00000010), DMASLVERR(
				0x00000020), DMADECERR(0x00000040), SGINTERR(0x00000100), SGSLVERR(
				0x00000200), SGDECERR(0x00000400), IOC_IRQ(0x00001000), DLY_IRQ(
				0x00002000), ERR_IRQ(0x00004000), MAX_READ_TRANSFER_LENGTH(
				0x1FFF),max_write_transfer_length_bytes((unsigned int) pow(2, 23)) {
	whole_memory_pointer = open("/dev/mem", O_RDWR | O_SYNC); // Open /dev/mem which represents the whole physical memory
	axidma_map_addr = (unsigned int *) mmap(NULL, 65535, PROT_READ | PROT_WRITE,
			MAP_SHARED, whole_memory_pointer, AXIDMA_ADDR_OFFSET); // Memory map AXI Lite register block
	source_addr = (uint64_t*) mmap(NULL,
			AXIDMA_MEMORY_MAPPING_WRITE_SIZE_DEFINE, PROT_READ | PROT_WRITE,
			MAP_SHARED, whole_memory_pointer, SOURCE_ADDR_OFFSET); // Memory map source address
	destination_addr = (uint64_t*) mmap(NULL,
			AXIDMA_MEMORY_MAPPING_READ_SIZE_DEFINE, PROT_READ | PROT_WRITE,
			MAP_SHARED, whole_memory_pointer, DESTINATION_ADDR_OFFSET); // Memory map destination address
	axidma_channel_timeout_us = 50*1000; //50 ms timeout
	read_transfer_length_bytes = 0x100;

}

Axidma::~Axidma(void) {
	munmap(axidma_map_addr, 65535);
	munmap(source_addr, AXIDMA_MEMORY_MAPPING_WRITE_SIZE_DEFINE);
	munmap(destination_addr, AXIDMA_MEMORY_MAPPING_READ_SIZE_DEFINE);
}

bool Axidma::init(unsigned int trans_read_length_bytes) {
	if (trans_read_length_bytes > MAX_READ_TRANSFER_LENGTH) {
		fprintf(stderr, "Error: The maximum read transfer length is %dWords\n",
				MAX_READ_TRANSFER_LENGTH);
		return (false);
	}

	read_transfer_length_bytes = trans_read_length_bytes;

	set_dma_register_value(MM2S_START_ADDRESS, SOURCE_ADDR_OFFSET);
	set_dma_register_value(MM2S_CONTROL_REGISTER, 0xf001);

	set_dma_register_value(S2MM_DESTINATION_ADDRESS, DESTINATION_ADDR_OFFSET);
	set_dma_register_value(S2MM_CONTROL_REGISTER, 0xf001);

	return (true);
}

void Axidma::set_dma_register_value(int register_offset, unsigned int value) {
	axidma_map_addr[register_offset >> 2] = value;
}

unsigned int Axidma::get_dma_register_value(int register_offset) {
	return (axidma_map_addr[register_offset >> 2]);
}

void Axidma::print_mm2s_status() {
	unsigned int status = get_dma_register_value(MM2S_STATUS_REGISTER);
	printf("Memory-mapped to stream status (0x%08x@0x%02x):", status,
			MM2S_STATUS_REGISTER);
	if (status & 0x00000001)
		printf(" halted");
	else
		printf(" running");
	if (status & 0x00000002)
		printf(" idle");
	if (status & 0x00000008)
		printf(" SGIncld");
	if (status & 0x00000010)
		printf(" DMAIntErr");
	if (status & 0x00000020)
		printf(" DMASlvErr");
	if (status & 0x00000040)
		printf(" DMADecErr");
	if (status & 0x00000100)
		printf(" SGIntErr");
	if (status & 0x00000200)
		printf(" SGSlvErr");
	if (status & 0x00000400)
		printf(" SGDecErr");
	if (status & 0x00001000)
		printf(" IOC_Irq");
	if (status & 0x00002000)
		printf(" Dly_Irq");
	if (status & 0x00004000)
		printf(" Err_Irq");
	printf("\n");
}

void Axidma::print_s2mm_status() {
	unsigned int status = get_dma_register_value(S2MM_STATUS_REGISTER);
	printf("Stream to memory-mapped status (0x%08x@0x%02x):", status,
			S2MM_STATUS_REGISTER);
	if (status & 0x00000001)
		printf(" halted");
	else
		printf(" running");
	if (status & 0x00000002)
		printf(" idle");
	if (status & 0x00000008)
		printf(" SGIncld");
	if (status & 0x00000010)
		printf(" DMAIntErr");
	if (status & 0x00000020)
		printf(" DMASlvErr");
	if (status & 0x00000040)
		printf(" DMADecErr");
	if (status & 0x00000100)
		printf(" SGIntErr");
	if (status & 0x00000200)
		printf(" SGSlvErr");
	if (status & 0x00000400)
		printf(" SGDecErr");
	if (status & 0x00001000)
		printf(" IOC_Irq");
	if (status & 0x00002000)
		printf(" Dly_Irq");
	if (status & 0x00004000)
		printf(" Err_Irq");
	printf("\n");
}

bool Axidma::check_mm2s_status(unsigned int chk_status) {
	unsigned int status = get_dma_register_value(MM2S_STATUS_REGISTER);
	if (chk_status == RUNNING && (status & HALTED) == 0)
		return (true);
	if (status & chk_status)
		return (true);
	return (false);
}

bool Axidma::check_s2mm_status(unsigned int chk_status) {
	unsigned int status = get_dma_register_value(S2MM_STATUS_REGISTER);
	if (chk_status == RUNNING && (status & HALTED) == 0)
		return (true);
	if (status & chk_status)
		return (true);
	return (false);
}

void Axidma::mm2s_pooling_sync(void) throw (AXIDMA_timeout_exception) {
	unsigned int status = get_dma_register_value(S2MM_STATUS_REGISTER);
	std::chrono::high_resolution_clock::time_point t1 =
			std::chrono::high_resolution_clock::now();
	while (!(check_mm2s_status(IOC_IRQ)) || !(check_mm2s_status(IDLE))) {
		std::chrono::high_resolution_clock::time_point t2 =
				std::chrono::high_resolution_clock::now();
		unsigned int time_span = std::chrono::duration_cast < time_span_us
				> (t2 - t1).count();
		if (time_span > axidma_channel_timeout_us) {
			throw axi_channel_timeout_excep;

		}
		status = get_dma_register_value(MM2S_STATUS_REGISTER);
	}
}

void Axidma::s2mm_pooling_sync(void) throw (AXIDMA_timeout_exception) {
	unsigned int status = get_dma_register_value(S2MM_STATUS_REGISTER);
	std::chrono::high_resolution_clock::time_point t1 =
			std::chrono::high_resolution_clock::now();
	while (!(check_s2mm_status(IOC_IRQ)) || !(check_s2mm_status(IDLE))) {
		std::chrono::high_resolution_clock::time_point t2 =
				std::chrono::high_resolution_clock::now();
		unsigned int time_span = std::chrono::duration_cast < time_span_us
				> (t2 - t1).count();
		if (time_span > axidma_channel_timeout_us) {
			throw axi_channel_timeout_excep;
		}
		status = get_dma_register_value(S2MM_STATUS_REGISTER);
	}
}

void Axidma::reset(void) {
	set_dma_register_value(S2MM_CONTROL_REGISTER, 4);
	set_dma_register_value(MM2S_CONTROL_REGISTER, 4);
}

void Axidma::stop(void) {
	set_dma_register_value(S2MM_CONTROL_REGISTER, 0);
	set_dma_register_value(MM2S_CONTROL_REGISTER, 0);
}

unsigned int Axidma::write(std::vector<uint64_t> * data) {

	unsigned int numBytes = data->size() * sizeof(uint64_t);

	const unsigned int transfer_size_words = 192;
	const unsigned int transfer_size_bytes = transfer_size_words * sizeof(uint64_t);



	std::copy(data->begin(), data->end(), source_addr);

	if ((numBytes > 0) && (numBytes <= max_write_transfer_length_bytes)) {

		source_addr[0] |= ((uint64_t) 1 & 0xFF) << 50;
		source_addr[0] |= ((uint64_t) (read_transfer_length_bytes
				/ sizeof(uint64_t)) & 0xFFF) << 51; //We use 8Bytes words in the S2MM bus

		set_dma_register_value(MM2S_LENGTH, numBytes);
		mm2s_pooling_sync();
		clear_mm2s_flags();
		return (numBytes);
	} else {
	    throw std::runtime_error("Write data on AXI bus over maximum transfer size");
	}

	return (0);
}

void Axidma::clear_mm2s_flags(void) {
	set_dma_register_value(MM2S_STATUS_REGISTER, 0x2); // Clear idle
	set_dma_register_value(MM2S_STATUS_REGISTER, 0x1000); // Clear IOC_Irq
}

unsigned int Axidma::read(std::vector<uint64_t> *data) {
	set_dma_register_value(S2MM_LENGTH, read_transfer_length_bytes);
	s2mm_pooling_sync();
	data->insert(data->end(), destination_addr,
			destination_addr + read_transfer_length_bytes / sizeof(uint64_t));

	clear_s2mm_flags();
	return (data->size());
}

void Axidma::clear_s2mm_flags(void) {
	set_dma_register_value(S2MM_STATUS_REGISTER, 0x2); // Clear idle
	set_dma_register_value(S2MM_STATUS_REGISTER, 0x1000); // Clear IOC_Irq
}

unsigned int Axidma::get_axidma_channel_timeout(void) {
	return (axidma_channel_timeout_us);
}

void Axidma::set_axidma_channel_timeout(unsigned int value) {
	axidma_channel_timeout_us = value;
}

unsigned int Axidma::get_read_transfer_length_bytes(void) {
	return (read_transfer_length_bytes);
}

#endif
