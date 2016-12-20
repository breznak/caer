#ifndef __AXIGPIO_CPP__
#define __AXIGPIO_CPP__

#include "axigpio.hpp"


Axigpio::Axigpio(unsigned int gpio_addr_offset) : AXIGPIO_ADDR_OFFSET(gpio_addr_offset),AXIGPIO_DATA(0x0000),AXIGPIO_TRI(0x0004),AXIGPIO_DATA2(0x0008),
        AXIGPIO_TRI2(0x000C),AXIGPIO_GIER(0x011C),AXIGPIO_IPIER(0x0128),AXIGPIO_IPISR(0x0120)
{
    	whole_memory_pointer = open("/dev/mem", O_RDWR | O_SYNC); // Open /dev/mem which represents the whole physical memory
    	gpio_map_addr = (unsigned int *)mmap(NULL, 65535, PROT_READ | PROT_WRITE, MAP_SHARED, whole_memory_pointer, AXIGPIO_ADDR_OFFSET); // Memory map AXI GPIO register block
}

Axigpio::~Axigpio()
{
    	munmap(gpio_map_addr,65535);
}

void Axigpio::set_register_value(int register_offset, unsigned int value)
{
    	gpio_map_addr[register_offset>>2] = value;
}

unsigned int Axigpio::get_register_value(int register_offset)
{
	    return gpio_map_addr[register_offset>>2];
}

/*
    direction_mask bit meaning:
    0 --> output
    1 --> input
*/
void Axigpio::configure_port_direction(unsigned int direction_mask)
{
    set_register_value(AXIGPIO_TRI, direction_mask);
}

void Axigpio::write(unsigned int value){
    set_register_value(AXIGPIO_DATA, value);
}

unsigned int Axigpio::read(void){
    return get_register_value(AXIGPIO_DATA);
}

#endif
