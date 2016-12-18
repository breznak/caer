#ifndef __AXIGPIO__
#define __AXIGPIO__

#include <fcntl.h>
#include <sys/mman.h>
//#include <curses.h>
#include <cstring>
#include <vector>
#include <unistd.h>

class Axigpio
{
    private:
        const unsigned int AXIGPIO_ADDR_OFFSET;
        const unsigned int AXIGPIO_DATA;
        const unsigned int AXIGPIO_TRI;
        const unsigned int AXIGPIO_DATA2;
        const unsigned int AXIGPIO_TRI2;
        const unsigned int AXIGPIO_GIER;
        const unsigned int AXIGPIO_IPIER;
        const unsigned int AXIGPIO_IPISR;

        int whole_memory_pointer;
        unsigned int* gpio_map_addr;

        unsigned int get_register_value(int register_offset);
        void set_register_value(int register_offset, unsigned int value);

    public:
        Axigpio(unsigned int gpio_addr_offset);
        ~Axigpio();
        void configure_port_direction(unsigned int direction_mask);
        void write(unsigned int value);
        unsigned int read(void);
};

#endif
