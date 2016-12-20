#ifndef __ZS_AXIDMA__
#define __ZS_AXIDMA__

#include "axidmalib.hpp"
//#include "axigpio.cpp"
#include <list>
#include <pthread.h>
#include <unistd.h>
#include <bitset>
#include <algorithm>

#define AXIDMA_DEVICE_DEFINE 0x40400000
#define DESTINATION_ADDR_OFFSET_DEFINE 0x0F000000
#define SOURCE_ADDR_OFFSET_DEFINE 0x0E000000
#define AXIGPIO_DEVICE_DEFINE 0X41200000

/** \brief ZS_axidma class. High level class for ZS - AXIDMA interfacing
 *
 * This class manages AXIDMA controller properly for ZS controller.
 */
class ZS_axidma {
private:
    pthread_t write_thread;
    pthread_mutex_t write_mxt;

    /** \brief Prepare the front data vector of write_data list to be write to ZS
     *
     *  @return number of bytes written to ZS
     */
    void write_to_axidma(void);
    std::list<std::vector<uint64_t>> write_data;
    std::list<bool> write_data_checklist;
    bool write_thread_running;

    //void read_from_axidma(void);
    //bool read_layer_finish;

    Axigpio axigpio;    //!< Object used to control the software reset
    Axidma axidma;      //!< Object used to manage AXIDMA engine

public:
    /** \brief Constructor
     */
    ZS_axidma(void);

    /** \brief Destructor
     */
    ~ZS_axidma(void);

    /** \brief AXIDMA channels initialization
     *
     *  @param read_transfer_length set the ZS read transfer length in bytes
     */
    void init(unsigned int read_tranfer_length);

    /** \brief Reset AXIDMA engine and all hardware modules in the programmable logic
     */
    void reset(void);

    /** \brief Stop both AXIDMA channels and the write thread
     */
    void stop(void);

    /** \brief Non-blocking write operation.
     *
     *  Add a new vector to write_data list to be written to ZS
     *  @param data vector to be added to write_data list
     */
    void write(std::vector<uint64_t>* data);

    /** \brief Bblocking read operation.
     *
     *  Read continually data from ZS untill zs_idle signal is catched.
     *  @param layer_data vector pointer where all data come out from ZS will be copied
     *  @return number of bytes read from zs in the actual layer
     */
    int readLayer(std::vector<uint64_t> *layer_data);

    /** \brief Write thread to be non-blocking write operations
     */
    friend void* write_thread_routine(void* arg);

    /** \brief Get actual write_thread_running values
     *
     *  @return write_thread_running value
     */
    bool is_write_thread_running(void);
};

#endif
