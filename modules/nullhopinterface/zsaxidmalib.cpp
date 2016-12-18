#ifndef __ZS_AXIDMA_CPP__
#define __ZS_AXIDMA_CPP__

#include "zsaxidmalib.hpp"

void* write_thread_routine(void* arg) {
    printf("Creating write thread...\n");
    ZS_axidma* zsaxidma = (ZS_axidma*) arg;
    while (zsaxidma->is_write_thread_running()) {

        if (!zsaxidma->write_data.empty()) {
            try {
                //Here the mutex act just like a semaphore
                pthread_mutex_lock(&zsaxidma->write_mxt);
                pthread_mutex_unlock(&zsaxidma->write_mxt);

                //Here we dont need to keep the mutex lock: data inside front are fine thanks to previous semaphore
                //And now we are just reading from the list, not modifying it
                zsaxidma->axidma.write(&zsaxidma->write_data.front());

                //Here we modify the list, so we need to lock it
                pthread_mutex_lock(&zsaxidma->write_mxt);
                zsaxidma->write_data.pop_front();
                pthread_mutex_unlock(&zsaxidma->write_mxt);
            } catch (AXIDMA_timeout_exception& ex) {
                printf(ex.what());
                printf("Write thread timeout\n");
               // exit(-1); //TODO REMOVE ME
                zsaxidma->stop();
                zsaxidma->init(zsaxidma->axidma.get_read_transfer_length_bytes());
            } catch (std::bad_alloc& ba) {
                printf("bad_alloc caught: %s. List size --> %d\n", ba.what(),
                        zsaxidma->write_data.size());
            }
        } else {
            //This microsleep is necessary to avoid the loop to iterate infinitely locking the CPU
            //Notice that it is MANDATORY keep it here to allow the mutex inside the if{} statement, otherwise the system will lock
            //into an infinite loop. If mutex are moved outside the if, the usleep can be remove but performance decrease will occour
            //value obtained trying multiple times
            usleep(70);
        }
    }

    printf("Destroying write thread...\n");
    pthread_detach(pthread_self()); //Necessary to avoid memory leak
    pthread_exit(NULL);
}

void ZS_axidma::write(std::vector<uint64_t> *data) {
    pthread_mutex_lock(&write_mxt);
    write_data.push_back(*data);
    pthread_mutex_unlock(&write_mxt);

}

ZS_axidma::ZS_axidma() :
        axidma(AXIDMA_DEVICE_DEFINE, SOURCE_ADDR_OFFSET_DEFINE,
        DESTINATION_ADDR_OFFSET_DEFINE), axigpio(AXIGPIO_DEVICE_DEFINE) {
    //read_layer_finish = false;
    pthread_mutex_init(&write_mxt, NULL);
    //write_thread = 1;
    write_thread_running = false;
}

ZS_axidma::~ZS_axidma(void) {
    pthread_cancel(write_thread);
}

void ZS_axidma::init(unsigned int read_tranfer_length) {
    reset();
    axidma.init(read_tranfer_length);
    pthread_create(&write_thread, NULL, write_thread_routine, (void*) this);
    write_thread_running = true;
}

void ZS_axidma::reset(void) {
    axidma.reset();
    axigpio.configure_port_direction(0x0);
    axigpio.write(0x1);
}

void ZS_axidma::stop(void) {
    write_thread_running = false;
    usleep(100);
    axidma.stop();
}

int ZS_axidma::readLayer(std::vector<uint64_t> *layer_data) {
    do {
        try {
            axidma.read(layer_data);
        } catch (AXIDMA_timeout_exception& ex) {
            printf(ex.what());
            printf("Read thread timeout\n");
            stop();
           // exit(-1); //TODO REMOVE ME
            init(axidma.get_read_transfer_length_bytes());
            return (-1);
        }

    } while ((layer_data->data()[layer_data->size() - 1] & 0x8000000000000000) == 0); //Check if ZS is IDLE - using != 0 we dont need to shift the data saving 1 instruction

    return (layer_data->size() * sizeof(uint64_t));

}

bool ZS_axidma::is_write_thread_running(void) {
    return (write_thread_running);
}

#endif

