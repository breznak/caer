/* C wrapper to NullHop Zynq interface
 *  Author: federico.corradi@gmail.com
 */
#ifndef __WRAPPER_H
#define __WRAPPER_H
#include <stdint.h>
#include <libcaer/events/frame.h>


#ifdef __cplusplus
extern "C" {
#endif

typedef struct zs_driver zs_driver;

zs_driver* newzs_driver(char * stringa);

int zs_driver_classify_image(zs_driver* v, int * picture);


#ifdef __cplusplus
}
#endif
#endif

