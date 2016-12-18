/* C wrapper to NullHop interface
 *  Author: federico.corradi@inilabs.com
 */
#include "classify.hpp"
#include "wrapper.h"

extern "C" {

zs_driver* newzs_driver(char * stringa) {
	return new zs_driver(stringa);
}

int zs_driver_classify_image(zs_driver* v, int * picture){
	return v->classify_image(picture);
}

}
