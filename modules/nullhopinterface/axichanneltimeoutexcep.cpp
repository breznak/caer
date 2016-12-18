/*
 * axichanneltimeoutexcep.cpp
 *
 *  Created on: Nov 23, 2016
 *      Author: hyo
 */
#ifndef __AXIDMA_TIMEOUT_EXCEPTION__
#define __AXIDMA_TIMEOUT_EXCEPTION__

#include <exception>
/** \brief AXIDMA_timeout_exception class.
 */
class AXIDMA_timeout_exception: public std::exception {
public:
	virtual const char* what() const throw () {
		return "Reached AXIDMA channel timeout exception.\n";
	}
};

#endif

