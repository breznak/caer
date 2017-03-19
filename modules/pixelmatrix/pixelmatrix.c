/*
 *
 *  Created on: Dec, 2016
 *      Author: federico.corradi@inilabs.com
 */

#include "pixelmatrix.h"
#include "base/mainloop.h"
#include "base/module.h"

// UDP
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>

#define NROWS 18
#define NCOLS 8
#define PORT 6500

const int pattern_a[NROWS * NCOLS] = {
	255,255,255,  0,255,255,255,  0,255,255,255,  0,255,255,  0,255,255,255,
	255,  0,255,  0,255,  0,255,  0,255,  0,255,  0,255,  0,  0,255,  0,255,
	255,  0,255,  0,255,  0,255,  0,255,  0,255,  0,255,  0,  0,255,  0,255,
	255,255,255,  0,255,255,255,  0,255,255,255,  0,255,255,  0,255,255,255,
	255,  0,  0,  0,255,  0,255,  0,255,  0,  0,  0,255,  0,  0,255,255,  0,
	255,  0,  0,  0,255,  0,255,  0,255,  0,  0,  0,255,  0,  0,255,  0,255,
	255,  0,  0,  0,255,  0,255,  0,255,  0,  0,  0,255,255,  0,255,  0,255,
	  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
};
const int pattern_c[NROWS * NCOLS] = {
	255,255,255,  0,255,255,255,  0,255,255,255,  0,255,  0,255,  0,  0,  0,
	255,  0,255,  0,255,  0,255,  0,255,  0,  0,  0,255,  0,255,  0,  0,  0,
	255,  0,255,  0,255,  0,255,  0,255,  0,  0,  0,255,  0,255,  0,  0,  0,
	255,255,255,  0,255,  0,255,  0,255,  0,  0,  0,255,255,  0,  0,  0,  0,
	255,255,  0,  0,255,  0,255,  0,255,  0,  0,  0,255,  0,255,  0,  0,  0,
	255,  0,255,  0,255,  0,255,  0,255,  0,  0,  0,255,  0,255,  0,  0,  0,
	255,  0,255,  0,255,255,255,  0,255,255,255,  0,255,  0,255,  0,  0,  0,
	  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
};
const int pattern_b[NROWS * NCOLS] = {
	255,255,255,  0,255,255,255,  0,255,  0,255,255,255,  0,255,255,255,  0,
	255,  0,  0,  0,255,  0,  0,  0,255,  0,255,  0,  0,  0,255,  0,  0,  0,
	255,  0,  0,  0,255,  0,  0,  0,255,  0,255,  0,  0,  0,255,  0,  0,  0,
	255,255,255,  0,255,  0,  0,  0,255,  0,255,255,255,  0,255,255,255,  0,
	  0,  0,255,  0,255,  0,  0,  0,255,  0,  0,  0,255,  0,  0,  0,255,  0,
	  0,  0,255,  0,255,  0,  0,  0,255,  0,  0,  0,255,  0,  0,  0,255,  0,
	255,255,255,  0,255,255,255,  0,255,  0,255,255,255,  0,255,255,255,  0,
	  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
};
const int pattern_d[NROWS * NCOLS] = {
	255,255,255,  0,255,255,255,  0,255,255,255,  0,255,  0,255,  0,  0,  0,
	255,  0,255,  0,255,  0,255,  0,255,  0,  0,  0,255,  0,255,  0,  0,  0,
	255,  0,255,  0,255,  0,255,  0,255,  0,  0,  0,255,  0,255,  0,  0,  0,
	255,255,  0,  0,255,255,255,  0,255,  0,  0,  0,255,255,  0,  0,  0,  0,
	255,  0,255,  0,255,  0,255,  0,255,  0,  0,  0,255,  0,255,  0,  0,  0,
	255,  0,255,  0,255,  0,255,  0,255,  0,  0,  0,255,  0,255,  0,  0,  0,
	255,255,255,  0,255,  0,255,  0,255,255,255,  0,255,  0,255,  0,  0,  0,
	  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
};

struct PMFilter_state {
	int clientSocket;	// UDP
	unsigned char buffer[NROWS * NCOLS * 3];	// ..
	struct sockaddr_in serverAddr; // ..
	socklen_t addr_size; 	// ..
};

typedef struct PMFilter_state *PMFilterState;

static bool caerPixelMatrixFilterInit(caerModuleData moduleData);
static void caerPixelMatrixFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerPixelMatrixFilterConfig(caerModuleData moduleData);
static void caerPixelMatrixFilterExit(caerModuleData moduleData);
static void caerPixelMatrixFilterReset(caerModuleData moduleData, uint16_t resetCallSourceID);

static struct caer_module_functions caerPixelMatrixFilterFunctions = { .moduleInit = &caerPixelMatrixFilterInit,
	.moduleRun = &caerPixelMatrixFilterRun, .moduleConfig = &caerPixelMatrixFilterConfig, .moduleExit =
		&caerPixelMatrixFilterExit, .moduleReset = &caerPixelMatrixFilterReset };

void caerPixelMatrixFilter(uint16_t moduleID, caerPolarityEventPacket polarity, char * classificationResults,
	int * classificationResultsId) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "PixelMatrix", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return;
	}

	caerModuleSM(&caerPixelMatrixFilterFunctions, moduleData, sizeof(struct PMFilter_state), 3, polarity,
		classificationResults, classificationResultsId);
}

static bool caerPixelMatrixFilterInit(caerModuleData moduleData) {

	PMFilterState state = moduleData->moduleState;

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	// UDP
	/*---- Create the socket. The three arguments are: ----*/
	/* 1) Internet domain 2) Stream socket 3) Default protocol (UDP in this case) */
	state->clientSocket = socket(AF_INET, SOCK_DGRAM, 0);
	if (state->clientSocket == -1) {
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Can't create UDP socket");
		exit(-1);
	}

	/*---- Configure settings of the server address struct ----*/
	/* Address family = Internet */
	state->serverAddr.sin_family = AF_INET;
	/* Set port number, using htons function to use proper byte order */
	state->serverAddr.sin_port = htons(PORT);
	/* Set IP address to localhost */
	state->serverAddr.sin_addr.s_addr = inet_addr("192.168.0.144");
	/* Set all bits of the padding field to 0 */
	memset(state->serverAddr.sin_zero, '\0', sizeof state->serverAddr.sin_zero);

	/*---- Connect the socket to the server using the address struct ----*/
	state->addr_size = sizeof state->serverAddr;
	if (connect(state->clientSocket, (struct sockaddr *) &state->serverAddr, state->addr_size) == 0) {
		caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Connected to Pixel Matrix LED Monitor");
	}
	else {
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
			"ERROR: failed to connect to Pixel Matrix LED Monitor");
	}

	// Nothing that can fail here.
	return (true);
}

static void caerPixelMatrixFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	caerPolarityEventPacket polarity = va_arg(args, caerPolarityEventPacket);
	char * classificationResults = va_arg(args, char*);
	int * classificationResultsId = va_arg(args, int*);

	// Only process packets with content.
	if (polarity == NULL) {
		return;
	} // polarity is not used at the moment

	PMFilterState state = moduleData->moduleState;

	if (classificationResultsId[0] == 0) {
		int counter = 0;
		int cc = 0;
		int xs = 0;
		bool started = false;
		for (int x = 0; x < NCOLS * NROWS; ++x) {
			// snake design
			if ( cc == NROWS  && started == true){
				started = false;
				cc = 0;
			}else if(cc == NROWS && started == false){
				started = true;
				cc = 0;
				xs = x-1;
			}
			state->buffer[counter] = 1 & 0xFF;
			if(started == true){
				int indexS =  xs+NROWS-cc; // invert ordering of index for even rows - snake design -
				state->buffer[counter + 0] = pattern_a[indexS] & 0xFF;
			}else{
				state->buffer[counter + 0] = pattern_a[x] & 0xFF;
			}
			state->buffer[counter + 2] = 0 & 0xFF;
			counter += 3;
			cc += 1;
		}
	}
	else if (classificationResultsId[0] == 1) {
		int counter = 0;
		int cc = 0;
		int xs = 0;
		bool started = false;
		for (int x = 0; x < NCOLS * NROWS; ++x) {
			// snake design
			if ( cc == NROWS  && started == true){
				started = false;
				cc = 0;
			}else if(cc == NROWS && started == false){
				started = true;
				cc = 0;
				xs = x-1;
			}
			state->buffer[counter] = 0 & 0xFF;
			if(started == true){
				int indexS =  xs+NROWS-cc; // invert ordering of index for even rows - snake design -
				state->buffer[counter + 1] = pattern_b[indexS] & 0xFF;
			}else{
				state->buffer[counter + 1] = pattern_b[x] & 0xFF;
			}
			state->buffer[counter + 2] = 0 & 0xFF;
			counter += 3;
			cc += 1;
		}
	}
	else if (classificationResultsId[0] == 2) {
		int counter = 0;
		int cc = 0;
		int xs = 0;
		bool started = false;
		for (int x = 0; x < NCOLS * NROWS; ++x) {
			// snake design
			if ( cc == NROWS  && started == true){
				started = false;
				cc = 0;
			}else if(cc == NROWS && started == false){
				started = true;
				cc = 0;
				xs = x-1;
			}
			state->buffer[counter] = 0 & 0xFF;
			if(started == true){
				int indexS =  xs+NROWS-cc; // invert ordering of index for even rows - snake design -
				state->buffer[counter + 2] = pattern_c[indexS] & 0xFF;
			}else{
				state->buffer[counter + 2] = pattern_c[x] & 0xFF;
			}
			state->buffer[counter + 1] = 0 & 0xFF;
			counter += 3;
			cc += 1;
		}
	}
	else if (classificationResultsId[0] == 3) {
		int counter = 0;
		int cc = 0;
		int xs = 0;
		bool started = false;
		for (int x = 0; x < NCOLS * NROWS; ++x) {
			// snake design
			if ( cc == NROWS  && started == true){
				started = false;
				cc = 0;
			}else if(cc == NROWS && started == false){
				started = true;
				cc = 0;
				xs = x-1;
			}
			state->buffer[counter] = 1 & 0xFF;
			if(started == true){
				int indexS =  xs+NROWS-cc; // invert ordering of index for even rows - snake design -
				state->buffer[counter + 0] = pattern_d[indexS] & 0xFF;
				state->buffer[counter + 2] = pattern_d[indexS] & 0xFF;
			}else{
				state->buffer[counter + 0] = pattern_d[x] & 0xFF;
				state->buffer[counter + 2] = pattern_d[x] & 0xFF;
			}
			counter += 3;
			cc += 1;
		}
	}

	// send info via udp
	if (send(state->clientSocket, state->buffer, NROWS * NCOLS * 3 * 4, 0) == -1) {
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Sending udp error %d", errno);
	}

}

static void caerPixelMatrixFilterConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	PMFilterState state = moduleData->moduleState;

}

static void caerPixelMatrixFilterExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	PMFilterState state = moduleData->moduleState;
}

static void caerPixelMatrixFilterReset(caerModuleData moduleData, uint16_t resetCallSourceID) {
	UNUSED_ARGUMENT(resetCallSourceID);

	PMFilterState state = moduleData->moduleState;

}

