/*
 * imagestreamer.c
 *
 *  Created on: Jan 12, 2016
 *      Authors:  federico.corradi@inilabs.com, chtekk
 */

#include "imagestreamerfilter.h"
#include "base/mainloop.h"
#include "base/module.h"

/* inlude std image library */
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "lib/stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "lib/stb_image.h"

#define STB_DEFINE
#include "lib/stb.h"

#define PNGSUITE_PRIMARY
/* end std image library */

/* federico's tools lib */
#include "lib/simple_matrix.h"
/* settings directories etc...*/
#include "settings.h"

struct ISFilter_state {
	int64_t **ImageMap;
	int32_t numSpikes;
	int32_t counter;
	int32_t counterImg;
	int16_t sizeMaxX;
	int16_t sizeMaxY;
	int8_t subSampleBy;
};

typedef struct ISFilter_state *ISFilterState;

static bool caerImageStreamerFilterInit(caerModuleData moduleData);
static void caerImageStreamerFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerImageStreamerFilterConfig(caerModuleData moduleData);
static void caerImageStreamerFilterExit(caerModuleData moduleData);
static bool allocateImageMap(ISFilterState state, int16_t sourceID);

static struct caer_module_functions caerImageStreamerFilterFunctions = { .moduleInit =
	&caerImageStreamerFilterInit, .moduleRun = &caerImageStreamerFilterRun, .moduleConfig =
	&caerImageStreamerFilterConfig, .moduleExit = &caerImageStreamerFilterExit };

void caerImageStreamerFilter(uint16_t moduleID, caerPolarityEventPacket polarity) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "ISFilter");

	caerModuleSM(&caerImageStreamerFilterFunctions, moduleData, sizeof(struct ISFilter_state), 1, polarity);
}

/*
* this function is a dummy write for std image library
*/
void dummy_write(void *context, void *data, int len)
{
   static char dummy[1024];
   if (len > 1024) len = 1024;
   memcpy(dummy, data, len);
}

static bool caerImageStreamerFilterInit(caerModuleData moduleData) {
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "numSpikes", 10000);
	sshsNodePutByteIfAbsent(moduleData->moduleNode, "subSampleBy", 0);

	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);
	ISFilterState state = moduleData->moduleState;

	state->numSpikes = sshsNodeGetInt(moduleData->moduleNode, "numSpikes");
	state->subSampleBy = sshsNodeGetByte(moduleData->moduleNode, "subSampleBy");

	// Nothing that can fail here.
	return (true);
}

static void caerImageStreamerFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerPolarityEventPacket polarity = va_arg(args, caerPolarityEventPacket);

	// Only process packets with content.
	// what if content is not a polarity event?
	if (polarity == NULL) {
		return;
	}

	ISFilterState state = moduleData->moduleState;

	// If the map is not allocated yet, do it.
	if (state->ImageMap == NULL) {
		if (!allocateImageMap(state, caerEventPacketHeaderGetEventSource(&polarity->packetHeader))) {
			// Failed to allocate memory, nothing to do.
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for ImageMap.");
			return;
		}
	}


	// Iterate over events and accumulate them 
	CAER_POLARITY_ITERATOR_VALID_START(polarity)
		// Get values on which to operate.
		//int64_t ts = caerPolarityEventGetTimestamp64(caerPolarityIteratorElement, polarity);
		uint16_t x = caerPolarityEventGetX(caerPolarityIteratorElement);
		uint16_t y = caerPolarityEventGetY(caerPolarityIteratorElement);
		int pol = caerPolarityEventGetPolarity(caerPolarityIteratorElement);
		int x_loop = 0;
		int y_loop = 0;

		// Apply sub-sampling.
		x = U16T(x >> state->subSampleBy);
		y = U16T(y >> state->subSampleBy);

		// Get value from map.
		int64_t current_value = state->ImageMap[x][y];

		//Update Map
	 	state->ImageMap[x][y] = current_value + pol;	
		state->counter += 1;

		//Generate Image from ImageMap
		if(state->counter >= state->numSpikes ){
			//we accumulated enough spikes..
			state->counterImg += 1;
			// init img/data/index coord
			ImageCoordinate *img_coor = malloc(sizeof(ImageCoordinate));
			ImageCoordinateInit(img_coor, state->sizeMaxX, state->sizeMaxY, 1); // 1 gray scale, 4 red green alpha
			//save to disk this image 
			char id_img[15];
			char ext[] =".png"; //we produce png output (possibles other formats supported by the library are bmp, ppm, TGA, psd, pnm, hdr, gif,..)
			char filename[255] = DIRECTORY_IMG ;
			sprintf(id_img, "%d", state->counterImg);
			strcat(filename, id_img); //append id_img
			strcat(filename, ext); //append extension 
			FILE *f = fopen(filename, "wb");
			for(x_loop = 0; x_loop < state->sizeMaxY*state->sizeMaxX*1; ++x_loop) { //*1 gray scale, *4 rbg + alpha
			        img_coor->index = x_loop;
				calculateCoordinates(img_coor, x_loop, state->sizeMaxX, state->sizeMaxY); 
				img_coor->image_data[x_loop] = state->ImageMap[img_coor->x][img_coor->y];
				//printf("data[%d] : %u x:%d y:%d\n" , x_loop, (unsigned int)img_coor->image_data[x_loop], img_coor->x, img_coor->y );
            		}
			//normalize image 0..255
		        normalizeImage(img_coor);
			// write image to png 
			stbi_write_png(filename, img_coor->sizeX, img_coor->sizeY, 1, img_coor->image_data, img_coor->sizeX);
			fclose(f);
			//close image and reset imagemap
			state->counter = 0;
			printf("\nImage Streamer: \t\t generated image number %d\n", state->counterImg);	
			for(x_loop=0; x_loop<= state->sizeMaxX; x_loop++){
				for(y_loop=0; y_loop<= state->sizeMaxY; y_loop++){
					state->ImageMap[x_loop][y_loop] = 0 ;
				}
			}		
			free(img_coor);
		}

	CAER_POLARITY_ITERATOR_VALID_END
}

static void caerImageStreamerFilterConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	ISFilterState state = moduleData->moduleState;

	state->numSpikes = sshsNodeGetInt(moduleData->moduleNode, "numSpikes");
	state->subSampleBy = sshsNodeGetByte(moduleData->moduleNode, "subSampleBy");
}

static void caerImageStreamerFilterExit(caerModuleData moduleData) {
	ISFilterState state = moduleData->moduleState;

	// Ensure map is freed.
	if (state->ImageMap != NULL) {
		free(state->ImageMap[0]);
		free(state->ImageMap);
		state->ImageMap = NULL;
	}
}

static bool allocateImageMap(ISFilterState state, int16_t sourceID) {
	// Get size information from source.
	sshsNode sourceInfoNode = caerMainloopGetSourceInfo((uint16_t) sourceID);
	int16_t sizeX = sshsNodeGetShort(sourceInfoNode, "dvsSizeX");
	int16_t sizeY = sshsNodeGetShort(sourceInfoNode, "dvsSizeY");

	// Initialize double-indirection contiguous 2D array, so that array[x][y]
	// is possible, see http://c-faq.com/aryptr/dynmuldimary.html for info.
	state->ImageMap = calloc((size_t) sizeX, sizeof(int64_t *));
	if (state->ImageMap == NULL) {
		return (false); // Failure.
	}

	state->ImageMap[0] = calloc((size_t) (sizeX * sizeY), sizeof(int64_t));
	if (state->ImageMap[0] == NULL) {
		free(state->ImageMap);
		state->ImageMap = NULL;

		return (false); // Failure.
	}

	for (size_t i = 1; i < (size_t) sizeX; i++) {
		state->ImageMap[i] = state->ImageMap[0] + (i * (size_t) sizeY);
	}

	// Assign max ranges for arrays (0 to MAX-1).
	state->sizeMaxX = (sizeX - 1);
	state->sizeMaxY = (sizeY - 1);

	// Init counters
	state->counter = 0;
	state->counterImg = 0;

	// TODO: size the map differently if subSampleBy is set!
	return (true);
}
