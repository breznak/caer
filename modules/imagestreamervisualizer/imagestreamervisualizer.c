/*
*  accumulates a fixed number of events and generates png pictures
*  it also displays png images when available
*  federico.corradi@inilabs.com
*/
#include "imagestreamervisualizer.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "modules/statistics/statistics.h"
#include "ext/portable_time.h"

#define GLFW_INCLUDE_GLEXT 1
#define GL_GLEXT_PROTOTYPES 1
#include <GLFW/glfw3.h>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

// We include glut.h so we can know which implementation we're using.
// We only support FreeGLUT currently.
#ifdef FREEGLUT
#include <GL/freeglut_ext.h>
#endif


#include "main.h"

#include <libcaer/events/polarity.h>

/* federico's matrix minimal lib */
#include "lib/simple_matrix.h"

/* inlude std image library */
#define STB_IMAGE_RESIZE_IMPLEMENTATION
#define STB_IMAGE_RESIZE_STATIC
#include "lib/stb_image_resize.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "lib/stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "lib/stb_image.h"

#define STB_DEFINE
#include "lib/stb.h"

#define PNGSUITE_PRIMARY
/* end std image library */



struct imagestreamervisualizer_state {
	GLFWwindow* window;
	uint32_t *eventRenderer;
	int16_t eventRendererSizeX;
	int16_t eventRendererSizeY;
	size_t eventRendererSlowDown;
	struct caer_statistics_state eventStatistics;
	uint32_t *frameRenderer;
	int32_t frameRendererSizeX;
	int32_t frameRendererSizeY;
	int32_t frameRendererPositionX;
	int32_t frameRendererPositionY;
	enum caer_frame_event_color_channels frameChannels;
	struct caer_statistics_state frameStatistics;
	int16_t subsampleRendering;
	int16_t subsampleCount;
   //image matrix	
   int64_t **ImageMap;
	int32_t numSpikes;
	int32_t counter;
	int32_t counterImg;
	int16_t sizeMaxX;
	int16_t sizeMaxY;
	int8_t subSampleBy;
};


typedef struct imagestreamervisualizer_state *imagestreamervisualizerState;

static bool caerImagestreamerVisualizerInit(caerModuleData moduleData);
static void caerImagestreamerVisualizerRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerImagestreamerVisualizerExit(caerModuleData moduleData);
static bool allocateEventRenderer(imagestreamervisualizerState state, int16_t sourceID);
static bool allocateFrameRenderer(imagestreamervisualizerState state, int16_t sourceID);
static bool allocateImageMap(imagestreamervisualizerState state, int16_t sourceID);

void framebuffer_size_callback(GLFWwindow* window);

static struct caer_module_functions caerImagestreamerVisualizerFunctions = { .moduleInit = &caerImagestreamerVisualizerInit, .moduleRun =
	&caerImagestreamerVisualizerRun, .moduleConfig =
NULL, .moduleExit = &caerImagestreamerVisualizerExit };

void caerImagestreamerVisualizer(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "ImageStreamerVisualizer");

	caerModuleSM(&caerImagestreamerVisualizerFunctions, moduleData, sizeof(struct imagestreamervisualizer_state), 2, polarity, frame);
}


static bool caerImagestreamerVisualizerInit(caerModuleData moduleData) {
	imagestreamervisualizerState state = moduleData->moduleState;
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "numSpikes", 7000);
   sshsNodePutByteIfAbsent(moduleData->moduleNode, "subSampleBy", 0);
   
	state->numSpikes = sshsNodeGetInt(moduleData->moduleNode, "numSpikes");
	state->subSampleBy = sshsNodeGetByte(moduleData->moduleNode, "subSampleBy");

	state->window = glfwCreateWindow(IMAGESTREAMERVISUALIZER_SCREEN_WIDTH, IMAGESTREAMERVISUALIZER_SCREEN_HEIGHT, "cAER Image Streamer Visualizer", NULL, NULL);
	if (state->window == NULL) {
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to create GLFW window for image streamer visualizer.");
		return (false);
	}
	
	glClearColor(0.0,0.0,0.0,0.0); // set white background color
	glColor3f(1.0f, 1.0f, 1.0f); // set the drawing color to black
	glMatrixMode(GL_PROJECTION); 
	glLoadIdentity();

	//make the window resizeable
	glfwSetFramebufferSizeCallback(state->window, framebuffer_size_callback);
	
	return (true);
}

static void caerImagestreamerVisualizerExit(caerModuleData moduleData) {
	imagestreamervisualizerState state = moduleData->moduleState;

	glfwDestroyWindow(state->window);

	glfwTerminate();

	// Ensure render maps are freed.
	if (state->eventRenderer != NULL) {
		free(state->eventRenderer);
		state->eventRenderer = NULL;
	}

	if (state->frameRenderer != NULL) {
		free(state->frameRenderer);
		state->frameRenderer = NULL;
	}

}

void framebuffer_size_callback(GLFWwindow* window)
{
	int width, height;
	glfwGetFramebufferSize(window, &width, &height);
	glViewport(0, 0, width, height);

}

static void caerImagestreamerVisualizerRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerPolarityEventPacket polarity = va_arg(args, caerPolarityEventPacket);

	// Only process packets with content.
	// what if content is not a polarity event?
	if (polarity == NULL) {
		return;
	}

	imagestreamervisualizerState state = moduleData->moduleState;

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

		//Update Map
		if(pol == 0){
			if(state->ImageMap[x][y] != INT64_MIN){
	 			state->ImageMap[x][y] = state->ImageMap[x][y] - 1;
			}
		}else{
			if(state->ImageMap[x][y] != INT64_MAX){
				state->ImageMap[x][y] = state->ImageMap[x][y] + 1;
			}
		}	
		state->counter += 1;

		//Generate Image from ImageMap
		if(state->counter >= state->numSpikes){
			uint8_t tmp;
			float tmp_v;
			float max_tmp_v = -100;
			float min_tmp_v = 2555;
			float mean = 0;
			float std = 0;
			//we accumulated enough spikes..
			state->counterImg += 1;
			// init img/data/index coord
			ImageCoordinate *img_coor = malloc(sizeof(ImageCoordinate));
			ImageCoordinateInit(img_coor, state->sizeMaxX, state->sizeMaxY, 1); // 1 gray scale, 4 red green alpha
			//save current image (of accumulated numSpikes) to disk 
			char id_img[15];
			char ext[] =".png"; // png output (possibles other formats supported by the library are bmp, ppm, TGA, psd, pnm, hdr, gif,..)
			char filename[255] = DIRECTORY_IMG ; //from settings.h
			sprintf(id_img, "%d", state->counterImg);
			strcat(filename, id_img); //append id_img
			strcat(filename, ext); //append extension 
			FILE *f = fopen(filename, "wb");
			int max_a = -1;
			int min_a = 2555;

			//get max min of imageMap sum min to make it unsigned integer compatible, then resize
			for(x_loop = 0; x_loop < state->sizeMaxX*state->sizeMaxY*1; ++x_loop) {
				img_coor->index = x_loop;
				calculateCoordinates(img_coor, x_loop, state->sizeMaxX, state->sizeMaxY);	
				max_a = MAX(max_a,state->ImageMap[img_coor->x][img_coor->y]);	
				min_a = MIN(min_a,state->ImageMap[img_coor->x][img_coor->y]);
				mean = mean + state->ImageMap[img_coor->x][img_coor->y];
			}
			mean = mean / state->sizeMaxX*state->sizeMaxY*1.0;
			for(x_loop = 0; x_loop < state->sizeMaxX*state->sizeMaxY*1; ++x_loop) {
				std = std + pow((state->ImageMap[img_coor->x][img_coor->y]-mean),2);
			}
			std = pow( (std / state->sizeMaxX*state->sizeMaxY*1.0), 0.5);
			unsigned char * image_map;
			image_map = (unsigned char*) malloc(state->sizeMaxX*state->sizeMaxY*1);
			for(x_loop = 0; x_loop < state->sizeMaxX*state->sizeMaxY*1; ++x_loop) {
				img_coor->index = x_loop;
				calculateCoordinates(img_coor, x_loop, state->sizeMaxX, state->sizeMaxY);
				tmp_v = ((((float) state->ImageMap[img_coor->x][state->sizeMaxY - img_coor->y])-mean)/std);		
				max_tmp_v = MAX(tmp_v,max_tmp_v);
				min_tmp_v = MIN(tmp_v,min_tmp_v);				
			}
			for(x_loop = 0; x_loop < state->sizeMaxX*state->sizeMaxY*1; ++x_loop) {
				img_coor->index = x_loop;
				calculateCoordinates(img_coor, x_loop, state->sizeMaxX, state->sizeMaxY);
				tmp_v = ((((float) state->ImageMap[img_coor->x][state->sizeMaxY - img_coor->y])-mean)/std);		
				tmp = round(((tmp_v - min_tmp_v)/(max_tmp_v - min_tmp_v))*254.0);
				image_map[x_loop] =  tmp & 0xFF; //uchar
			}
			unsigned char *small_img;
			small_img = (unsigned char*) malloc(SIZE_IMG*SIZE_IMG*1);
			//resize
			stbir_resize_uint8(image_map, state->sizeMaxX, state->sizeMaxY, 0, small_img, SIZE_IMG, SIZE_IMG, 0, 1);
			min_a = 255;
			max_a = -1;
			for(x_loop = 0; x_loop < SIZE_IMG*SIZE_IMG*1; ++x_loop) {
				max_a = MAX(max_a,small_img[x_loop]);	
				min_a = MIN(min_a,small_img[x_loop]);
			}
			//normalize
			for(x_loop = 0; x_loop < SIZE_IMG*SIZE_IMG*1; ++x_loop) {
				small_img[x_loop] = (unsigned char) round(((((float)small_img[x_loop]-min_a))/(max_a-min_a))*254);
			}
			//write image
			stbi_write_png(filename, SIZE_IMG, SIZE_IMG, 1, small_img, SIZE_IMG*1);
			fclose(f);
			state->counter = 0;
			printf("\nImage Streamer: \t\t generated image number %d\n", state->counterImg);	
			for(x_loop=0; x_loop<= state->sizeMaxX; x_loop++){
				for(y_loop=0; y_loop<= state->sizeMaxY; y_loop++){
					state->ImageMap[x_loop][y_loop] = 0 ;
				}
			}		
		
      free(img_coor);
      free(image_map);
      
	   //select context/window
	   glfwMakeContextCurrent(state->window);

	   #define checkImageWidth SIZE_IMG
	   #define checkImageHeight SIZE_IMG
	   static GLubyte checkImage[checkImageHeight][checkImageWidth][4];
	   static GLuint texName;
      //init image for glfw
	   int i, j, c;
	      for (i = 0; i < checkImageHeight; i++) {
	         for (j = 0; j < checkImageWidth; j++) {
	       c = i * checkImageHeight + j;
	       //printf("%d", small_img[c]);
		    //c = ((((i&0x8)==0)^((j&0x8))==0))*255; 	//checkboard texture opengl
		    checkImage[i][j][0] = (GLubyte) small_img[c];
		    checkImage[i][j][1] = (GLubyte) small_img[c];
		    checkImage[i][j][2] = (GLubyte) small_img[c];
		    checkImage[i][j][3] = (GLubyte) 255;
	         }
	      }

	   //init texture
	   glClearColor (0.0, 0.0, 0.0, 0.0);
	   glShadeModel(GL_FLAT);
	   glEnable(GL_DEPTH_TEST);
	   glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	   glGenTextures(1, &texName);
	   glBindTexture(GL_TEXTURE_2D, texName);
	   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, 
			      GL_NEAREST);
	   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, 
			      GL_NEAREST);
	   glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, checkImageWidth, 
			   checkImageHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, 
			   checkImage);

	   //display
    	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	   glEnable(GL_TEXTURE_2D);
	   glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	   glBindTexture(GL_TEXTURE_2D, texName);
	   glBegin(GL_QUADS);
	   glTexCoord2f(0.0, 0.0); glVertex2f(0.0, 0.0); //glVertex3f(-2.0, -1.0, 0.0);
	   glTexCoord2f(0.0, 1.0); glVertex2f(0.0, 1.0); //glVertex3f(-2.0, 1.0, 0.0);
	   glTexCoord2f(1.0, 1.0); glVertex2f(1.0, 1.0); //glVertex3f(0.0, 1.0, 0.0);
	   glTexCoord2f(1.0, 0.0); glVertex2f(1.0, 0.0); //glVertex3f(0.0, -1.0, 0.0);

	   glEnd();
	   glPopMatrix();
	   glFlush();
	   glDisable(GL_TEXTURE_2D);

	   //do glfw update
	   glfwSwapBuffers(state->window);
	   glfwPollEvents();
	
      //free memory
      free(small_img);
   }
   CAER_POLARITY_ITERATOR_VALID_END

}

static bool allocateImageMap(imagestreamervisualizerState state, int16_t sourceID) {
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

static bool allocateEventRenderer(imagestreamervisualizerState state, int16_t sourceID) {
	// Get size information from source.
	sshsNode sourceInfoNode = caerMainloopGetSourceInfo((uint16_t) sourceID);
	int16_t sizeX = sshsNodeGetShort(sourceInfoNode, "dvsSizeX");
	int16_t sizeY = sshsNodeGetShort(sourceInfoNode, "dvsSizeY");

	state->eventRenderer = calloc((size_t) (sizeX * sizeY), sizeof(uint32_t));
	if (state->eventRenderer == NULL) {
		return (false); // Failure.
	}

	// Assign maximum sizes for event renderer.
	state->eventRendererSizeX = sizeX;
	state->eventRendererSizeY = sizeY;

	return (true);
}

static bool allocateFrameRenderer(imagestreamervisualizerState state, int16_t sourceID) {
	// Get size information from source.
	sshsNode sourceInfoNode = caerMainloopGetSourceInfo((uint16_t) sourceID);
	int16_t sizeX = sshsNodeGetShort(sourceInfoNode, "apsSizeX");
	int16_t sizeY = sshsNodeGetShort(sourceInfoNode, "apsSizeY");

	state->frameRenderer = calloc((size_t) (sizeX * sizeY ), sizeof(uint16_t));
	if (state->frameRenderer == NULL) {
		return (false); // Failure.
	}

	// Assign maximum sizes and defaults for frame renderer.
	state->frameRendererSizeX = sizeX;
	state->frameRendererSizeY = sizeY;
	state->frameRendererPositionX = 0;
	state->frameRendererPositionY = 0;
	state->frameChannels = 1;

	return (true);
}
