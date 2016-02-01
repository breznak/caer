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
#include <string.h>

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
#include "ext/simplematrix/simple_matrix.h"

/* inlude std image library */
#include "ext/stblib/stb_image_write.h"
#include "ext/stblib/stb_image_resize.h"

//#include "lib/stb_image_resize.h"

#define STB_IMAGE_IMPLEMENTATION
#include "ext/stblib/stb_image.h"

#define STB_DEFINE
#include "ext/stblib/stb.h"

#define PNGSUITE_PRIMARY
/* end std image library */

#define TITLE_WINDOW "cAER Image Streamer Visualizer"
#define TITLE_WINDOW_RECORDING "cAER Image Streamer Visualizer: Recordings png.."
#define TESTING 0		// keyboard "r" or "t" (recording or testing) "s" (stop) real-time test network, stores images in /tmp/ as defined in header file .h
#define TRAINING_POSITIVES 1	// keyboard "p" (positives) record pngs and store them in positive folder
#define TRAINING_NEGATIVES 2	// keyboard "n" (negatives) record pngs and store them in negative folder
				// keyboard "s" stop saving png, generations on visualizer keeps going


extern int8_t savepng_state = 0; //default state -> do not save png
extern int8_t mode = 0;		 //default mode -> do nothing 

struct imagestreamervisualizer_state {
	GLFWwindow* window;
	uint32_t *eventRenderer;
	int16_t eventRendererSizeX;
	int16_t eventRendererSizeY;
	size_t eventRendererSlowDown;
	struct caer_statistics_state eventStatistics;
	int16_t subsampleRendering;
	int16_t subsampleCount;
	//save output files
	int8_t savepng;
	int8_t mode;
        //image matrix	
        int64_t **ImageMap;
	int32_t numSpikes;
	int32_t counter;
	int32_t counterImg;
	int16_t sizeMaxX;
	int16_t sizeMaxY;
	int8_t subSampleBy;
	// frame
	uint16_t *frameRenderer;
	int32_t frameRendererSizeX;
	int32_t frameRendererSizeY;
	int32_t frameRendererPositionX;
	int32_t frameRendererPositionY;
	enum caer_frame_event_color_channels frameChannels;
};


typedef struct imagestreamervisualizer_state *imagestreamervisualizerState;

static bool caerImagestreamerVisualizerInit(caerModuleData moduleData);
static void caerImagestreamerVisualizerRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerImagestreamerVisualizerExit(caerModuleData moduleData);
static bool allocateEventRenderer(imagestreamervisualizerState state, int16_t sourceID);
static bool allocateImageMap(imagestreamervisualizerState state, int16_t sourceID);

void framebuffer_size_callback(GLFWwindow* window);
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);

static struct caer_module_functions caerImagestreamerVisualizerFunctions = { .moduleInit = &caerImagestreamerVisualizerInit, .moduleRun =
	&caerImagestreamerVisualizerRun, .moduleConfig =
NULL, .moduleExit = &caerImagestreamerVisualizerExit };

void caerImagestreamerVisualizer(uint16_t moduleID, caerPolarityEventPacket polarity, char ** file_string, caerFrameEventPacket frame, char ** file_string_frame) {
	
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "ImageStreamerVisualizer");

	caerModuleSM(&caerImagestreamerVisualizerFunctions, moduleData, sizeof(struct imagestreamervisualizer_state), 4, polarity, file_string, frame, file_string_frame);

	return;
}


static bool caerImagestreamerVisualizerInit(caerModuleData moduleData) {
	imagestreamervisualizerState state = moduleData->moduleState;
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "numSpikes", 7000);
   	sshsNodePutByteIfAbsent(moduleData->moduleNode, "subSampleBy", 0);
	sshsNodePutByteIfAbsent(moduleData->moduleNode, "savepng", 0);
   
	state->numSpikes = sshsNodeGetInt(moduleData->moduleNode, "numSpikes");
	state->subSampleBy = sshsNodeGetByte(moduleData->moduleNode, "subSampleBy");
	state->savepng = sshsNodeGetByte(moduleData->moduleNode, "savepng");

	state->window = glfwCreateWindow(IMAGESTREAMERVISUALIZER_SCREEN_WIDTH, IMAGESTREAMERVISUALIZER_SCREEN_HEIGHT, TITLE_WINDOW, NULL, NULL);
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
	//key control save image
	glfwSetKeyCallback(state->window, key_callback);
	
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

}

void framebuffer_size_callback(GLFWwindow* window){
	int width, height;
	glfwGetFramebufferSize(window, &width, &height);
	glViewport(0, 0, width, height);

}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods){

    // R start recording or T start testing  
    if ((key == GLFW_KEY_R || key == GLFW_KEY_T) && action == GLFW_PRESS){
	printf("\nImage Streamer Filter: start saving png files to hw\n");
	glfwSetWindowTitle(window, TITLE_WINDOW_RECORDING);
	savepng_state = 1;
	mode = TESTING; //testing
    }
    // S stop recording
    if (key == GLFW_KEY_S  && action == GLFW_PRESS){
	printf("\nImage Streamer Filter: stop saving png files\n");
	glfwSetWindowTitle(window, TITLE_WINDOW);
	savepng_state = 0; //stop testing
	mode = TESTING;
    }
    // P start recording positive examples
    if (key == GLFW_KEY_N  && action == GLFW_PRESS){
	printf("\nImage Streamer Filter: start capturing positive png in /yourpath/pos/ \n");
	glfwSetWindowTitle(window, TITLE_WINDOW);
	savepng_state = 1;
	mode = TRAINING_NEGATIVES; //training positives examples
    }
    // N start recording negative examples
    if (key == GLFW_KEY_P  && action == GLFW_PRESS){
	printf("\nImage Streamer Filter: start capturing positive png in /yourpath/pos/ \n");
	glfwSetWindowTitle(window, TITLE_WINDOW);
	savepng_state = 1;
	mode = TRAINING_POSITIVES; //training negative examples 
    }
 
    
}


static void caerImagestreamerVisualizerRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerPolarityEventPacket polarity = va_arg(args, caerPolarityEventPacket);
	char ** file_string = va_arg(args, char **);
	caerFrameEventPacket frame = va_arg(args, caerFrameEventPacket);
	char ** file_string_frame = va_arg(args, char **);

	// Only process packets with content.
	// what if content is not a polarity event?
	if (polarity == NULL && frame == NULL) {
		return;
	}

	imagestreamervisualizerState state = moduleData->moduleState;

	//update saving state
	state->savepng = savepng_state;
	state->mode = mode;

	// If the map is not allocated yet, do it.
	if (state->ImageMap == NULL) {
		if (!allocateImageMap(state, caerEventPacketHeaderGetEventSource(&polarity->packetHeader))) {
			// Failed to allocate memory, nothing to do.
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for ImageMap.");
			return;
		}
	}

	// init frame
	if ( frame != NULL) {
		caerFrameEvent currFrameEvent;

		for (int32_t i = caerEventPacketHeaderGetEventNumber(&frame->packetHeader) - 1; i >= 0; i--) {
			currFrameEvent = caerFrameEventPacketGetEvent(frame, i);

			// Only operate on the last valid frame.
			if (caerFrameEventIsValid(currFrameEvent)) {
				// Copy the frame content to the permanent frameRenderer.
				// Use frame sizes to correctly support small ROI frames.
				state->frameRendererSizeX = caerFrameEventGetLengthX(currFrameEvent);
				state->frameRendererSizeY = caerFrameEventGetLengthY(currFrameEvent);
				state->frameRendererPositionX = caerFrameEventGetPositionX(currFrameEvent);
				state->frameRendererPositionY = caerFrameEventGetPositionY(currFrameEvent);
				state->frameChannels = caerFrameEventGetChannelNumber(currFrameEvent);

				state->frameRenderer = caerFrameEventGetPixelArrayCGFormat(currFrameEvent);

				break;
			}
		}

		if(state->savepng == 1 && state->frameRenderer != NULL){
			state->counterImg += 1;
			int frame_loop = 0;

			//save current frame image as png to disk
			char id_img[15];
			char ext[] =".png"; // png output (possibles other formats supported by the library are bmp, ppm, TGA, psd, pnm, hdr, gif,..)
			//check in which mode we are (testing/training/none)
			char filename[255] = DIRECTORY_IMG;	
			if(state->mode == TESTING){
				strcat(filename,"_testing_frame_"); 
			}
			if(state->mode == TRAINING_POSITIVES){
				strcat(filename,"_pos_frame_"); 
			}
			if(state->mode == TRAINING_NEGATIVES){
				strcat(filename,"_neg_frame_");
			}
			sprintf(id_img, "%d", state->counterImg);
			strcat(filename, id_img); //append id_img
			strcat(filename, ext); //append extension 
			FILE *f = fopen(filename, "wb");

			//cast frame to unsigned char
			unsigned char *frame_img;
			frame_img = (unsigned char*) malloc(state->frameRendererSizeX*state->frameRendererSizeY*1);
			for(frame_loop = 0; frame_loop < state->frameRendererSizeX*state->frameRendererSizeY*1; ++frame_loop) {
				frame_img[frame_loop] =  (state->frameRenderer[frame_loop] >> 8) & 0xFF; 
			}
			stbi_write_png(filename, state->frameRendererSizeX,state->frameRendererSizeY, 1, frame_img, state->frameRendererSizeX*1);
			fclose(f);
			*file_string = strdup(filename);
			if(*file_string ==  NULL){
				caerLog(CAER_LOG_DEBUG, moduleData->moduleSubSystemString, "Error file name for png image not valid..");
				return;
			} 
            free(frame_img);
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
			int max_a = -1;
			int min_a = 2555;

			//we accumulated enough spikes..
			state->counterImg += 1;
			// init img/data/index coord
			ImageCoordinate *img_coor = malloc(sizeof(ImageCoordinate));
			ImageCoordinateInit(img_coor, state->sizeMaxX, state->sizeMaxY, 1); // 1 gray scale, 4 red green alpha

			//normalize image vector, convert it to 0..255, resize to SIZE_IMG*SIZE_IMG and update/save png 
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
			small_img = (unsigned char*) malloc(SIZE_IMG_W*SIZE_IMG_H*1);
			//resize
			stbir_resize_uint8(image_map, state->sizeMaxX, state->sizeMaxY, 0, small_img, SIZE_IMG_W, SIZE_IMG_H, 0, 1);
			min_a = 255;
			max_a = -1;
			for(x_loop = 0; x_loop < SIZE_IMG_W*SIZE_IMG_H*1; ++x_loop) {
				max_a = MAX(max_a,small_img[x_loop]);	
				min_a = MIN(min_a,small_img[x_loop]);
			}
			//normalize
			for(x_loop = 0; x_loop < SIZE_IMG_W*SIZE_IMG_H*1; ++x_loop) {
				small_img[x_loop] = (unsigned char) round(((((float)small_img[x_loop]-min_a))/(max_a-min_a))*254);
			}
			// if Recording key pressed (R), write image to disk
			if(state->savepng == 1){
				//save current image (of accumulated numSpikes) to disk 
				char id_img[15];
				char ext[] =".png"; // png output (possibles other formats supported by the library are bmp, ppm, TGA, psd, pnm, hdr, gif,..)
				//check in which mode we are (testing/training/none)
				char filename[255] = DIRECTORY_IMG;	
				if(state->mode == TESTING){
					strcat(filename,"_testing_spikes_"); 
				}
				if(state->mode == TRAINING_POSITIVES){
					strcat(filename,"_pos_spikes_"); 
				}
				if(state->mode == TRAINING_NEGATIVES){
					strcat(filename,"_neg_spikes_");
				}
				sprintf(id_img, "%d", state->counterImg);
				strcat(filename, id_img); //append id_img
				strcat(filename, ext); //append extension 
				FILE *f = fopen(filename, "wb");
				stbi_write_png(filename, SIZE_IMG_W, SIZE_IMG_H, 1, small_img, SIZE_IMG_W*1);
				fclose(f);
				*file_string = strdup(filename);
				//printf("File copied %s\n", *file_string);
				if(*file_string ==  NULL){
					caerLog(CAER_LOG_DEBUG, moduleData->moduleSubSystemString, "Error file name for png image not valid..");
					return;
				} 
			}
			state->counter = 0;
			//printf("\nImage Streamer: \t\t generated image number %d\n", state->counterImg);	
			for(x_loop=0; x_loop<= state->sizeMaxX; x_loop++){
				for(y_loop=0; y_loop<= state->sizeMaxY; y_loop++){
					state->ImageMap[x_loop][y_loop] = 0 ;
				}
			}				


	   // free chunks of memory
       free(img_coor);
       free(image_map);
       free(state->frameRenderer);
       state->frameRenderer = NULL;

	   //select context/window
	   glfwMakeContextCurrent(state->window);

	   #define checkImageWidth SIZE_IMG_W
	   #define checkImageHeight SIZE_IMG_H
	   static GLubyte checkImage[checkImageHeight][checkImageWidth][4];
	   static GLuint texName;
      	   //init image for viewer 
	   int i, j, c;
	      for (i = 0; i < checkImageHeight; i++) {
	         for (j = 0; j < checkImageWidth; j++) {
	       	    c = j * checkImageHeight + i;
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
	   glTexCoord2f(0.0, 0.0); glVertex3f(-1.0f, 1.0f, 0.0f); // Top Left 
	   glTexCoord2f(0.0, 1.0); glVertex3f( 1.0f, 1.0f, 0.0f); // Top Right
	   glTexCoord2f(1.0, 1.0); glVertex3f( 1.0f,-1.0f, 0.0f); // Bottom Right
	   glTexCoord2f(1.0, 0.0); glVertex3f(-1.0f,-1.0f, 0.0f); // Bottom Left
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

