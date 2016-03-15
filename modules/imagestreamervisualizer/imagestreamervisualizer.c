/*
 *  accumulates a fixed number of events and generates png pictures
 *  it also displays png images when available
 *  federico.corradi@inilabs.com
 */

#include "imagestreamervisualizer.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "ext/portable_time.h"


static bool caerImagestreamerVisualizerInit(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerImagestreamerVisualizerRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerImagestreamerVisualizerExit(caerModuleData moduleData);
static int caerImagestreamerVisualizerRenderThread(void *moduleData);
static bool initializeFrameRenderer(caerImagestreamerVisualizerState state, int16_t sourceID);

static struct caer_module_functions caerImagestreamerVisualizerFunctions = { .moduleInit =
	&caerImagestreamerVisualizerInit, .moduleRun = &caerImagestreamerVisualizerRun, .moduleConfig =
NULL, .moduleExit = &caerImagestreamerVisualizerExit };

void caerImagestreamerVisualizer(uint16_t moduleID, unsigned char * disp_img, const int disp_img_size, double * classific_results, int * classific_sizes, int max_img_qty) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "ImageStreamerVisualizer"); //"ImageStreamerVisualizer");
        
	caerModuleSM(&caerImagestreamerVisualizerFunctions, moduleData, sizeof(struct imagestreamervisualizer_state), 5,
		disp_img, disp_img_size, classific_results, classific_sizes, max_img_qty);

	return;
}

static bool caerImagestreamerVisualizerInit(caerModuleData moduleData, size_t argsNumber, va_list args) {
	
    caerImagestreamerVisualizerState state = moduleData->moduleState;

    sshsNodePutByteIfAbsent(moduleData->moduleNode, "savepng", 0);
    sshsNodePutByteIfAbsent(moduleData->moduleNode, "mode", 0);
       
    // Start separate rendering thread. Decouples presentation from
    // data processing and preparation. Communication over properly
    // locked bitmap.
    
/*  if(!al_init()) {
        fprintf(stderr, "failed to initialize allegro!\n");
        return -1;
    }
    state->displayWindow = al_create_display(IMAGESTREAMERVISUALIZER_SCREEN_WIDTH,IMAGESTREAMERVISUALIZER_SCREEN_HEIGHT);
    if( state->displayWindow == NULL) {
        fprintf(stderr, "failed to create display!\n");
        return -1;
    }
    state->bb = al_get_backbuffer(state->displayWindow);
    al_set_window_title(state->displayWindow, "Image Generator: Accumulated spikes");
    al_clear_to_color(al_map_rgb(0,0,0));
    al_flip_display();
    if(!al_install_keyboard()) {
        fprintf(stderr, "failed to initialize the keyboard!\n");
        return -1;
    }
    state->event_queue = al_create_event_queue();
    if(!state->event_queue) {
        fprintf(stderr, "failed to create event_queue!\n");
        return -1;
    }
    
    al_register_event_source(state->event_queue, al_get_keyboard_event_source());

*/


    if (!atomic_load_explicit(&state->vis_state.running, memory_order_relaxed)) {
		if (!caerVisualizerInit(&state->vis_state, IMAGESTREAMERVISUALIZER_SCREEN_WIDTH, IMAGESTREAMERVISUALIZER_SCREEN_HEIGHT, 2, true)) {
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to initialize image streamer visualizer.");
			return (false);
		}
    }

    thrd_create(&state->renderingThread, &caerImagestreamerVisualizerRenderThread, moduleData);
    
    return (true);

}

static int caerImagestreamerVisualizerRenderThread(void *moduleData) {

	caerModuleData data = moduleData;
	caerImagestreamerVisualizerState state = data->moduleState;
	
	while (atomic_load_explicit(&data->running, memory_order_relaxed)) {
		if (atomic_load_explicit(&state->vis_state.running, memory_order_relaxed)) {
			caerVisualizerUpdateScreen(&state->vis_state);
		}
	}
	
	return (thrd_success);
}

static void caerImagestreamerVisualizerExit(caerModuleData moduleData) {
	caerImagestreamerVisualizerState state = moduleData->moduleState;

	// Wait on rendering thread.
	thrd_join(state->renderingThread, NULL);

	if (atomic_load(&state->vis_state.running)) {
		caerVisualizerExit(&state->vis_state);
	}
	
}

static void caerImagestreamerVisualizerRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);
	
	caerImagestreamerVisualizerState state = moduleData->moduleState;

	// Interpret variable arguments (same as above in main function).
	unsigned char * disp_img = va_arg(args, char *);
	const int DISPLAY_IMG_SIZE = va_arg(args, const int);
    double * classific_results = va_arg(args, double *);
    int * classific_sizes = va_arg(args, int *);
    int max_img_qty = va_arg(args, int);
    
    //update saving state, might have been changed by user key inputs
    sshsNodePutByte(moduleData->moduleNode, "savepng", savepng_state);
    sshsNodePutByte(moduleData->moduleNode, "mode", mode);
            
    //create one frame event packet, one single frame
    caerFrameEventPacket my_frame_packet = caerFrameEventPacketAllocate(1, moduleData->moduleID, 0, IMAGESTREAMERVISUALIZER_SCREEN_WIDTH, IMAGESTREAMERVISUALIZER_SCREEN_HEIGHT, 1);
    //get that single frame
	caerFrameEvent my_frame = caerFrameEventPacketGetEvent(my_frame_packet,0);

	
    // only proceed if we have a display image
    if (disp_img == NULL){
        return;
    }
   
    #define checkImageWidth 256 //DISPLAY_IMG_SIZE; not working
    #define checkImageHeight 256 //DISPLAY_IMG_SIZE; not working
    unsigned char *small_img = disp_img;

	//now put stuff into the frame
	int c,counter;
	counter = 0;
	for(int i=0; i<IMAGESTREAMERVISUALIZER_SCREEN_HEIGHT; i++){
	    for(int y=0; y<IMAGESTREAMERVISUALIZER_SCREEN_HEIGHT; y++){
	        //c = y * DISPLAY_IMG_SIZE + i;
	        my_frame->pixels[counter] = (uint16_t)(rand()%65000);//(uint16_t)(small_img[c]<<8); 
	        counter += 1;
	    }
	}
	
    //add info to the frame
    caerFrameEventSetLengthXLengthYChannelNumber(my_frame, IMAGESTREAMERVISUALIZER_SCREEN_WIDTH, IMAGESTREAMERVISUALIZER_SCREEN_HEIGHT,
	1, my_frame_packet);
	//valido
	caerFrameEventValidate(my_frame,my_frame_packet);
	
	//update bitmap
	caerVisualizerUpdate(&my_frame_packet->packetHeader, &state->vis_state);
	free(my_frame_packet);
	
    //static GLubyte checkImage[checkImageHeight][checkImageWidth][4];
    //static GLuint texName;

/*    ALLEGRO_EVENT keypressed;
    bool get_event = al_get_next_event(state->event_queue, &keypressed);
    if( get_event ){
        switch(keypressed.type){
          case ALLEGRO_EVENT_KEY_DOWN:
            if(keypressed.keyboard.keycode == ALLEGRO_KEY_T){
                printf("\nImage Streamer Filter: start saving png files to hw\n");
                savepng_state = 1;
                mode = TESTING; //testing
            }
            if(keypressed.keyboard.keycode == ALLEGRO_KEY_S){
                printf("\nImage Streamer Filter: stop saving png files\n");
	            savepng_state = 0; //stop testing
	            mode = TESTING;
            }          
            if(keypressed.keyboard.keycode == ALLEGRO_KEY_N){
                printf("\nImage Streamer Filter: start capturing negative pngs\n");
                savepng_state = 1;
	            mode = TRAINING_NEGATIVES; 
            }      
            if(keypressed.keyboard.keycode == ALLEGRO_KEY_P){
                printf("\nImage Streamer Filter: start capturing positive pngs\n");
                savepng_state = 1;
	            mode = TRAINING_POSITIVES;
            }     	
         case ALLEGRO_EVENT_KEY_UP:
            if(keypressed.keyboard.keycode == ALLEGRO_KEY_T){}
	        if(keypressed.keyboard.keycode == ALLEGRO_KEY_S){}
            if(keypressed.keyboard.keycode == ALLEGRO_KEY_N){}
            if(keypressed.keyboard.keycode == ALLEGRO_KEY_P){}
        }
        get_event = false;
    }

    //lock bitmap
    al_set_target_bitmap(state->bb);
    ALLEGRO_LOCKED_REGION * lock;
    lock = al_lock_bitmap(state->bb, ALLEGRO_PIXEL_FORMAT_ABGR_8888_LE, ALLEGRO_LOCK_WRITEONLY);
    unsigned char *ptr = (lock->data);
    ALLEGRO_COLOR color;
    unsigned char red,green,blue,alpha;
    int c;
    //init to zero
    //al_clear_to_color(al_map_rgb(0,0,0));
    for (int x = 0; x != IMAGESTREAMERVISUALIZER_SCREEN_WIDTH; ++x) {
        for (int y = 0; y != IMAGESTREAMERVISUALIZER_SCREEN_HEIGHT; ++y) {
            color = al_map_rgb(0,0,0);
            al_put_pixel(x, y, color); 
        }
    }    
    for (int x = 0; x != DISPLAY_IMG_SIZE; ++x) {
         for (int y = 0; y != DISPLAY_IMG_SIZE; ++y) {
             c = y * DISPLAY_IMG_SIZE + x;
             color = al_map_rgb(small_img[c],small_img[c],small_img[c]);
             al_put_pixel(x, y, color); 
         }
    }

    //draw red square if classification is successfull
    if (classific_results[0] > 0 ){
        //printf("Classification results availale\n");
        //printf("%f\n", classific_results[0]);
        if(classific_results[0] > THR){
            for (int x = 0; x != DISPLAY_IMG_SIZE; ++x) {
                for (int y = 0; y != 5; ++y) {
                   c = y * DISPLAY_IMG_SIZE + x; 
                   color = al_map_rgb((int)(classific_results[0]*255),0,0);
                   al_put_pixel(x, y, color); 
                }
            }
            for (int x = 0; x != DISPLAY_IMG_SIZE; ++x) {
                for (int y = DISPLAY_IMG_SIZE; y != DISPLAY_IMG_SIZE-5; y--) {
                   c = y * DISPLAY_IMG_SIZE + x; 
                   color = al_map_rgb((int)(classific_results[0]*255),0,0);
                   al_put_pixel(x, y, color); 
                }
            }            
            for (int y = 0; y != DISPLAY_IMG_SIZE; ++y) {
                for (int x = 0; x != 5; x++) {
                   c = y * DISPLAY_IMG_SIZE + x; 
                   color = al_map_rgb((int)(classific_results[0]*255),0,0);
                   al_put_pixel(x, y, color); 
                }
            }            
            for (int y = 0; y != DISPLAY_IMG_SIZE; ++y) {
                for (int x = DISPLAY_IMG_SIZE; x != DISPLAY_IMG_SIZE-5; x--) {
                   c = y * DISPLAY_IMG_SIZE + x; 
                   color = al_map_rgb((int)(classific_results[0]*255),0,0);
                   al_put_pixel(x, y, color); 
                }
            }   
        }
    }

    //unlock
    al_unlock_bitmap(state->bb);
    //flip
    al_flip_display();
    */
}



