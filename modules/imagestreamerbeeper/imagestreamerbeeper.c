/*
 *  beeps if classific_results is != 0
 *  federico.corradi@inilabs.com
 */

#include "imagestreamerbeeper.h"
#include "base/mainloop.h"
#include "base/module.h"
#include <allegro5/allegro_audio.h>
#include <allegro5/allegro_acodec.h>


static bool caerImagestreamerBeeperInit(caerModuleData moduleData);
static void caerImagestreamerBeeperRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerImagestreamerBeeperExit(caerModuleData moduleData);

static struct caer_module_functions caerImagestreamerBeeperFunctions = { .moduleInit = &caerImagestreamerBeeperInit,
	.moduleRun = &caerImagestreamerBeeperRun, .moduleConfig =
	NULL, .moduleExit = &caerImagestreamerBeeperExit };

void caerImagestreamerBeeper(uint16_t moduleID, double  * classific_results, int max_img_qty) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "ImageStreamerBeeper", PROCESSOR);
	if (moduleData == NULL) {
		return;
	}

	caerModuleSM(&caerImagestreamerBeeperFunctions, moduleData, sizeof(struct imagestreamerbeeper_state), 2, classific_results, max_img_qty);
}

static bool caerImagestreamerBeeperInit(caerModuleData moduleData) {

	caerImagestreamerBeeperState state = moduleData->moduleState;
	sshsNodePutDoubleIfAbsent(moduleData->moduleNode, "detThreshold", 0.96);
	state->detThreshold = sshsNodeGetDouble(moduleData->moduleNode, "detThreshold");

	if (!al_init()) {
		fprintf(stderr, "failed to initialize allegro!\n");
		return (false);
	}
	if (!al_install_audio()) {
		fprintf(stderr, "failed to initialize audio!\n");
		return (false);
	}
	if (!al_init_acodec_addon()) {
		fprintf(stderr, "failed to initialize audio codecs!\n");
		return (false);
	}
	if (!al_reserve_samples(1)) {
		fprintf(stderr, "failed to reserve samples!\n");
		return (false);
	}

	return (true);
}

static void caerImagestreamerBeeperExit(caerModuleData moduleData) {
	caerImagestreamerBeeperState state = moduleData->moduleState;

}

static void caerImagestreamerBeeperRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);
	
	// Interpret variable arguments (same as above in main function).
	double  * res = va_arg(args, double*);
	int max_img_qty = va_arg(args, int);

	caerImagestreamerBeeperState state = moduleData->moduleState;
	state->detThreshold = sshsNodeGetDouble(moduleData->moduleNode, "detThreshold");

	ALLEGRO_SAMPLE *sample = NULL;

	sample = al_load_sample(AUDIO_BEEP_FILE);
	al_reserve_samples(1);
	
	for(int uu=0; uu< 1 ; uu++){
	        if(res[uu] != 0.0f){
			if ( res[uu] >= state->detThreshold) {
			if (!sample) {
					printf("Audio clip sample not loaded!\n");
				}
				else {
					al_play_sample(sample, 100.0, 0.0, 1.0, ALLEGRO_PLAYMODE_ONCE, 0);
					al_rest(0.06);
				}
			}
		}
	}
	al_destroy_sample(sample);

}
