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

void caerImagestreamerBeeper(uint16_t moduleID, caerEventPacketHeader imagestreamer, double * classific_results) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "ImageStreamerVisualizer");
	if (moduleData == NULL) {
		return;
	}

	caerModuleSM(&caerImagestreamerBeeperFunctions, moduleData, 2, imagestreamer, classific_results);
}

static bool caerImagestreamerBeeperInit(caerModuleData moduleData) {

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
	caerEventPacketHeader imagestreamer = va_arg(args, caerEventPacketHeader);
	double * classific_results = va_arg(args, double *);

	ALLEGRO_SAMPLE *sample = NULL;

	if (*classific_results != 0) {
		sample = al_load_sample(AUDIO_BEEP_FILE);
		al_reserve_samples(1);
		if (!sample) {
			printf("Audio clip sample not loaded!\n");
		}
		else {
			al_play_sample(sample, 100.0, 0.0, 1.0, ALLEGRO_PLAYMODE_ONCE, 0);
			al_rest(0.06);
		}
		al_destroy_sample(sample);
	}

}
