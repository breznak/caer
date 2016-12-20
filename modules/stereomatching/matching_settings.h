#ifndef STEREOMATCHING_SETTINGS_H_
#define STEREOMATCHING_SETTINGS_H_

enum StereoMatchingAlg { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3, STEREO_3WAY=4 };

struct StereoMatchingSettings_struct {
	int alg;
	int doMatching;
	int captureDelay;
	char * loadFileName_extrinsic;
	char * loadFileName_intrinsic;
};

typedef struct StereoMatchingSettings_struct *StereoMatchingSettings;


#endif /* STEREOMATCHING_SETTINGS_H_ */
