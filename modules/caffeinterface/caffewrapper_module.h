/* Caffe Wrapper interface
 * Author: federico.corradi@inilabs.com
 */
const char * caerCaffeWrapper(uint16_t moduleID, int * classifyhist, int size,
		char *classificationResults,
		caerFrameEventPacket *networkActivity, int sizeDisplay);


