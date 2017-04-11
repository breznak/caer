#ifndef OPTICFLOW_WRAPPER_H_
#define OPTICFLOW_WRAPPER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "opticflow_settings.h"

typedef struct OpticFlow OpticFlow;

OpticFlow *OpticFlow_init(OpticFlowSettings settings);
void OpticFlow_destroy(OpticFlow *calibClass);
void OpticFlow_updateSettings(OpticFlow *calibClass);
bool OpticFlow_doOpticFlow(OpticFlow *calibClass, caerFrameEvent * frame, caerFrameEvent * frameInput, int sizeX, int sizeY);

#ifdef __cplusplus
}
#endif

#endif /* OPTICFLOW_WRAPPER_H_ */
