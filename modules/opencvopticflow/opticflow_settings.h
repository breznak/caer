#ifndef OPTICFLOW_SETTINGS_H_
#define OPTICFLOW_SETTINGS_H_

struct OpticFlowSettings_struct {
        bool doOpticFlow;
        uint32_t imageWidth;
        uint32_t imageHeigth;
};

typedef struct OpticFlowSettings_struct *OpticFlowSettings;

#endif /* OPTICFLOW_SETTINGS_H_ */
