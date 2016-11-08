#include "dynapse_common.h"

static inline const char *chipIDToName(int16_t chipID, bool withEndSlash) {
    switch (chipID) {
        case 64:
            return ((withEndSlash) ? ("DYNAPSEFX2/") : ("DYNAPSEFX2"));
            break;
    }
    
    return ((withEndSlash) ? ("Unknown/") : ("Unknown"));
}

static void mainloopDataNotifyIncrease(void *p) {
    caerMainloopData mainloopData = p;
    
    atomic_fetch_add_explicit(&mainloopData->dataAvailable, 1, memory_order_release);
}

static void mainloopDataNotifyDecrease(void *p) {
    caerMainloopData mainloopData = p;
    
    // No special memory order for decrease, because the acquire load to even start running
    // through a mainloop already synchronizes with the release store above.
    atomic_fetch_sub_explicit(&mainloopData->dataAvailable, 1, memory_order_relaxed);
}

static void moduleShutdownNotify(void *p) {
    sshsNode moduleNode = p;
    
    // Ensure parent also shuts down (on disconnected device for example).
    sshsNodePutBool(moduleNode, "running", false);
}


static void createCoarseFineBiasSetting(sshsNode biasNode, const char *biasName,  const char *coarseValue,
                                        uint8_t fineValue, const char *hlbias, const char *currentLevel,
                                        const char *sex, bool enabled) {
    // Add trailing slash to node name (required!).
    size_t biasNameLength = strlen(biasName);
    char biasNameFull[biasNameLength + 2];
    memcpy(biasNameFull, biasName, biasNameLength);
    biasNameFull[biasNameLength] = '/';
    biasNameFull[biasNameLength + 1] = '\0';
    
    
    // Create configuration node for this particular bias.
    sshsNode biasConfigNode = sshsGetRelativeNode(biasNode, biasNameFull);
    
    // Add bias settings.
    sshsNodePutStringIfAbsent(biasConfigNode, "coarseValue", coarseValue);
    sshsNodePutShortIfAbsent(biasConfigNode, "fineValue", I16T(fineValue));
    sshsNodePutStringIfAbsent(biasConfigNode, "BiasLowHi", hlbias);
    sshsNodePutStringIfAbsent(biasConfigNode, "currentLevel", currentLevel);
    sshsNodePutStringIfAbsent(biasConfigNode, "sex", sex);
    sshsNodePutStringIfAbsent(biasConfigNode, "enable", enabled);
}


static void createDefaultConfiguration(caerModuleData moduleData, struct caer_dynapse_info *devInfo) {
    // Device related configuration has its own sub-node.
    sshsNode deviceConfigNode = sshsGetRelativeNode(moduleData->moduleNode, chipIDToName(devInfo->chipID, true));
    
    // Chip biases, based on testing defaults.
    sshsNode biasNode = sshsGetRelativeNode(deviceConfigNode, "bias/");
    
    createCoarseFineBiasSetting(biasNode,"C0_IF_BUF_P","15p",0,"HighBias","Normal","PBias",true);
   /* C0_IF_RFR_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C0_IF_NMDA_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C0_IF_DC_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C0_IF_TAU1_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C0_IF_TAU2_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C0_IF_THR_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C0_IF_AHW_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C0_IF_AHTAU_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C0_IF_AHTHR_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C0_IF_CASC_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C0_PULSE_PWLK_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C0_PS_WEIGHT_INH_S_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C0_PS_WEIGHT_INH_F_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C0_PS_WEIGHT_EXC_S_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C0_PS_WEIGHT_EXC_F_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C0_NPDPII_TAU_S_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C0_NPDPII_THR_S_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C0_NPDPII_TAU_F_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C0_NPDPII_THR_F_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C0_NPDPIE_TAU_S_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C0_NPDPIE_THR_S_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C0_NPDPIE_TAU_F_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C0_NPDPIE_THR_F_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C0_R2R_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C1_IF_BUF_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C1_IF_RFR_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C1_IF_NMDA_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C1_IF_DC_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C1_IF_TAU1_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C1_IF_TAU2_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C1_IF_THR_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C1_IF_AHW_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C1_IF_AHTAU_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C1_IF_AHTHR_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C1_IF_CASC_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C1_PULSE_PWLK_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C1_PS_WEIGHT_INH_S_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C1_PS_WEIGHT_INH_F_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C1_PS_WEIGHT_EXC_S_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C1_PS_WEIGHT_EXC_F_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C1_NPDPII_TAU_S_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C1_NPDPII_THR_S_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C1_NPDPII_TAU_F_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C1_NPDPII_THR_F_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C1_NPDPIE_TAU_S_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C1_NPDPIE_THR_S_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C1_NPDPIE_TAU_F_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C1_NPDPIE_THR_F_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C1_R2R_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C2_IF_BUF_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C2_IF_RFR_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C2_IF_NMDA_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C2_IF_DC_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C2_IF_TAU1_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C2_IF_TAU2_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C2_IF_THR_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C2_IF_AHW_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C2_IF_AHTAU_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C2_IF_AHTHR_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C2_IF_CASC_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C2_PULSE_PWLK_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C2_PS_WEIGHT_INH_S_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C2_PS_WEIGHT_INH_F_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C2_PS_WEIGHT_EXC_S_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C2_PS_WEIGHT_EXC_F_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C2_NPDPII_TAU_S_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C2_NPDPII_THR_S_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C2_NPDPII_TAU_F_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C2_NPDPII_THR_F_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C2_NPDPIE_TAU_S_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C2_NPDPIE_THR_S_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C2_NPDPIE_TAU_F_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C2_NPDPIE_THR_F_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C2_R2R_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C3_IF_BUF_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C3_IF_RFR_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C3_IF_NMDA_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C3_IF_DC_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C3_IF_TAU1_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C3_IF_TAU2_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C3_IF_THR_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C3_IF_AHW_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C3_IF_AHTAU_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C3_IF_AHTHR_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C3_IF_CASC_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C3_PULSE_PWLK_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C3_PS_WEIGHT_INH_S_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C3_PS_WEIGHT_INH_F_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C3_PS_WEIGHT_EXC_S_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C3_PS_WEIGHT_EXC_F_N,15p,0,HighBias,Normal,NBias,BiasEnable,
    C3_NPDPII_TAU_S_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C3_NPDPII_THR_S_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C3_NPDPII_TAU_F_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C3_NPDPII_THR_F_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C3_NPDPIE_TAU_S_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C3_NPDPIE_THR_S_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C3_NPDPIE_TAU_F_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C3_NPDPIE_THR_F_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    C3_R2R_P,15p,0,HighBias,Normal,PBias,BiasEnable,
    U_Buffer,3.2u,80,NONE,NONE,NONE,NONE,
    U_SSP,SPECIAL,7,NONE,NONE,NONE,NONE,
    U_SSN,SPECIAL,15,NONE,NONE,NONE,NONE,
    D_Buffer,3.2u,80,NONE,NONE,NONE,NONE,
    D_SSP,SPECIAL,7,NONE,NONE,NONE,NONE,
    D_SSN,SPECIAL,15,NONE,NONE,NONE,NONE,*/
    
    
}

static void sendDefaultConfiguration(caerModuleData moduleData, struct caer_dynapse_info *devInfo) {
    // Device related configuration has its own sub-node.
    sshsNode deviceConfigNode = sshsGetRelativeNode(moduleData->moduleNode, chipIDToName(devInfo->chipID, true));
    
}


bool caerInputDYNAPSEInit(caerModuleData moduleData, uint16_t deviceType) {

    caerLog(CAER_LOG_DEBUG, moduleData->moduleSubSystemString, "Initializing module ...");

    // USB port/bus/SN settings/restrictions.
    // These can be used to force connection to one specific device at startup.
    sshsNodePutShortIfAbsent(moduleData->moduleNode, "busNumber", 0);
    sshsNodePutShortIfAbsent(moduleData->moduleNode, "devAddress", 0);
    sshsNodePutStringIfAbsent(moduleData->moduleNode, "serialNumber", "");
    
    // Add auto-restart setting.
    sshsNodePutBoolIfAbsent(moduleData->moduleNode, "autoRestart", true);
    
    /// Start data acquisition, and correctly notify mainloop of new data and module of exceptional
    // shutdown cases (device pulled, ...).
    char *serialNumber = sshsNodeGetString(moduleData->moduleNode, "serialNumber");
    caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString, "Initializing module ... %d", deviceType);

    moduleData->moduleState = caerDeviceOpen(1, CAER_DEVICE_DYNAPSE, 0, 0, NULL);
    
    free(serialNumber);
    
    if (moduleData->moduleState == NULL) {
        // Failed to open device.
        return (false);
    }

    // Let's take a look at the information we have on the device.
    struct caer_dynapse_info dynapse_info = caerDynapseInfoGet(moduleData->moduleState);
    
    printf("%s --- ID: %d, Master: %d,  Logic: %d,  ChipID: %d.\n",
           dynapse_info.deviceString, dynapse_info.deviceID,
           dynapse_info.deviceIsMaster, dynapse_info.logicVersion, dynapse_info.chipID);
    
    
    sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");
    
    sshsNodePutLong(sourceInfoNode, "highestTimestamp", -1);
    sshsNodePutShort(sourceInfoNode, "logicVersion", dynapse_info.logicVersion);
    sshsNodePutBool(sourceInfoNode, "deviceIsMaster", dynapse_info.deviceIsMaster);
    sshsNodePutShort(sourceInfoNode, "deviceID", dynapse_info.deviceID);
    sshsNodePutShort(sourceInfoNode, "chipID", dynapse_info.chipID);

    
    // Generate source string for output modules.
    size_t sourceStringLength = (size_t) snprintf(NULL, 0, "#Source %" PRIu16 ": %s\r\n", moduleData->moduleID,
                                                  chipIDToName(dynapse_info.chipID, false));
    
    char sourceString[sourceStringLength + 1];
    snprintf(sourceString, sourceStringLength + 1, "#Source %" PRIu16 ": %s\r\n", moduleData->moduleID,
             chipIDToName(dynapse_info.chipID, false));
    sourceString[sourceStringLength] = '\0';
    
    sshsNodePutString(sourceInfoNode, "sourceString", sourceString);
    
    // Generate sub-system string for module.
    size_t subSystemStringLength = (size_t) snprintf(NULL, 0, "%s[SN %s, %" PRIu8 ":%" PRIu8 "]",
                                                     moduleData->moduleSubSystemString, dynapse_info.deviceSerialNumber, dynapse_info.deviceUSBBusNumber,
                                                     dynapse_info.deviceUSBDeviceAddress);
    
    char subSystemString[subSystemStringLength + 1];
    snprintf(subSystemString, subSystemStringLength + 1, "%s[SN %s, %" PRIu8 ":%" PRIu8 "]",
             moduleData->moduleSubSystemString, dynapse_info.deviceSerialNumber, dynapse_info.deviceUSBBusNumber,
             dynapse_info.deviceUSBDeviceAddress);
    subSystemString[subSystemStringLength] = '\0';
    
    caerModuleSetSubSystemString(moduleData, subSystemString);
    

    // Ensure good defaults for data acquisition settings.
    // No blocking behavior due to mainloop notification, and no auto-start of
    // all producers to ensure cAER settings are respected.
    caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_DATAEXCHANGE,
                        CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, false);
    caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_DATAEXCHANGE,
                        CAER_HOST_CONFIG_DATAEXCHANGE_START_PRODUCERS, false);
    caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_DATAEXCHANGE,
                        CAER_HOST_CONFIG_DATAEXCHANGE_STOP_PRODUCERS, true);
    
    // Create default settings and send them to the device.
    //createDefaultConfiguration(moduleData, &dynapse_info);
    //sendDefaultConfiguration(moduleData, &dynapse_info);
    
    // Start data acquisition.
    bool ret = caerDeviceDataStart(moduleData->moduleState, &mainloopDataNotifyIncrease, &mainloopDataNotifyDecrease,
                                   caerMainloopGetReference(), &moduleShutdownNotify, moduleData->moduleNode);
    
    if (!ret) {
        // Failed to start data acquisition, close device and exit.
        caerDeviceClose((caerDeviceHandle *) &moduleData->moduleState);
        
        return (false);
    }
    
    // Device related configuration has its own sub-node.
    sshsNode deviceConfigNode = sshsGetRelativeNode(moduleData->moduleNode, chipIDToName(dynapse_info.chipID, true));
    

    sshsNode biasNode = sshsGetRelativeNode(deviceConfigNode, "bias/");
    
    size_t biasNodesLength = 0;
    sshsNode *biasNodes = sshsNodeGetChildren(biasNode, &biasNodesLength);
    
    
    return (true);

}

void caerInputDYNAPSEExit(caerModuleData moduleData) {
    // Device related configuration has its own sub-node.
    struct caer_dynapse_info devInfo = caerDynapseInfoGet(moduleData->moduleState);
    sshsNode deviceConfigNode = sshsGetRelativeNode(moduleData->moduleNode, chipIDToName(devInfo.chipID, true));

    
    caerDeviceDataStop(moduleData->moduleState);
    
    caerDeviceClose((caerDeviceHandle *) &moduleData->moduleState);
    
    if (sshsNodeGetBool(moduleData->moduleNode, "autoRestart")) {
        // Prime input module again so that it will try to restart if new devices detected.
        sshsNodePutBool(moduleData->moduleNode, "running", true);
    }
}

void caerInputDYNAPSERun(caerModuleData moduleData, size_t argsNumber, va_list args) {
    UNUSED_ARGUMENT(argsNumber);
    
    // Interpret variable arguments (same as above in main function).
    caerEventPacketContainer *container = va_arg(args, caerEventPacketContainer *);
    
    *container = caerDeviceDataGet(moduleData->moduleState);
    
    if (*container != NULL) {
        caerMainloopFreeAfterLoop((void (*)(void *)) &caerEventPacketContainerFree, *container);
        
        sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");
        sshsNodePutLong(sourceInfoNode, "highestTimestamp",
                        caerEventPacketContainerGetHighestEventTimestamp(*container));
        
        // Detect timestamp reset and call all reset functions for processors and outputs.
        caerEventPacketHeader special = caerEventPacketContainerGetEventPacket(*container, SPECIAL_EVENT);
        
        if ((special != NULL) && (caerEventPacketHeaderGetEventNumber(special) == 1)
            && (caerSpecialEventPacketFindEventByType((caerSpecialEventPacket) special, TIMESTAMP_RESET) != NULL)) {
            caerMainloopResetProcessors(moduleData->moduleID);
            caerMainloopResetOutputs(moduleData->moduleID);
            
            // Update master/slave information.
            struct caer_dynapse_info devInfo = caerDynapseInfoGet(moduleData->moduleState);
            sshsNodePutBool(sourceInfoNode, "deviceIsMaster", devInfo.deviceIsMaster);
        }
    }
}

