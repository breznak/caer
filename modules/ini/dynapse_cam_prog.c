#include "base/module.h"

#include <math.h>
#include <stdatomic.h>
#include <sys/types.h>
#include <fcntl.h>
#include <time.h>

#include "main.h"
#include "dynapse_common.h"
#include <libcaer/events/packetContainer.h>
#include <libcaer/events/spike.h>


bool caerCamProgInit(caerModuleData moduleData);

bool caerCamProgInit(caerModuleData moduleData) {

	caerInputDynapseState state = moduleData->moduleState;

	sshsNode spikeNode = sshsGetRelativeNode(moduleData->moduleNode, "camProg/");

	sshsNodePutBoolIfAbsent(spikeNode, "doProg", false);
	sshsNodePutIntIfAbsent(spikeNode, "chip_id", DYNAPSE_CONFIG_DYNAPSE_U2); // default is chip U2
	sshsNodePutIntIfAbsent(spikeNode, "col_addr", 13);
	sshsNodePutIntIfAbsent(spikeNode, "row_addr", 3);
	sshsNodePutIntIfAbsent(spikeNode, "cam_addr", 1);
	sshsNodePutIntIfAbsent(spikeNode, "core_id", true);
	sshsNodePutIntIfAbsent(spikeNode, "core_s", true);
	sshsNodePutIntIfAbsent(spikeNode, "address", true);
	sshsNodePutBoolIfAbsent(spikeNode, "ei", true);
	sshsNodePutBoolIfAbsent(spikeNode, "fs", 15);

	return (true);
}





