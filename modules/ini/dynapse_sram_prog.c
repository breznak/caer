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


bool caerSramProgInit(caerModuleData moduleData);
void caerSramProgExit(caerModuleData moduleData);

bool caerSramProgInit(caerModuleData moduleData) {

	caerInputDynapseState state = moduleData->moduleState;

	sshsNode spikeNode = sshsGetRelativeNode(moduleData->moduleNode, "sramProg/");

	sshsNodePutBoolIfAbsent(spikeNode, "doProg", false);
	sshsNodePutIntIfAbsent(spikeNode, "chip_id", DYNAPSE_CONFIG_DYNAPSE_U2); // default is chip U2
	sshsNodePutIntIfAbsent(spikeNode, "neuron_id", 0);
	sshsNodePutIntIfAbsent(spikeNode, "sram_addr", 0);
	sshsNodePutIntIfAbsent(spikeNode, "core_id", 0);
	sshsNodePutIntIfAbsent(spikeNode, "dest_core_id", 0);
	sshsNodePutIntIfAbsent(spikeNode, "dx", 0);
	sshsNodePutBoolIfAbsent(spikeNode, "sx", true);
	sshsNodePutIntIfAbsent(spikeNode, "dy", 0);
	sshsNodePutBoolIfAbsent(spikeNode, "sy", true);
	sshsNodePutIntIfAbsent(spikeNode, "virtual_core_id", 3);

	return (true);
}



