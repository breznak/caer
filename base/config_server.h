#ifndef CONFIG_SERVER_H_
#define CONFIG_SERVER_H_

#include "main.h"

enum caer_config_actions {
	CAER_CONFIG_NODE_EXISTS = 0,
	CAER_CONFIG_ATTR_EXISTS = 1,
	CAER_CONFIG_GET = 2,
	CAER_CONFIG_PUT = 3,
	CAER_CONFIG_ERROR = 4,
	CAER_CONFIG_GET_CHILDREN = 5,
	CAER_CONFIG_GET_ATTRIBUTES = 6,
	CAER_CONFIG_GET_TYPES = 7,
};

void caerConfigServerStart(void);
void caerConfigServerStop(void);

#endif /* CONFIG_SERVER_H_ */
