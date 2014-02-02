#ifndef SSHS_H_
#define SSHS_H_

// Common includes, useful for everyone.
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>

// SSHS node
typedef struct sshs_node *sshsNode;

enum sshs_node_attr_value_type {
	BOOL = 0, BYTE = 1, SHORT = 2, INT = 3, LONG = 4, FLOAT = 5, DOUBLE = 6, STRING = 7,
};

union sshs_node_attr_value {
	bool boolean;
	uint8_t ubyte;
	uint16_t ushort;
	uint32_t uint;
	uint64_t ulong;
	float ffloat;
	double ddouble;
	char *string;
};

enum sshs_node_node_events {
	CHILD_NODE_ADDED = 0,
};

enum sshs_node_attribute_events {
	ATTRIBUTE_ADDED = 0, ATTRIBUTE_MODIFIED = 1,
};

sshsNode sshsNodeNew(const char *nodeName, sshsNode parent);
const char *sshsNodeGetName(sshsNode node);
const char *sshsNodeGetPath(sshsNode node);
void sshsNodeAddNodeListener(sshsNode node, void *userData,
	void (*node_changed)(sshsNode node, void *userData, enum sshs_node_node_events event, sshsNode changeNode));
void sshsNodeRemoveNodeListener(sshsNode node, void *userData,
	void (*node_changed)(sshsNode node, void *userData, enum sshs_node_node_events event, sshsNode changeNode));
void sshsNodeAddAttrListener(sshsNode node, void *userData,
	void (*attribute_changed)(sshsNode node, void *userData, enum sshs_node_attribute_events event,
		const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue));
void sshsNodeRemoveAttrListener(sshsNode node, void *userData,
	void (*attribute_changed)(sshsNode node, void *userData, enum sshs_node_attribute_events event,
		const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue));
bool sshsNodeAttrExists(sshsNode node, const char *key, enum sshs_node_attr_value_type type);
union sshs_node_attr_value sshsNodeGetAttribute(sshsNode node, const char *key, enum sshs_node_attr_value_type type);
bool sshsNodePutBoolIfAbsent(sshsNode node, const char *key, bool value);
void sshsNodePutBool(sshsNode node, const char *key, bool value);
bool sshsNodeGetBool(sshsNode node, const char *key);
bool sshsNodePutByteIfAbsent(sshsNode node, const char *key, uint8_t value);
void sshsNodePutByte(sshsNode node, const char *key, uint8_t value);
uint8_t sshsNodeGetByte(sshsNode node, const char *key);
bool sshsNodePutShortIfAbsent(sshsNode node, const char *key, uint16_t value);
void sshsNodePutShort(sshsNode node, const char *key, uint16_t value);
uint16_t sshsNodeGetShort(sshsNode node, const char *key);
bool sshsNodePutIntIfAbsent(sshsNode node, const char *key, uint32_t value);
void sshsNodePutInt(sshsNode node, const char *key, uint32_t value);
uint32_t sshsNodeGetInt(sshsNode node, const char *key);
bool sshsNodePutLongIfAbsent(sshsNode node, const char *key, uint64_t value);
void sshsNodePutLong(sshsNode node, const char *key, uint64_t value);
uint64_t sshsNodeGetLong(sshsNode node, const char *key);
bool sshsNodePutFloatIfAbsent(sshsNode node, const char *key, float value);
void sshsNodePutFloat(sshsNode node, const char *key, float value);
float sshsNodeGetFloat(sshsNode node, const char *key);
bool sshsNodePutDoubleIfAbsent(sshsNode node, const char *key, double value);
void sshsNodePutDouble(sshsNode node, const char *key, double value);
double sshsNodeGetDouble(sshsNode node, const char *key);
bool sshsNodePutStringIfAbsent(sshsNode node, const char *key, const char *value);
void sshsNodePutString(sshsNode node, const char *key, const char *value);
char *sshsNodeGetString(sshsNode node, const char *key);
void sshsNodeExportNodeToXML(sshsNode node, int outFd);
void sshsNodeExportSubTreeToXML(sshsNode node, int outFd);
bool sshsNodeImportNodeFromXML(sshsNode node, int inFd, bool strict);
bool sshsNodeImportSubTreeFromXML(sshsNode node, int inFd, bool strict);
bool sshsNodeStringToNodeConverter(sshsNode node, const char *key, const char *type, const char *value);
const char **sshsNodeGetChildNames(sshsNode node, size_t *numNames);
const char **sshsNodeGetAttributeKeys(sshsNode node, size_t *numKeys);
enum sshs_node_attr_value_type *sshsNodeGetAttributeTypes(sshsNode node, const char *key, size_t *numTypes);

// Helper functions
const char *sshsHelperTypeToStringConverter(enum sshs_node_attr_value_type type);
enum sshs_node_attr_value_type sshsHelperStringToTypeConverter(const char *typeString);
char *sshsHelperValueToStringConverter(enum sshs_node_attr_value_type type, union sshs_node_attr_value value);
bool sshsHelperStringToValueConverter(enum sshs_node_attr_value_type type, const char *valueString,
	union sshs_node_attr_value *value);

// SSHS
typedef struct sshs *sshs;
typedef void (*sshsErrorLogCallback)(const char *msg);

sshs sshsGetGlobal(void);
void sshsSetGlobalErrorLogCallback(sshsErrorLogCallback error_log_cb);
sshs sshsNew(void);
bool sshsExistsNode(sshs st, const char *nodePath);
sshsNode sshsGetNode(sshs st, const char *nodePath);
bool sshsExistsRelativeNode(sshsNode node, const char *nodePath);
sshsNode sshsGetRelativeNode(sshsNode node, const char *nodePath);
bool sshsBeginTransaction(sshs st, char *nodePaths[], size_t nodePathsLength);
bool sshsEndTransaction(sshs st, char *nodePaths[], size_t nodePathsLength);

#endif /* SSHS_H_ */
