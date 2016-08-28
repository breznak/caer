#include "sshs_internal.h"

#if defined(__GNUC__) || defined(__clang__)
	#if defined(__USE_MINGW_ANSI_STDIO)
		#define ATTRIBUTE_FORMAT __attribute__ ((format (gnu_printf, 2, 3)))
	#else
		#define ATTRIBUTE_FORMAT __attribute__ ((format (printf, 2, 3)))
	#endif
#else
	#define ATTRIBUTE_FORMAT
#endif

static void sshsHelperAllocSprintf(char **strp, const char *format, ...) ATTRIBUTE_FORMAT;

// Put NULL in *strp on failure (memory allocation failure).
static void sshsHelperAllocSprintf(char **strp, const char *format, ...) {
	va_list argptr;

	va_start(argptr, format);
	size_t printLength = (size_t) vsnprintf(NULL, 0, format, argptr);
	va_end(argptr);

	*strp = malloc(printLength + 1);
	if (*strp == NULL) {
		return;
	}

	va_start(argptr, format);
	vsnprintf(*strp, printLength + 1, format, argptr);
	va_end(argptr);
}

// Return NULL on unknown type. Do not free returned strings!
const char *sshsHelperTypeToStringConverter(enum sshs_node_attr_value_type type) {
	// Convert the value and its type into a string for XML output.
	switch (type) {
		case SSHS_BOOL:
			return ("bool");

		case SSHS_BYTE:
			return ("byte");

		case SSHS_SHORT:
			return ("short");

		case SSHS_INT:
			return ("int");

		case SSHS_LONG:
			return ("long");

		case SSHS_FLOAT:
			return ("float");

		case SSHS_DOUBLE:
			return ("double");

		case SSHS_STRING:
			return ("string");

		case SSHS_UNKNOWN:
		default:
			return (NULL); // UNKNOWN TYPE.
	}
}

// Return -1 on unknown type.
enum sshs_node_attr_value_type sshsHelperStringToTypeConverter(const char *typeString) {
	if (typeString == NULL) {
		return (SSHS_UNKNOWN); // NULL STRING.
	}

	// Convert the value string back into the internal type representation.
	if (strcmp(typeString, "bool") == 0) {
		return (SSHS_BOOL);
	}
	else if (strcmp(typeString, "byte") == 0) {
		return (SSHS_BYTE);
	}
	else if (strcmp(typeString, "short") == 0) {
		return (SSHS_SHORT);
	}
	else if (strcmp(typeString, "int") == 0) {
		return (SSHS_INT);
	}
	else if (strcmp(typeString, "long") == 0) {
		return (SSHS_LONG);
	}
	else if (strcmp(typeString, "float") == 0) {
		return (SSHS_FLOAT);
	}
	else if (strcmp(typeString, "double") == 0) {
		return (SSHS_DOUBLE);
	}
	else if (strcmp(typeString, "string") == 0) {
		return (SSHS_STRING);
	}

	return (SSHS_UNKNOWN); // UNKNOWN TYPE.
}

// Return NULL on failure (either memory allocation or unknown type / faulty conversion).
// Strings returned by this function need to be free()'d after use!
char *sshsHelperValueToStringConverter(enum sshs_node_attr_value_type type, union sshs_node_attr_value value) {
	// Convert the value and its type into a string for XML output.
	char *valueString;

	switch (type) {
		case SSHS_BOOL:
			// Manually generate true or false.
			if (value.boolean) {
				valueString = strdup("true");
			}
			else {
				valueString = strdup("false");
			}

			break;

		case SSHS_BYTE:
			sshsHelperAllocSprintf(&valueString, "%" PRIi8, value.ibyte);
			break;

		case SSHS_SHORT:
			sshsHelperAllocSprintf(&valueString, "%" PRIi16, value.ishort);
			break;

		case SSHS_INT:
			sshsHelperAllocSprintf(&valueString, "%" PRIi32, value.iint);
			break;

		case SSHS_LONG:
			sshsHelperAllocSprintf(&valueString, "%" PRIi64, value.ilong);
			break;

		case SSHS_FLOAT:
			sshsHelperAllocSprintf(&valueString, "%g", (double) value.ffloat);
			break;

		case SSHS_DOUBLE:
			sshsHelperAllocSprintf(&valueString, "%g", value.ddouble);
			break;

		case SSHS_STRING:
			valueString = strdup(value.string);
			break;

		case SSHS_UNKNOWN:
		default:
			valueString = NULL; // UNKNOWN TYPE.
			break;
	}

	return (valueString);
}

// Return false on failure (unknown type / faulty conversion), the content of
// value is undefined. For the STRING type, the returned value.string is a copy
// of the input string. Remember to free() it after use!
bool sshsHelperStringToValueConverter(enum sshs_node_attr_value_type type, const char *valueString,
	union sshs_node_attr_value *value) {
	if (valueString == NULL || value == NULL) {
		// It is possible for a string value to be NULL, namely when it is
		// an empty string in the XML file. Handle this case here.
		if (type == SSHS_STRING && valueString == NULL && value != NULL) {
			value->string = strdup("");
			if (value->string == NULL) {
				return (false); // MALLOC FAILURE.
			}

			return (true);
		}

		return (false); // NULL STRING.
	}

	switch (type) {
		case SSHS_BOOL:
			// Boolean uses custom true/false strings.
			if (strcmp(valueString, "true") == 0) {
				value->boolean = true;
			}
			else {
				value->boolean = false;
			}

			break;

		case SSHS_BYTE:
			if (sscanf(valueString, "%" SCNi8, &value->ibyte) != 1) {
				return (false); // CONVERSION FAILURE.
			}

			break;

		case SSHS_SHORT:
			if (sscanf(valueString, "%" SCNi16, &value->ishort) != 1) {
				return (false); // CONVERSION FAILURE.
			}

			break;

		case SSHS_INT:
			if (sscanf(valueString, "%" SCNi32, &value->iint) != 1) {
				return (false); // CONVERSION FAILURE.
			}

			break;

		case SSHS_LONG:
			if (sscanf(valueString, "%" SCNi64, &value->ilong) != 1) {
				return (false); // CONVERSION FAILURE.
			}

			break;

		case SSHS_FLOAT:
			if (sscanf(valueString, "%g", &value->ffloat) != 1) {
				return (false); // CONVERSION FAILURE.
			}

			break;

		case SSHS_DOUBLE:
			if (sscanf(valueString, "%lg", &value->ddouble) != 1) {
				return (false); // CONVERSION FAILURE.
			}

			break;

		case SSHS_STRING:
			value->string = strdup(valueString);
			if (value->string == NULL) {
				return (false); // MALLOC FAILURE.
			}

			break;

		case SSHS_UNKNOWN:
		default:
			return (false); // UNKNOWN TYPE.
	}

	return (true);
}
