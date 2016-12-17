#include <stdio.h>
#include <stdlib.h>

typedef struct {
	uint16_t r,g,b;
} COLOUR;

COLOUR GetColour(double v, double vmin, double vmax);

