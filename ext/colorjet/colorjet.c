#include "colorjet.h"

COLOUR GetColour(double v, double vmin, double vmax)
{
   COLOUR c = {0,0,0}; //{65535, 65535, 65535}; // white
   double dv;
   double value;

   if (v < vmin)
      v = vmin;
   if (v > vmax)
      v = vmax;
   dv = vmax - vmin;

   if (v < (vmin + dv / 4)) {
      c.r = 0;
      value = ( 4 * (v - vmin) / dv ) * 65535;
      if (value > 30000)
    	  c.g = 30000;
      else if (value < 0)
    	  c.g = 0;
      else
    	  c.g = (uint16_t) value;
   } else if (v < (vmin + dv / 2)) {
      c.r = 0;
      value = (1 + 4 * (vmin + dv / 4 - v) / dv) * 65535;
      if (value > 30000)
    	  c.b = 30000;
      else if (value < 0)
    	  c.b = 0;
      else
    	  c.b = (uint16_t) value;
   } else if (v < (vmin + dv * 3 / 4)) {
      c.b = 0;
      value = (4 * (v - vmin - dv / 2) / dv) * 65535;
      if (value > 30000)
    	  c.r = 30000;
      else if (value < 0)
    	  c.r = 0;
      else
    	  c.r = (uint16_t) value;
   } else {
      c.b = 0;
      value = (1 + 4 * (vmin + dv * 3 / 4 - v) / dv) * 65535;
      if (value > 30000)
    	  c.g = 30000;
      else if (value < 0)
    	  c.g = 0;
      else
    	  c.g = (uint16_t) value;
   }

   return(c);
}
