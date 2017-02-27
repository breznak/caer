/* OpenDV Interface
 *  Author: federico.corradi@inilabs.com
 */

#ifndef __OPENDVDISPLAY_H
#define __OPENDVDISPLAY_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <algorithm>
#include <iosfwd>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <libcaer/events/frame.h>
#include "modules/activityindicator/activityindicator.h"
#include "opencvdisplay_module.h"

using std::string;

class MyOpenCV {
private:

public:
	void generate(AResults res, caerFrameEvent *single_frame);
};

#endif
