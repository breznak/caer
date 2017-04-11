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

using std::string;

class OpenCV {
private:

public:
	void generate(int nIn, int nOut, caerFrameEvent *single_frame, int sizeX, int sizeY);
};

#endif
