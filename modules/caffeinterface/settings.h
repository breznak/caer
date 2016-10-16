/* Caffe Interface for deep learning
 *  Author: federico.corradi@inilabs.com
 */

// Choose your network and it will run in real-time :-)
// DVS FACE DETECTION - remember to enable doSaveHist_png in imagegenerator
// also set the detTreshold in caffeinterface
//#define NET_MODEL "modules/caffeinterface/caffe_models/faces_36x36/lenet.prototxt";
//#define NET_WEIGHTS  "modules/caffeinterface/caffe_models/faces_36x36/binary.caffemodel";
//#define NET_MEAN "modules/caffeinterface/caffe_models/faces_36x36/mean.mean";
//#define NET_VAL "modules/caffeinterface/caffe_models/faces_36x36/val.txt";
// MNIST
#define NET_MODEL "modules/caffeinterface/caffe_models/mnist/lenet.prototxt";
#define NET_WEIGHTS  "modules/caffeinterface/caffe_models/mnist/lenet_iter_10000.caffemodel";
#define NET_MEAN "modules/caffeinterface/caffe_models/mnist/mean.mean";
#define NET_VAL "modules/caffeinterface/caffe_models/mnist/val.txt";
