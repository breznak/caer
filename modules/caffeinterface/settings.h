/* Caffe Interface for deep learning
 *  Author: federico.corradi@inilabs.com
 */

#define USE_CUDNN 2

// Choose your network and it will run in real-time :-)
// DVS FACE DETECTION - remember to enable doSaveHist_png in imagegenerator
// also set the detTreshold in caffeinterface
#define NET_MODEL "modules/caffeinterface/caffe_models/faces_36x36/lenet.prototxt";
#define NET_WEIGHTS  "modules/caffeinterface/caffe_models/faces_36x36/binary.caffemodel";
#define NET_MEAN "modules/caffeinterface/caffe_models/faces_36x36/mean.mean";
#define NET_VAL "modules/caffeinterface/caffe_models/faces_36x36/val.txt";
// PREDATOR
//#define NET_MODEL "modules/caffeinterface/caffe_models/predator_36x36/predator.prototxt";
//#define NET_WEIGHTS  "modules/caffeinterface/caffe_models/predator_36x36/binary.caffemodel";
//#define NET_MEAN "modules/caffeinterface/caffe_models/predator_36x36/mean.mean";
//#define NET_VAL "modules/caffeinterface/caffe_models/predator_36x36/val.txt";
// MNIST
//#define NET_MODEL "modules/caffeinterface/caffe_models/mnist_28x28/lenet.prototxt";
//#define NET_WEIGHTS  "modules/caffeinterface/caffe_models/mnist_28x28/lenet.caffemodel";
//#define NET_MEAN "modules/caffeinterface/caffe_models/mnist_28x28/mean.mean";
//#define NET_VAL "modules/caffeinterface/caffe_models/mnist_28x28/val.txt";
// ROSHAMBO
//#define NET_MODEL "modules/caffeinterface/caffe_models/roshambo_64x64/NullHop.prototxt";
//#define NET_WEIGHTS "modules/caffeinterface/caffe_models/roshambo_64x64/NullHop.caffemodel";
//#define NET_MEAN "modules/caffeinterface/caffe_models/roshambo_64x64/mean.mean";
//#define NET_VAL "modules/caffeinterface/caffe_models/roshambo_64x64/NullHop_val.prototxt";
//#define NET_MODEL "modules/caffeinterface/caffe_models/roshambo_64x64/NullHop.prototxt"
//#define NET_WEIGHTS "modules/caffeinterface/caffe_models/roshambo_64x64/NullHop.caffemodel"
//#define NET_MEAN "modules/caffeinterface/caffe_models/roshambo_64x64/mean.mean"
//#define NET_VAL "modules/caffeinterface/caffe_models/roshambo_64x64/NullHop_val.prototxt"
