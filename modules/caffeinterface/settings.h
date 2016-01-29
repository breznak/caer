/* Caffe Interface for deep learning
*  Author: federico.corradi@inilabs.com
*/

// Choose your network and it will run in real-time :-)

// CAFFENET .. more than 1000 classes
#define NET_MODEL "/home/ubuntu/caffe/models/bvlc_reference_caffenet/deploy.prototxt";
#define NET_WEIGHTS  "/home/ubuntu/caffe/models/bvlc_reference_caffenet/bvlc_reference_caffenet.caffemodel";
#define NET_MEAN "/home/ubuntu/caffe/data/ilsvrc12/imagenet_mean.binaryproto";
#define NET_VAL "/home/ubuntu/caffe/data/ilsvrc12/synset_words.txt";

// MNIST
//#define NET_MODEL "/home/ubuntu/caffe/examples/mnist/lenet.prototxt";
//#define NET_WEIGHTS  "/home/ubuntu/caffe/examples/mnist/lenet_iter_30000.caffemodel";
//#define NET_MEAN "/home/ubuntu/caffe/data/ilsvrc12/imagenet_mean.binaryproto";
//#define NET_VAL "/home/ubuntu/caffe/data/mnist/val.txt";

//  DVS FACE DETECTION
//#define NET_MODEL "/home/ubuntu/caffe/examples/_netfacedetection_good/in_24_d_3_c_20_k_5_p_2_c_50_k_5_p_2_ip_500_ip_2/lenet.prototxt";
//#define NET_WEIGHTS  "/home/ubuntu/caffe/examples/_netfacedetection_good/in_24_d_3_c_20_k_5_p_2_c_50_k_5_p_2_ip_500_ip_2/lenet_7000_spikes_iter_60000.caffemodel";
//#define NET_MEAN "/home/ubuntu/caffe/data/ilsvrc12/imagenet_mean.binaryproto";
//#define NET_VAL "/home/ubuntu/caffe/data/facedetection/txt/val.txt";
