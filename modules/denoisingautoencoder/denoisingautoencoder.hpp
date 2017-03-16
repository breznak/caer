/* Denoising Auto-Encoder - pre-training -
 *  Author: federico.corradi@inilabs.com
 */

#ifndef __OPENDVDISPLAY_H
#define __OPENDVDISPLAY_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <algorithm>
#include <math.h>
#include <fstream>
#include <iostream>
#include <random>
#include <iosfwd>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <libcaer/events/frame.h>
#include "denoisingautoencoder_module.h"

#define IS_TEST_SA 0

#define ATD at<double>
#define elif else if

#define RELU 0
#define SIGMOID 1
#define TANH 2

using namespace cv;
using namespace std;

using std::string;

class MyDenAutoEncoder {

private:


public:

	typedef struct SparseAutoencoder{
	    Mat W1;
	    Mat W2;
	    Mat b1;
	    Mat b2;
	    Mat W1grad;
	    Mat W2grad;
	    Mat b1grad;
	    Mat b2grad;
	    double cost;
	}SA;

	typedef struct SparseAutoencoderActivation{
	    Mat aInput;
	    Mat aHidden;
	    Mat aOutput;
	    Mat zHidden;
	    Mat zOutput;
	}SAA;

    int epochs = 100; // per image
    double lrate = 0.1;
    int T = 200;
    double epsilon0 = 80.0;
    double f = 0.999;
    double pi = 0.5;
    double pf = 0.99;
    SA sa;
    SAA acti;
    int hiddenSize = 200;
    bool init_done = false;
    Mat trainX; // vector
    int batch_size = 300;
    int counter_img = 0;
    vector<Mat> vec;

	//
	int NL_Method = SIGMOID;
	double probDestruction = 1.0;
	int batch;

	void generate(caerFrameEvent *single_frame_in, caerFrameEvent *encoders);

	// internals
	Mat concatenateMat(vector<Mat> &vec);
	int ReverseInt (int i);
	void read_Mnist(string filename, vector<Mat> &vec);
	void read_Mnist_Label(string filename, Mat &mat);
	Mat getBernoulliMatrix(int height, int width, double prob);
	Mat sigmoid(Mat &M);
	Mat dsigmoid(Mat &a);
	Mat Tanh(Mat &M);
	Mat dTanh(Mat &M);
	Mat ReLU(Mat& M);
	Mat dReLU(Mat& M);
	Mat non_Linearity(Mat &M);
	Mat d_non_Linearity(Mat &M);
	void weightRandomInit(int inputsize, int hiddensize, int nsamples, double epsilon);
	void getSparseAutoencoderActivation( Mat &data, SAA &acti);
	void sparseAutoencoderCost( Mat &corruptedData, Mat &data, double lambda, double sparsityParam, double beta);
	void gradientChecking( Mat &data, double lambda, double sparsityParam, double beta);
	void trainSparseAutoencoder( Mat &data, int hiddenSize, double lambda, double sparsityParam, double beta);
	void readData(Mat &x, Mat &y, string xpath, string ypath, int number_of_images);
	void saveWeight2txt(Mat &data);

};

#endif
