/* OpenCV Interface
 *  Author: federico.corradi@inilabs.com
 */
#include "denoisingautoencoder.hpp"
#include <string>

void MyDenAutoEncoder::generate(caerFrameEvent *single_frame, caerFrameEvent *encoders) {

	// Loading img
	cv::Mat imgInput = cv::Mat(FRAMESIZE, FRAMESIZE, CV_8UC3);
	for (size_t i = 0; i < FRAMESIZE; i++) {
		for (size_t j = 0; j < FRAMESIZE; j++) {
			imgInput.at<cv::Vec3b>(i, j)[0] = caerFrameEventGetPixelForChannel(*single_frame, i, j, 0);
			imgInput.at<cv::Vec3b>(i, j)[1] = caerFrameEventGetPixelForChannel(*single_frame, i, j, 0);
			imgInput.at<cv::Vec3b>(i, j)[2] = caerFrameEventGetPixelForChannel(*single_frame, i, j, 0);
		}
	}

	// run denoising auto-encoder
	// push in array of arrays for batch mode
	for (int i = 0; i < 1; ++i) {
		Mat tpmat = Mat::zeros(FRAMESIZE, FRAMESIZE, CV_8UC1);
		for (int r = 0; r < FRAMESIZE; ++r) {
			for (int c = 0; c < FRAMESIZE; ++c) {
				unsigned char temp = 0;
				tpmat.at<uchar>(r, c) = caerFrameEventGetPixelForChannel(*single_frame, r, c, 0);
			}
		}
		vec.push_back(tpmat);
	}
	trainX = concatenateMat(vec);
	batch = trainX.cols / 100; //trainX.cols / 100;

	// pre-processing data.
	Scalar mean, stddev;
	meanStdDev(trainX, mean, stddev);
	Mat normX = trainX - mean[0];
	normX.copyTo(trainX);

	if (counter_img == batch_size) {
		if (!init_done) {
			int nfeatures = trainX.rows;
			int nsamples = trainX.cols;
			caerLog(CAER_LOG_NOTICE, __func__ , "Initializing random weights for Denoising Auto-Encoder");
			weightRandomInit(nfeatures, hiddenSize, nsamples, 0.12);
			init_done = true;
			caerLog(CAER_LOG_NOTICE, __func__ , "Initialization done for Denoising Auto-Encoder");

		}
		trainSparseAutoencoder(trainX, hiddenSize, 5e-4, 0.1, 3);
		counter_img = 0;
		trainX.release();
		for(size_t ii=0; ii< batch_size; ii++){
			vec.pop_back();
		}
	}
	counter_img++;

	//cout<<"Totally sa.W1.rows "<<sa.W1.rows<<" sa.W1.cols" << sa.W1.cols<<endl;
	// sa.W1.rows -> number of filters
	// sa.W1.cols -> FRAMESIZE x FRAMESIZE
	int encfil = 15;// (rand() % (int) (sa.W1.rows + 1));
	Mat normDisp;
    normalize(sa.W1, normDisp, 0, 255,NORM_MINMAX);

	for (int j = 0; j < FRAMESIZE * FRAMESIZE; j++) {
		int x = j % FRAMESIZE;
		int y = j / FRAMESIZE;
		if(normDisp.rows > 0 && normDisp.cols > 3){
		    int val = (int) round(normDisp.at<double>(encfil,j));
			//std::cout << val << std::endl;
			caerFrameEventSetPixelForChannel(*encoders, x, y, 0,  val* 256);
			caerFrameEventSetPixelForChannel(*encoders, x, y, 1,  val* 256);
			caerFrameEventSetPixelForChannel(*encoders, x, y, 2,  val* 256);
		}
		//	caerFrameEventSetPixelForChannel(*encoders, x, y, 0, (int)floor(sa.W1.ATD(encfil, j)));
	}

}

Mat MyDenAutoEncoder::concatenateMat(vector<Mat> &vec) {

	int height = vec[0].rows;
	int width = vec[0].cols;
	Mat res = Mat::zeros(height * width, vec.size(), CV_64FC1);
	for (int i = 0; i < vec.size(); i++) {
		Mat img(height, width, CV_64FC1);

		vec[i].convertTo(img, CV_64FC1);
		// reshape(int cn, int rows=0), cn is num of channels.
		Mat ptmat = img.reshape(0, height * width);
		Rect roi = cv::Rect(i, 0, ptmat.cols, ptmat.rows);
		Mat subView = res(roi);
		ptmat.copyTo(subView);
	}
	divide(res, 255.0, res);
	return res;
}

int MyDenAutoEncoder::ReverseInt(int i) {
	unsigned char ch1, ch2, ch3, ch4;
	ch1 = i & 255;
	ch2 = (i >> 8) & 255;
	ch3 = (i >> 16) & 255;
	ch4 = (i >> 24) & 255;
	return ((int) ch1 << 24) + ((int) ch2 << 16) + ((int) ch3 << 8) + ch4;
}

Mat MyDenAutoEncoder::getBernoulliMatrix(int height, int width, double prob) {
	Mat res = Mat::zeros(height, width, CV_64FC1);
	std::default_random_engine e;
	std::bernoulli_distribution b(prob);
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			bool tmp = b(e);
			if (tmp)
				res.ATD(i, j) = 1.0;
			}
		}
	return res;
}

Mat MyDenAutoEncoder::sigmoid(Mat &M) {
	Mat temp;
	exp(-M, temp);
	return 1.0 / (temp + 1.0);
}

Mat MyDenAutoEncoder::dsigmoid(Mat &a) {
	Mat res = 1.0 - a;
	res = res.mul(a);
	return res;
}

Mat MyDenAutoEncoder::Tanh(Mat &M) {
	Mat res(M);
	for (int i = 0; i < res.rows; i++) {
		for (int j = 0; j < res.cols; j++) {
			res.ATD(i, j) = tanh(M.ATD(i, j));
		}
	}
	return res;
}

Mat MyDenAutoEncoder::dTanh(Mat &M) {
	Mat res = Mat::ones(M.rows, M.cols, CV_64FC1);
	Mat temp;
	pow(M, 2.0, temp);
	res -= temp;
	return res;
}

Mat MyDenAutoEncoder::ReLU(Mat& M) {
	Mat res(M);
	for (int i = 0; i < M.rows; i++) {
		for (int j = 0; j < M.cols; j++) {
			if (M.ATD(i, j) < 0.0) res.ATD(i, j) = 0.0;
		}
	}
	return res;
}

Mat MyDenAutoEncoder::dReLU(Mat& M) {
	Mat res = Mat::zeros(M.rows, M.cols, CV_64FC1);
	for (int i = 0; i < M.rows; i++) {
		for (int j = 0; j < M.cols; j++) {
			if (M.ATD(i, j) > 0.0) res.ATD(i, j) = 1.0;
		}
	}
	return res;
}

Mat MyDenAutoEncoder::non_Linearity(Mat &M) {
	if (NL_Method == RELU) {
		return ReLU(M);
	}
	else if (NL_Method == TANH) {
		return Tanh(M);
	}
	else {
		return sigmoid(M);
	}
}

Mat MyDenAutoEncoder::d_non_Linearity(Mat &M) {
	if (NL_Method == RELU) {
		return dReLU(M);
	}
	else if (NL_Method == TANH) {
		return dTanh(M);
	}
	else {
		return dsigmoid(M);
	}
}

void MyDenAutoEncoder::weightRandomInit(int inputsize, int hiddensize, int nsamples, double epsilon) {

	double *pData;
	sa.W1 = Mat::ones(hiddensize, inputsize, CV_64FC1);
	for (int i = 0; i < hiddensize; i++) {
		pData = sa.W1.ptr<double>(i);
		for (int j = 0; j < inputsize; j++) {
			pData[j] = randu<double>();
		}
	}
	sa.W1 = sa.W1 * (2 * epsilon) - epsilon;

	sa.W2 = Mat::ones(inputsize, hiddensize, CV_64FC1);
	for (int i = 0; i < inputsize; i++) {
		pData = sa.W2.ptr<double>(i);
		for (int j = 0; j < hiddensize; j++) {
			pData[j] = randu<double>();
		}
	}
	sa.W2 = sa.W2 * (2 * epsilon) - epsilon;

	sa.b1 = Mat::ones(hiddensize, 1, CV_64FC1);
	for (int j = 0; j < hiddensize; j++) {
		sa.b1.ATD(j, 0) = randu<double>();
	}
	sa.b1 = sa.b1 * (2 * epsilon) - epsilon;

	sa.b2 = Mat::ones(inputsize, 1, CV_64FC1);
	for (int j = 0; j < inputsize; j++) {
		sa.b2.ATD(j, 0) = randu<double>();
	}
	sa.b2 = sa.b2 * (2 * epsilon) - epsilon;

	sa.W1grad = Mat::zeros(hiddensize, inputsize, CV_64FC1);
	sa.W2grad = Mat::zeros(inputsize, hiddensize, CV_64FC1);
	sa.b1grad = Mat::zeros(hiddensize, 1, CV_64FC1);
	sa.b2grad = Mat::zeros(inputsize, 1, CV_64FC1);
	sa.cost = 0.0;
}

void MyDenAutoEncoder::getSparseAutoencoderActivation(Mat &data, SAA &acti) {
	data.copyTo(acti.aInput);
	//std::cout << "cpy " <<  data.cols << std::endl;

	acti.zHidden = sa.W1 * acti.aInput + repeat(sa.b1, 1, data.cols);
	acti.aHidden = non_Linearity(acti.zHidden);
	acti.zOutput = sa.W2 * acti.aHidden + repeat(sa.b2, 1, data.cols);
	acti.aOutput = non_Linearity(acti.zOutput);
}

void MyDenAutoEncoder::sparseAutoencoderCost(Mat &corruptedData, Mat &data, double lambda, double sparsityParam,
	double beta) {

	int nfeatures = data.rows;
	int nsamples = data.cols;
	//std::cout << "Acti" << std::endl;

	getSparseAutoencoderActivation(corruptedData, acti);

	Mat errtp = acti.aOutput - data;
	pow(errtp, 2.0, errtp);
	errtp /= 2.0;
	double Jcost = sum(errtp)[0] / nsamples;
	// now calculate pj which is the average activation of hidden units
	Mat pj;
	reduce(acti.aHidden, pj, 1, CV_REDUCE_SUM);
	pj /= nsamples;
	// the second part is weight decay part
	double Jweight = 0.0;
	Mat temp;
	pow(sa.W1, 2.0, temp);
	Jweight += sum(temp)[0];
	pow(sa.W2, 2.0, temp);
	Jweight += sum(temp)[0];
	Jweight *= lambda / 2.0;
	// the third part of overall cost function is the sparsity part
	temp = sparsityParam / pj;
	log(temp, temp);
	errtp = temp * sparsityParam;
	temp = (1 - sparsityParam) / (1 - pj);
	log(temp, temp);
	errtp += temp * (1 - sparsityParam);
	double Jsparse = sum(errtp)[0] * beta;
	sa.cost = Jcost + Jweight + Jsparse;

	// following are for calculating the grad of weights.
	Mat delta3 = -(data - acti.aOutput);
	delta3 = delta3.mul(d_non_Linearity(acti.aOutput));
	Mat temp2 = -sparsityParam / pj + (1 - sparsityParam) / (1 - pj);
	temp2 *= beta;
	Mat delta2 = sa.W2.t() * delta3 + repeat(temp2, 1, nsamples);
	delta2 = delta2.mul(d_non_Linearity(acti.aHidden));
	Mat nablaW1 = delta2 * acti.aInput.t();
	Mat nablaW2 = delta3 * acti.aHidden.t();
	Mat nablab1, nablab2;
	delta3.copyTo(nablab2);
	delta2.copyTo(nablab1);
	sa.W1grad = nablaW1 / nsamples + lambda * sa.W1;
	sa.W2grad = nablaW2 / nsamples + lambda * sa.W2;
	reduce(nablab1, sa.b1grad, 1, CV_REDUCE_SUM);
	reduce(nablab2, sa.b2grad, 1, CV_REDUCE_SUM);
	sa.b1grad /= nsamples;
	sa.b2grad /= nsamples;
}

void MyDenAutoEncoder::gradientChecking(Mat &data, double lambda, double sparsityParam, double beta) {

	//Gradient Checking (remember to disable this part after you're sure the
	//cost function and dJ function are correct)
	sparseAutoencoderCost(data, data, lambda, sparsityParam, beta);
	Mat w1g(sa.W1grad);
	cout << " test sparse autoencoder !!!! " << endl;
	double epsilon = 1e-4;
	for (int i = 0; i < sa.W1.rows; i++) {
		for (int j = 0; j < sa.W1.cols; j++) {
			double memo = sa.W1.ATD(i, j);
			sa.W1.ATD(i, j) = memo + epsilon;
			sparseAutoencoderCost(data, data, lambda, sparsityParam, beta);
			double value1 = sa.cost;
			sa.W1.ATD(i, j) = memo - epsilon;
			sparseAutoencoderCost(data, data, lambda, sparsityParam, beta);
			double value2 = sa.cost;
			double tp = (value1 - value2) / (2 * epsilon);
			cout<<i<<", "<<j<<", "<<tp<<", "<<w1g.ATD(i, j)<<", "<<w1g.ATD(i, j) / tp<<endl;
			sa.W1.ATD(i, j) = memo;
		}
	}
}

void MyDenAutoEncoder::trainSparseAutoencoder(Mat &data, int hiddenSize, double lambda, double sparsityParam,
	double beta) {

	if (IS_TEST_SA) {
		gradientChecking(data, lambda, sparsityParam, beta);
	}
	else {
		cout << "Sparse Autoencoder Learning................" << endl;
		// define the velocity vectors.
		Mat v_W1 = Mat::zeros(sa.W1.rows, sa.W1.cols, CV_64FC1);
		Mat v_W2 = Mat::zeros(sa.W2.rows, sa.W2.cols, CV_64FC1);
		Mat v_b1 = Mat::zeros(sa.b1.rows, sa.b1.cols, CV_64FC1);
		Mat v_b2 = Mat::zeros(sa.b2.rows, sa.b2.cols, CV_64FC1);

		//double epsilont;
		//double pt;
		int t_start= t;
		for (t; t < (t_start+epochs); t++) {

			if (t > T)
				pt = pf;
			else
				pt = (double) t / T * pi + (1 - (double) t / T) * pf;
			epsilont = epsilon0 * pow(f, t);

			int randomNum = ((long) rand() + (long) rand()) % (data.cols - batch);
			Rect roi = Rect(randomNum, 0, batch, data.rows);
			Mat batchX = data(roi);

			Mat corrupted = getBernoulliMatrix(batchX.rows, batchX.cols, probDestruction);
			corrupted = batchX.mul(corrupted);

			//std::cout << "cost" << std::endl;
			sparseAutoencoderCost(corrupted, batchX, lambda, sparsityParam, beta);

			v_W1 = pt * v_W1 - (1 - pt) * epsilont * (lrate * sa.W1grad);
			v_W2 = pt * v_W2 - (1 - pt) * epsilont * (lrate * sa.W2grad);
			v_b1 = pt * v_b1 - (1 - pt) * epsilont * (lrate * sa.b1grad);
			v_b2 = pt * v_b2 - (1 - pt) * epsilont * (lrate * sa.b2grad);

			sa.W1 += v_W1;
			sa.W2 += v_W2;
			sa.b1 += v_b1;
			sa.b2 += v_b2;

			caerLog(CAER_LOG_NOTICE, __func__ , "epoch %d Cost function value =  %f", t,  sa.cost);

		}
	}
}


