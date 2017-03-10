#include "opticflow.hpp"

OpticFlow::OpticFlow(OpticFlowSettings settings) {
	this->settings = settings;

	updateSettings();
}

void OpticFlow::updateSettings(void) {

	// Update board size.
	flowSize.width = settings->imageWidth;
	flowSize.height = settings->imageWidth;

}

bool OpticFlow::doOpticFlow(caerFrameEvent * frame, caerFrameEvent * frameInput, int sizeX, int sizeY) {

	//int channum = caerFrameEventGetChannelNumber(*frameInput);
	//std::cout << "ch " << channum << std::endl;

	// Loading frameInput to a mat
	cv::Mat imgInput = cv::Mat(sizeX, sizeY, CV_8UC3);
	for (size_t i = 0; i < sizeX; i++) {
		for (size_t j = 0; j < sizeY; j++) {
			imgInput.at<Vec3b>(i,j)[0] = caerFrameEventGetPixelForChannel(*frameInput, i, j, 0);
			imgInput.at<Vec3b>(i,j)[1] = caerFrameEventGetPixelForChannel(*frameInput, i, j, 1);
			imgInput.at<Vec3b>(i,j)[2]  = caerFrameEventGetPixelForChannel(*frameInput, i, j, 2);
		}
	}

	// Loading generated frame to a mat
	cv::Mat imgOutput = cv::Mat(sizeX, sizeY, CV_8UC3);
	for (size_t i = 0; i < sizeX; i++) {
		for (size_t j = 0; j < sizeY; j++) {
			imgOutput.at<Vec3b>(i,j)[0]= caerFrameEventGetPixelForChannel(*frame, i, j, 0);
			imgOutput.at<Vec3b>(i,j)[1] = caerFrameEventGetPixelForChannel(*frame, i, j, 1);
			imgOutput.at<Vec3b>(i,j)[2] = caerFrameEventGetPixelForChannel(*frame, i, j, 2);
		}
	}

	if (init == false) {

		cout << "rows: " << imgInput.rows << " cols: " << imgInput.cols;
		cout << " ch: " << imgInput.channels() << " " << imgInput.isContinuous() << endl;

		//Mat for storing the downsized images.
		smallImage =  Mat(imgInput.rows / ratio, imgInput.cols / ratio, CV_8UC3, Scalar(43, 87, 12));
		oldSmallImage = Mat(imgInput.rows / ratio, imgInput.cols / ratio, CV_8UC3, Scalar(43, 87, 12));

		//Matrics for storing the values got after doing
		//the optic flow operations.
		diff = Mat(imgInput.rows / ratio, imgInput.cols / ratio, CV_64FC3, Scalar(43, 87, 12));
		dx = Mat(imgInput.rows / ratio, imgInput.cols / ratio, CV_64FC3, Scalar(43, 87, 12));
		dy = Mat(imgInput.rows / ratio, imgInput.cols / ratio, CV_64FC3, Scalar(43, 87, 12));

		iRows = imgInput.rows;
		iCols = imgInput.cols;

		init = true;

		//cv::namedWindow("debug",0);
	}
	else {

		//Declare iterators for iterating over the original
		//and downsized mat of frames.
		MatIterator_<Vec3b> og_it, og_end, sm_it, sm_end;

		int tp = 0;

		//This is a custom implementation of the downsizing
		//of the original image. We just subsample the original image
		//and store it in the new Mat.
		for (og_it = imgInput.begin<Vec3b>(), og_end = imgInput.end<Vec3b>(), sm_it = smallImage.begin<Vec3b>(), sm_end =
			smallImage.end<Vec3b>(); og_it != og_end && sm_it != sm_end; og_it += ratio, ++sm_it) {
			if (tp % (iCols / ratio) == 0 && tp != 0) {
				og_it += iCols * (ratio - 1);
			}
			(*sm_it)[0] = (*og_it)[0];
			(*sm_it)[1] = (*og_it)[1];
			(*sm_it)[2] = (*og_it)[2];
			tp++;
		}

		//Blur the downsized image inplace to reduce noise
		//due to subsampling and otherwise.
		blur(smallImage, smallImage, Size(3, 3));

		//Convert the Mat to use the float data type as
		//our calculations will involve decimal
		//point numbers.
		Mat smallImage_64F, oldSmallImage_64F;
		smallImage.convertTo(smallImage_64F, CV_64FC3);
		oldSmallImage.convertTo(oldSmallImage_64F, CV_64FC3);

		//This is the first operation - taking the diff
		//of current and previous frame.
		diff = smallImage_64F - oldSmallImage_64F;

		//Now we take the dx at every point in image. The
		//following kernel is used for this purpose...
		Mat kern = (Mat_<char>(3, 3) << 1, 0, -1, 2, 0, -2, 1, 0, -1);
		filter2D(smallImage_64F, dx, smallImage_64F.depth(), kern);

		//Now we take the dy at every point in image. The
		//following kernel is used for this purpose...
		kern = (Mat_<char>(3, 3) << -1, -2, -1, 0, 0, 0, 1, 2, 1);
		filter2D(smallImage_64F, dy, smallImage_64F.depth(), kern);

		//We estimate optic flow in every image window
		//of size 21x21. The following data structures
		//are declared for these operations.
		int sqSize = 21;
		Mat tp1(sqSize * sqSize, 2, CV_64FC1, 1);
		Mat tp2(sqSize * sqSize, 1, CV_64FC1, 1);
		Mat motionXMat(diff.rows, diff.cols, CV_64FC1, 1);
		Mat motionYMat(diff.rows, diff.cols, CV_64FC1, 1);
		Mat motionMat = Mat::zeros(diff.rows, diff.cols, CV_64FC3);

		smallImage.copyTo(motionMat);
		smallImage.copyTo(oldSmallImage);

		//Convert the image to grayscale to be able to use
		//it with openCV functions.
		Mat curGImage;
		cvtColor(smallImage, curGImage, CV_BGR2GRAY);

		//Get corners in the current frame using the inbuilt
		//goodFeaturesToTrack function. We find optical flow
		//only on these points. More details about this are
		//available on my blog post on
		// http://mayankrajoria.com/blog
		Mat corners, showImage;
		goodFeaturesToTrack(curGImage, corners, 0, 0.05, 0.02, noArray(), 3, false, 0.04);

		for (int l = 0; l < corners.rows; l++) {
			float* curPoint = corners.ptr<float>(l);
			int i = curPoint[1], j = curPoint[0] * 3;
			if (i < 15)
				continue;
			tp = 0;
			for (int k = i - sqSize / 2; k < i + 1 + sqSize / 2; k++) {
				double* curx = dx.ptr<double>(k);
				double* cury = dy.ptr<double>(k);
				double* curd = diff.ptr<double>(k);
				for (int l = j - 30; l < j + 31; l = l + 3) {

					double* curtp1 = tp1.ptr<double>(tp);
					double* curtp2 = tp2.ptr<double>(tp);
					curtp1[0] = curx[l];
					curtp1[1] = cury[l];
					curtp2[0] = curd[l];
					tp++;
				}
			}

			//Do the optic flow calculation for
			//the current window and also apply
			//non linear repeated estimation.
			Mat fir = (tp1.t() * tp1);
			if (determinant(fir) <= 0.5) {
				continue;
			}
			fir = (fir).inv(DECOMP_LU);
			Mat sec = fir * tp1.t();
			Mat pre;
			sec.copyTo(pre);
			sec = sec * tp2;
			int it = 3;
			while (it--) {
				Mat rb = tp1 * sec - tp2;
				sec = sec - pre * rb;
			}

			//Magnitude of the optic flow claculated
			//in current window.
			double len = sqrt(
				sec.ptr<double>(0)[0] * sec.ptr<double>(0)[0] + sec.ptr<double>(1)[0] * sec.ptr<double>(1)[0]);

			//If magnitude is more than threshold, plot an
			//arrow for this flow in the image.
			if (len > 0.05) {
				double m = 0 + sec.ptr<double>(1)[0] / sec.ptr<double>(0)[0];
				double p = -1 / m;
				MyLine(motionMat, Point(j / 3, i), Point(j / 3, i));
				if (sec.ptr<double>(0)[0] > 0) {
					MyLine(motionMat, Point(j / 3 + 3 / sqrt(1 + p * p), i - 3 * p / (sqrt(1 + p * p))),
						Point(j / 3 + 25 * len / sqrt(1 + m * m), i - 25 * m * len / (sqrt(1 + m * m))));
					MyLine(motionMat, Point(j / 3 - 3 / sqrt(1 + p * p), i + 3 * p / (sqrt(1 + p * p))),
						Point(j / 3 + 25 * len / sqrt(1 + m * m), i - 25 * m * len / (sqrt(1 + m * m))));
				}
				else {
					MyLine(motionMat, Point(j / 3 + 3 / sqrt(1 + p * p), i - 3 * p / (sqrt(1 + p * p))),
						Point(j / 3 - 25 * len / sqrt(1 + m * m), i + 25 * m * len / (sqrt(1 + m * m))));
					MyLine(motionMat, Point(j / 3 - 3 / sqrt(1 + p * p), i + 3 * p / (sqrt(1 + p * p))),
						Point(j / 3 - 25 * len / sqrt(1 + m * m), i + 25 * m * len / (sqrt(1 + m * m))));
				}
			}
		}

		//Show some outputs of the blurred image
		//and the other with optic flow arrows overlayed
		//onto it.
		//cv::imshow("debug", smallImage);
		//cv::imshow("Camera feed motion", motionMat);

		//Store the current frame as the old frame.
		smallImage.copyTo(oldSmallImage);

		//imgInput.convertTo(imgInput, CV_8UC3);
		for (int j = 0; j < sizeX; j++) {
		    for (int i = 0; i < sizeY; i++) {
		        //int color = (int) imgInput.at<char>(i,j)[0];
		        int bgrPixel = (int)motionMat.at<Vec3b>(i,j)[0];//(int) imgInput.at<uchar>(i,j,0);
		        caerFrameEventSetPixelForChannel(*frame, i, j, 0, bgrPixel*256);
		        bgrPixel = (int)motionMat.at<Vec3b>(i,j)[1];
		        caerFrameEventSetPixelForChannel(*frame, i, j, 1, bgrPixel*256);
		        bgrPixel = (int)motionMat.at<Vec3b>(i,j)[2];
		        caerFrameEventSetPixelForChannel(*frame, i, j, 2, bgrPixel*256);

		    }
		}

	} // else init done
	return (true);

}

void OpticFlow::MyLine(Mat img, Point start, Point end) {
	int thickness = 1;
	int lineType = 8;
	line(img, start, end, Scalar(20, 210, 25), 1, 8);
}
