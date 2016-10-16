/* Caffe Interface for deep learning
 *  Author: federico.corradi@inilabs.com
 */
#include "classify.hpp"
#include "settings.h"

using namespace caffe;
// NOLINT(build/namespaces)
using std::string;

void MyClass::file_set(char * i, double *b, double thr, bool printoutputs,
		caerFrameEvent *single_frame, bool showactivations) {
	MyClass::file_i = i;

	if (file_i != NULL) {

		//std::cout << "\n---------- Prediction for " << file_i << " started ----------\n" << std::endl;
		cv::Mat img = cv::imread(file_i, 0);
		cv::Mat img2;
		img.convertTo(img2, CV_32FC1);
		img2 = img2 * 0.00390625; // normalize 0,255 to 1

		CHECK(!img.empty()) << "Unable to decode image " << file_i;
		std::vector<Prediction> predictions = MyClass::Classify(img2, 5,
				single_frame, showactivations);

		/* Print the top N predictions. */
		for (size_t i = 0; i < predictions.size(); ++i) {
			Prediction p = predictions[i];
			if (printoutputs) {
				std::cout << "\n" << std::fixed << std::setprecision(4)
						<< p.second << " - \"" << p.first << "\"" << std::endl;
			}
			// for face detection net
			if (p.first.compare("FACE") == 0 && p.second > thr) {
				*b = p.second;
				std::cout << "\n" << p.second << " DETECTION " << std::endl;
			}
		}
	}
}

char * MyClass::file_get() {
	return file_i;
}

void MyClass::init_network() {

	//::google::InitGoogleLogging(0);
	string model_file = NET_MODEL
	;
	string trained_file = NET_WEIGHTS
	;
	string mean_file = NET_MEAN
	;
	string label_file = NET_VAL
	;
	MyClass::Classifier(model_file, trained_file, mean_file, label_file);

	return;

}

void MyClass::Classifier(const string& model_file, const string& trained_file,
		const string& mean_file, const string& label_file) {
#ifdef CPU_ONLY
	Caffe::set_mode(Caffe::CPU);
#else
	Caffe::set_mode(Caffe::GPU);
#endif

	/* Load the network. */
	net_.reset(new Net<float>(model_file, TEST));
	net_->CopyTrainedLayersFrom(trained_file);

	CHECK_EQ(net_->num_inputs(), 1) << "Network should have exactly one input.";
	CHECK_EQ(net_->num_outputs(), 1)
			<< "Network should have exactly one output.";

	Blob<float>* input_layer = net_->input_blobs()[0];
	num_channels_ = input_layer->channels();
	CHECK(num_channels_ == 3 || num_channels_ == 1)
			<< "Input layer should have 1 or 3 channels.";
	input_geometry_ = cv::Size(input_layer->width(), input_layer->height());

	/* Load the binaryproto mean file. */
	//SetMean(mean_file);
	/* Load labels. */
	std::ifstream labels(label_file.c_str());
	CHECK(labels) << "Unable to open labels file " << label_file;
	string line;
	while (std::getline(labels, line))
		labels_.push_back(string(line));

	Blob<float>* output_layer = net_->output_blobs()[0];
	CHECK_EQ(labels_.size(), output_layer->channels())
			<< "Number of labels is different from the output layer dimension.";
}

static bool PairCompare(const std::pair<float, int>& lhs,
		const std::pair<float, int>& rhs) {
	return lhs.first > rhs.first;
}

/* Return the indices of the top N values of vector v. */
static std::vector<int> Argmax(const std::vector<float>& v, int N) {
	std::vector<std::pair<float, int> > pairs;
	for (size_t i = 0; i < v.size(); ++i)
		pairs.push_back(std::make_pair(v[i], i));
	std::partial_sort(pairs.begin(), pairs.begin() + N, pairs.end(),
			PairCompare);

	std::vector<int> result;
	for (int i = 0; i < N; ++i)
		result.push_back(pairs[i].second);
	return result;
}

/* Return the top N predictions. */
std::vector<Prediction> MyClass::Classify(const cv::Mat& img, int N,
		caerFrameEvent *single_frame, bool showactivations) {
	std::vector<float> output = Predict(img, single_frame, showactivations);

	N = std::min<int>(labels_.size(), N);
	std::vector<int> maxN = Argmax(output, N);
	std::vector<Prediction> predictions;
	for (int i = 0; i < N; ++i) {
		int idx = maxN[i];
		predictions.push_back(std::make_pair(labels_[idx], output[idx]));
	}

	return predictions;
}

/* Load the mean file in binaryproto format. */
void MyClass::SetMean(const string& mean_file) {
	BlobProto blob_proto;
	ReadProtoFromBinaryFileOrDie(mean_file.c_str(), &blob_proto);

	/* Convert from BlobProto to Blob<float> */
	Blob<float> mean_blob;
	mean_blob.FromProto(blob_proto);
	CHECK_EQ(mean_blob.channels(), num_channels_)
			<< "Number of channels of mean file doesn't match input layer.";

	/* The format of the mean file is planar 32-bit float BGR or grayscale. */
	std::vector<cv::Mat> channels;
#ifdef CPU_ONLY
	float* data = mean_blob.mutable_cpu_data();
#else
	float* data = mean_blob.mutable_gpu_data();
#endif
	for (int i = 0; i < num_channels_; ++i) {
		/* Extract an individual channel. */
		cv::Mat channel(mean_blob.height(), mean_blob.width(), CV_32FC1, data);
		channels.push_back(channel);
		data += mean_blob.height() * mean_blob.width();
	}

	/* Merge the separate channels into a single image. */
	cv::Mat mean;
	cv::merge(channels, mean);

	/* Compute the global mean pixel value and create a mean image
	 * filled with this value. */
	cv::Scalar channel_mean = cv::mean(mean);
	mean_ = cv::Mat(input_geometry_, mean.type(), channel_mean);

}

std::vector<float> MyClass::Predict(const cv::Mat& img,
		caerFrameEvent *single_frame, bool showactivations) {

	Blob<float>* input_layer = net_->input_blobs()[0];
	input_layer->Reshape(1, num_channels_, input_geometry_.height,
			input_geometry_.width);
	/* Forward dimension change to all layers. */
	net_->Reshape();

	std::vector<cv::Mat> input_channels;
	WrapInputLayer(&input_channels);

	Preprocess(img, &input_channels);
	net_->ForwardPrefilled(); //Prefilled();

	//IF WE ENABLE VISUALIZATION IN REAL TIME
	if (showactivations) {
		const vector<shared_ptr<Layer<float> > >& layers = net_->layers();

		//image vector containing all layer activations
		vector < vector<cv::Mat> > layersVector;

		std::vector<int> ntot, ctot, htot, wtot, n_image_per_layer;

		// net blobs
		const vector<shared_ptr<Blob<float>>>&this_layer_blobs =
		net_->blobs();

		// we want all activations of all layers this_layer_blobs.size()
		for (int i = 0; i < this_layer_blobs.size(); i++) {

			int n, c, h, w;
			float data;

			if(strcmp(layers[i]->type(),"Convolution")  != 0 &&
			   strcmp(layers[i]->type(),"ReLU") != 0   &&
			   strcmp(layers[i]->type(),"Pooling") != 0 &&
			   strcmp(layers[i]->type(),"InnerProduct") != 0 ){
				 continue;
			}

			n = this_layer_blobs[i]->num();
			c = this_layer_blobs[i]->channels();
			h = this_layer_blobs[i]->height();
			w = this_layer_blobs[i]->width();

			// new image Vector For all Activations of this Layer
			std::vector<cv::Mat> imageVector;

			//go over all channels/filters/activations
			ntot.push_back(n);
			ctot.push_back(c);
			htot.push_back(h);
			wtot.push_back(w);
			n_image_per_layer.push_back(n * c);
			for (int num = 0; num < n; num++) {
				//go over all channels
				for (int chan_num = 0; chan_num < c; chan_num++) {
					//go over h,w produce image
					cv::Mat newImage = cv::Mat::zeros(h, w, CV_32F);
					for (int hh = 0; hh < h; hh++) {
						//go over w
						for (int ww = 0; ww < w; ww++) {
							data = this_layer_blobs[i]->data_at(num, chan_num,
									hh, ww);
							newImage.at<float>(hh, ww) = data;
						}
					}
					//std::cout << layers[i]->type() << std::endl;
					//cv::normalize(newImage, newImage, 0.0, 65535, cv::NORM_MINMAX, -1);
					/*if(strcmp(layers[i]->type(),"Convolution") == 0){
					    cv::normalize(newImage, newImage, 0.0, 65535, cv::NORM_MINMAX, -1);
					}
					if(strcmp(layers[i]->type(),"ReLU") == 0){
					    cv::normalize(newImage, newImage, 0.0, 65535, cv::NORM_MINMAX, -1);
					}
					if(strcmp(layers[i]->type(),"Pooling") == 0){
					    cv::normalize(newImage, newImage, 0.0, 65535, cv::NORM_MINMAX, -1);
					}*/
					//if(strcmp(layers[i]->type(),"InnerProduct") == 0){
					//	;
					//}else{
						cv::normalize(newImage, newImage, 0.0, 65535, cv::NORM_MINMAX, -1);
					//}
					//cv::normalize(newImage, newImage, 0.0, 65535, cv::NORM_MINMAX, -1);
					imageVector.push_back(newImage);
				}
			}
			layersVector.push_back(imageVector);
		}

		//do the graphics only plot convolutional layers
		//divide the y in equal parts , one row per layer
		int counter_y = -1, counter_x = -1;

		// now use a copy of the frame and then copy it back
		caerFrameEvent tmp_frame;
		tmp_frame = *single_frame;

		// mat final Frame of activations
		cv::Mat1f frame_activity(tmp_frame->lengthX, tmp_frame->lengthY);
		int size_y_single_image = floor(tmp_frame->lengthY / layersVector.size()); // num layers
		for (int layer_num = 0; layer_num < layersVector.size(); layer_num++) { //layersVector.size()
			counter_y += 1; // count y position of image (layers)
			counter_x = -1; // reset counter_x

			// loop over all in/out filters for this layer
			for (int img_num = 0; img_num < layersVector[layer_num].size();
					img_num++) {

				counter_x += 1; // count number of images on x (filters)

				int size_x_single_image = floor(
						tmp_frame->lengthX / layersVector[layer_num].size());

				cv::Size sizeI(size_x_single_image, size_y_single_image);
				cv::Mat1f rescaled; //rescaled image

				cv::resize(layersVector[layer_num][img_num], rescaled, sizeI); //resize image
				cv::Mat data_tp = cv::Mat(rescaled.cols, rescaled.rows, CV_32F);
				cv::transpose(rescaled, data_tp);

				int xloc, yloc;
				xloc = (size_x_single_image) * counter_x;
				yloc = (size_y_single_image) * counter_y;

				data_tp.copyTo(
						frame_activity.rowRange(xloc, xloc + rescaled.cols).colRange(
								yloc, yloc + rescaled.rows));
			}
		}

		cv::Mat data_frame = cv::Mat(frame_activity.cols, frame_activity.rows, CV_16UC3);
		cv::transpose(frame_activity, data_frame);

	    // normalize output into [0,65535]
	    //cv::normalize(data_frame, data_frame, 0.0, 65535, cv::NORM_MINMAX, -1);

		/*cv::Scalar avg,sdv;
		cv::meanStdDev(data_frame, avg, sdv);
		sdv.val[0] = sqrt(data_frame.cols*data_frame.rows*sdv.val[0]*sdv.val[0]);
		cv::Mat image_32f;
		data_frame.convertTo(image_32f,CV_32F,1/sdv.val[0],-avg.val[0]/sdv.val[0]);*/

		// copy activations image into frame
		for (int y = 0; y < tmp_frame->lengthY; y++) {
			for (int x = 0; x < tmp_frame->lengthX; x++) {
				caerFrameEventSetPixel(tmp_frame, x, y, (uint16_t) data_frame.at<float>(y, x));
			}
		}
		*single_frame = tmp_frame;
	}//if show activations
	else{
		single_frame = NULL;
	}

	/* Copy the output layer to a std::vector */
	Blob<float>* output_layer = net_->output_blobs()[0];

#ifdef CPU_ONLY
	const float* begin = output_layer->cpu_data();
#else
	const float* begin = output_layer->gpu_data();
#endif
	const float* end = begin + output_layer->channels();

	return std::vector<float>(begin, end);
}

/* Wrap the input layer of the network in separate cv::Mat objects
 * (one per channel). This way we save one memcpy operation and we
 * don't need to rely on cudaMemcpy2D. The last preprocessing
 * operation will write the separate channels directly to the input
 * layer. */
void MyClass::WrapInputLayer(std::vector<cv::Mat>* input_channels) {
	Blob<float>* input_layer = net_->input_blobs()[0];

	int width = input_layer->width();
	int height = input_layer->height();
	float* input_data = input_layer->mutable_cpu_data();
	for (int i = 0; i < input_layer->channels(); ++i) {
		cv::Mat channel(height, width, CV_32FC1, input_data);
		input_channels->push_back(channel);
		input_data += width * height;
	}
}

void MyClass::Preprocess(const cv::Mat& img,
		std::vector<cv::Mat>* input_channels) {
	/* Convert the input image to the input image format of the network. */

	// std::cout << " Preprocess --- img.channnels() " << img.channels() << ", num_channels_" << num_channels_ << std::endl;
	cv::Mat sample;
	if (img.channels() == 3 && num_channels_ == 1)
		cv::cvtColor(img, sample, cv::COLOR_BGR2GRAY);
	else if (img.channels() == 4 && num_channels_ == 1)
		cv::cvtColor(img, sample, cv::COLOR_BGRA2GRAY);
	else if (img.channels() == 4 && num_channels_ == 3)
		cv::cvtColor(img, sample, cv::COLOR_BGRA2BGR);
	else if (img.channels() == 1 && num_channels_ == 3)
		cv::cvtColor(img, sample, cv::COLOR_GRAY2BGR);
	else
		sample = img;

	cv::Mat sample_resized;
	if (sample.size() != input_geometry_)
		cv::resize(sample, sample_resized, input_geometry_);
	else
		sample_resized = sample;

	cv::Mat sample_float;
	if (num_channels_ == 3)
		sample_resized.convertTo(sample_float, CV_32FC3);
	else
		sample_resized.convertTo(sample_float, CV_32FC1);

	cv::Mat sample_normalized;
	mean_ = cv::Mat::zeros(1, 1, CV_64F); //TODO remove, compute mean_ from mean_file and adapt size for subtraction.
	//std::cout << " Preprocess: mean_size " << mean_.size() << std::endl;

	cv::subtract(sample_float, mean_, sample_normalized);

	/* This operation will write the separate BGR planes directly to the
	 * input layer of the network because it is wrapped by the cv::Mat
	 * objects in input_channels. */
	cv::split(sample_normalized, *input_channels);

	CHECK(reinterpret_cast<float*>(input_channels->at(0).data)
#ifdef CPU_ONLY
			== net_->input_blobs()[0]->cpu_data())
#else
			== net_->input_blobs()[0]->gpu_data())
#endif
			<< "Input channels are not wrapping the input layer of the network.";
}

