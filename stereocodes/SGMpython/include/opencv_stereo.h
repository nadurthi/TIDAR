#ifndef OPENCV_STEREO
#define OPENCV_STEREO


#include <pybind11/eigen.h>
#include <pybind11/numpy.h>


#include <opencv2/core/cuda.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/utility.hpp>
#include "opencv2/cudastereo.hpp"

#include <string>

#include <opencv2/core/eigen.hpp>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

class Algo_openCV{
public:
	Algo_openCV(std::string opt );

	Eigen::MatrixXf  compute_stereo_bm(Eigen::MatrixXf Xleft, Eigen::MatrixXf Xright);
	Eigen::MatrixXf  compute_stereo_bp(Eigen::MatrixXf Xleft, Eigen::MatrixXf Xright);
	Eigen::MatrixXf  compute_stereo_csbp(Eigen::MatrixXf Xleft, Eigen::MatrixXf Xright);
	Eigen::MatrixXf  compute_stereo_csgm(Eigen::MatrixXf Xleft, Eigen::MatrixXf Xright);

	cv::Ptr<cv::cuda::StereoBM> bm;
	cv::Ptr<cv::cuda::StereoBeliefPropagation>  bp;
	cv::Ptr<cv::cuda::StereoConstantSpaceBP>  csbp;
	cv::Ptr<cv::cuda::StereoSGM>  csgm;
};


// #include<opencv2/opencv.hpp>
// using namespace cv;

#endif