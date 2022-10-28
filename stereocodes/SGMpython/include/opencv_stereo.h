#ifndef OPENCV_STEREO
#define OPENCV_STEREO

#include <memory>
#include <string>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/embed.h>

// #include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include <opencv2/core/cuda.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/utility.hpp>
#include "opencv2/cudastereo.hpp"

#include <stdlib.h>
#include <iostream>
#include <string>
#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <random>
#include <utility>
#include <thread>
#include <map>
#include <unordered_map>
#include <queue>
#include <array>
#include <iomanip>
#include <memory>

// #include <opencv2/core/eigen.hpp>
#include <nlohmann/json.hpp>
using json = nlohmann::json;
namespace py = pybind11;




class Algo_openCV{
public:
	Algo_openCV(std::string opt );

	py::array_t<float>  compute_stereo_bm(py::array_t<uint8_t>& Xleft, py::array_t<uint8_t>& Xright);
	py::array_t<int16_t> compute_stereo_bp(py::array_t<uint8_t>& Xleft, py::array_t<uint8_t>& Xright);
	py::array_t<int16_t> compute_stereo_csbp(py::array_t<uint8_t>& Xleft, py::array_t<uint8_t>& Xright);
	py::array_t<int16_t> compute_stereo_csgm(py::array_t<uint8_t>& Xleft, py::array_t<uint8_t>& Xright);

	cv::Ptr<cv::cuda::StereoBM> bm;
	cv::Ptr<cv::cuda::StereoBeliefPropagation>  bp;
	cv::Ptr<cv::cuda::StereoConstantSpaceBP>  csbp;
	cv::Ptr<cv::cuda::StereoSGM>  csgm;
};


// #include<opencv2/opencv.hpp>
// using namespace cv;

#endif