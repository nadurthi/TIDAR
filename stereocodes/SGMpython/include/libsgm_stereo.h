#ifndef LIBSGM_STEREO
#define LIBSGM_STEREO

#include <memory>
#include <string>
#include <nlohmann/json.hpp>
#include <libsgm.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/embed.h>

// #include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/core/eigen.hpp>
#include <iostream>


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
// #include <opencv2/ximgproc/disparity_filter.hpp>
#include "opencv2/ximgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

using namespace cv;
using namespace cv::ximgproc;

#define ASSERT_MSG(expr, msg) \
								if (!(expr)) { \
																std::cerr << msg << std::endl; \
																std::exit(EXIT_FAILURE); \
								} \

								
using json = nlohmann::json;
namespace py = pybind11;


class Algo_libsgm{
public:
Algo_libsgm(std::string opt );
py::array_t<int16_t> getDisparity_cpu(py::array_t<uint8_t>& Xleft, py::array_t<uint8_t>& Xright);
py::array_t<int16_t> getDisparity_cuda(py::array_t<uint8_t>& Xleft, py::array_t<uint8_t>& Xright);
py::array_t<int16_t> getDisparity_gpu(py::array_t<uint8_t>& Xleft, py::array_t<uint8_t>& Xright);

std::unique_ptr<sgm::StereoSGM> ssgm_ptr;
std::unique_ptr<sgm::StereoSGM> ssgm_cuda_ptr;
std::unique_ptr<sgm::LibSGMWrapper> sgmgpu_ptr;
sgm::StereoSGM::Parameters param;

Ptr<DisparityWLSFilter> wls_filter;

};



#endif