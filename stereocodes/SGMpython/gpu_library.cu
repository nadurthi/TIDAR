#include <sstream>
#include <iostream>
#include <cuda_runtime.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/embed.h>  // py::scoped_interpreter

#include <cuda_runtime.h>
#include <nlohmann/json.hpp>
#include "pybind11_json.h"

// //eigen`
// #include <Eigen/Core>
// #include <Eigen/Geometry>


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
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/filesystem.hpp>
using namespace boost::filesystem;
using json = nlohmann::json;

#include <libsgm.h>


template <typename T>
__global__ void kernel
(T *vec, T scalar, int num_elements)
{
  unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < num_elements) {
    vec[idx] = vec[idx] * scalar;
  }
}

template <typename T>
void run_kernel
(T *vec, T scalar, int num_elements)
{
  dim3 dimBlock(256, 1, 1);
  dim3 dimGrid(ceil((T)num_elements / dimBlock.x));
  
  kernel<T><<<dimGrid, dimBlock>>>
    (vec, scalar, num_elements);

  cudaError_t error = cudaGetLastError();
  if (error != cudaSuccess) {
    std::stringstream strstr;
    strstr << "run_kernel launch failed" << std::endl;
    strstr << "dimBlock: " << dimBlock.x << ", " << dimBlock.y << std::endl;
    strstr << "dimGrid: " << dimGrid.x << ", " << dimGrid.y << std::endl;
    strstr << cudaGetErrorString(error);
    throw strstr.str();
  }
}

template <typename T>
void map_array(pybind11::array_t<T> vec, T scalar)
{
  pybind11::buffer_info ha = vec.request();

  if (ha.ndim != 1) {
    std::stringstream strstr;
    strstr << "ha.ndim != 1" << std::endl;
    strstr << "ha.ndim: " << ha.ndim << std::endl;
    throw std::runtime_error(strstr.str());
  }

  int size = ha.shape[0];
  int size_bytes = size*sizeof(T);
  T *gpu_ptr;
  cudaError_t error = cudaMalloc(&gpu_ptr, size_bytes);

  if (error != cudaSuccess) {
    throw std::runtime_error(cudaGetErrorString(error));
  }

  T* ptr = reinterpret_cast<T*>(ha.ptr);
  error = cudaMemcpy(gpu_ptr, ptr, size_bytes, cudaMemcpyHostToDevice);
  if (error != cudaSuccess) {
    throw std::runtime_error(cudaGetErrorString(error));
  }

  run_kernel<T>(gpu_ptr, scalar, size);

  error = cudaMemcpy(ptr, gpu_ptr, size_bytes, cudaMemcpyDeviceToHost);
  if (error != cudaSuccess) {
    throw std::runtime_error(cudaGetErrorString(error));
  }

  error = cudaFree(gpu_ptr);
  if (error != cudaSuccess) {
    throw std::runtime_error(cudaGetErrorString(error));
  }
}

class Algo_libsgm{
public:
Algo_libsgm(std::string opt ){
  auto options=json::parse(opt);

	int disp_size = options["libSGM"]["max_disparity"];
	// int P1 = options["libSGM"]["P1"]; //7
	// int P2 = options["libSGM"]["P2"];
 //  float uniqueness = options["libSGM"]["uniqueness"];
	// int num_paths = options["libSGM"]["num_paths"];
	// int num_paths = options["libSGM"]["max_disparity"];
	// int LR_max_diff = options["libSGM"]["LR_max_diff"];

	// const sgm::PathType path_type = num_paths == 8 ? sgm::PathType::SCAN_8PATH : sgm::PathType::SCAN_4PATH;
	// const int input_depth = 16;
	// const int output_depth = 16;

 //  const int h = options["cameras"]["height"];
 //  const int w = options["cameras"]["width"];
   
	// const sgm::StereoSGM::Parameters param(P1, P2, uniqueness, false, path_type, min_disp, LR_max_diff);
	// sgm::StereoSGM ssgm(w, h, disp_size, input_depth, output_depth, sgm::EXECUTE_INOUT_HOST2HOST, param);

	// sgm::LibSGMWrapper sgmgpu(disp_size,P1,P2,uniqueness,false,path_type,min_disp, LR_max_diff);

}
Eigen::MatrixXf getDisparity(const Eigen::Ref<const Eigen::MatrixXf> &Xleft,const Eigen::Ref<const Eigen::MatrixXf> &Xright){
	// Mat left,right;
	// cv.eigen2cv(Xleft,left);
	// cv.eigen2cv(Xright,right);
	// cv::Mat disparity(left.size(), CV_16S);
	// cv::Mat disparity_gpu;

	// ssgm.execute(left.data, right.data, disparity.data);
	// cv::Mat mask = disparity == ssgm.get_invalid_disparity();
	// disparity.setTo(cv::Scalar(0, 0, 0), mask);

	// try {
	// 		cv::cuda::GpuMat d_left, d_right, d_disparity;
	// 		d_left.upload(left);
	// 		d_right.upload(right);
	// 		sgmgpu.execute(d_left, d_right, d_disparity);
	// 		d_disparity.download(disparity_gpu);
	// }
	// catch (const cv::Exception& e) {
	// 		std::cerr << e.what() << std::endl;
	// 		return e.code == cv::Error::GpuNotSupported ? 1 : -1;
	// }
	// cv::Mat maskgpu = disparity_gpu == sgmgpu.getInvalidDisparity();
	// disparity_gpu.setTo(0, maskgpu);

  Eigen::MatrixXf Xdisp;
 //  cv.cv2eigen(disparity_gpu,Xdisp);
  return Xdisp;
  
}

std::unique_ptr<sgm::StereoSGM> ssgm_ptr;
std::unique_ptr<sgm::LibSGMWrapper> sgmgpu_ptr;

};

void take_json(const json& s) {
    std::cout << "This function took an nlohmann::json instance as argument: " << s << std::endl;
}
json return_json() {
    json j = {{"value", 1}};

    std::cout << "This function returns an nlohmann::json instance: "  << j << std::endl;

    return j;
}

namespace py = pybind11;
PYBIND11_MODULE(gpu_library, m)
{
  m.def("multiply_with_scalar", map_array<double>);
  
  m.def("take_json", &take_json, "pass py::object to a C++ function that takes an nlohmann::json");
  m.def("return_json", &return_json, "return py::object from a C++ function that returns an nlohmann::json");

  py::class_<Algo_libsgm>(m, "Algo_libsgm")
        .def(py::init<const std::string &>());
        // .def("getDisparity", &Algorithms::getDisparity);
        
}
