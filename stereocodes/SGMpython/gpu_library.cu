#include <sstream>
#include <iostream>
#include <cuda_runtime.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
// #include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/embed.h>  // py::scoped_interpreter

#include <cuda_runtime.h>
#include <nlohmann/json.hpp>
#include "pybind11_json.h"

// //eigen`
// #include <Eigen/Core>
// #include <Eigen/Geometry>


#include "libsgm_stereo.h"
#include "opencv_stereo.h"
#include <iostream>
#include <vector>
#include <string>
using json = nlohmann::json;



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



void take_json(const json& s) {
    std::cout << "This function took an nlohmann::json instance as argument: " << s << std::endl;
}
json return_json() {
    json j = {{"value", 1}};

    std::cout << "This function returns an nlohmann::json instance: "  << j << std::endl;

    return j;
}


namespace py = pybind11;

py::array_t<uint8_t> flipcvMat(py::array_t<uint8_t>& img)
{
    size_t rows = img.shape(0);
    size_t cols = img.shape(1);
    auto channels = img.shape(2);
    std::cout << "rows: " << rows << " cols: " << cols << " channels: " << channels << std::endl;
    auto type = CV_8UC3;

    cv::Mat cvimg2(rows, cols, type, (unsigned char*)img.data());

    cv::imwrite("/source/test.png", cvimg2); // OK

    cv::Mat cvimg3(rows, cols, type);
    cv::flip(cvimg2, cvimg3, 0);

    cv::imwrite("/source/testout.png", cvimg3); // OK

    py::array_t<uint8_t> output(
                                py::buffer_info(
                                cvimg3.data,
                                sizeof(uint8_t), //itemsize
                                py::format_descriptor<uint8_t>::format(),
                                3, // ndim
                                std::vector<size_t> {rows, cols , 3}, // shape
                                std::vector<size_t> { sizeof(uint8_t) * cols * 3, sizeof(uint8_t) * 3, sizeof(uint8_t)} // strides
    )
    );
    return output;
}




PYBIND11_MODULE(gpu_library, m)
{
  m.def("multiply_with_scalar", map_array<double>);
  
  m.def("take_json", &take_json, "pass py::object to a C++ function that takes an nlohmann::json");
  m.def("return_json", &return_json, "return py::object from a C++ function that returns an nlohmann::json");
  m.def("flipcvMat", &flipcvMat, "flipcvMat");



  py::class_<Algo_libsgm>(m, "Algo_libsgm")
        .def(py::init<const std::string &>())
        .def("getDisparity_cpu", &Algo_libsgm::getDisparity_cpu)
        .def("getDisparity_cuda", &Algo_libsgm::getDisparity_cuda)
        .def("getDisparity_gpu", &Algo_libsgm::getDisparity_gpu);

    py::class_<Algo_openCV>(m, "Algo_openCV")
        .def(py::init<const std::string &>())
        .def("compute_stereo_bm", &Algo_openCV::compute_stereo_bm)
        .def("compute_stereo_bp", &Algo_openCV::compute_stereo_bp)
        .def("compute_stereo_csbp", &Algo_openCV::compute_stereo_csbp)
        .def("compute_stereo_csgm", &Algo_openCV::compute_stereo_csgm);
        
}
