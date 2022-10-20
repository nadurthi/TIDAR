
#include "libsgm_stereo.h"



Algo_libsgm::Algo_libsgm(std::string opt ){
  auto options=json::parse(opt);

	int disp_size = options["libSGM"]["max_disparity"];
	int min_disp = options["libSGM"]["min_disparity"];
	int P1 = options["libSGM"]["P1"]; //7
	int P2 = options["libSGM"]["P2"];
  float uniqueness = options["libSGM"]["uniqueness"];
	int num_paths = options["libSGM"]["num_paths"];
	int max_disparity = options["libSGM"]["max_disparity"];
	int LR_max_diff = options["libSGM"]["LR_max_diff"];

  

  wls_filter = createDisparityWLSFilterGeneric(false);
	wls_filter->setDepthDiscontinuityRadius(options["WLS"]["radius"]); //0.5*16
	wls_filter->setLambda(options["WLS"]["lambda"]); //8000
	wls_filter->setSigmaColor(options["WLS"]["sigma"]);//1.5

 //   int disp_size = 128;
	// int P1 = 20;
	// int P2 = 84;
	// float uniqueness = 0.95;
	// int num_paths = 4;
	// int min_disp = 0;
	// int LR_max_diff = 1;
   //   int h =  376;// options["cameras"]["height"];
   // int w =  1241; //options["cameras"]["width"];

   int h =   options["cameras"]["height"];
   int w =   options["cameras"]["width"];


	// ASSERT_MSG(!left.empty() && !right.empty(), "imread failed.");
	// ASSERT_MSG(left.size() == right.size() && left.type() == right.type(), "input images must be same size and type.");
	// ASSERT_MSG(left.type() == CV_8U || left.type() == CV_16U, "input image format must be CV_8U or CV_16U.");
	// ASSERT_MSG(disp_size == 64 || disp_size == 128 || disp_size == 256, "disparity size must be 64, 128 or 256.");
	// ASSERT_MSG(num_paths == 4 || num_paths == 8, "number of scanlines must be 4 or 8.");

	sgm::PathType path_type = num_paths == 8 ? sgm::PathType::SCAN_8PATH : sgm::PathType::SCAN_4PATH;
	// int input_depth = left.type() == CV_8U ? 8 : 16;
	int input_depth = 8;
	int output_depth = 16;

	param.P1=P1;
	param.P2=P2;
	param.uniqueness=uniqueness;
	param.subpixel = false;
	param.path_type=path_type;
	param.min_disp=min_disp;
	param.LR_max_diff=LR_max_diff;
	ssgm_ptr = std::unique_ptr<sgm::StereoSGM >( new sgm::StereoSGM(w, h, disp_size, input_depth, output_depth, sgm::EXECUTE_INOUT_HOST2HOST, param) );
	ssgm_cuda_ptr = std::unique_ptr<sgm::StereoSGM >( new sgm::StereoSGM(w, h, disp_size, input_depth, output_depth, sgm::EXECUTE_INOUT_CUDA2CUDA, param) );

	sgmgpu_ptr = std::unique_ptr<sgm::LibSGMWrapper>( new sgm::LibSGMWrapper(disp_size,P1,P2,uniqueness,false,path_type,min_disp, LR_max_diff) );

}
py::array_t<int16_t>  Algo_libsgm::getDisparity_cpu(py::array_t<uint8_t>& Xleft,py::array_t<uint8_t>& Xright){

	cv::Mat left(Xleft.shape(0), Xleft.shape(1), CV_8U, (unsigned char*)Xleft.data());
	cv::Mat right(Xright.shape(0), Xright.shape(1), CV_8U, (unsigned char*)Xright.data());

	

	// sgm::StereoSGM::Parameters param(P1, P2, uniqueness, false, path_type, min_disp, LR_max_diff);
	// sgm::StereoSGM ssgm(left.cols, left.rows, disp_size, input_depth, output_depth, sgm::EXECUTE_INOUT_HOST2HOST, param);

	cv::Mat disparity(left.size(), CV_16S);

	ssgm_ptr->execute(left.data, right.data, disparity.data);
	cv::Mat mask = disparity == ssgm_ptr->get_invalid_disparity();
	disparity.setTo(cv::Scalar(0, 0, 0), mask);

	size_t rows = disparity.rows;
	size_t cols = disparity.cols;
	py::array_t<int16_t> output(
                                py::buffer_info(
                                disparity.data,
                                sizeof(int16_t), //itemsize
                                py::format_descriptor<int16_t>::format(),
                                2, // ndim
                                std::vector<size_t> {rows, cols}, // shape
                                std::vector<size_t> { sizeof(int16_t) * cols , sizeof(int16_t) } // strides
    )
    );

	return output;
  
}
py::array_t<int16_t>  Algo_libsgm::getDisparity_cuda(py::array_t<uint8_t>& Xleft,py::array_t<uint8_t>& Xright){

	cv::Mat left(Xleft.shape(0), Xleft.shape(1), CV_8U, (unsigned char*)Xleft.data());
	cv::Mat right(Xright.shape(0), Xright.shape(1), CV_8U, (unsigned char*)Xright.data());
	cv::cuda::GpuMat d_left, d_right, d_disparity;
	d_left.upload(left);
	d_right.upload(right);


	cv::Mat disparity;

	ssgm_cuda_ptr->execute(d_left.data, d_right.data, d_disparity.data);
	d_disparity.download(disparity);
	cv::Mat mask = disparity == ssgm_cuda_ptr->get_invalid_disparity();
	disparity.setTo(0, mask);

	size_t rows = disparity.rows;
	size_t cols = disparity.cols;
	py::array_t<int16_t> output(
                                py::buffer_info(
                                disparity.data,
                                sizeof(int16_t), //itemsize
                                py::format_descriptor<int16_t>::format(),
                                2, // ndim
                                std::vector<size_t> {rows, cols}, // shape
                                std::vector<size_t> { sizeof(int16_t) * cols , sizeof(int16_t) } // strides
    )
    );

	return output;
  
}
py::array_t<int16_t>  Algo_libsgm::getDisparity_gpu(py::array_t<uint8_t>& Xleft,py::array_t<uint8_t>& Xright){
	cv::Mat left(Xleft.shape(0), Xleft.shape(1), CV_8U, (unsigned char*)Xleft.data());
	cv::Mat right(Xright.shape(0), Xright.shape(1), CV_8U, (unsigned char*)Xright.data());

	cv::Mat disparity_gpu,filtered_disp;

	
	cv::cuda::GpuMat d_left, d_right, d_disparity;
	d_left.upload(left);
	d_right.upload(right);
	sgmgpu_ptr->execute(d_left, d_right, d_disparity);
	d_disparity.download(disparity_gpu);
	cv::Mat maskgpu = disparity_gpu == sgmgpu_ptr->getInvalidDisparity();
	disparity_gpu.setTo(0, maskgpu);


	// auto wls = cv::ximgproc::DisparityWLSFilter();
	
	wls_filter->filter(disparity_gpu,left,filtered_disp);
	disparity_gpu=filtered_disp;

  size_t rows = disparity_gpu.rows;
	size_t cols = disparity_gpu.cols;
	py::array_t<int16_t> output(
                                py::buffer_info(
                                disparity_gpu.data,
                                sizeof(int16_t), //itemsize
                                py::format_descriptor<int16_t>::format(),
                                2, // ndim
                                std::vector<size_t> {rows, cols}, // shape
                                std::vector<size_t> { sizeof(int16_t) * cols , sizeof(int16_t) } // strides
    )
    );
	return output;
  
}
