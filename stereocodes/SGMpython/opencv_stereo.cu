
#include "opencv_stereo.h"


Algo_openCV::Algo_openCV(std::string opt ){
	auto options=json::parse(opt);

	
	
	int numDisparities = options["opencv_stereo"]["max_disparity"];
	int blockSize = options["opencv_stereo"]["blockSize"]; //19

	int ndisp = options["opencv_stereo"]["max_disparity"];
	int iters = options["opencv_stereo"]["iters"];
	int levels  = options["opencv_stereo"]["levels"];
	int msg_type = CV_32F;

	int nr_plane = options["opencv_stereo"]["nr_plane"];


	bm = cv::cuda::createStereoBM(128, 3);

	
    bp = cv::cuda::createStereoBeliefPropagation(ndisp, iters, levels, msg_type);
    
	
    csbp = cv::cuda::createStereoConstantSpaceBP(ndisp, iters, levels, nr_plane,  msg_type);

    int minDisparity = options["opencv_stereo"]["min_disparity"];
    int max_disparity = options["opencv_stereo"]["max_disparity"];
    int P1 = options["opencv_stereo"]["P1"];
    int P2 = options["opencv_stereo"]["P2"];
    int uniquenessRatio = options["opencv_stereo"]["uniquenessRatio"];
    int mode = cv::cuda::StereoSGM::MODE_HH4;
	csgm = cv::cuda::createStereoSGM(minDisparity, max_disparity, P1, P2, uniquenessRatio, mode);


}

py::array_t<float>   Algo_openCV::compute_stereo_bm(py::array_t<uint8_t>& Xleft, py::array_t<uint8_t>& Xright){
	cv::Mat left(Xleft.shape(0), Xleft.shape(1), CV_8U, (unsigned char*)Xleft.data());
	cv::Mat right(Xright.shape(0), Xright.shape(1), CV_8U, (unsigned char*)Xright.data());
	
	cv::cuda::GpuMat d_left, d_right;
	d_left.upload(left);
    d_right.upload(right);
	
	cv::Mat disp,disp2;
	cv::cuda::GpuMat d_disp;
	 
    bm->compute(d_left, d_right, d_disp);
    d_disp.download(disp);

    disp.convertTo(disp2,CV_32F,1,0);

    size_t rows = disp2.rows;
	size_t cols = disp2.cols;
	py::array_t<float> output(
                                py::buffer_info(
                                disp2.data,
                                sizeof(float), //itemsize
                                py::format_descriptor<float>::format(),
                                2, // ndim
                                std::vector<size_t> {rows, cols}, // shape
                                std::vector<size_t> { sizeof(float) * cols , sizeof(float) } // strides
    )
    );

	return output;

}


py::array_t<int16_t>  Algo_openCV::compute_stereo_bp(py::array_t<uint8_t>& Xleft, py::array_t<uint8_t>& Xright){
	cv::Mat left(Xleft.shape(0), Xleft.shape(1), CV_8U, (unsigned char*)Xleft.data());
	cv::Mat right(Xright.shape(0), Xright.shape(1), CV_8U, (unsigned char*)Xright.data());
	

	cv::cuda::GpuMat d_left, d_right;
	cv::cuda::GpuMat d_disp(left.size(), CV_16S);
	cv::Mat disp(left.size(), CV_16S);

	d_left.upload(left);
    d_right.upload(right);

    bp->compute(d_left, d_right, d_disp);
    d_disp.download(disp);

    size_t rows = disp.rows;
	size_t cols = disp.cols;
	py::array_t<int16_t> output(
                                py::buffer_info(
                                disp.data,
                                sizeof(int16_t), //itemsize
                                py::format_descriptor<int16_t>::format(),
                                2, // ndim
                                std::vector<size_t> {rows, cols}, // shape
                                std::vector<size_t> { sizeof(int16_t) * cols , sizeof(int16_t) } // strides
    )
    );

	return output;
}


py::array_t<int16_t>  Algo_openCV::compute_stereo_csbp(py::array_t<uint8_t>& Xleft, py::array_t<uint8_t>& Xright){
	cv::Mat left(Xleft.shape(0), Xleft.shape(1), CV_8U, (unsigned char*)Xleft.data());
	cv::Mat right(Xright.shape(0), Xright.shape(1), CV_8U, (unsigned char*)Xright.data());


	 cv::cuda::GpuMat d_left, d_right;
	 cv::cuda::GpuMat d_disp(left.size(), CV_16S);
	 cv::Mat disp(left.size(), CV_16S);

	d_left.upload(left);
    d_right.upload(right);

    csbp->compute(d_left, d_right, d_disp);
    d_disp.download(disp);

    size_t rows = disp.rows;
	size_t cols = disp.cols;
	py::array_t<int16_t> output(
                                py::buffer_info(
                                disp.data,
                                sizeof(int16_t), //itemsize
                                py::format_descriptor<int16_t>::format(),
                                2, // ndim
                                std::vector<size_t> {rows, cols}, // shape
                                std::vector<size_t> { sizeof(int16_t) * cols , sizeof(int16_t) } // strides
    )
    );

	return output;
}


py::array_t<int16_t> Algo_openCV::compute_stereo_csgm(py::array_t<uint8_t>& Xleft, py::array_t<uint8_t>& Xright){


	cv::Mat left(Xleft.shape(0), Xleft.shape(1), CV_8U, (unsigned char*)Xleft.data());
	cv::Mat right(Xright.shape(0), Xright.shape(1), CV_8U, (unsigned char*)Xright.data());
	
	cv::cuda::GpuMat d_left, d_right;
	d_left.upload(left);
    d_right.upload(right);
	
	cv::Mat disp,disp2;
	cv::cuda::GpuMat d_disp;
	 
    csgm->compute(d_left, d_right, d_disp);
    d_disp.download(disp);

    disp.convertTo(disp2,CV_32F,1,0);

    size_t rows = disp.rows;
	size_t cols = disp.cols;
	py::array_t<int16_t> output(
                                py::buffer_info(
                                disp.data,
                                sizeof(int16_t), //itemsize
                                py::format_descriptor<int16_t>::format(),
                                2, // ndim
                                std::vector<size_t> {rows, cols}, // shape
                                std::vector<size_t> { sizeof(int16_t) * cols , sizeof(int16_t) } // strides
    )
    );

	return output;

}

