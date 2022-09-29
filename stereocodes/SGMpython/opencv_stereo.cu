
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


	cv::Ptr<cv::cuda::StereoBM> bm = cv::cuda::createStereoBM(numDisparities, blockSize);

	
    cv::Ptr<cv::cuda::StereoBeliefPropagation>  bp = cv::cuda::createStereoBeliefPropagation(ndisp, iters, levels, msg_type);
    
	
    cv::Ptr<cv::cuda::StereoConstantSpaceBP>  csbp = cv::cuda::createStereoConstantSpaceBP(ndisp, iters, levels, nr_plane,  msg_type);

    int minDisparity = options["opencv_stereo"]["min_disparity"];
    int max_disparity = options["opencv_stereo"]["max_disparity"];
    int P1 = options["opencv_stereo"]["P1"];
    int P2 = options["opencv_stereo"]["P2"];
    int uniquenessRatio = options["opencv_stereo"]["uniquenessRatio"];
    int mode = cv::cuda::StereoSGM::MODE_HH4;
	cv::Ptr<cv::cuda::StereoSGM>  csgm = cv::cuda::createStereoSGM(minDisparity, max_disparity, P1, P2, uniquenessRatio, mode);


}

Eigen::MatrixXf  Algo_openCV::compute_stereo_bm(Eigen::MatrixXf Xleft, Eigen::MatrixXf Xright){
	 cv::Mat left,right,left2,right2;
	cv::eigen2cv(Xleft,left);
	cv::eigen2cv(Xright,right);
	cv::cuda::GpuMat d_left, d_right;

	// if (left.channels() > 1 || right.channels() > 1)
	// 	cv::cvtColor(left, left2, cv::COLOR_BGR2GRAY);
	//     cv::cvtColor(right, right2, cv::COLOR_BGR2GRAY);
	//     d_left.upload(left2);
 //    	d_right.upload(right2);
	// }
	// else{
	// 	d_left.upload(left);
	//     d_right.upload(right);
	// }

	// // cv::Mat disparity(left.size(), CV_16S);

	 
	cv::cuda::GpuMat d_disp(left.size(), CV_8U);
	 
    bm->compute(d_left, d_right, d_disp);

    cv::Mat disp(left.size(), CV_8U);
    d_disp.download(disp);

    Eigen::MatrixXf Xdisp;
	cv::cv2eigen(disp,Xdisp);
	return Xdisp;

}


Eigen::MatrixXf  Algo_openCV::compute_stereo_bp(Eigen::MatrixXf Xleft, Eigen::MatrixXf Xright){
	 cv::Mat left,right;
	cv::eigen2cv(Xleft,left);
	cv::eigen2cv(Xright,right);
	// cv::Mat disparity(left.size(), CV_16S);

	 cv::cuda::GpuMat d_left, d_right;
	 cv::cuda::GpuMat d_disp(left.size(), CV_8U);
	 cv::Mat disp(left.size(), CV_8U);

	 d_left.upload(left);
    d_right.upload(right);

    bp->compute(d_left, d_right, d_disp);
    d_disp.download(disp);

    Eigen::MatrixXf Xdisp;
	cv::cv2eigen(disp,Xdisp);
	return Xdisp;
}


Eigen::MatrixXf  Algo_openCV::compute_stereo_csbp(Eigen::MatrixXf Xleft, Eigen::MatrixXf Xright){
	 cv::Mat left,right;
	cv::eigen2cv(Xleft,left);
	cv::eigen2cv(Xright,right);


	 cv::cuda::GpuMat d_left, d_right;
	 cv::cuda::GpuMat d_disp(left.size(), CV_8U);
	 cv::Mat disp(left.size(), CV_8U);

	 d_left.upload(left);
    d_right.upload(right);

    csbp->compute(d_left, d_right, d_disp);
    d_disp.download(disp);

    Eigen::MatrixXf Xdisp;
	cv::cv2eigen(disp,Xdisp);
	return Xdisp;
}


Eigen::MatrixXf  Algo_openCV::compute_stereo_csgm(Eigen::MatrixXf Xleft, Eigen::MatrixXf Xright){
	 cv::Mat left,right;
	cv::eigen2cv(Xleft,left);
	cv::eigen2cv(Xright,right);
	cv::Mat disparity(left.size(), CV_16S);

	 cv::cuda::GpuMat d_left, d_right;
	 cv::cuda::GpuMat d_disp(left.size(), CV_8U);
	 cv::Mat disp(left.size(), CV_8U);

	 d_left.upload(left);
    d_right.upload(right);

    csgm->compute(d_left, d_right, d_disp);
    d_disp.download(disp);

    Eigen::MatrixXf Xdisp;
	cv::cv2eigen(disp,Xdisp);
	return Xdisp;
}

