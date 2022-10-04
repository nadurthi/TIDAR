#include <memory>
#include <string>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/embed.h>

#include <pybind11/eigen.h>
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

#include <opencv2/core/eigen.hpp>


int main(int argc, char* argv[]){
	cv::Mat left_src,left,right_src,right;

	left_src = cv::imread("1.png");
	right_src = cv::imread("2.png");
	cv::cvtColor(left_src, left, cv::COLOR_BGR2GRAY);
	cv::cvtColor(right_src, right, cv::COLOR_BGR2GRAY);

	cv::Ptr<cv::cuda::StereoBM> bm = cv::cuda::createStereoBM(128, 3);

	int minDisparity = 0;
    int max_disparity = 128;
    int P1 = 20;
    int P2 = 84;
    int uniquenessRatio = 5;
    int mode = cv::cuda::StereoSGM::MODE_HH4;
	cv::Ptr<cv::cuda::StereoSGM>   csgm = cv::cuda::createStereoSGM(minDisparity, max_disparity, P1, P2, uniquenessRatio, mode);

	cv::cuda::GpuMat d_left, d_right;

	d_left.upload(left);
	d_right.upload(right);
	
	cv::Mat disp,disp2;
	cv::cuda::GpuMat d_disp;
	
	std::cout <<"begin working"<<std::endl; 
    csgm->compute(d_left, d_right, d_disp);
    d_disp.download(disp);
    disp.convertTo(disp2,CV_32F,1,0);


    double minVal; 
	double maxVal; 
	cv::Point minLoc; 
	cv::Point maxLoc;

	cv::minMaxLoc( disp, &minVal, &maxVal, &minLoc, &maxLoc );

	std::cout << "min val: " << minVal << std::endl;
	std::cout << "max val: " << maxVal << std::endl;
    std::cout <<"done working .. "<< disp.depth()<< "   "<< disp.channels()<< "   "<< disp.type()<<std::endl;
    std::cout <<"done working .. "<< disp.at<int16_t>(maxLoc.y,maxLoc.x) <<std::endl;
    std::cout <<"done working .. "<< disp2.at<float>(maxLoc.y,maxLoc.x) <<std::endl;
    
}

