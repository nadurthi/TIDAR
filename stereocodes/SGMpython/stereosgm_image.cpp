/*
   Copyright 2016 Fixstars Corporation

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

   http ://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
 */

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/pybind11.h>
#include <pybind11/embed.h>  // py::scoped_interpreter

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

#include <libsgm.h>

#define ASSERT_MSG(expr, msg) \
								if (!(expr)) { \
																std::cerr << msg << std::endl; \
																std::exit(EXIT_FAILURE); \
								} \


using timerdict = std::map<std::string,std::vector<float> >;
using timerdictptr = std::shared_ptr<timerdict>;
struct Timer {
								Timer(const std::string &k,timerdictptr &tptr );
								~Timer();

								std::chrono::time_point<std::chrono::high_resolution_clock> t1,t2;
								timerdictptr Tptr;
								std::string key;
};

Timer::Timer(const std::string &k,timerdictptr &tptr ){
								Tptr=tptr;
								key=k;
								t1 = std::chrono::high_resolution_clock::now();
}
Timer::~Timer(){
								t2 = std::chrono::high_resolution_clock::now();

								int sec=std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count();
								int msec=std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
								std::chrono::duration<double, std::milli> fp_ms = t2-t1;

								(*Tptr)[key].push_back( fp_ms.count() );


}


int main(int argc, char* argv[])
{
								cv::CommandLineParser parser(argc, argv,
																																					"{@left_dir   | <none> | path to input left image                                                            }"
																																					"{@right_dir  | <none> | path to input right image                                                           }"
																																					"{disp_size   |    128 | maximum possible disparity value                                                    }"
																																					"{P1          |     7 | penalty on the disparity change by plus or minus 1 between nieghbor pixels          }"
																																					"{P2          |    84 | penalty on the disparity change by more than 1 between neighbor pixels              }"
																																					"{uniqueness  |   0.95 | margin in ratio by which the best cost function value should be at least second one }"
																																					"{num_paths   |      8 | number of scanlines used in cost aggregation                                        }"
																																					"{min_disp    |      0 | minimum disparity value                                                             }"
																																					"{LR_max_diff |      1 | maximum allowed difference between left and right disparity                         }"
																																					"{help h      |        | display this help and exit                                                          }"
																																					"{dispdir     |        | disparisty folder                                                                   }");

								if (parser.has("help")) {
																parser.printMessage();
																return 0;
								}

								path dispfolder(parser.get<cv::String>("dispdir"));
								path leftimgpath,leftdir(parser.get<cv::String>("@left_dir"));
								path rightimgpath,rightdirpath(parser.get<cv::String>("@right_dir"));
								directory_iterator end_itr;


								for(auto& entry : boost::make_iterator_range(directory_iterator(leftdir), {})) {
																// std::cout << entry << "\n";
																leftimgpath = entry.path();

																rightimgpath = rightdirpath / leftimgpath.filename();
																// std::cout<<leftimgpath<<std::endl;
																// std::cout<<rightimgpath<<std::endl;
																break;
								}


								cv::Mat left = cv::imread(leftimgpath.string(), cv::IMREAD_GRAYSCALE);
								cv::Mat right = cv::imread(rightimgpath.string(), cv::IMREAD_GRAYSCALE);


								auto type = left.type();
								std::string r;

								uchar depth = type & CV_MAT_DEPTH_MASK;
								uchar chans = 1 + (type >> CV_CN_SHIFT);

								switch ( depth ) {
								case CV_8U:  r = "8U"; break;
								case CV_8S:  r = "8S"; break;
								case CV_16U: r = "16U"; break;
								case CV_16S: r = "16S"; break;
								case CV_32S: r = "32S"; break;
								case CV_32F: r = "32F"; break;
								case CV_64F: r = "64F"; break;
								default:     r = "User"; break;
								}

								r += "C";
								r += (chans+'0');

								std::cout << "image type = " << r << std::endl;
								if (!parser.check()) {
																parser.printErrors();
																parser.printMessage();
																std::exit(EXIT_FAILURE);
								}

								const int disp_size = parser.get<int>("disp_size");
								const int P1 = parser.get<int>("P1");
								const int P2 = parser.get<int>("P2");
								const float uniqueness = parser.get<float>("uniqueness");
								const int num_paths = parser.get<int>("num_paths");
								const int min_disp = parser.get<int>("min_disp");
								const int LR_max_diff = parser.get<int>("LR_max_diff");

								ASSERT_MSG(!left.empty() && !right.empty(), "imread failed.");
								ASSERT_MSG(left.size() == right.size() && left.type() == right.type(), "input images must be same size and type.");
								ASSERT_MSG(left.type() == CV_8U || left.type() == CV_16U, "input image format must be CV_8U or CV_16U.");
								ASSERT_MSG(disp_size == 64 || disp_size == 128 || disp_size == 256, "disparity size must be 64, 128 or 256.");
								ASSERT_MSG(num_paths == 4 || num_paths == 8, "number of scanlines must be 4 or 8.");

								const sgm::PathType path_type = num_paths == 8 ? sgm::PathType::SCAN_8PATH : sgm::PathType::SCAN_4PATH;
								const int input_depth = left.type() == CV_8U ? 8 : 16;
								const int output_depth = 16;

								const sgm::StereoSGM::Parameters param(P1, P2, uniqueness, false, path_type, min_disp, LR_max_diff);
								sgm::StereoSGM ssgm(left.cols, left.rows, disp_size, input_depth, output_depth, sgm::EXECUTE_INOUT_HOST2HOST, param);

								sgm::LibSGMWrapper sgmgpu(disp_size,P1,P2,uniqueness,false,path_type,min_disp, LR_max_diff);

								// cycle through the directory


								cv::Mat disparity(left.size(), CV_16S);
								auto tmap =  std::make_shared<timerdict>();
								cv::Mat disparity_gpu;
								for(auto& entry : boost::make_iterator_range(directory_iterator(leftdir), {})) {
																std::cout << entry << "\n";
																leftimgpath = entry.path();

																rightimgpath = rightdirpath / leftimgpath.filename();
																cv::Mat left = cv::imread(leftimgpath.string(), -1);
																cv::Mat right = cv::imread(rightimgpath.string(), -1);
																{
																								Timer tt("libsgm",tmap);
																								ssgm.execute(left.data, right.data, disparity.data);
																}
																cv::Mat mask = disparity == ssgm.get_invalid_disparity();
																disparity.setTo(cv::Scalar(0, 0, 0), mask);
																auto pp = dispfolder / leftimgpath.filename();
																cv::imwrite(pp.string(), disparity);



																try {
																								Timer tt("libsgm_gpu",tmap);
																								cv::cuda::GpuMat d_left, d_right, d_disparity;
																								d_left.upload(left);
																								d_right.upload(right);
																								sgmgpu.execute(d_left, d_right, d_disparity);
																								d_disparity.download(disparity_gpu);
																}
																catch (const cv::Exception& e) {
																								std::cerr << e.what() << std::endl;
																								return e.code == cv::Error::GpuNotSupported ? 1 : -1;
																}


																cv::Mat maskgpu = disparity_gpu == sgmgpu.getInvalidDisparity();
																disparity_gpu.setTo(0, maskgpu);
																pp = dispfolder / "gpu"/leftimgpath.filename();
																cv::imwrite(pp.string(), disparity_gpu);
								}

								ofstream myfile,myfilegpu;
								auto pp = dispfolder / path("timing.txt");
								auto ppgpu = dispfolder / "gpu" / path("timing.txt");
								myfile.open (pp.string());
								myfilegpu.open (ppgpu.string());
								for(auto mm:tmap->at("libsgm") ) {
																myfile << std::fixed << std::setw(11) << std::setprecision(6) << std::setfill('0') << mm<<"\n";
								}
								for(auto mm:tmap->at("libsgm_gpu") ) {
																myfilegpu << std::fixed << std::setw(11) << std::setprecision(6) << std::setfill('0') << mm<<"\n";
								}
								myfile.close();
								myfilegpu.close();

								ssgm.execute(left.data, right.data, disparity.data);

								// create mask for invalid disp
								cv::Mat mask = disparity == ssgm.get_invalid_disparity();

								// show image
								cv::Mat disparity_8u, disparity_color;
								disparity.convertTo(disparity_8u, CV_8U, 255. / disp_size);
								cv::applyColorMap(disparity_8u, disparity_color, cv::COLORMAP_JET);
								disparity_8u.setTo(0, mask);
								disparity_color.setTo(cv::Scalar(0, 0, 0), mask);
								if (left.type() != CV_8U)
																cv::normalize(left, left, 0, 255, cv::NORM_MINMAX, CV_8U);

								std::vector<cv::Mat> images = { disparity_8u, disparity_color, left };
								std::vector<std::string> titles = { "disparity", "disparity color", "input" };

								std::cout << "Hot keys:" << std::endl;
								std::cout << "\tESC - quit the program" << std::endl;
								std::cout << "\ts - switch display (disparity | colored disparity | input image)" << std::endl;

								int mode = 0;
								while (true) {

																cv::setWindowTitle("image", titles[mode]);
																cv::imshow("image", images[mode]);

																const char c = cv::waitKey(0);
																if (c == 's')
																								mode = (mode < 2 ? mode + 1 : 0);
																if (c == 27)
																								break;
								}

								return 0;
}
