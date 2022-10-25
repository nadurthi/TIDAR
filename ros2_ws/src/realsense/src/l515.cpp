#include <librealsense2/rs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <memory>
// for Size
#include <opencv2/core/types.hpp>
// for CV_8UC3
#include <opencv2/core/hal/interface.h>

//#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
//#include "realsense/msg/color_depth.hpp"  

//#include <sensor_msgs/image_encodings.h>
//#include <image_transport/image_transport.h>
//#include "rclcpp/rclcpp.hpp"
//#include "sensor_msgs/msg/image.hpp"
//#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
//#include <stdio.h>
#include<Eigen/Dense>
#include<fstream>
#include <vector>



#include <iostream>
using namespace cv;
using std::placeholders::_1;
using namespace Eigen;
/*
 This example introduces the concept of spatial stream alignment.
 For example usecase of alignment, please check out align-advanced and measure demos.
 The need for spatial alignment (from here "align") arises from the fact
 that not all camera streams are captured from a single viewport.
 Align process lets the user translate images from one viewport to another.
 That said, results of align are synthetic streams, and suffer from several artifacts:
 1. Sampling - mapping stream to a different viewport will modify the resolution of the frame
               to match the resolution of target viewport. This will either cause downsampling or
               upsampling via interpolation. The interpolation used needs to be of type
               Nearest Neighbor to avoid introducing non-existing values.
 2. Occlussion - Some pixels in the resulting image correspond to 3D coordinates that the original
               sensor did not see, because these 3D points were occluded in the original viewport.
               Such pixels may hold invalid texture values.
*/

// This example assumes camera with depth and color
// streams, and direction lets you define the target stream



class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber"),align_to_color(RS2_STREAM_COLOR)
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "optitrack", 1, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      
      


    // Create a pipeline to easily configure and start the camera
    

    cfg.enable_stream(RS2_STREAM_DEPTH,640, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR,640, 480, RS2_FORMAT_BGR8, 30);
    pipe.start(cfg);

    X=Eigen::MatrixXf::Zero(10000,9);    
    cntc=0;
    }
	void saveData()
	{
		//https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
		const static IOFormat CSVFormat(FullPrecision, DontAlignCols, ", ", "\n");

		std::ofstream file("simulation/Poses.csv");
		if (file.is_open())
		{
		    file << X.format(CSVFormat);
		    file.close();
		}
	} 
  
    void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
    {
      	std::cout<< msg->pose.position.x<<" " << msg->pose.position.y<<" " << msg->pose.position.z << std::endl;
      	X(cntc,0)=msg->header.stamp.sec;
      	X(cntc,1)=msg->header.stamp.nanosec;
      	X(cntc,2)=msg->pose.position.x;
      	X(cntc,3)=msg->pose.position.y;
      	X(cntc,4)=msg->pose.position.z;
      	X(cntc,5)=msg->pose.orientation.x;
      	X(cntc,6)=msg->pose.orientation.y;
      	X(cntc,7)=msg->pose.orientation.z;
      	X(cntc,8)=msg->pose.orientation.w;
      	
      	
      	// Using the align object, we block the application until a frameset is available
        rs2::frameset frameset = pipe.wait_for_frames();
		
        
        // Align all frames to color viewport
        frameset = align_to_color.process(frameset);
    

        // With the aligned frameset we proceed as usual
        auto depth = frameset.get_depth_frame();
        auto color = frameset.get_color_frame();
        //auto colorized_depth = c.colorize(depth);
		
		const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();
        
		Mat colormat(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
		//Mat depthcolormat(Size(w, h), CV_8UC3, (void*)colorized_depth.get_data(), Mat::AUTO_STEP);
		Mat depthmat(Size(w, h), CV_16UC3, (void*)depth.get_data(), Mat::AUTO_STEP);
		
		//Mat colormat2;
		//cvtColor(colormat, colormat2, COLOR_RGBA2BGRA );
        imshow("Display window", colormat);
        //imshow("Depth", depthcolormat);
        int key = (waitKey(10) & 0xFF);
    	
    	char cbuf[30],dbuf[30];
		sprintf( cbuf, "simulation/color_%05d.png", cntc );
		sprintf( dbuf, "simulation/depth_%05d.png", cntc );
		
    	imwrite(cbuf, colormat);
    	imwrite(dbuf, depthmat);
        if (key == 'q') 
        	return; // stop capturing by pressing q
        	
       cntc=cntc+1;
        
    }
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rs2::colorizer c;                     // Helper to colorize depth images
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::align align_to_color;
    Eigen::MatrixXf X;
	int cntc;
};

int main(int argc, char * argv[]) try
{	
	rclcpp::init(argc, argv);
	auto mp = std::make_shared<MinimalSubscriber>();
	 
	 rclcpp::spin(mp);
	mp->saveData();
  	//rclcpp::executors::SingleThreadedExecutor executor;
	//executor.add_node(mp);
	//executor.spin();
  

	rclcpp::shutdown();
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}


