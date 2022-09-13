#####################################################
##               Read bag from file                ##
#####################################################


# First import library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
# Import argparse for command-line options
import argparse
# Import os.path for file path manipulation
import os.path
import os

# Create object for parsing command-line options
parser = argparse.ArgumentParser(description="Read recorded bag file and display depth stream in jet colormap.\
                                Remember to change the stream fps and format to match the recorded.")
# Add argument which takes path to a bag file as an input
parser.add_argument("-i", "--input", type=str, help="Path to the bag file")
# Parse the command line arguments to an object
args = parser.parse_args()
# Safety if no parameter have been given
if not args.input:
    print("No input paramater have been given.")
    print("For help type --help")
    exit()
# Check if the given file have bag extension
if os.path.splitext(args.input)[1] != ".bag":
    print("The given file is not of correct file format.")
    print("Only .bag files are accepted")
    exit()

pc = rs.pointcloud()
points = rs.points()


path = '/media/na0043/misc/DATA/ship/color_image_bgra_files'

os.makedirs(os.path.join(path,'rgbd'),exist_ok=True)
os.makedirs(os.path.join(path,'ply'),exist_ok=True)
cnt=0
try:
    # Create pipeline
    pipeline = rs.pipeline()

    # Create a config object
    config = rs.config()

    # Tell config that we will use a recorded device from file to be used by the pipeline through playback.
    rs.config.enable_device_from_file(config, args.input)

    # Configure the pipeline to stream the depth stream
    # Change this parameters according to the recorded bag file resolution
    config.enable_stream(rs.stream.depth, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgba8, 30)

    # Start streaming from file
    pipeline.start(config)

    # Create opencv window to render image in
    cv2.namedWindow("Depth Stream", cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow("Color Stream", cv2.WINDOW_AUTOSIZE)
    
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Create colorizer object
    colorizer = rs.colorizer()

    # Streaming loop
    while True:
        # Get frameset of depth
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        # Get depth frame
        depth_frame = aligned_frames.get_depth_frame()
        color_frame=aligned_frames.get_color_frame()
        # Colorize depth frame to jet colormap
        depth_color_frame = colorizer.colorize(depth_frame)

        # Convert depth_frame to numpy array to render image in opencv
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_color_image = np.asanyarray(depth_color_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        print(color_image.shape,"  ...  ",depth_image.shape)
        color_image_bgra = cv2.cvtColor(color_image, cv2.COLOR_RGBA2BGRA )
        # cvtColor(src,dst,CV_YUV2BGR_YUY2);

        # Render image in opencv window
        cv2.imshow("Depth Stream", depth_color_image)
        cv2.imshow("Color Stream", color_image_bgra)
        np.savez(os.path.join(path,'rgbd','data_%d'%cnt),depth=depth_image,color=color_image)

        # Create save_to_ply object
        # ply = rs.save_to_ply(os.path.join(path,'ply','data_%d.ply'%cnt))
        #
        # # Set options to the desired values
        # # In this example we'll generate a textual PLY with normals (mesh is already created by default)
        # ply.set_option(rs.save_to_ply.option_ply_binary, True)
        # ply.set_option(rs.save_to_ply.option_ply_normals, False)


        # Apply the processing block to the frameset which contains the depth frame and the texture
        # ply.process(colorized)


        pc.map_to(color_frame)
        points = pc.calculate(depth_frame)

        # points = pc.calculate(depth_frame)
        # pc.map_to(color_frame)
        points.export_to_ply(os.path.join(path,'ply','data_%d.ply'%cnt), color_frame)

        cnt+=1

        key = cv2.waitKey(1)
        # if pressed escape exit program
        if key == 27:
            cv2.destroyAllWindows()
            break

finally:
    pass