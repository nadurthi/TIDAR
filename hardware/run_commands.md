# velodyne
source install/setup.bash
ros2 launch velodyne velodyne-all-nodes-VLP16-composed-launch.py

# tis cam
tcam-capture

## to read all cameras into opencv
python TIScamreaders.py

