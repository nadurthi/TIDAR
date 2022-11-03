# installs
sudo apt install ros-foxy-ament-cmake-nose
sudo apt-get install ros-foxy-diagnostic-updater
sudo apt-get install libpcap-dev

# velodyne
git clone -b foxy-devel git@github.com:ros-drivers/velodyne.git

source install/setup.bash
ros2 launch velodyne velodyne-all-nodes-VLP16-composed-launch.py

# tis cam
tcam-capture

## to read all cameras into opencv
python TIScamreaders.py


# not working wired ethernet
$ sudo nmcli networking off
$ sudo nmcli networking on

