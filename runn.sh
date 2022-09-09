export DISP=/run/user/1000/gvfs/sftp:host=10.42.0.1,user=na0043/media/na0043/misc/DATA/StereoDatasets/stereo2012/predictions/libsgm_nano_pth8_d128_p1_7_p2_84

export TRAINING=/run/user/1000/gvfs/sftp:host=10.42.0.1,user=na0043/media/na0043/misc/DATA/StereoDatasets/stereo2012/data_stereo_flow/training

export RIGHT=/run/user/1000/gvfs/sftp:host=10.42.0.1,user=na0043/media/na0043/misc/DATA/StereoDatasets/stereo2012/data_stereo_flow/training/colored_1

export LEFT=/run/user/1000/gvfs/sftp:host=10.42.0.1,user=na0043/media/na0043/misc/DATA/StereoDatasets/stereo2012/data_stereo_flow/training/colored_0

./sgm ${TRAINING} 7 84

./stereo_test ${LEFT} ${RIGHT} -dispdir=${DISP}

