SHELL := /bin/bash

all: colcon launch_all #run_cam run_charuco_decoder run_apriltag_detector

colcon:
	colcon build --symlink-install --packages-select apriltag_pose_detector

launch_all:
	. install/setup.bash && \
	ros2 launch nedo_bringup all.launch.py

run_cam:
	ros2 run usb_cam usb_cam_node_exe --ros-args --params-file ./src/usb_cam/params.yaml

run_charuco_decoder:
	. install/setup.bash && \
	ros2 launch charuco_detector hand_eye_calibration.launch.py

run_apriltag_detector:
	. install/setup.bash && \
	ros2 run apriltag_pose_detector apriltag_pose_detector_node \
		--ros-args \
		-p image_topic:=/camera/image_raw \
		-p camera_info_topic:=/camera/camera_info \
		-p tag_size_m:=0.055 \
		-p tag_id:=-1 \
		-p apriltag_dictionary:=3 \
		-p publish_debug_image:=true