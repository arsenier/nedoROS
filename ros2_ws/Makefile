SHELL := /bin/bash

all: colcon source run_cam

colcon:
	colcon build --packages-select charuco_calibrator

source:
	source install/setup.bash

run_charuco:
	ros2 launch charuco_calibrator start.launch.py

run_cam:
	ros2 run usb_cam usb_cam_node_exe --ros-args --params-file ./src/usb_cam/params.yaml

run_cam_old:
	ros2 run usb_cam_publisher camera_node \
  --ros-args \
  -p device:=/dev/video4 -p width:=1920 -p height:=1080 -p fps:=30 \
  -p scale:=0.5 -p mono:=false \
  -p publish_raw:=true -p publish_compressed:=true \
  -p raw_every_n:=3 -p jpeg_quality:=80 -p target_fps:=30.0

run_charuco_decoder:
	cd src
	colcon build
	source install/setup.bash
	ros2 launch charuco_detector hand_eye_calibration.launch.py 