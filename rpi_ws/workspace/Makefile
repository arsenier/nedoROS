.PHONY: build

build:
	colcon build --symlink-install

lidar:
	ros2 launch lslidar_driver lslidar_launch.py

zmrobo:
	ros2 launch base_serial.launch.py