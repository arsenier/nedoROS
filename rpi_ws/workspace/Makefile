.PHONY: build

build:
	colcon build --symlink-install

lidar:
	. install/setup.sh && ros2 launch lslidar_driver lslidar_launch.py

# zmrobo:
# 	ros2 launch base_serial.launch.py

pi_driver:
	. install/setup.sh && ros2 run pi_driver pi_driver

all:
	. install/setup.sh && ros2 launch all_launch.py
