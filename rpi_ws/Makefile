.PHONY: vizanti vizanti_rws

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=
# export ROS_LOCALHOST_ONLY=1
# -e RMW_IMPLEMENTATION=$(RMW_IMPLEMENTATION)

vizanti:
	docker run --rm -it --net=host --ipc=host --pid=host --privileged --name vizanti-ros2 -e ROS_DOMAIN_ID=$(ROS_DOMAIN_ID) -e USE_RWS=false -v /dev/shm:/dev/shm vizanti:2.0

vizanti_build:
	cd vizanti && docker build -f docker/Dockerfile -t vizanti:2.0 . --build-arg ROS_VERSION=humble

# vizanti_rws:
# 	docker run --rm -it --net=host --name vizanti-ros2 -e ROS_DOMAIN_ID=$(ROS_DOMAIN_ID) -e RMW_IMPLEMENTATION=$(RMW_IMPLEMENTATION) -e USE_RWS=true -v /dev/shm:/dev/shm vizanti:2.0

vizanti_into:
	docker exec -it "vizanti-ros2" /bin/bash

zmrobo_run:
	bash docker/run.bash

zmrobo_into:
	bash docker/into.bash

zmrobo_stop:
	bash docker/stop.bash

zmrobo_build:
	cd docker && bash build.bash
