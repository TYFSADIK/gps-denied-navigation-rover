.PHONY: test ros
EIGEN ?= $(shell brew --prefix eigen 2>/dev/null)/include/eigen3
test:
	c++ -std=c++17 -O2 -Wall -Iros2_ws/src/gdn_rover/include -I$(EIGEN) -I/usr/include/eigen3 ros2_ws/src/gdn_rover/test/test_core.cpp -o /tmp/gdn_test && /tmp/gdn_test
	python3 -m unittest -q
ros:   # build the ROS 2 package in a Jazzy container (no ROS install needed on the Mac)
	docker run --rm -v $(CURDIR)/ros2_ws:/ws -w /ws ros:jazzy-ros-base bash -c "apt-get update -qq && apt-get install -y -qq libeigen3-dev ros-jazzy-nav-msgs >/dev/null && . /opt/ros/jazzy/setup.sh && colcon build && colcon test && colcon test-result --all"
