#!/bin/bash

# clear existing virtual environments
deactivate
rm -rf venv

# build the ros environment
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

cd ros2/f1_PC
rm -r install
rm -r build 
colcon build

path=$(cd "$MY_PATH" && pwd)
source $path/install/setup.bash
echo "source $path/install/setup.bash" >> ~/.bashrc
cd ..
cd ..

echo "ROS2 environment have been built!"


# create python virtual environment
python3 -m venv venv
source venv/bin/activate


# install the external dependencies
cd external_dependencies/crazymocap
pip install -e .
cd ..
cd aimotion_f1tenth_utils
pip install -e .
cd ..
cd ..
echo "External dependencies have been installed"


# install the manager framework
pip install -e .
echo "Fleet manager have been installed!"