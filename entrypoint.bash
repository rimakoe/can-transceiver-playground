#!/bin/bash

sudo chown -R $(whoami):$(whoami) /home/user/
python -m pip install --update pip
pip install -e ./tools/cantools
cd tools/can-transceiver-lib
mkdir build
cd build
cmake ..
sudo make install
cd ../../../
rosdep update
rosdep install -r --rosdistro iron --from-paths ros2_ws/src --ignore-src -y
cd ros2_ws
colcon build
bash
