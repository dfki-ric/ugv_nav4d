#!/bin/bash

if dpkg -s doxygen xdot qttools5-dev qttools5-dev-tools libopenscenegraph-dev libeigen3-dev libcgal-dev libpcl-dev libboost-filesystem-dev libboost-serialization-dev libboost-system-dev &>/dev/null; then
  echo "All os_dependencies are installed alread. To update them run:"
  echo "# apt-get update && apt-get upgrade"
else
  sudo apt-get update
  sudo apt-get install -y doxygen
  sudo apt-get install -y xdot
  sudo apt-get install -y qttools5-dev qttools5-dev-tools
  sudo apt-get install -y libopenscenegraph-dev libeigen3-dev libcgal-dev libpcl-dev
  sudo apt-get install -y libboost-filesystem-dev libboost-serialization-dev libboost-system-dev
fi
