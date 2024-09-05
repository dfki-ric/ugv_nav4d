#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo
    echo "Please specify a prefix. If you want system wide installation use '/usr/local' as prefix. for a local install use e.g. './install'"
    echo "In case you use a local prefix, please source the generated env.sh script in this folder to set up the paths before building the main library"
    echo
    exit -1
fi

if [[ $1 != .* && $1 != /* ]]; then
    echo
    echo -e "\033[1;33mPlease specify a global path or a local path with '/' or './' \033[1;0m"
    echo
    exit -1
fi

#download and build (in subshell to return to original folder)

INSTALLPATH=$(realpath $1)

mkdir -p build
cd build
cmake -DCMAKE_INSTALL_PREFIX=${INSTALLPATH} ..
make install

if [[ $1 = .* ]]; then
    echo
    echo -e "\033[1;33m'source ${INSTALLPATH}/env.sh' in this folder before building the main library \033[1;0m"
    echo
fi
