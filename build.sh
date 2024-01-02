#!/bin/bash

###################################################################
#
#                          BUILD FUNCTIONS
#
###################################################################

cmake .
cmake --build . -j4
make install
mkdir -p /usr/local/include/wiringPiLite
cp -r inc/* /usr/local/include/wiringPiLite/