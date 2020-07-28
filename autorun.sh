#! /usr/bin/env bash
#
# This script runs on startup.
# There is a line at the bottom of .xsessionrc that executes this script.
#
# Jordan Ford
# 

# What directory is this script located in?
DIR="$( cd "$(dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# Go to the directory containing our binaries.
cd $DIR/build

# Rebuild our binaries.
cmake .. -DCMAKE_BUILD_TYPE=Release
make

# Run the teleop binary on startup.
terminator -e './teleop && bash || bash'
