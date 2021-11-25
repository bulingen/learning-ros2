#!/bin/bash

# this file should always be sourced when running ros2
source $(echo $PWD)/install/setup.bash || echo -e "\nHave you run 'colcon build'?"

# source python virtual env to isolate dependencies
source ./venv/bin/activate

# add site-packages from python venv to PYTHONPATH so that ros sees them
PYTHON_VERSION=$(python -c"import sys; print(str(sys.version_info.major) + '.' + str(sys.version_info.minor))")
LOCAL_PYTHON_PACKAGES="./venv/lib/python$PYTHON_VERSION/site-packages"
export PYTHONPATH=$LOCAL_PYTHON_PACKAGES:$PYTHONPATH
