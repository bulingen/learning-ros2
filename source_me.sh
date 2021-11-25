#!/bin/bash

source $(echo $PWD)/install/setup.bash || echo -e "\nHave you run 'colcon build'?"
source ./venv/bin/activate
