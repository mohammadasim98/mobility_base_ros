#! /bin/bash

# Perform an SDK Install
#./sdk_install.bash
bash <(wget -q -O - https://bitbucket.org/DataspeedInc/mobility_base_ros/raw/master/mobility_base/scripts/sdk_install.bash)

# Update with apt-get
#sudo apt-get update && sudo apt-get upgrade && rosdep update

echo "SDK update: Done"

