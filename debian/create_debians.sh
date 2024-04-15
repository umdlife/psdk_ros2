#!/bin/bash
apt update
apt install -y --no-install-recommends python3-bloom python3-catkin-pkg dpkg-dev debhelper dh-python nlohmann-json3-dev

# store the current dir
CUR_DIR=$(pwd)

# Update ROS deps
rosdep init
rosdep update

rosdep keys --from-paths . --ignore-src --rosdistro humble --os ubuntu:jammy | \
  xargs rosdep resolve --rosdistro humble --os ubuntu:jammy | \
  awk '/#apt/{getline; print}' > ./rosdep_requirements.txt
apt install -y --no-install-recommends $(cat ./rosdep_requirements.txt)

source /opt/ros/humble/setup.bash

PACKAGE_LIST=(
  psdk_ros2/psdk_interfaces \
  psdk_ros2/psdk_wrapper
)

for PACKAGE in ${PACKAGE_LIST[@]}; do
    echo ""
    echo "Creating debian for $PACKAGE..."

    # We have to go to the ROS package parent directory
    cd $PACKAGE;
    bloom-generate rosdebian --ros-distro humble --os-name ubuntu --os-version jammy
    debian/rules "binary --parallel --dpkg-shlibdeps-params=--ignore-missing-info"

    cd ..
    DEB_FILE=$(find *.deb);
    if ! [[ $? -eq 0 ]]; then 
        exit 1
    fi
    dpkg -i $DEB_FILE
    cp $DEB_FILE $CUR_DIR
    rm *.deb *.ddeb
    cd $CUR_DIR

done

echo "Complete!"
