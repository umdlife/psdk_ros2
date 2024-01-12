#!/bin/bash

sudo apt update && sudo apt install -y python3-pip dpkg-dev debhelper dh-python libopus-dev ffmpeg libavcodec-dev libavformat-dev libavfilter-dev
sudo ln -snf /usr/lib/x86_64-linux-gnu/libopus.a /usr/local/lib
sudo pip3 install rosdep bloom
sudo rosdep init
sudo mv psdk_ros2/psdk/debian/50-my-packages.list /etc/ros/rosdep/sources.list.d
sudo mv psdk_ros2/psdk/debian/rosdep.yaml /

# store the current dir
CUR_DIR=$(pwd)

# Update ROS deps
sudo rosdep update

PACKAGE_LIST=(
            psdk_ros2/psdk_interfaces \
            psdk_ros2/psdk_wrapper
)

for PACKAGE in ${PACKAGE_LIST[@]}; do
    echo ""
    echo "Creating debian for $PACKAGE..."

    # We have to go to the ROS package parent directory
    cd $PACKAGE;
    sudo bloom-generate rosdebian --ros-distro humble
    ls debian
    debian/rules "binary --parallel --dpkg-shlibdeps-params=--ignore-missing-info"

    cd ..
    DEB_FILE=$(find *.deb);
    if ! [[ $? -eq 0 ]]; then 
        exit 1
    fi
    dpkg -i $DEB_FILE
    rm *.deb *.ddeb
    cd $CUR_DIR

done

echo "Complete!"
