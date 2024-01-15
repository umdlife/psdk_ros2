#!/bin/bash

apt update
apt install -y --no-install-recommends python3-pip dpkg-dev debhelper dh-python libopus-dev ffmpeg libavcodec-dev libavformat-dev libavfilter-dev
ln -snf /usr/lib/x86_64-linux-gnu/libopus.a /usr/local/lib
pip3 install rosdep bloom
rosdep init
mv psdk_ros2/psdk/debian/50-my-packages.list /etc/ros/rosdep/sources.list.d
mv psdk_ros2/psdk/debian/rosdep.yaml /

# store the current dir
CUR_DIR=$(pwd)

# Update ROS deps
rosdep update

rosdep keys --from-paths . --ignore-src --rosdistro humble | \
  xargs rosdep resolve --rosdistro humble | \
  awk '/#apt/{getline; print}' > ./rosdep_requirements.txt
apt install -y --no-install-recommends $(cat ./rosdep_requirements.txt)

PACKAGE_LIST=(
  psdk_ros2/psdk_interfaces \
  psdk_ros2/psdk_wrapper
)

for PACKAGE in ${PACKAGE_LIST[@]}; do
    echo ""
    echo "Creating debian for $PACKAGE..."

    # We have to go to the ROS package parent directory
    cd $PACKAGE;
    bloom-generate rosdebian --ros-distro humble
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
