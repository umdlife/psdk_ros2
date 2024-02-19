#!/bin/bash
locale  # check for UTF-8

apt update && apt install locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

apt install software-properties-common
add-apt-repository universe

apt update && apt install curl -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

apt update
apt install -y --no-install-recommends python3-pip python3-bloom python3-catkin-pkg dpkg-dev debhelper dh-python nlohmann-json3-dev
pip3 install rosdep
rosdep init
cp psdk_ros2/debian/50-my-packages.list /etc/ros/rosdep/sources.list.d
cp psdk_ros2/debian/rosdep.yaml /

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
