# ROS2 KUKA Robot Descriptions Fork

This fork is identical to the original `kuka_robot_descriptions` repository, except that it has some minor changes to work with the `kuka_kontrol` package.

The original KUKA Robot Descriptions GitHub Repository can be found at the link below.<br>
[KUKA Robot Descriptions GitHub](https://github.com/kroshu/kuka_robot_descriptions)<br>

#### Table of Contents
[Installation](#Installation)<br>

## Installation
It is necessary to install the corresponding fork of the `kuka_drivers` package alongside this forked repository. The original and forked KUKA Drivers GitHub Repositories can be found at the links below.<br>
[Original KUKA Drivers GitHub](https://github.com/kroshu/kuka_drivers)<br>
[Forked KUKA Drivers GitHub](https://github.com/thinclab/kuka_drivers/tree/jazzy)<br>

It is recommended to have a separate workspace for this robot descriptions repository and the drivers to simplify the build process; use the commands below to create the `/kuka_ws` and to clone the forks into the `/src` directory.

    mkdir -p ~/kuka_ws/src
    cd ~/kuka_ws/src
    git clone -b jazzy https://github.com/thinclab/kuka_drivers.git
    git clone -b jazzy https://github.com/thinclab/kuka_robot_descriptions.git

After you have cloned the fork, go to `~/kuka_ws` and resolve dependencies.

    cd ~/kuka_ws
    rosdep install --from-paths src --ignore-src -r -y

Then, build the package in the workspace using the command below.

    MAKEFLAGS=`getconf _NPROCESSORS_ONLN` colcon build --continue-on-error --parallel-workers 4 --symlink-install --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release

Make sure to source your workspace if you plan to utilize these repositories.

    source ~/kuka_ws/install/setup.bash

 You can add the `source` command to your `~/.bashrc` using the commands below so that this happens automatically when you open a new terminal.

    echo "source ~/cam_ws/install/setup.bash" >> ~/.bashrc
