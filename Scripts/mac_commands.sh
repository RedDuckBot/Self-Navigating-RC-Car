#!/bin/bash

#ROS functions

function buildpy() {
    sudo colcon build --symlink-install
    source /home/macbot_user/.bashrc
}
