#!/bin/bash

#ROS functions

function buildpy() {
    colcon build --symlink-install
    source /home/macbot_user/.bashrc
}
