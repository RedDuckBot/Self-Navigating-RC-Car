#!/bin/bash

#ROS functions

function buildpy() {
    colcon build --symlink-install
    source /root/.bashrc
}
