#!/usr/bin/env bash

function ros_louie() {
    export ROS_MASTER_URI=http://10.8.33.42:11311
    export ROS_IP=$(hostname -I)

    env | grep ROS_MASTER_URI
    env | grep ROS_IP
}

function ros_dewey() {
    export ROS_MASTER_URI=http://10.8.33.41:11311
    export ROS_IP=$(hostname -I)

    env | grep ROS_MASTER_URI
    env | grep ROS_IP
}

function ros_huey() {
    export ROS_MASTER_URI=http://10.8.33.136:11311
    export ROS_IP=$(hostname -I)

    env | grep ROS_MASTER_URI
    env | grep ROS_IP
}

function ros_local() {
    export ROS_MASTER_URI=http://localhost:11311
    export ROS_IP=$(hostname -I)

    env | grep ROS_MASTER_URI
    env | grep ROS_IP
}
