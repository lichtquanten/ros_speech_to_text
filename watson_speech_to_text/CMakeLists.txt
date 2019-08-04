cmake_minimum_required(VERSION 2.8.3)
project(watson_speech_to_text)

find_package(catkin REQUIRED)

catkin_package(CATKIN_DEPENDS
    audio_io_msgs
    rospy
    message_runtime
    speech_to_text_msgs
    std_msgs
)
