// *************************************************************************
//
// Copyright (c) 2021 Andrei Gramakov. All rights reserved.
//
// This file is licensed under the terms of the MIT license.
// For a copy, see: https://opensource.org/licenses/MIT
//
// site:    https://agramakov.me
// e-mail:  mail@agramakov.me
//
// *************************************************************************

#include "ros/ros.h"
#include "zakhar_msgs/EmotionCoreDataDescriptor.h"
#include "zakhar_msgs/EmotionCoreWrite.h"
#include "zakhar_msgs/EmotionParams.h"


bool EmotionCoreWrite_handler(zakhar_msgs::EmotionCoreWrite::Request &req,
                              zakhar_msgs::EmotionCoreWrite::Response &res) {
    res.result = "done";
    return true;
}

bool EmotionCoreDataDescriptor_handler(zakhar_msgs::EmotionCoreDataDescriptor::Request &req,
                                       zakhar_msgs::EmotionCoreDataDescriptor::Response &res) {
    res.result = "done";
    return true;
}

bool FillMsg(zakhar_msgs::EmotionParams &msg){
    msg.params_json = "{}";
    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "zakhar_emotion_core");
    ros::NodeHandle n;

    ros::ServiceServer emotion_core_write_srv           = n.advertiseService("EmotionCoreWrite", EmotionCoreWrite_handler);
    ros::ServiceServer emotion_core_data_descriptor_srv = n.advertiseService("EmotionCoreDataDescriptor.", EmotionCoreDataDescriptor_handler);
    ros::Publisher emotion_core_emotion_params_pub      = n.advertise<zakhar_msgs::EmotionParams>("/EmotionParams", 10);
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        zakhar_msgs::EmotionParams msg;
        FillMsg(msg);
        ROS_INFO("%s", msg.params_json);
        emotion_core_emotion_params_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return -1;
}
