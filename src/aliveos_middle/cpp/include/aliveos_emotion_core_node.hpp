// *************************************************************************
//
// Copyright (c) 2021 Andrei Gramakov. All rights reserved.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
// site:    https://agramakov.me
// e-mail:  mail@agramakov.me
//
// *************************************************************************

#pragma once

#include "AnimalEmotionCore/AnimalEmotionCore.hpp"
#include "json.hpp"
#include "ros/ros.h"
#include "aliveos_msgs/EmotionCoreDataDescriptor.h"
#include "aliveos_msgs/EmotionCoreWrite.h"
#include "aliveos_msgs/EmotionParams.h"

using namespace std;
using json = nlohmann::json;


class ZakharEmotionCoreNode {
protected:
    ZakharEmotionCoreNode();
    static ZakharEmotionCoreNode *instance;
    static AnimalEmotionCore *core;

    ros::NodeHandle n;
    ros::ServiceServer emotion_core_write_srv;
    ros::ServiceServer emotion_core_data_descriptor_srv;
    ros::Publisher emotion_core_emotion_params_pub;
    ros::Rate loop_rate;

    aliveos_msgs::EmotionParams current_msg;
    json current_param_json;

    void FillMsg();

public:
    // Singletons should not be cloneable.
    ZakharEmotionCoreNode(ZakharEmotionCoreNode &other) = delete;

    // Singletons should not be assignable.
    void operator=(const ZakharEmotionCoreNode &) = delete;

    static ZakharEmotionCoreNode *GetInstance();

    static bool handler_EmotionCoreWrite_handler(aliveos_msgs::EmotionCoreWrite::Request &req,
                                                 aliveos_msgs::EmotionCoreWrite::Response &res);

    static bool handler_EmotionCoreDataDescriptor(aliveos_msgs::EmotionCoreDataDescriptor::Request &req,
                                                  aliveos_msgs::EmotionCoreDataDescriptor::Response &res);

    void Start();
};
