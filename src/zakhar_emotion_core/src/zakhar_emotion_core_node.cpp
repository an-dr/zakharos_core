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

#include <chrono>
#include "json.hpp"
#include "ros/ros.h"
#include "zakhar_emotion_core_node.hpp"
#include "zakhar_msgs/EmotionCoreDataDescriptor.h"
#include "zakhar_msgs/EmotionCoreWrite.h"
#include "zakhar_msgs/EmotionParams.h"


using namespace std;
using namespace std::chrono;
using json = nlohmann::json;

ZakharEmotionCoreNode *ZakharEmotionCoreNode::instance = nullptr;

AnimalEmotionCore *ZakharEmotionCoreNode::core = nullptr;

//TODO:move to utils or smth
static long millis() {
    auto ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    return ms.count();
}

ZakharEmotionCoreNode::ZakharEmotionCoreNode() : loop_rate(1) {
    core                             = new AnimalEmotionCore();
    emotion_core_write_srv           = n.advertiseService("EmotionCoreWrite", handler_EmotionCoreWrite_handler);
    emotion_core_data_descriptor_srv = n.advertiseService("EmotionCoreDataDescriptor", handler_EmotionCoreDataDescriptor);
    emotion_core_emotion_params_pub  = n.advertise<zakhar_msgs::EmotionParams>("/EmotionParams", 10);
}

ZakharEmotionCoreNode *ZakharEmotionCoreNode::GetInstance() {
    if (instance == nullptr) {
        instance = new ZakharEmotionCoreNode();
    }
    return instance;
}

bool ZakharEmotionCoreNode::handler_EmotionCoreWrite_handler(zakhar_msgs::EmotionCoreWrite::Request &req,
                                                             zakhar_msgs::EmotionCoreWrite::Response &res) {
    ROS_DEBUG("%s\r\n", __func__);
    res.result = "done";
    if (req.temp_val_per_sec > 0) {
        TemporaryCoreImpact_t d = {
                .change_per_sec = (float) req.temp_val_per_sec,
                .param_name     = req.temp_param_name,
                .delta_value    = (float) req.value};
        core->WriteTempImpact(d);
    } else {
        SensorDataStruct_t s = {
                .sensor_name = req.sensor_name,
                .value       = (float) req.value};
        core->WriteSensorData(s);
    }
    ROS_DEBUG("%s: done\r\n", __func__);
    return true;
}

bool ZakharEmotionCoreNode::handler_EmotionCoreDataDescriptor(zakhar_msgs::EmotionCoreDataDescriptor::Request &req,
                                                              zakhar_msgs::EmotionCoreDataDescriptor::Response &res) {
    ROS_DEBUG("%s\r\n", __func__);
    InDataDescriptorStruct_t dsc = {
            .sensor_name = req.sensor_name,
            .val_min     = (int) req.val_min,
            .val_max     = (int) req.val_max};

    ROS_DEBUG("%s: Got weights_json - %s\r\n", __func__, req.weights_json.c_str());

    auto weights = json::parse(req.weights_json);
    for (auto i = weights.begin(); i != weights.end(); i++) {
        json w_json(*i);  //element
        SensorDataWeightStruct_t w = {
                .core_param_name = i.key(),
                .weight          = i.value().get<float>()};
        dsc.weights.push_back(w);
    }
    core->AddSensorDataDescriptor(dsc);
    res.result = "done";
    ROS_DEBUG("%s: done\r\n", __func__);
    return true;
}

void ZakharEmotionCoreNode::FillMsg() {
    const CoreParamsMap_t *params = core->GetParams();
    ROS_WARN("Time: %d ms", core->time_ms);
    for (auto i = params->begin(); i != params->end(); i++) {
        ROS_WARN("Emotion core - %s:%f", i->first.c_str(), i->second);
        current_param_json[i->first] = i->second;
    }
    current_msg.params_json = current_param_json.dump();
}


void ZakharEmotionCoreNode::Start() {
    long t_start;
    int t_delta;
    while (ros::ok()) {
        t_start = millis();
        FillMsg();
        emotion_core_emotion_params_pub.publish(current_msg);
        ros::spinOnce();
        loop_rate.sleep();

        t_delta = (int) (millis() - t_start);
        core->WriteTime(t_delta);
    }
}
