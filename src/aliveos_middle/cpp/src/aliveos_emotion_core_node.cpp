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
#include <string>
#include "aliveos_emotion_core_node.hpp"
#include "aliveos_msgs/EmotionCoreDataDescriptor.h"
#include "aliveos_msgs/EmotionCoreWrite.h"
#include "aliveos_msgs/EmotionParams.h"
#include "json.hpp"
#include "ros/ros.h"


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

    string write_srv_name;
    string data_descriptor_srv_name;
    string eparams_topic_name;

    ros::param::get("SRV_ECORE_W", write_srv_name);
    ros::param::get("SRV_ECORE_DDSC", data_descriptor_srv_name);
    ros::param::get("TOPIC_EPARAM", eparams_topic_name);

    emotion_core_write_srv           = n.advertiseService(write_srv_name, handler_EmotionCoreWrite_handler);
    emotion_core_data_descriptor_srv = n.advertiseService(data_descriptor_srv_name, handler_EmotionCoreDataDescriptor);
    emotion_core_emotion_params_pub  = n.advertise<aliveos_msgs::EmotionParams>(eparams_topic_name, 10);
}

ZakharEmotionCoreNode *ZakharEmotionCoreNode::GetInstance() {
    if (instance == nullptr) {
        instance = new ZakharEmotionCoreNode();
    }
    return instance;
}

bool ZakharEmotionCoreNode::handler_EmotionCoreWrite_handler(aliveos_msgs::EmotionCoreWrite::Request &req,
                                                             aliveos_msgs::EmotionCoreWrite::Response &res) {
    ROS_DEBUG("%s got: {sensor_name:%s, value:%d, temp_param_name:%s, temp_val_per_sec:%d}",
              __func__, req.sensor_name.c_str(), req.value, req.temp_param_name.c_str(), req.temp_val_per_sec);
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
    return true;
}

bool ZakharEmotionCoreNode::handler_EmotionCoreDataDescriptor(aliveos_msgs::EmotionCoreDataDescriptor::Request &req,
                                                              aliveos_msgs::EmotionCoreDataDescriptor::Response &res) {
    ROS_INFO("%s", __func__);
    InDataDescriptorStruct_t dsc = {
            .sensor_name = req.sensor_name,
            .val_min     = (int) req.val_min,
            .val_max     = (int) req.val_max};

    ROS_DEBUG("%s: %s weights_json - %s", __func__, req.sensor_name.c_str(), req.weights_json.c_str());

    auto weights = json::parse(req.weights_json);
    for (auto i = weights.begin(); i != weights.end(); i++) {
        json w_json(*i);  //element
        SensorDataWeightStruct_t w = {
                .core_param_name = w_json["parameter"].get<std::string>(),
                .weight          = w_json["value"].get<float>()};
        dsc.weights.push_back(w);
    }
    core->AddSensorDataDescriptor(dsc);
    res.result = "done";
    return true;
}

void ZakharEmotionCoreNode::FillMsg() {
    const CoreParamsMap_t *params = core->GetParams();
    ROS_DEBUG("\nTime: %d ms", core->time_ms);
    string report;
    for (auto i = params->begin(); i != params->end(); i++) {
        report += i->first + ": " + to_string(i->second) + "\n";
        current_param_json[i->first] = i->second;
    }
    ROS_DEBUG(report.c_str());
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
