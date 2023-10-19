// Copyright (c) 2018 - for information on the respective copyright owner
// see the NOTICE file and/or the repository https://github.com/boschresearch/fmi_adapter.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cassert>

#include <exception>
#include <map>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "fmi_adapter/FMIAdapter.h"

#include <fmilib.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "fmi_adapter_node");
  ros::NodeHandle n("~");

  ROS_INFO("SET: fmuPath\n");
  std::string fmuPath;
  if (!n.getParam("fmu_path", fmuPath)) {
    ROS_ERROR("Parameter 'fmu_path' not specified!");
    throw std::runtime_error("Parameter 'fmu_path' not specified!");
  }

  double stepSizeAsDouble = 0.5;
  n.getParam("step_size", stepSizeAsDouble);
ROS_INFO("SET: step_size: %f\n",stepSizeAsDouble);

  ros::Duration stepSize(stepSizeAsDouble);
ROS_INFO("SET: ROS step: %f\n",stepSize.toSec());

  ROS_INFO("SET: adapter\n");
  fmi_adapter::FMIAdapter adapter(fmuPath, stepSize);
  ROS_INFO("END: adapter\n");
  
  const std::vector<fmi2_import_variable_t*> InitParam = adapter.getParameters();
  
  for(fmi2_import_variable_t* x : InitParam)
  {
//ROS_INFO("InitParam: '%s', '%f'", fmi2_import_get_variable_name(x), fmi2_import_get_real_variable_start(fmi2_import_get_variable_as_real(x)));
	if(fmi2_import_get_causality(x)==fmi2_causality_enu_parameter)
	{
	adapter.setInitialValue(fmi2_import_get_variable_name(x), fmi2_import_get_real_variable_start(fmi2_import_get_variable_as_real(x)));
	}
  }
  
//  for (const std::string & name : adapter.getParameterNames()) {
    //ROS_INFO("FMU has parameter '%s'", name.c_str());
//  }
  adapter.initializeFromROSParameters(n);

ROS_INFO("END: initializeFromROSParameters\n");
  
  std::map<std::string, ros::Subscriber> subscribers;
  for (const std::string& name : adapter.getInputVariableNames()) {
    std::string rosifiedName = fmi_adapter::FMIAdapter::rosifyName(name);
    ros::Subscriber subscriber =
        n.subscribe<std_msgs::Float64>(rosifiedName, 1000, [&adapter, name](const std_msgs::Float64::ConstPtr& msg) {
          std::string myName = name;
//ROS_INFO("Set: Sub:'%s', data:'%f'\n",name.c_str(),msg->data);
          adapter.setInputValue(myName, ros::Time::now(), msg->data);
        });
    subscribers[name] = subscriber;
  }
ROS_INFO("END: Create Subscriber\n");

  std::map<std::string, ros::Publisher> publishers;
  for (const std::string& name : adapter.getOutputVariableNames()) {
    std::string rosifiedName = fmi_adapter::FMIAdapter::rosifyName(name);
    publishers[name] = n.advertise<std_msgs::Float64>(rosifiedName, 1000);
  }

ROS_INFO("END: Create Publisher\n");
  
  adapter.exitInitializationMode(ros::Time::now());
  //adapter.exitInitializationMode(ros::Time(0.0));
ROS_INFO("END: exitInitializationMode\n");

  double updatePeriod = 1;  // Default is 0.01s
  n.getParam("update_period", updatePeriod);

  ros::Timer timer = n.createTimer(ros::Duration(updatePeriod), [&](const ros::TimerEvent& event) {
    if (adapter.getSimulationTime() < event.current_expected) {
ROS_INFO("Start: doStepsUntil\n");
//      adapter.doStepsUntil(event.current_expected);
      adapter.doStepsUntil(ros::Time::now());
//      adapter.doStep();
ROS_INFO("End: doStepsUntil\n");
    } else {
      ROS_INFO("Simulation time %f is greater than timer's time %f. Is your step size to large?",
               adapter.getSimulationTime().toSec(), event.current_expected.toSec());
    }
    for (const std::string& name : adapter.getOutputVariableNames()) {
      std_msgs::Float64 msg;
      msg.data = adapter.getOutputValue(name);
      publishers[name].publish(msg);
//ROS_INFO("Topic:'%s', data:'%f' \n",name.c_str(), msg.data);
    }
  });

  ros::spin();

  return 0;
}
