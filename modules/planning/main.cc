/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/planning/planning_node.h"

#include "gflags/gflags.h"
#include "modules/common/log.h"
#include "third_party/ros/include/ros/ros.h"

int main(int argc, char **argv) {
  // 初始化Google日志库。argv[0]通常是程序名称，这一行会为程序设置日志输出功能
  google::InitGoogleLogging(argv[0]);
  // 解析命令行传入的参数，并将其存储在argc和argv中
  google::ParseCommandLineFlags(&argc, &argv, true);

  ros::init(argc, argv, "planning");
  // 规划模块核心类，负责执行规划任务
  ::apollo::planning::PlanningNode planning_node;
  // 启用规划节点
  planning_node.Run();

  return 0;
}
