/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/routing/routing_component.h"

#include <utility>

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/routing/common/routing_gflags.h"

DECLARE_string(flagfile);

namespace apollo {
namespace routing {

bool RoutingComponent::Init() {
  // 1.加载配置文件
  RoutingConfig routing_conf;
  // 从 DAG 配置中读取 RoutingConfig proto，其中包含 TopicConfig（话题名称）
  ACHECK(cyber::ComponentBase::GetProtoConfig(&routing_conf))
      << "Unable to load routing conf file: "
      << cyber::ComponentBase::ConfigFilePath();

  AINFO << "Config file: " << cyber::ComponentBase::ConfigFilePath()
        << " is loaded.";
  
  apollo::cyber::proto::RoleAttributes attr;
  attr.set_channel_name(routing_conf.topic_config().routing_response_topic());
  auto qos = attr.mutable_qos_profile();
  qos->set_history(apollo::cyber::proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
  qos->set_reliability(
      apollo::cyber::proto::QosReliabilityPolicy::RELIABILITY_RELIABLE);
  qos->set_durability(
      apollo::cyber::proto::QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL);
  // 2. 创建实时响应 Writer
  response_writer_ = node_->CreateWriter<routing::RoutingResponse>(attr);

  apollo::cyber::proto::RoleAttributes attr_history;
  attr_history.set_channel_name(
      routing_conf.topic_config().routing_response_history_topic());
  auto qos_history = attr_history.mutable_qos_profile();
  qos_history->set_history(
      apollo::cyber::proto::QosHistoryPolicy::HISTORY_KEEP_LAST);  // 只保留最后一条
  qos_history->set_reliability(
      apollo::cyber::proto::QosReliabilityPolicy::RELIABILITY_RELIABLE); // 可靠传输
  qos_history->set_durability(
      apollo::cyber::proto::QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL); // 晚加入的订阅者也能收到最后一条消息（类似 latched topic）
  // 3. 创建历史响应 Writer
  response_history_writer_ =
      node_->CreateWriter<routing::RoutingResponse>(attr_history);
  std::weak_ptr<RoutingComponent> self =
      std::dynamic_pointer_cast<RoutingComponent>(shared_from_this());
  // 4. 启动定时器
  // 设计目的：让其他模块（如 Planning、HMI）即使错过了初始路由响应，也能在 1 秒内收到最新的路由结果
  timer_.reset(new ::apollo::cyber::Timer(
      FLAGS_routing_response_history_interval_ms,    // 默认 1000ms
      [self, this]() {
        auto ptr = self.lock();
        if (ptr) {
          std::lock_guard<std::mutex> guard(this->mutex_);
          if (this->response_ != nullptr) {
            auto timestamp = apollo::cyber::Clock::NowInSeconds();
            response_->mutable_header()->set_timestamp_sec(timestamp);
            this->response_history_writer_->Write(*response_);
          }
        }
      },
      false));
  timer_->Start();
  // 5.初始核心路由
  // 加载拓扑地图并创建 Navigator，同时检查 Navigator 是否就绪
  return routing_.Init().ok() && routing_.Start().ok();
}

bool RoutingComponent::Proc(
    const std::shared_ptr<routing::RoutingRequest>& request) {
  auto response = std::make_shared<routing::RoutingResponse>();
  if (!routing_.Process(request, response.get())) {
    return false;
  }
  common::util::FillHeader(node_->Name(), response.get());
  response_writer_->Write(response);
  {
    std::lock_guard<std::mutex> guard(mutex_);
    response_ = std::move(response);
  }
  return true;
}

}  // namespace routing
}  // namespace apollo
