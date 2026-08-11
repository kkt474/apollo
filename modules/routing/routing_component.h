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

#pragma once

#include <memory>

#include "modules/routing/routing.h"

namespace apollo {
namespace routing {

class RoutingComponent final
    : public ::apollo::cyber::Component<routing::RoutingRequest> {
 public:
  RoutingComponent() = default;
  ~RoutingComponent() = default;

 public:
  bool Init() override;
  bool Proc(
      const std::shared_ptr<routing::RoutingRequest>& request) override;

 private:
// 发布实时路由响应到 /apollo/routing_response
  std::shared_ptr<::apollo::cyber::Writer<routing::RoutingResponse>>
      response_writer_ = nullptr;
// 发布历史路由响应到 /apollo/routing_response_history
  std::shared_ptr<::apollo::cyber::Writer<routing::RoutingResponse>>
      response_history_writer_ = nullptr;
// 核心路由逻辑对象（Navigator + 地图）
  Routing routing_;
// 缓存最近一次路由结果
  std::shared_ptr<routing::RoutingResponse> response_ = nullptr;
// 定时器，周期性重发历史响应
  std::unique_ptr<::apollo::cyber::Timer> timer_;
// 保护 response_ 的线程安全
  std::mutex mutex_;
};

CYBER_REGISTER_COMPONENT(RoutingComponent)

}  // namespace routing
}  // namespace apollo
