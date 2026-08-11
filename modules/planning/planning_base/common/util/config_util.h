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

/**
 * @file config_util.h
 * @brief 配置工具类头文件
 *
 * 功能说明：
 * 提供规划模块配置文件的加载、合并和覆盖功能
 * 支持默认配置与用户自定义配置的灵活组合
 */

#pragma once

#include <algorithm>
#include <string>

#include "cyber/common/file.h"

namespace apollo {
/**
 * @brief Apollo最外层命名空间
 *
 * C++语法说明：
 * - namespace：用于逻辑分组相关代码，避免命名冲突
 * - apollo是Apollo自动驾驶平台的最外层命名空间
 */
namespace planning {

/**
 * @class ConfigUtil
 * @brief 配置工具类
 *
 * 功能说明：
 * 提供静态方法用于处理配置文件的加载和合并
 * 支持默认配置与用户自定义配置的两种组合方式
 *
 * 设计模式：
 * - 工具类模式：所有方法都是静态的，无需实例化
 * - 模板方法：使用模板支持不同类型的Protobuf配置
 *
 * C++语法说明：
 * - class：类声明关键字
 * - public：公有成员访问限定符
 * - static：静态方法，可通过类名直接调用，无需创建对象
 *
 * 使用示例：
 * @code
 *   PlanningConfig config;
 *   ConfigUtil::LoadMergedConfig<T>(default_path, user_path, &config);
 * @endcode
 */
class ConfigUtil {
 public:
  /**
   * @brief 将名称转换为路径格式
   *
   * @param name 要转换的名称
   * @return std::string 转换后的名称（全小写，用于路径）
   *
   * 功能说明：
   * 将传入的名称转换为路径格式
   * 主要要求是所有字符必须为小写
   *
   * 算法流程：
   * 1. 遍历名称中的每个字符
   * 2. 使用std::transform将所有字符转换为小写
   *
   * C++语法说明：
   * - static：静态成员函数
   *   - 可以通过 ConfigUtil::TransformToPathName() 直接调用
   *   - 不需要创建 ConfigUtil 对象实例
   * - const std::string& name：
   *   - const：输入参数不可被修改
   *   - std::string&：字符串引用，避免拷贝
   * - std::string：标准库字符串类型
   *
   * 示例：
   * @code
   *   std::string name = "LaneFollowConfig";
   *   std::string path_name = ConfigUtil::TransformToPathName(name);
   *   // path_name = "lanefollowconfig"
   * @endcode
   */
  static std::string TransformToPathName(const std::string& name);

  /**
   * @brief 获取完整的规划类名称
   *
   * @param class_name 不带命名空间的类名
   * @return std::string 带有完整命名空间的类名
   *
   * 功能说明：
   * 将类名与规划模块的命名空间组合
   * 生成完整的类名字符串，用于动态类创建
   *
   * 算法流程：
   * 在class_name前拼接 "apollo::planning::" 命名空间前缀
   *
   * C++语法说明：
   * - "apollo::planning::" + class_name：
   *   字符串拼接操作
   *   :: 是命名空间作用域运算符
   *
   * 示例：
   * @code
   *   std::string full_name = ConfigUtil::GetFullPlanningClassName("PublicRoadPlanner");
   *   // full_name = "apollo::planning::PublicRoadPlanner"
   * @endcode
   */
  static std::string GetFullPlanningClassName(const std::string& class_name);

  /**
   * @brief 加载并合并配置
   *
   * @tparam T Protobuf消息类型模板参数
   * @param default_config_path 默认配置文件路径
   *        如果config_path中的参数未定义，则使用此文件中的默认值
   * @param config_path 用户自定义配置文件路径
   * @param config 输出参数，加载后的配置数据
   * @return bool 加载是否成功
   *
   * 功能说明：
   * 先加载默认配置，然后用用户自定义配置合并覆盖
   * 用户的配置会与默认配置合并，只覆盖用户指定的部分
   *
   * 算法流程：
   * 1. 调用 LoadConfig<T> 加载默认配置到config
   * 2. 调用 LoadConfig<T> 加载用户配置到 specific_config
   * 3. 调用 config->MergeFrom(specific_config) 合并配置
   *
   * C++语法说明：
   * - template <typename T>：
   *   函数模板，T是泛型类型参数
   *   允许函数接受任何Protobuf消息类型
   * - T* config：指针参数，用于输出结果
   * - CHECK_NOTNULL(config)：
   *   宏定义，用于运行时断言
   *   检查config指针是否为nullptr
   *   如果为nullptr，程序会报错并终止
   * - apollo::cyber::common::LoadConfig<T>：
   *   Apollo Cyber RT框架的配置加载函数
   *   模板参数T指定要加载的Protobuf类型
   * - AERROR：
   *   Apollo的错误日志宏
   *   用于记录错误级别的日志
   * - AWARN：
   *   Apollo的警告日志宏
   *   用于记录警告级别的日志
   * - config->MergeFrom(spcific_config)：
   *   Protobuf消息的合并方法
   *   将specific_config的值合并到config中
   *   只有specific_config中设置的字段才会覆盖
   *
   * 示例：
   * @code
   *   PlanningConfig default_config;
   *   PlanningConfig user_config;
   *   if (ConfigUtil::LoadMergedConfig(
   *       "/default/planning_config.pb.txt",
   *       "/user/planning_config.pb.txt",
   *       &default_config)) {
   *     // 成功加载，default_config包含合并后的配置
   *   }
   * @endcode
   */
  template <typename T>
  static bool LoadMergedConfig(const std::string& default_config_path,
                               const std::string& config_path, T* config);

  /**
   * @brief 加载配置并优先使用用户定义配置
   *
   * @tparam T Protobuf消息类型模板参数
   * @param default_config_path 默认配置文件路径
   *        如果用户配置不存在，则使用此文件
   * @param config_path 用户自定义配置文件路径
   * @param config 输出参数，加载后的配置数据
   * @return bool 加载是否成功
   *
   * 功能说明：
   * 优先尝试加载用户自定义配置
   * 如果用户配置不存在或加载失败，则使用默认配置
   *
   * 算法流程：
   * 1. 首先尝试从config_path加载用户配置
   * 2. 如果加载成功，直接返回
   * 3. 如果加载失败，则从default_config_path加载默认配置
   *
   * C++语法说明：
   * - if (!apollo::cyber::common::GetProtoFromFile(config_path, config))：
   *   首先尝试加载用户配置
   *   GetProtoFromFile与LoadConfig的区别：
   *   - LoadConfig：使用Protobuf的文本格式
   *   - GetProtoFromFile：使用Protobuf的二进制或文本格式
   * - return true：
   *   用户配置加载成功，直接返回
   * - if (!apollo::cyber::common::GetProtoFromFile(default_config_path, config))：
   *   加载默认配置
   *   使用AERROR记录错误日志
   *
   * 与LoadMergedConfig的区别：
   * - LoadMergedConfig：合并两个配置，用户的值覆盖默认值
   * - LoadOverridedConfig：二选一，要么用用户配置，要么用默认配置
   *
   * 示例：
   * @code
   *   PlanningConfig config;
   *   if (ConfigUtil::LoadOverridedConfig(
   *       "/default/planning_config.pb.txt",
   *       "/user/planning_config.pb.txt",
   *       &config)) {
   *     // 成功加载，config包含用户配置或默认配置
   *   }
   * @endcode
   */
  template <typename T>
  static bool LoadOverridedConfig(const std::string& default_config_path,
                                 const std::string& config_path, T* config);
};

/**
 * @brief LoadMergedConfig模板函数实现
 *
 * @tparam T Protobuf消息类型
 * @param default_config_path 默认配置路径
 * @param config_path 用户配置路径
 * @param config 输出参数
 * @return bool 加载是否成功
 *
 * 实现详解：
 * 1. 使用CHECK_NOTNULL确保config指针有效
 * 2. 加载默认配置
 * 3. 尝试加载用户配置
 * 4. 合并两个配置
 */
template <typename T>
bool ConfigUtil::LoadMergedConfig(const std::string& default_config_path,
                                  const std::string& config_path, T* config) {
  // 检查config指针是否为nullptr
  // CHECK_NOTNULL是Apollo框架的断言宏
  // 如果config为nullptr，程序会报错并终止执行
  CHECK_NOTNULL(config);

  // 步骤1：加载默认配置
  // LoadConfig<T>是Apollo Cyber RT框架的模板函数
  // 用于从配置文件加载Protobuf消息
  // 如果加载失败，记录错误日志但继续执行（使用默认值）
  if (!apollo::cyber::common::LoadConfig<T>(default_config_path, config)) {
    // AERROR是Apollo的错误日志宏
    // << 运算符用于拼接日志内容，类似cout
    AERROR << "Failed to load default config file:" << default_config_path;
  }

  // 步骤2：加载用户自定义配置
  // specific_config存储用户自定义的配置值
  T spcific_config;

  // 尝试从用户配置路径加载配置
  // 如果加载失败，记录警告日志并使用默认配置
  if (!apollo::cyber::common::LoadConfig<T>(config_path, &spcific_config)) {
    // AWARN是Apollo的警告日志宏
    // 提示用户配置加载失败，但不影响程序运行
    AWARN << "can not load user defined config file[" << config_path
          << "], use default config instead";
    return true;  // 返回true，因为使用默认配置也是可接受的结果
  }

  // 步骤3：合并配置
  // MergeFrom是Protobuf消息的成员函数
  // 将specific_config的值合并到config中
  // 只有specific_config中明确设置的值才会覆盖默认值
  // 未设置的字段保持默认值
  config->MergeFrom(spcific_config);
  return true;
}

/**
 * @brief LoadOverridedConfig模板函数实现
 *
 * @tparam T Protobuf消息类型
 * @param default_config_path 默认配置路径
 * @param config_path 用户配置路径
 * @param config 输出参数
 * @return bool 加载是否成功
 *
 * 实现详解：
 * 1. 优先尝试加载用户配置
 * 2. 如果用户配置存在，直接使用
 * 3. 如果用户配置不存在，加载默认配置
 */
template <typename T>
bool ConfigUtil::LoadOverridedConfig(const std::string& default_config_path,
                                    const std::string& config_path,
                                    T* config) {
  // 检查config指针是否为nullptr
  CHECK_NOTNULL(config);

  // 步骤1：优先尝试加载用户配置
  // GetProtoFromFile从指定路径加载Protobuf消息
  // 如果加载成功，config会被用户配置填充
  if (apollo::cyber::common::GetProtoFromFile(config_path, config)) {
    return true;  // 用户配置加载成功，直接返回
  }

  // 步骤2：用户配置不存在或不有效，加载默认配置
  if (!apollo::cyber::common::GetProtoFromFile(default_config_path, config)) {
    // 默认配置加载也失败，记录错误并返回false
    AERROR << "Failed to load config file using the default config:"
           << default_config_path;
    return false;  // 返回false表示加载失败
  }

  return true;  // 成功使用默认配置
}

}  // namespace planning
}  // namespace apollo
