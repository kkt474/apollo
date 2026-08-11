/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
 * @file fallback_path.h
 * @brief 后备路径生成器头文件
 *
 * 功能说明：
 * 定义了后备路径生成器类的接口
 * 当主路径规划失败时，生成安全的fallback路径
 *
 * 应用场景：
 * - 主路径规划失败时的安全保障
 * - 开放空间规划中的后备轨迹
 * - 紧急情况下的安全路径生成
 *
 * 设计理念：
 * 1. 继承自PathGeneration基类
 * 2. 标准化三阶段处理：边界决定、路径优化、路径评估
 * 3. 使用插件机制动态加载
 *
 * 工作流程：
 * 1. Init() - 初始化，加载配置
 * 2. Process() - 执行路径生成主流程
 * 3. DecidePathBounds() - 决定路径边界
 * 4. OptimizePath() - 优化路径
 * 5. AssessPath() - 评估并选择最佳路径
 *
 * C++语法说明：
 * - class：类声明关键字
 * - public：公有继承
 * - virtual：虚函数，支持运行时多态
 * - override：重写基类虚函数
 * - std::shared_ptr：共享所有权智能指针
 * - std::string：标准库字符串
 * - std::vector：动态数组容器
 * - #pragma once：头文件保护
 */

/**
 * @brief 头文件保护指令
 *
 * C++语法说明：
 * - #pragma once：
 *   预处理指令，防止头文件被多次包含
 *   现代编译器支持，比#ifndef...#define...#endif更简洁
 *   优点：无需手动写保护宏，编译器自动处理
 *   注意：不是标准C++的一部分，但主流编译器都支持
 */
#pragma once

/**
 * @brief C++标准库头文件
 *
 * C++语法说明：
 * - #include <memory>：
 *   智能指针头文件
 *   提供std::shared_ptr、std::unique_ptr等智能指针
 *   智能指针可以自动管理内存，避免内存泄漏
 *
 * - #include <string>：
 *   标准库字符串类
 *   std::string是处理字符串的常用类型
 *   提供字符串拼接、查找、比较等功能
 *
 * - #include <vector>：
 *   动态数组容器
 *   std::vector是C++中最常用的容器之一
 *   支持随机访问和动态大小调整
 */
#include <memory>
#include <string>
#include <vector>

/**
 * @brief Protobuf配置头文件
 *
 * 功能说明：
 * - fallback_path.pb.h：
 *   Protobuf编译生成的代码
 *   定义了FallbackPathConfig配置结构
 *   包含后备路径生成器的所有配置参数
 *
 * C++语法说明：
 * - "modules/planning/tasks/fallback_path/proto/..."：
 *   使用引号表示自定义头文件路径
 *   protobuf生成的代码通常放在proto子目录
 */
#include "modules/planning/tasks/fallback_path/proto/fallback_path.pb.h"

/**
 * @brief Cyber RT插件管理器头文件
 *
 * 功能说明：
 * - cyber/plugin_manager/plugin_manager.h：
 *   Apollo Cyber RT的插件系统
 *   允许动态加载和创建Task实例
 *
 * C++语法说明：
 * - CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN：
 *   宏，用于注册插件到插件管理器
 */
#include "cyber/plugin_manager/plugin_manager.h"

/**
 * @brief 路径生成基类头文件
 *
 * 功能说明：
 * - path_generation.h：
 *   定义了PathGeneration类
 *   所有路径生成任务继承自此类
 *   提供路径生成的通用接口和流程
 *
 * C++语法说明：
 * - PathGeneration：
 *   路径生成基类
 *   FallbackPath继承自PathGeneration
 *   继承关系：FallbackPath -> PathGeneration -> Task -> Decider
 */
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"

/**
 * @namespace apollo::planning
 * @brief Apollo规划模块命名空间
 *
 * C++语法说明：
 * - namespace apollo：
 *   Apollo最外层命名空间
 *   所有Apollo代码都在此命名空间下
 * - namespace planning：
 *   规划子命名空间
 */
namespace apollo {
namespace planning {

/**
 * @class FallbackPath
 * @brief 后备路径生成器类
 *
 * 功能说明：
 * 当主路径规划失败时，生成安全的fallback路径
 * 继承自PathGeneration类，是Task的派生类
 *
 * 继承关系：
 * FallbackPath -> PathGeneration -> Task -> Decider
 *
 * 设计模式：
 * - 模板方法模式：基类定义算法骨架
 * - 策略模式：不同任务有不同行为
 * - 插件模式：动态加载
 *
 * C++语法说明：
 * - class FallbackPath :
 *   类声明，定义一个名为FallbackPath的类
 *
 * - public PathGeneration：
 *   公有继承自PathGeneration类
 *   public继承表示：
 *   - 基类的public成员仍是public
 *   - 基类的protected成员仍是protected
 *   - 基类的private成员不可直接访问
 */
class FallbackPath : public PathGeneration {
 public:
  /**
   * @brief 初始化后备路径生成器
   *
   * @param config_dir 配置文件目录
   * @param name 任务名称
   * @param injector 依赖注入器指针
   * @return bool 初始化是否成功
   *
   * 功能说明：
   * 初始化生成器的内部状态和配置
   *
   * 重写说明：
   * 该方法重写了基类PathGeneration的Init方法
   *
   * C++语法说明：
   * - bool Init(...)：
   *   返回布尔值表示成功/失败
   *
   * - const std::string& config_dir：
   *   常量引用，传入配置文件目录路径
   *   const保证配置不会被修改
   *   引用避免拷贝大型字符串
   *
   * - const std::string& name：
   *   常量引用，传入任务名称
   *
   * - const std::shared_ptr<DependencyInjector>& injector：
   *   共享指针的常量引用
   *   - shared_ptr：多个所有者共享对象
   *   - const引用：既不拷贝也不获得所有权
   *
   * - override：
   *   C++11关键字，显式表示重写基类虚函数
   *   编译器会检查基类是否有可重写的对应虚函数
   *   如果没有，编译错误
   *   作用：防止拼写错误、签名不匹配等
   */
  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;

 private:
  /**
   * @brief 处理路径生成
   *
   * @param frame 规划帧数据
   * @param reference_line_info 参考线信息
   * @return apollo::common::Status 处理状态
   *
   * 功能说明：
   * 后备路径生成的核心处理函数
   * 执行完整的路径生成流程
   *
   * 重写说明：
   * 该方法重写了基类PathGeneration的Process方法
   *
   * C++语法说明：
   * - apollo::common::Status：
   *   Apollo通用状态类，包含错误码和消息
   *   用于表示操作的成功/失败状态
   *
   * - Frame*：
   *   原始指针，指向规划帧
   *   使用指针因为frame需要被修改
   *
   * - ReferenceLineInfo*：
   *   参考线信息指针
   *   用于访问和修改参考线相关信息
   *
   * - override：
   *   重写基类虚函数
   */
  apollo::common::Status Process(
      Frame* frame, ReferenceLineInfo* reference_line_info) override;

  /**
   * @brief 计算所有路径边界
   *
   * @param boundary 输出：计算的路径边界
   * @return bool 计算是否成功
   *
   * 功能说明：
   * 决定所有可能的路径边界
   * 路径边界定义了车辆可以行驶的区域
   *
   * 算法说明：
   * - 分析自车周围环境
   * - 考虑静态和动态障碍物
   * - 生成可行的路径边界
   *
   * C++语法说明：
   * - std::vector<PathBoundary>* boundary：
   *   指向路径边界向量的指针
   *   用于输出多个路径边界
   *   使用指针允许函数修改外部变量
   *
   * - PathBoundary：
   *   路径边界类
   *   定义了路径的上下边界
   */
  bool DecidePathBounds(std::vector<PathBoundary>* boundary);

  /**
   * @brief 为每个路径边界优化路径
   *
   * @param path_boundaries 输入的路径边界
   * @param candidate_path_data 输出：候选路径数据
   * @return bool 优化是否成功
   *
   * 功能说明：
   * 根据给定的路径边界生成优化路径
   * 输出多个候选路径供后续评估
   *
   * 算法说明：
   * - 使用优化算法生成路径
   * - 考虑平滑性、安全性等因素
   * - 生成多个候选路径
   *
   * C++语法说明：
   * - const std::vector<PathBoundary>& path_boundaries：
   *   常量引用，输入的路径边界
   *   const保证不会被修改
   *   引用避免拷贝大型向量
   *
   * - std::vector<PathData>* candidate_path_data：
   *   指针输出参数
   *   用于输出多个候选路径
   *
   * - PathData：
   *   路径数据类
   *   包含路径点和相关属性
   */
  bool OptimizePath(const std::vector<PathBoundary>& path_boundaries,
                    std::vector<PathData>* candidate_path_data);

  /**
   * @brief 评估每条路径的可行性并选择最佳路径
   *
   * @param candidate_path_data 输入的候选路径
   * @param final_path 输出：最佳路径
   * @return bool 评估是否成功
   *
   * 功能说明：
   * 评估所有候选路径的可行性
   * 选择最佳路径作为最终输出
   *
   * 评估标准：
   * - 路径的平滑性
   * - 路径的安全性
   * - 路径的可行性（如曲率限制）
   *
   * C++语法说明：
   * - std::vector<PathData>* candidate_path_data：
   *   指针输入/输出参数
   *   输入候选路径，函数会修改其内容
   *
   * - PathData* final_path：
   *   指针输出参数
   *   用于输出最佳路径
   */
  bool AssessPath(std::vector<PathData>* candidate_path_data,
                  PathData* final_path);

 private:
  /**
   * @brief 后备路径生成器配置
   *
   * 功能说明：
   * 存储后备路径生成器的配置参数
   * 从protobuf配置文件中加载
   *
   * C++语法说明：
   * - FallbackPathConfig：
   *   Protobuf消息类型
   *   包含后备路径生成器的所有配置参数
   *
   * - config_：
   *   成员变量命名约定：xxx_后缀
   *   表示这是类的成员变量
   */
  FallbackPathConfig config_;
};

/**
 * @brief Cyber插件注册宏
 *
 * 功能说明：
 * 将FallbackPath类注册为Task类型的插件
 * 允许系统在运行时发现和加载这个路径生成器
 *
 * C++语法说明：
 * - CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN：
 *   Apollo Cyber RT的插件注册宏
 *   这是一个类级别的注册机制
 *   展开后会在全局插件管理器中注册此类
 *
 * - apollo::planning::FallbackPath：
 *   要注册的类名（完整命名空间）
 *
 * - Task：
 *   注册的插件基类型
 *   FallbackPath是Task的派生类
 *
 * 使用方式：
 * 通过插件管理器可以动态创建FallbackPath实例
 * 而无需直接引用具体的类名
 * 这实现了类创建的解耦
 */
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::FallbackPath, Task)

/**
 * @namespace命名空间结束
 *
 * C++语法说明：
 * - }  // namespace planning：
 *   结束planning命名空间
 *   注释方便阅读和导航
 *
 * - }  // namespace apollo：
 *   结束apollo命名空间
 */
}  // namespace planning
}  // namespace apollo