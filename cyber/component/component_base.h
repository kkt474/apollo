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
 * @file component_base.h
 * @brief Cyber RT组件基类头文件
 *
 * 功能说明：
 * 定义了Cyber RT中所有组件(Component)的基类
 * 组件是Cyber RT的基本计算单元，通过发布-订阅模式进行数据通信
 *
 * 核心概念：
 * - Component：组件，Cyber RT中的基本计算模块
 * - Reader：读者，订阅话题接收数据
 * - Writer：作者，发布数据到话题
 * - Node：节点，组件运行的载体
 * - Scheduler：调度器，管理组件的执行调度
 *
 * 组件生命周期：
 * 1. 构造：创建组件对象
 * 2. 初始化(Initialize)：加载配置，创建Reader/Writer
 * 3. 运行：接收和处理数据
 * 4. 关闭(Shutdown)：清理资源，移除任务
 *
 * 设计模式：
 * - 模板方法模式：提供Initialize框架，子类实现Init
 * - RAII模式：资源获取即初始化
 *
 * C++语法说明：
 * - std::enable_shared_from_this：让类能安全地返回shared_ptr this
 * - std::atomic：原子变量，用于线程安全的状态标志
 * - = 0：纯虚函数声明
 * - template：模板编程
 * - override：重写基类虚函数
 **/
#ifndef CYBER_COMPONENT_COMPONENT_BASE_H_
/**
 * @brief 头文件保护宏
 *
 * #ifndef/#define/#endif：
 * 防止头文件被重复包含
 * 标准做法：文件名大写，::替换为__
 */

#define CYBER_COMPONENT_COMPONENT_BASE_H_
/**
 * @brief 头文件保护宏定义
 */

#include <atomic>
/**
 * @brief 原子操作头文件
 *
 * 提供std::atomic原子类型
 * 用于多线程环境下的无锁同步
 * 常用于计数器、状态标志等
 */

#include <memory>
/**
 * @brief 智能指针头文件
 *
 * 提供：
 * - std::shared_ptr：引用计数智能指针
 * - std::unique_ptr：独占所有权智能指针
 * - std::weak_ptr：弱引用智能指针
 */

#include <string>
/**
 * @brief 字符串头文件
 *
 * 提供std::string字符串类
 */

#include <vector>
/**
 * @brief 动态数组头文件
 *
 * 提供std::vector容器
 */

#include "gflags/gflags.h"
/**
 * @brief Google Flags命令行参数库
 *
 * 提供DEFINE_xxx宏定义配置参数
 * gflags允许通过命令行或配置文件设置参数
 */

#include "cyber/proto/component_conf.pb.h"
/**
 * @brief 组件配置Protobuf消息头文件
 *
 * 包含：
 * - ComponentConfig：组件配置消息
 * - TimerComponentConfig：定时器组件配置
 * Protobuf是一种结构化数据序列化协议
 */

#include "cyber/class_loader/class_loader.h"
/**
 * @brief 类加载器头文件
 *
 * ClassLoader支持运行时动态加载类
 * 用于插件系统和组件的延迟加载
 */

#include "cyber/common/environment.h"
/**
 * @brief 环境变量头文件
 *
 * 提供环境变量访问接口
 */

#include "cyber/common/file.h"
/**
 * @brief 文件操作头文件
 *
 * 提供文件读写工具函数
 */

#include "cyber/node/node.h"
/**
 * @brief 节点头文件
 *
 * Node是Cyber RT中的通信节点
 * 提供话题发布/订阅接口
 */

#include "cyber/scheduler/scheduler.h"
/**
 * @brief 调度器头文件
 *
 * Scheduler管理任务的调度执行
 * 支持多线程和实时调度
 */

namespace apollo {
/**
 * @brief Apollo项目主命名空间
 */

namespace cyber {
/**
 * @brief Cyber RT框架命名空间
 *
 * Cyber RT是Apollo的实时计算框架
 */

using apollo::cyber::proto::ComponentConfig;
/**
 * @brief 使用ComponentConfig类型
 *
 * using声明：将其他命名空间的类型引入当前作用域
 * 简化后续代码中的类型引用
 */

using apollo::cyber::proto::TimerComponentConfig;
/**
 * @brief 使用TimerComponentConfig类型
 */

/**
 * @brief 组件基类
 *
 * @tparam M 模板参数，消息类型
 *
 * 功能说明：
 * 所有Cyber RT组件的基类
 * 提供组件的通用生命周期管理
 *
 * 设计特点：
 * - 继承std::enable_shared_from_this：允许获取this的shared_ptr
 * - 纯虚函数Init：子类必须实现具体初始化逻辑
 * - 模板方法模式：Initialize框架已定，子类实现Init
 *
 * C++语法说明：
 * - class ComponentBase：类声明
 * - public std::enable_shared_from_this<ComponentBase>：
 *   公有继承模板类
 *   enable_shared_from_this提供shared_from_this()方法
 *   允许在成员函数中获取当前对象的shared_ptr
 */
class ComponentBase : public std::enable_shared_from_this<ComponentBase> {
 public:
  /**
   * @brief 组件基类公有接口
   */

  /**
   * @brief 读者类型别名
   *
   * template模板语法：
   * using Reader = xxx;
   * 为已有类型定义别名
   *
   * @tparam M 消息类型模板参数
   * using Reader<M> = cyber::Reader<M>
   * 简化后续代码：Reader<M>等价于cyber::Reader<M>
   *
   * 用法示例：Reader<Chassis>表示接收Chassis消息的Reader
   */
  template <typename M>
  using Reader = cyber::Reader<M>;

  /**
   * @brief 虚析构函数
   *
   * virtual ~ComponentBase() {}
   * 虚析构函数确保通过基类指针删除派生类对象时
   * 能正确调用派生类的析构函数
   *
   * 重要性：
   * 如果没有virtual析构函数，
   * delete base_ptr可能只调用基类析构函数
   * 导致派生类资源泄漏
   */
  virtual ~ComponentBase() {}

  /**
   * @brief 组件初始化接口（ComponentConfig版本）
   *
   * @param config 组件配置消息
   * @return bool 初始化是否成功
   *
   * 功能说明：
   * 组件初始化的模板方法
   * 框架已定义初始化流程，子类实现Init
   *
   * 初始化流程：
   * 1. 加载配置文件
   * 2. 调用子类Init进行具体初始化
   *
   * C++语法说明：
   * - virtual bool Initialize(...)：虚函数，允许子类重写
   * - const ComponentConfig& config：
   *   const引用参数，避免拷贝
   * - return false：默认实现返回false，表示未实现
   */
  virtual bool Initialize(const ComponentConfig& config) { return false; }

  /**
   * @brief 组件初始化接口（TimerComponentConfig版本）
   *
   * @param config 定时器组件配置
   * @return bool 初始化是否成功
   *
   * 功能说明：
   * 定时器组件的初始化接口
   * 支持定时触发执行的组件
   */
  virtual bool Initialize(const TimerComponentConfig& config) { return false; }

  /**
   * @brief 关闭组件
   *
   * 功能说明：
   * 优雅关闭组件，清理资源
   *
   * 关闭流程：
   * 1. 检查是否已经关闭（避免重复关闭）
   * 2. 调用Clear清理子类资源
   * 3. 关闭所有Reader
   * 4. 从调度器移除任务
   *
   * C++语法说明：
   * - virtual void Shutdown()：虚函数
   * - std::atomic<bool> is_shutdown_：原子布尔变量
   * - is_shutdown_.exchange(true)：
   *   原子操作：将is_shutdown_设置为true并返回旧值
   *   如果返回true说明已经被其他线程设置为true
   *   这是一个线程安全的"检查并设置"操作
   * - for (auto& reader : readers_)：
   *   范围for循环遍历读者列表
   *   auto&自动推导类型，引用避免拷贝
   * - scheduler::Instance()->RemoveTask(...)：
   *   单例模式获取调度器实例
   *   调用RemoveTask移除该组件的任务
   */
  virtual void Shutdown() {
    if (is_shutdown_.exchange(true)) {
      /**
       * @brief 检查是否已关闭
       *
       * exchange是原子操作：
       * - 设置新值true
       * - 返回旧值
       * - 如果旧值是true，说明已经被关闭
       */
      return;  /**< 已关闭，直接返回 */
    }

    Clear();  /**< 清理子类资源 */

    for (auto& reader : readers_) {
      /**
       * @brief 遍历关闭所有Reader
       *
       * auto& reader：自动类型推导为vector中的元素引用
       * 引用避免拷贝，提高效率
       */
      reader->Shutdown();
      /**
       * @brief 关闭单个Reader
       */
    }

    scheduler::Instance()->RemoveTask(node_->Name());
    /**
     * @brief 从调度器移除任务
     *
     * scheduler::Instance()：获取调度器单例
     * node_->Name()：获取节点名称作为任务标识
     */
  }

  /**
   * @brief 获取Protobuf配置
   *
   * @tparam T Protobuf消息类型模板参数
   * @param config 输出参数，存储读取的配置
   * @return bool 是否读取成功
   *
   * 功能说明：
   * 从配置文件读取Protobuf消息
   *
   * 算法流程：
   * 1. 调用common::GetProtoFromFile读取文件
   * 2. 解析为指定的Protobuf消息类型
   *
   * C++语法说明：
   * - template <typename T>：
   *   函数模板，允许传递任意Protobuf消息类型
   * - bool GetProtoConfig(T* config) const：
   *   const成员函数，不会修改对象状态
   *   T*是指向任意Protobuf消息类型的指针
   * - common::GetProtoFromFile：
   *   从文件读取并解析Protobuf消息
   */
  template <typename T>
  bool GetProtoConfig(T* config) const {
    return common::GetProtoFromFile(config_file_path_, config);
    /**
     * @brief 调用文件工具读取配置
     *
     * common::GetProtoFromFile(file_path, config)：
     * - file_path_：类成员，存储配置文件的路径
     * - config：输出参数，解析后的消息存储在此
     * - 返回true表示成功，false表示失败
     */
  }

 protected:
  /**
   * @brief 受保护的成员函数（子类接口）
   *
   * protected成员：
   * - 基类成员：子类可以访问
   * - 其他代码：不能访问
   */

  /**
   * @brief 子类具体初始化接口（纯虚函数）
   *
   * @return bool 初始化是否成功
   *
   * 功能说明：
   * 子类必须实现此函数进行具体初始化
   * 由Initialize模板方法调用
   *
   * C++语法说明：
   * - virtual bool Init() = 0;
   * - = 0表示纯虚函数
   * - 包含纯虚函数的类是抽象类
   * - 不能直接实例化，必须由派生类实现
   */
  virtual bool Init() = 0;

  /**
   * @brief 清理资源
   *
   * 功能说明：
   * 子类重写此函数清理自定义资源
   * 默认实现为空
   *
   * C++语法说明：
   * - virtual void Clear() { return; }
   * - 有默认实现的虚函数
   * - 子类可以选择重写或使用默认实现
   */
  virtual void Clear() { return; }

  /**
   * @brief 获取配置文件路径
   *
   * @return const std::string& 配置文件的常量引用
   *
   * 功能说明：
   * 提供配置文件路径的只读访问
   *
   * C++语法说明：
   * - const std::string& ConfigFilePath() const
   * - 第一个const：返回类型是常量引用，不可修改
   * - 第二个const：成员函数不会修改对象状态
   */
  const std::string& ConfigFilePath() const { return config_file_path_; }

  /**
   * @brief 加载配置文件
   *
   * @param config 组件配置消息
   *
   * 功能说明：
   * 从ComponentConfig加载配置文件路径
   * 支持环境变量和绝对路径
   *
   * 算法流程：
   * 1. 如果配置提供了config_file_path：
   *    - 使用GetFilePathWithEnv查找文件（支持环境变量）
   *    - 如果找不到，使用配置的路径
   * 2. 如果配置提供了flag_file_path：
   *    - 同样处理
   *    - 调用SetCommandLineOption设置flagfile
   *
   * C++语法说明：
   * - void LoadConfigFiles(...)：
   *   重载版本，接收ComponentConfig
   * - if (!config.config_file_path().empty())：
   *   检查配置中的文件路径是否非空
   * - config.config_file_path()：
   *   Protobuf消息的getter方法
   *   返回配置的config_file_path字段
   * - common::GetFilePathWithEnv(path, env_var, &result)：
   *   在环境变量指定的路径列表中查找文件
   * - google::SetCommandLineOption("flagfile", ...)：
   *   设置gflags的命令行参数
   */
  void LoadConfigFiles(const ComponentConfig& config) {
    if (!config.config_file_path().empty()) {
      /**
       * @brief 处理配置文件路径
       */
      if (!common::GetFilePathWithEnv(config.config_file_path(),
                                      "APOLLO_CONF_PATH", &config_file_path_)) {
        /**
         * @brief 在APOLLO_CONF_PATH中查找配置文件
         *
         * GetFilePathWithEnv返回false表示未找到
         */
        AERROR << "conf file [" << config.config_file_path() << "] not found!";
        /**
         * @brief 输出错误日志
         *
         * AERROR：Cyber RT错误日志宏
         * <<：流输出运算符，拼接字符串
         */
        config_file_path_ = config.config_file_path();
        /**< @brief 找不到时使用原始路径 */
      } else {
        AINFO << "use config file: " << config_file_path_;
        /**< @brief 成功找到，记录日志 */
      }
    }

    if (!config.flag_file_path().empty()) {
      /**
       * @brief 处理flag文件路径
       */
      std::string flag_file_path = config.flag_file_path();
      /**< @brief 复制一份，避免修改原配置 */

      if (!common::GetFilePathWithEnv(config.flag_file_path(),
                                      "APOLLO_FLAG_PATH", &flag_file_path)) {
        /**
         * @brief 在APOLLO_FLAG_PATH中查找flag文件
         */
        AERROR << "flag file [" << config.flag_file_path() << "] not found!";
      } else {
        AINFO << "use flag file: " << flag_file_path;
        /**< @brief 成功找到，记录日志 */
      }

      google::SetCommandLineOption("flagfile", flag_file_path.c_str());
      /**
       * @brief 设置gflags的flagfile参数
       *
       * google::SetCommandLineOption：
       * 运行时设置gflags参数
       * "flagfile"：参数名
       * flag_file_path.c_str()：参数字符串
       *
       * 设置后，gflags会读取该文件中的所有FLAGS定义
       */
    }
  }

  /**
   * @brief 加载配置文件（TimerComponentConfig版本）
   *
   * @param config 定时器组件配置
   *
   * 功能说明：
   * 与ComponentConfig版本类似
   * 支持定时器组件的配置加载
   */
  void LoadConfigFiles(const TimerComponentConfig& config) {
    if (!config.config_file_path().empty()) {
      if (!common::GetFilePathWithEnv(config.config_file_path(),
                                      "APOLLO_CONF_PATH", &config_file_path_)) {
        AERROR << "conf file [" << config.config_file_path() << "] not found!";
        config_file_path_ = config.config_file_path();
      } else {
        AINFO << "use config file: " << config_file_path_;
      }
    }

    if (!config.flag_file_path().empty()) {
      std::string flag_file_path = config.flag_file_path();
      if (!common::GetFilePathWithEnv(config.flag_file_path(),
                                      "APOLLO_FLAG_PATH", &flag_file_path)) {
        AERROR << "flag file [" << config.flag_file_path() << "] not found!";
      } else {
        AINFO << "use flag file: " << flag_file_path;
      }
      google::SetCommandLineOption("flagfile", flag_file_path.c_str());
    }
  }

  /**
   * @brief 组件成员变量
   *
   * protected成员变量：子类可直接访问
   */

  /**
   * @brief 关闭状态标志
   *
   * @brief std::atomic<bool>：原子布尔类型
   *
   * 原子类型特点：
   * - 多个线程同时访问时，保证数据一致性
   * - 无需锁即可实现线程安全
   * - 适用于计数器、状态标志等
   *
   * 初始化={false}：
   * - 使用花括号初始化
   * - 初始值为false（未关闭）
   */
  std::atomic<bool> is_shutdown_ = {false};

  /**
   * @brief 组件关联的节点
   *
   * @brief std::shared_ptr<Node>：节点智能指针
   *
   * 智能指针特点：
   * - shared_ptr：引用计数智能指针
   * - 最后一个使用者销毁时释放对象
   * - 避免手动内存管理
   *
   * nullptr初始化：
   * - 初始为空指针
   * - 后续由子类初始化
   */
  std::shared_ptr<Node> node_ = nullptr;

  /**
   * @brief 配置文件路径
   *
   * @brief std::string：字符串类型
   *
   * 存储组件配置文件的路径
   * 由LoadConfigFiles函数设置
   */
  std::string config_file_path_ = "";

  /**
   * @brief 读者列表
   *
   * @brief std::vector<std::shared_ptr<ReaderBase>>：
   * - std::vector：动态数组容器
   * - std::shared_ptr<ReaderBase>：Reader智能指针
   *
   * vector特点：
   * - 动态大小，可伸缩
   * - 支持push_back添加元素
   * - 支持迭代器遍历
   *
   * 存储组件创建的所有Reader
   * 用于Shutdown时统一关闭
   */
  std::vector<std::shared_ptr<ReaderBase>> readers_;
};

}  // namespace cyber
/**
 * @brief Cyber命名空间结束标记
 */

}  // namespace apollo
/**
 * @brief Apollo命名空间结束标记
 */

#endif  // CYBER_COMPONENT_COMPONENT_BASE_H_
/**
 * @brief 头文件保护宏结束
 */
