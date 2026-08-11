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

/**
 * @file message_util.h
 * @brief 消息工具头文件
 *
 * 功能说明：
 * 提供Protobuf消息的通用工具函数
 * 包括：填充消息头、导出消息、计算消息指纹
 *
 * 核心概念：
 * - Header：消息头，包含模块名、时间戳、序列号等元数据
 * - Sequence Number：序列号，用于消息追踪和排序
 * - Timestamp：时间戳，记录消息产生时间
 * - Message Fingerprint：消息指纹，用于消息去重或缓存
 *
 * C++语法说明：
 * - #pragma once：头文件保护
 * - template + SFINAE：模板偏特化技术
 * - std::enable_if：类型选择
 * - std::is_base_of：类型检查
 * - std::atomic：原子变量
 * - inline函数：内联函数建议
 * - std::hash：哈希函数对象
 **/

/**
 * @brief 头文件保护
 *
 * #pragma once：
 * 防止头文件被重复包含的预处理指令
 * 现代编译器普遍支持，简洁高效
 */
#pragma once

#include <memory>
/**
 * @brief 智能指针头文件
 *
 * 提供：
 * - std::shared_ptr：引用计数智能指针
 * - std::unique_ptr：独占所有权智能指针
 */

#include <string>
/**
 * @brief 字符串头文件
 *
 * 提供std::string字符串类
 */

#include "absl/strings/str_cat.h"
/**
 * @brief Abseil字符串工具头文件
 *
 * Abseil是Google开发的C++基础库
 * absl::StrCat：高性能字符串拼接
 * 比std::stringstream或+操作符更高效
 */

#include "google/protobuf/message.h"
/**
 * @brief Protobuf消息基类头文件
 *
 * google::protobuf::Message：
 * 所有Protobuf消息的基类
 * 提供序列化、描述符获取等功能
 */

#include "cyber/common/file.h"
/**
 * @brief Cyber RT文件工具头文件
 *
 * 提供：
 * - DirectoryExists：检查目录是否存在
 * - EnsureDirectory：确保目录存在
 * - SetProtoToASCIIFile：写入Protobuf到文本文件
 */

#include "cyber/time/clock.h"
/**
 * @brief Cyber RT时钟头文件
 *
 * 提供：
 * - cyber::Clock：Cyber RT时间服务
 * - Clock::NowInSeconds()：获取当前时间（秒）
 */

namespace apollo {
/**
 * @brief Apollo项目主命名空间
 */

namespace common {
/**
 * @brief 通用工具命名空间
 */

namespace util {
/**
 * @brief 工具模块命名空间
 *
 * apollo::common::util：
 * 通用工具函数集合
 */

/**
 * @brief 填充消息头
 *
 * @tparam T Protobuf消息类型模板参数
 * @param module_name 模块名称
 * @param msg 输出参数，指向要填充的消息
 *
 * 功能说明：
 * 为Protobuf消息填充标准头信息
 * 包括：模块名、时间戳、序列号
 *
 * 算法流程：
 * 1. 获取当前时间戳
 * 2. 设置模块名称
 * 3. 设置时间戳
 * 4. 原子递增并获取序列号
 *
 * C++语法说明：
 * - template + SFINAE：
 *   使用std::enable_if和std::is_base_of实现模板偏特化
 *   只允许google::protobuf::Message的子类使用此函数
 *
 * - typename std::enable_if<condition, int>::type = 0：
 *   SFINAE技术，通过模板参数限制T的类型
 *   std::is_base_of<Base, Derived>：检查是否是基类
 *
 * - std::atomic<uint64_t>：
 *   原子变量，保证序列号生成的线程安全
 *   多个线程同时调用不会产生重复序列号
 *
 * - sequence_num.fetch_add(1)：
 *   原子递增操作
 *   返回旧值，原子地加1
 *
 * - static_cast<unsigned int>(...)：
 *   类型转换，将uint64_t转为unsigned int
 *
 * @code
 *   Chassis chassis;
 *   FillHeader("planning", &chassis);
 *   // chassis.header().module_name() == "planning"
 *   // chassis.header().sequence_num() > 0
 * @endcode
 */
template <typename T, typename std::enable_if<
                          std::is_base_of<google::protobuf::Message, T>::value,
                          int>::type = 0>
/**
 * @brief 模板参数说明：
 *
 * typename std::enable_if<Cond, T>::type = 0：
 * - std::enable_if：模板元编程工具
 * - 第一个模板参数Cond：条件，为true时有效
 * - 第二个模板参数T：默认int
 * - = 0：默认值
 *
 * std::is_base_of<A, B>::value：
 * - 检查A是否是B的基类
 * - ::value：获取编译时常量
 *
 * 整体效果：
 * 只有当T继承自google::protobuf::Message时
 * 才会实例化此模板函数
 */
static void FillHeader(const std::string& module_name, T* msg) {
  /**
   * @brief 序列号计数器
   *
   * static std::atomic<uint64_t>：
   * - static：静态变量，程序生命周期内唯一
   * - std::atomic：原子类型
   * - uint64_t：无符号64位整数
   * - = {0}：初始化为0
   *
   * 特点：
   * - 线程安全
   * - 每次调用fetch_add返回唯一递增的值
   */
  static std::atomic<uint64_t> sequence_num = {0};

  auto* header = msg->mutable_header();
  /**
   * @brief 获取消息头的可变指针
   *
   * msg->mutable_header()：
   * - Protobuf消息的方法
   * - mutable_前缀表示可修改
   * - 返回Header对象的指针
   */

  double timestamp = ::apollo::cyber::Clock::NowInSeconds();
  /**
   * @brief 获取当前时间戳
   *
   * ::apollo::cyber::Clock::NowInSeconds()：
   * - ::前缀：全局命名空间，避免歧义
   * - Clock::NowInSeconds()：获取当前时间（秒）
   * - 返回double类型的秒数
   */

  header->set_module_name(module_name);
  /**
   * @brief 设置模块名称
   *
   * header->set_module_name(name)：
   * - Protobuf消息的setter方法
   * - 设置header的module_name字段
   */

  header->set_timestamp_sec(timestamp);
  /**
   * @brief 设置时间戳
   *
   * header->set_timestamp_sec(sec)：
   * - 设置header的timestamp_sec字段
   * - 单位：秒
   */

  header->set_sequence_num(
      static_cast<unsigned int>(sequence_num.fetch_add(1)));
  /**
   * @brief 设置序列号
   *
   * sequence_num.fetch_add(1)：
   * - 原子递增操作
   * - 返回递增前的值
   * - 保证每个消息获得唯一序列号
   *
   * static_cast<unsigned int>(...)：
   * - C++类型转换
   * - uint64_t -> unsigned int
   * - 序列号通常不需要64位
   */
}

/**
 * @brief 导出消息到文件
 *
 * @tparam T Protobuf消息类型模板参数
 * @param msg 要导出的消息智能指针
 * @param dump_dir 导出目录，默认"/tmp"
 * @return bool 是否成功导出
 *
 * 功能说明：
 * 将Protobuf消息以ASCII文本格式导出到文件
 * 用于调试、记录日志、或保存消息历史
 *
 * 算法流程：
 * 1. 检查消息是否为空
 * 2. 获取消息类型名称
 * 3. 构建导出路径：dump_dir/type_name/sequence_num.pb.txt
 * 4. 确保目录存在
 * 5. 序列化消息并写入文件
 *
 * C++语法说明：
 * - const std::shared_ptr<T>&：
 *   常量引用，避免拷贝
 *   shared_ptr管理消息生命周期
 *
 * - std::shared_ptr<T>：
 *   引用计数智能指针
 *   自动释放内存，防止泄漏
 *
 * - T::descriptor()->full_name()：
 *   获取Protobuf消息的完整类型名
 *   descriptor()返回消息描述符
 *   full_name()返回完全限定名称
 *
 * - absl::StrCat：
 *   高效字符串拼接
 *   避免多次内存分配
 *
 * @code
 *   auto msg = std::make_shared<Chassis>();
 *   DumpMessage(msg, "/tmp/apollo");
 *   // 写入 /tmp/apollo/apollo.common_msgs.Chassis/123.pb.txt
 * @endcode
 */
template <typename T, typename std::enable_if<
                          std::is_base_of<google::protobuf::Message, T>::value,
                          int>::type = 0>
/**
 * @brief 同样的SFINAE模板约束
 *
 * 确保只有Protobuf消息类型才能使用此函数
 */
bool DumpMessage(const std::shared_ptr<T>& msg,
                 /**
                  * @brief 消息智能指针
                  *
                  * const std::shared_ptr<T>&：
                  * - const：不能修改指针本身
                  * - shared_ptr：智能指针
                  * - &：引用，避免拷贝
                  */
                 const std::string& dump_dir = "/tmp") {
  /**
   * @brief 默认导出目录
   *
   * = "/tmp"：
   * 默认参数值
   * 如果不指定，导出到/tmp目录
   */

  if (!msg) {
    /**
     * @brief 检查消息是否为空
     *
     * if (!msg)：
     * - shared_ptr支持bool转换
     * - 空指针返回true
     */
    AWARN << "Message to be dumped is nullptr!";
    /**
     * @brief 输出警告日志
     *
     * AWARN：Apollo警告日志宏
     */
    return false;
    /**
     * @brief 空消息，导出失败
     */
  }

  auto type_name = T::descriptor()->full_name();
  /**
   * @brief 获取消息类型完整名称
   *
   * T::descriptor()：
   * - Protobuf消息的静态方法
   * - 返回指向Descriptor的指针
   * - Descriptor包含消息的元信息
   *
   * ->full_name()：
   * - 获取消息的完全限定名称
   * - 格式："apollo.common_msgs.Chassis"
   */

  std::string dump_path = dump_dir + "/" + type_name;
  /**
   * @brief 构建导出目录路径
   *
   * dump_dir + "/" + type_name：
   * - 字符串拼接
   * - 示例："/tmp/apollo.common_msgs.Chassis"
   *
   * 使用+操作符：
   * - std::string重载了+
   * - 可能多次内存分配，不够高效
   * - 对于短路径可接受
   */

  if (!cyber::common::DirectoryExists(dump_path)) {
    /**
     * @brief 检查目录是否存在
     *
     * cyber::common::DirectoryExists(path)：
     * - 返回bool
     * - true：目录存在
     * - false：目录不存在
     */
    if (!cyber::common::EnsureDirectory(dump_path)) {
      /**
       * @brief 确保目录存在
       *
       * EnsureDirectory：
       * - 递归创建目录
       * - 类似mkdir -p
       * - 返回bool：是否成功
       */
      AERROR << "Cannot enable dumping for '" << type_name
             << "' because the path " << dump_path
             << " cannot be created or is not a directory.";
      /**
       * @brief 输出错误日志
       *
       * AERROR：Apollo错误日志宏
       * <<：流输出，拼接字符串
       */
      return false;
      /**
       * @brief 创建目录失败
       */
    }
  }

  auto sequence_num = msg->header().sequence_num();
  /**
   * @brief 获取消息序列号
   *
   * msg->header().sequence_num()：
   * - header()：获取消息头（const引用）
   * - sequence_num()：获取序列号
   *
   * 使用auto推导类型
   */

  return cyber::common::SetProtoToASCIIFile(
      *msg, absl::StrCat(dump_path, "/", sequence_num, ".pb.txt"));
  /**
   * @brief 导出消息到文件
   *
   * *msg：
   * - 解引用shared_ptr
   * - 得到实际的Message引用
   *
   * absl::StrCat(...)：
   * - 高效字符串拼接
   * - 参数可以是多种类型
   * - 构造完整文件路径
   *
   * cyber::common::SetProtoToASCIIFile：
   * - 写入Protobuf消息为ASCII格式
   * - 返回bool：是否成功
   *
   * 文件命名格式：
   * /tmp/apollo.common_msgs.Chassis/123.pb.txt
   */
}

/**
 * @brief 计算消息指纹
 *
 * @param message 输入消息（常量引用）
 * @return size_t 消息的哈希值
 *
 * 功能说明：
 * 计算Protobuf消息的哈希指纹
 * 用于消息去重、缓存键、或快速比较
 *
 * 算法流程：
 * 1. 将消息序列化为字节串
 * 2. 对字节串计算哈希值
 * 3. 返回哈希值
 *
 * C++语法说明：
 * - inline函数：
 *   建议编译器内联
 *   适用于短小、频繁调用的函数
 *   减少函数调用开销
 *
 * - const google::protobuf::Message&：
 *   - const：不能修改消息
 *   - Message&：引用，避免拷贝
 *
 * - message.SerializeToString(&proto_bytes)：
 *   - Protobuf内置方法
 *   - 将消息序列化为字节串
 *   - 存储到string中
 *
 * - std::hash<std::string>：
 *   - 标准库哈希函数对象
 *   - 对string计算哈希
 *
 * - hash_fn(proto_bytes)：
 *   - 调用哈希函数
 *   - 返回哈希值
 *
 * @code
 *   Chassis chassis1, chassis2;
 *   // ... 设置chassis1和chassis2相同的数据
 *   size_t fp1 = MessageFingerprint(chassis1);
 *   size_t fp2 = MessageFingerprint(chassis2);
 *   if (fp1 == fp2) {
 *     // 可能是相同消息
 *   }
 * @endcode
 */
inline size_t MessageFingerprint(const google::protobuf::Message& message) {
  /**
   * @brief inline函数
   *
   * inline关键字：
   * - 建议编译器将函数调用内联
   * - 减少函数调用开销
   * - 适用于短小函数
   * - 编译器可能忽略此建议
   */

  static std::hash<std::string> hash_fn;
  /**
   * @brief 静态哈希函数对象
   *
   * static std::hash<std::string>：
   * - static：静态局部变量，函数退出后仍存在
   * - 只初始化一次
   * - 避免每次调用都构造
   */

  std::string proto_bytes;
  /**
   * @brief 序列化字节串
   *
   * std::string：
   * - 作为字节缓冲区
   * - 可以存储任意字节序列
   */

  message.SerializeToString(&proto_bytes);
  /**
   * @brief 序列化消息到字符串
   *
   * SerializeToString：
   * - Protobuf内置方法
   * - 将消息序列化为二进制字节串
   * - 存储到proto_bytes中
   *
   * 注意：
   * - 二进制格式，非人类可读
   * - 与文本格式不同
   */

  return hash_fn(proto_bytes);
  /**
   * @brief 计算并返回哈希值
   *
   * hash_fn(proto_bytes)：
   * - 调用哈希函数
   * - 输入：序列化后的字节串
   * - 输出：size_t类型的哈希值
   *
   * 返回值用途：
   * - 消息去重
   * - 缓存键
   * - 快速比较
   *
   * 注意：
   * - size_t是平台相关
   * - 通常是uint64_t或uint32_t
   */
}

}  // namespace util
/**
 * @brief util命名空间结束标记
 */

}  // namespace common
/**
 * @brief common命名空间结束标记
 */

}  // namespace apollo
/**
 * @brief apollo命名空间结束标记
 */
