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
 * @file file.cc
 * @brief Apollo Cyber RT文件操作工具实现文件
 *
 * 功能说明：
 * 实现了Cyber RT框架下的文件操作工具类，提供文件读写、目录操作、路径处理等功能
 * 支持Protobuf消息的ASCII/二进制/JSON格式序列化与反序列化
 *
 * 核心概念：
 * - Protobuf：Google Protocol Buffers，结构化数据序列化协议
 * - File Descriptor：文件描述符，操作系统打开文件的标识
 * - ZeroCopyStream：零拷贝流，避免数据复制提高性能
 * - ASCII/Binary/JSON：Protobuf支持的三种序列化格式
 *
 * 文件格式说明：
 * - ASCII格式：人类可读的文本格式，扩展名为.prototxt
 * - Binary格式：二进制格式，扩展名为.bin，效率高但不可读
 * - JSON格式：JSON格式，扩展名为.json，跨平台性好
 *
 * 系统调用说明：
 * - open/read/write/close：Unix文件I/O系统调用
 * - stat：获取文件状态信息
 * - mkdir/rmdir：目录创建删除
 * - opendir/readdir/closedir：目录遍历
 * - glob/globfree：通配符模式匹配
 *
 * C++语法说明：
 * - std::fstream：文件流类
 * - std::ios::binary：二进制模式标志
 * - errno：系统错误码变量
 * - struct stat：文件状态结构体
 * - glob_t：glob模式匹配结果结构体
 **/

#include "cyber/common/file.h"
/**
 * @brief 文件工具头文件
 *
 * 包含文件操作函数的声明
 */

#include <dirent.h>
/**
 * @brief POSIX目录操作头文件
 *
 * 提供：
 * - DIR：目录流类型
 * - opendir/readdir/closedir：目录操作函数
 * - dirent：目录项结构体
 * - dirent->d_type：目录项类型
 * - DT_DIR/DT_REG：目录/普通文件标志
 */

#include <fcntl.h>
/**
 * @brief 文件控制头文件
 *
 * 提供：
 * - open：打开文件的系统调用
 * - O_RDONLY/O_WRONLY/O_RDWR：读写模式标志
 * - O_CREAT/O_TRUNC：创建/截断标志
 * - S_IRWXU/S_IRWXG/S_IRWXO：权限标志
 */

#include <glob.h>
/**
 * @brief 通配符模式匹配头文件
 *
 * 提供：
 * - glob/globfree：通配符模式匹配函数
 * - glob_t：匹配结果结构体
 * - GLOB_TILDE：支持~扩展的标志
 */

#include <sys/mman.h>
/**
 * @brief 内存映射头文件
 *
 * 提供：
 * - mmap：内存映射函数
 * - munmap：解除内存映射
 */

#include <sys/stat.h>
/**
 * @brief 文件状态头文件
 *
 * 提供：
 * - stat/fstat：获取文件状态
 * - struct stat：文件状态结构体
 * - S_ISDIR/S_ISREG：判断文件/目录类型宏
 * - S_IFDIR/S_IFREG：文件类型标志
 */

#include <sys/types.h>
/**
 * @brief 系统数据类型头文件
 *
 * 提供基本系统类型定义
 */

#include <unistd.h>
/**
 * @brief Unix标准头文件
 *
 * 提供：
 * - close：关闭文件描述符
 * - read/write：读写文件
 * - getcwd：获取当前工作目录
 * - unlink：删除文件
 */

#include <cerrno>
/**
 * @brief C错误码头文件
 *
 * 提供：
 * - errno：系统错误码变量
 * - EEXIST：文件已存在错误
 * - strerror：将错误码转为字符串
 */

#include <cstddef>
/**
 * @brief 标准C类型头文件
 *
 * 提供：
 * - size_t：无符号大小类型
 * - ssize_t：有符号大小类型
 */

#include <fstream>
/**
 * @brief 文件流头文件
 *
 * 提供：
 * - std::fstream：文件流类
 * - std::ifstream：输入文件流
 * - std::ofstream：输出文件流
 * - std::ios：I/O流控制
 */

#include <string>
/**
 * @brief 字符串头文件
 *
 * 提供：
 * - std::string：字符串类
 */

#include "google/protobuf/util/json_util.h"
/**
 * @brief Protobuf JSON工具头文件
 *
 * 提供：
 * - JsonParseOptions：JSON解析选项
 * - JsonStringToMessage：JSON字符串转Protobuf消息
 */

#include "nlohmann/json.hpp"
/**
 * @brief JSON库头文件
 *
 * nlohmann::json：C++ JSON库
 * 用于解析和生成JSON数据
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

namespace common {
/**
 * @brief Cyber通用工具命名空间
 *
 * 包含文件、日志、时间等工具
 */

using std::istreambuf_iterator;
/**
 * @brief 使用istreambuf_iterator类型
 *
 * 用于遍历输入流缓冲区
 */

using std::string;
/**
 * @brief 使用std::string
 */

using std::vector;
/**
 * @brief 使用std::vector
 */

/**
 * @brief 将Protobuf消息写入ASCII格式文件
 *
 * @param message Protobuf消息引用
 * @param file_descriptor 已打开的文件描述符
 * @return bool 是否写入成功
 *
 * 功能说明：
 * 使用文件描述符将Protobuf消息以ASCII格式写入文件
 * 使用零拷贝流提高性能
 *
 * 算法流程：
 * 1. 检查文件描述符有效性
 * 2. 创建FileOutputStream
 * 3. 使用TextFormat::Print序列化消息
 * 4. 关闭输出流和文件描述符
 *
 * C++语法说明：
 * - const google::protobuf::Message&：常量引用，避免拷贝
 * - new FileOutputStream：堆分配，调用者负责生命周期
 * - delete：显式释放内存
 * - ZeroCopyOutputStream：零拷贝输出流接口
 */
bool SetProtoToASCIIFile(const google::protobuf::Message &message,
                         int file_descriptor) {
  using google::protobuf::TextFormat;
  /**< @brief 使用TextFormat进行ASCII格式化 */
  using google::protobuf::io::FileOutputStream;
  /**< @brief 文件输出流 */
  using google::protobuf::io::ZeroCopyOutputStream;
  /**< @brief 零拷贝输出流基类 */

  if (file_descriptor < 0) {
    /**
     * @brief 检查文件描述符有效性
     *
     * 有效的文件描述符是非负整数
     */
    AERROR << "Invalid file descriptor.";
    return false;
  }

  ZeroCopyOutputStream *output = new FileOutputStream(file_descriptor);
  /**
   * @brief 创建零拷贝输出流
   *
   * new：堆分配对象
   * FileOutputStream：接受文件描述符的输出流
   * 注意：这里分配在堆内存，需要手动delete
   */

  bool success = TextFormat::Print(message, output);
  /**
   * @brief 序列化消息到输出流
   *
   * TextFormat::Print：
   * - 将Protobuf消息格式化为ASCII文本
   * - 输出到ZeroCopyOutputStream
   * 返回：是否成功
   */

  delete output;
  /**
   * @brief 释放输出流内存
   *
   * delete：显式析构对象，释放内存
   * 与前面的new配对
   */

  close(file_descriptor);
  /**
   * @brief 关闭文件描述符
   *
   * close：Unix系统调用，关闭文件描述符
   */

  return success;
  /**
   * @brief 返回写入是否成功
   */
}

/**
 * @brief 将Protobuf消息写入ASCII格式文件（文件名版本）
 *
 * @param message Protobuf消息引用
 * @param file_name 文件名
 * @return bool 是否写入成功
 *
 * 功能说明：
 * 打开文件后调用SetProtoToASCIIFile重载版本
 *
 * 算法流程：
 * 1. 使用open系统调用打开文件
 * 2. 检查文件是否成功打开
 * 3. 调用SetProtoToASCIIFile写入
 *
 * C++语法说明：
 * - open(file, flags, mode)：Unix打开文件系统调用
 * - O_WRONLY：只写模式
 * - O_CREAT：文件不存在则创建
 * - O_TRUNC：如果文件存在则截断
 * - S_IRWXU：所有者读写执行权限
 */
bool SetProtoToASCIIFile(const google::protobuf::Message &message,
                         const std::string &file_name) {
  int fd = open(file_name.c_str(), O_WRONLY | O_CREAT | O_TRUNC, S_IRWXU);
  /**
   * @brief 打开文件
   *
   * open系统调用：
   * - file_name.c_str()：将string转为C字符串
   * - O_WRONLY | O_CREAT | O_TRUNC：组合标志
   *   - O_WRONLY：只写模式
   *   - O_CREAT：文件不存在则创建
   *   - O_TRUNC：截断已有文件
   * - S_IRWXU：权限标志，所有者读写执行
   *
   * 返回：文件描述符（非负整数），失败返回-1
   */

  if (fd < 0) {
    /**
     * @brief 检查文件是否成功打开
     */
    AERROR << "Unable to open file " << file_name << " to write.";
    return false;
  }

  return SetProtoToASCIIFile(message, fd);
  /**
   * @brief 调用重载版本写入数据
   */
}

/**
 * @brief 将字符串写入ASCII格式文件
 *
 * @param content 字符串内容
 * @param file_name 文件名
 * @return bool 是否写入成功
 *
 * 功能说明：
 * 直接将字符串内容写入文件，不经过Protobuf
 *
 * 算法流程：
 * 1. 打开文件
 * 2. 使用write系统调用写入
 * 3. 关闭文件描述符
 *
 * C++语法说明：
 * - ssize_t write(int fd, const void* buf, size_t count)：
 *   Unix写文件系统调用
 *   - fd：文件描述符
 *   - buf：数据缓冲区指针
 *   - count：写入字节数
 *   - 返回：实际写入字节数，-1表示错误
 * - content.size()：返回字符串字节数
 */
bool SetStringToASCIIFile(const std::string &content,
                          const std::string &file_name) {
  int fd = open(file_name.c_str(), O_WRONLY | O_CREAT | O_TRUNC, S_IRWXU);
  /**
   * @brief 打开文件
   */
  if (fd < 0) {
    AERROR << "Unable to open file " << file_name << " to write.";
    return false;
  }

  /**
   * @brief 写入字符串数据到文件
   *
   * write系统调用：
   * - fd：文件描述符
   * - content.c_str()：获取C字符串指针
   * - content.size()：获取字符串大小
   *
   * 返回：实际写入的字节数
   */
  ssize_t bytes_written = write(fd, content.c_str(), content.size());

  if (bytes_written < 0) {
    /**
     * @brief 检查写入是否成功
     *
     * bytes_written < 0 表示写入错误
     */
    AERROR << "Failed to write to file.";
    close(fd);  /**
     * @brief 确保文件描述符被关闭
     *
     * 即使发生错误也要关闭文件描述符
     * 避免资源泄漏
     */
    return false;
  }

  close(fd);  /**
   * @brief 关闭文件描述符
   *
   * 写入成功后也要关闭文件
   */

  return true;
}

/**
 * @brief 从ASCII格式文件读取Protobuf消息
 *
 * @param file_name 文件名
 * @param message Protobuf消息指针（输出参数）
 * @return bool 是否读取成功
 *
 * 功能说明：
 * 从ASCII格式的prototxt文件解析Protobuf消息
 *
 * 算法流程：
 * 1. 打开文件
 * 2. 创建FileInputStream
 * 3. 使用TextFormat::Parse解析
 * 4. 关闭输入流和文件描述符
 *
 * C++语法说明：
 * - O_RDONLY：只读模式
 * - ZeroCopyInputStream：零拷贝输入流
 * - TextFormat::Parse：解析ASCII格式文本
 */
bool GetProtoFromASCIIFile(const std::string &file_name,
                           google::protobuf::Message *message) {
  using google::protobuf::TextFormat;
  /**< @brief 使用TextFormat */
  using google::protobuf::io::FileInputStream;
  /**< @brief 文件输入流 */
  using google::protobuf::io::ZeroCopyInputStream;
  /**< @brief 零拷贝输入流 */

  int file_descriptor = open(file_name.c_str(), O_RDONLY);
  /**
   * @brief 以只读模式打开文件
   *
   * O_RDONLY：只读模式
   */
  if (file_descriptor < 0) {
    AERROR << "Failed to open file " << file_name << " in text mode.";
    /**
     * @brief 打开失败
     */
    return false;
  }

  ZeroCopyInputStream *input = new FileInputStream(file_descriptor);
  /**
   * @brief 创建零拷贝输入流
   */

  bool success = TextFormat::Parse(input, message);
  /**
   * @brief 解析文件内容
   *
   * TextFormat::Parse：
   * - 从ZeroCopyInputStream读取
   * - 解析为Protobuf消息
   * - 结果存入message指针
   */

  if (!success) {
    /**
     * @brief 解析失败
     */
    AERROR << "Failed to parse file " << file_name << " as text proto.";
  }

  delete input;
  /**
   * @brief 释放输入流内存
   */
  close(file_descriptor);
  /**
   * @brief 关闭文件描述符
   */
  return success;
}

/**
 * @brief 将Protobuf消息写入二进制格式文件
 *
 * @param message Protobuf消息引用
 * @param file_name 文件名
 * @return bool 是否写入成功
 *
 * 功能说明：
 * 以二进制格式序列化Protobuf消息，效率高但不可读
 *
 * 算法流程：
 * 1. 打开文件流（二进制模式）
 * 2. 调用SerializeToOstream
 *
 * C++语法说明：
 * - std::fstream：文件流类
 * - std::ios::out：输出模式
 * - std::ios::trunc：截断模式
 * - std::ios::binary：二进制模式
 * - SerializeToOstream：Protobuf二进制序列化方法
 */
bool SetProtoToBinaryFile(const google::protobuf::Message &message,
                          const std::string &file_name) {
  std::fstream output(file_name,
                      std::ios::out | std::ios::trunc | std::ios::binary);
  /**
   * @brief 打开二进制文件流
   *
   * std::fstream构造参数：
   * - file_name：文件名
   * - std::ios::out：输出模式
   * - std::ios::trunc：截断已有文件
   * - std::ios::binary：二进制模式（不进行换行转换）
   *
   * 注意：二进制模式不使用，文件流会直接写入原始字节
   */

  return message.SerializeToOstream(&output);
  /**
   * @brief 序列化消息到输出流
   *
   * SerializeToOstream：
   * - Protobuf内置方法
   * - 将消息以二进制格式序列化
   * - 输出到ostream
   *
   * 返回：是否成功
   */
}

/**
 * @brief 从二进制格式文件读取Protobuf消息
 *
 * @param file_name 文件名
 * @param message Protobuf消息指针（输出参数）
 * @return bool 是否读取成功
 *
 * 功能说明：
 * 从二进制格式的.bin文件解析Protobuf消息
 *
 * 算法流程：
 * 1. 打开文件流（二进制模式）
 * 2. 检查流状态
 * 3. 调用ParseFromIstream解析
 *
 * C++语法说明：
 * - std::ios::in：输入模式
 * - input.good()：检查流状态是否正常
 * - ParseFromIstream：Protobuf二进制反序列化方法
 */
bool GetProtoFromBinaryFile(const std::string &file_name,
                            google::protobuf::Message *message) {
  std::fstream input(file_name, std::ios::in | std::ios::binary);
  /**
   * @brief 以二进制只读模式打开文件
   */

  if (!input.good()) {
    /**
     * @brief 检查文件流是否正常
     *
     * good()返回true如果：
     * - 文件成功打开
     * - 未到达文件尾
     * - 未发生错误
     */
    AERROR << "Failed to open file " << file_name << " in binary mode.";
    return false;
  }

  if (!message->ParseFromIstream(&input)) {
    /**
     * @brief 解析二进制文件
     *
     * ParseFromIstream：
     * - 从istream读取二进制数据
     * - 反序列化为Protobuf消息
     */
    AERROR << "Failed to parse file " << file_name << " as binary proto.";
    return false;
  }

  return true;
}

/**
 * @brief 从文件读取Protobuf消息（自动检测格式）
 *
 * @param file_name 文件名
 * @param message Protobuf消息指针（输出参数）
 * @return bool 是否读取成功
 *
 * 功能说明：
 * 自动检测文件格式（ASCII/二进制），优先尝试二进制解析
 *
 * 算法流程：
 * 1. 检查文件是否存在
 * 2. 如果扩展名是.bin，优先尝试二进制格式
 * 3. 如果二进制失败，尝试ASCII格式
 * 4. 否则先尝试ASCII，失败后尝试二进制
 *
 * C++语法说明：
 * - std::equal：比较两个范围是否相等
 * - kBinExt.rbegin()/rend()：反向迭代器
 * - file_name.rbegin()：从后向前比较
 */
bool GetProtoFromFile(const std::string &file_name,
                      google::protobuf::Message *message) {
  if (!PathExists(file_name)) {
    /**
     * @brief 检查文件是否存在
     */
    AERROR << "File [" << file_name << "] does not exist! ";
    return false;
  }

  /**
   * @brief 如果扩展名是.bin，优先尝试二进制解析
   *
   * 使用后缀匹配判断文件类型
   */
  static const std::string kBinExt = ".bin";
  if (std::equal(kBinExt.rbegin(), kBinExt.rend(), file_name.rbegin())) {
    /**
     * @brief std::equal比较算法
     *
     * 比较两个范围是否相等：
     * - kBinExt.rbegin()到rend()：.bin的反向范围
     * - file_name.rbegin()：文件名的反向迭代器起点
     *
     * 如果文件以.bin结尾
     */
    return GetProtoFromBinaryFile(file_name, message) ||
           GetProtoFromASCIIFile(file_name, message);
    /**
     * @brief 二进制失败后尝试ASCII
     */
  }

  return GetProtoFromASCIIFile(file_name, message) ||
         GetProtoFromBinaryFile(file_name, message);
  /**
   * @brief 默认先尝试ASCII，失败后尝试二进制
   */
}

/**
 * @brief 从JSON格式文件读取Protobuf消息
 *
 * @param file_name 文件名
 * @param message Protobuf消息指针（输出参数）
 * @return bool 是否读取成功
 *
 * 功能说明：
 * 使用nlohmann_json解析JSON文件，转换为Protobuf消息
 *
 * 算法流程：
 * 1. 打开JSON文件
 * 2. 使用nlohmann_json解析
 * 3. 转换为JSON字符串
 * 4. 使用JsonStringToMessage转换为Protobuf
 *
 * C++语法说明：
 * - std::ifstream：输入文件流
 * - ifs.is_open()：检查文件是否成功打开
 * - nlohmann::json：JSON解析库
 * - JsonStringToMessage：Protobuf JSON解析工具
 * - JsonParseOptions：解析选项
 * - options.ignore_unknown_fields：忽略未知字段
 */
bool GetProtoFromJsonFile(const std::string &file_name,
                          google::protobuf::Message *message) {
  using google::protobuf::util::JsonParseOptions;
  /**< @brief JSON解析选项 */
  using google::protobuf::util::JsonStringToMessage;
  /**< @brief JSON字符串转消息工具 */

  std::ifstream ifs(file_name);
  /**
   * @brief 打开JSON文件
   *
   * std::ifstream：输入文件流
   */
  if (!ifs.is_open()) {
    /**
     * @brief 检查文件是否成功打开
     */
    AERROR << "Failed to open file " << file_name;
    return false;
  }

  nlohmann::json Json;
  /**
   * @brief 创建JSON对象
   *
   * nlohmann::json：
   * C++ JSON库，用于解析和生成JSON
   */
  ifs >> Json;
  /**
   * @brief 从文件流读取JSON
   *
   * operator>>：重载的流提取运算符
   * 从ifs读取JSON数据到Json对象
   */
  ifs.close();
  /**
   * @brief 关闭文件流
   */

  JsonParseOptions options;
  /**
   * @brief 创建解析选项
   */
  options.ignore_unknown_fields = true;
  /**
   * @brief 设置忽略未知字段
   *
   * 允许JSON中有多余字段而不报错
   */

  google::protobuf::util::Status dump_status;
  /**
   * @brief 解析状态
   *
   * google::protobuf::util::Status：
   * 表示操作结果状态
   */
  return (JsonStringToMessage(Json.dump(), message, options).ok());
  /**
   * @brief 转换为Protobuf消息
   *
   * Json.dump()：将JSON对象转为字符串
   * JsonStringToMessage：解析JSON字符串为Protobuf
   * .ok()：检查转换是否成功
   */
}

/**
 * @brief 读取文件内容到字符串
 *
 * @param file_name 文件名
 * @param content 字符串指针（输出参数）
 * @return bool 是否读取成功
 *
 * 功能说明：
 * 读取整个文件内容到字符串
 *
 * 算法流程：
 * 1. 打开文件
 * 2. 使用stringstream读取全部内容
 * 3. 转换为字符串
 *
 * C++语法说明：
 * - std::ifstream：输入文件流
 * - std::stringstream：字符串流，用于拼接
 * - fin.rdbuf()：获取文件流缓冲区
 * - str_stream.str()：获取字符串流的内容
 */
bool GetContent(const std::string &file_name, std::string *content) {
  std::ifstream fin(file_name);
  /**
   * @brief 打开文件
   */
  if (!fin) {
    /**
     * @brief 检查文件是否打开成功
     */
    return false;
  }

  std::stringstream str_stream;
  /**
   * @brief 创建字符串流
   *
   * std::stringstream：
   * 可用于字符串的读写操作
   */
  str_stream << fin.rdbuf();
  /**
   * @brief 读取文件流缓冲区内容
   *
   * rdbuf()：返回流缓冲区的指针
   * operator<<：将缓冲区内容写入stringstream
   * 这会读取整个文件内容
   */

  *content = str_stream.str();
  /**
   * @brief 获取字符串内容
   *
   * str()：返回stringstream的内容字符串
   * 赋值给输出参数
   */

  return true;
}

/**
 * @brief 获取绝对路径
 *
 * @param prefix 前缀路径
 * @param relative_path 相对路径
 * @return std::string 绝对路径
 *
 * 功能说明：
 * 将相对路径与前缀结合生成绝对路径
 *
 * 算法流程：
 * 1. 如果relative_path为空，返回prefix
 * 2. 如果prefix为空或relative_path已是绝对路径，返回relative_path
 * 3. 如果prefix以/结尾，直接拼接
 * 4. 否则prefix + "/" + relative_path
 *
 * C++语法说明：
 * - string.front()：获取首字符引用
 * - string.back()：获取尾字符引用
 */
std::string GetAbsolutePath(const std::string &prefix,
                            const std::string &relative_path) {
  if (relative_path.empty()) {
    /**
     * @brief 如果相对路径为空
     */
    return prefix;
  }

  /**
   * @brief 如果前缀为空或相对路径已是绝对路径
   */
  if (prefix.empty() || relative_path.front() == '/') {
    /**
     * @brief relative_path.front()：获取首字符
     */
    return relative_path;
  }

  if (prefix.back() == '/') {
    /**
     * @brief 如果前缀以/结尾
     *
     * prefix.back()：获取最后一个字符
     */
    return prefix + relative_path;
  }

  return prefix + "/" + relative_path;
  /**
   * @brief 需要在中间添加/
   */
}

/**
 * @brief 检查路径是否存在
 *
 * @param path 路径
 * @return bool 是否存在
 *
 * 功能说明：
 * 使用stat系统调用检查文件/目录是否存在
 *
 * C++语法说明：
 * - struct stat：文件状态结构体
 * - stat(path, &info)：获取文件状态
 * - info.st_mode：文件类型和权限
 *
 * struct stat成员：
 * - st_mode：文件类型和权限
 * - st_size：文件大小
 * - st_atime/mtime/ctime：访问/修改/状态改变时间
 */
bool PathExists(const std::string &path) {
  struct stat info;
  /**
   * @brief 文件状态结构体
   *
   * struct stat：
   * 包含文件的各种属性信息
   */
  return stat(path.c_str(), &info) == 0;
  /**
   * @brief 检查stat调用是否成功
   *
   * stat返回0表示成功，-1表示失败
   * &info：结构体引用，用于接收结果
   */
}

/**
 * @brief 检查路径是否为绝对路径
 *
 * @param path 路径
 * @return bool 是否为绝对路径
 *
 * 功能说明：
 * 绝对路径以/开头
 *
 * C++语法说明：
 * - path.front()：获取首字符
 */
bool PathIsAbsolute(const std::string &path) {
  if (path.empty()) {
    /**
     * @brief 空字符串不是绝对路径
     */
    return false;
  }

  return path.front() == '/';
  /**
   * @brief 如果首字符是/，则是绝对路径
   */
}

/**
 * @brief 检查目录是否存在
 *
 * @param directory_path 目录路径
 * @return bool 目录是否存在
 *
 * 功能说明：
 * 检查路径是否存在且是一个目录
 *
 * C++语法说明：
 * - S_IFDIR：目录类型标志
 * - st_mode & S_IFDIR：按位与判断是否是目录
 * - S_ISDIR(st_mode)：判断是否目录的宏
 */
bool DirectoryExists(const std::string &directory_path) {
  struct stat info;
  /**
   * @brief 文件状态结构体
   */
  return stat(directory_path.c_str(), &info) == 0 && (info.st_mode & S_IFDIR);
  /**
   * @brief 两个条件：
   * 1. stat调用成功（路径存在）
   * 2. st_mode与S_IFDIR按位与结果非零（是目录）
   *
   * 注意：S_IFDIR只是标志位，需要用&操作来测试
   */
}

/**
 * @brief 使用通配符模式查找文件
 *
 * @param pattern 通配符模式（如"/path/*.txt"）
 * @return std::vector<std::string> 匹配的文件路径列表
 *
 * 功能说明：
 * 使用glob函数查找匹配通配符的所有文件
 *
 * 算法流程：
 * 1. 调用glob查找匹配
 * 2. 遍历结果列表
 * 3. 释放glob资源
 *
 * C++语法说明：
 * - glob_t：glob操作结果结构体
 * - globs.gl_pathc：匹配的文件数量
 * - globs.gl_pathv：匹配的文件路径数组指针
 * - GLOB_TILDE：支持~作为home目录扩展
 * - globfree：释放glob分配的资源
 */
std::vector<std::string> Glob(const std::string &pattern) {
  glob_t globs = {};
  /**
   * @brief 初始化glob结果结构体
   *
   * ={}：零初始化，所有成员设为0
   */

  std::vector<std::string> results;
  /**
   * @brief 存储结果的向量
   */

  if (glob(pattern.c_str(), GLOB_TILDE, nullptr, &globs) == 0) {
    /**
     * @brief 执行glob匹配
     *
     * glob函数：
     * - pattern.c_str()：要匹配的模式
     * - GLOB_TILDE：标志位，展开~为home目录
     * - nullptr：错误回调函数（可选）
     * - &globs：结果结构体引用
     *
     * 返回0表示成功，非0表示失败
     */

    for (size_t i = 0; i < globs.gl_pathc; ++i) {
      /**
       * @brief 遍历所有匹配的文件
       *
       * globs.gl_pathc：匹配的文件数量
       * globs.gl_pathv：指向路径数组的指针
       */
      results.emplace_back(globs.gl_pathv[i]);
      /**
       * @brief 添加到结果列表
       *
       * emplace_back：原地构造字符串
       * globs.gl_pathv[i]：第i个匹配路径
       */
    }
  }

  globfree(&globs);
  /**
   * @brief 释放glob资源
   *
   * 必须调用globfree释放glob分配的内存
   */

  return results;
}

/**
 * @brief 复制文件
 *
 * @param from 源文件路径
 * @param to 目标文件路径
 * @return bool 是否复制成功
 *
 * 功能说明：
 * 将源文件复制到目标路径
 * 如果源文件无法打开，使用系统cp命令复制
 *
 * 算法流程：
 * 1. 尝试以二进制流打开源文件
 * 2. 如果打开失败，使用cp命令
 * 3. 否则以二进制流写入目标
 *
 * C++语法说明：
 * - std::ios::binary：二进制模式
 * - src.rdbuf()：获取源文件流缓冲区
 * - dst << src.rdbuf()：复制整个缓冲区内容
 * - std::system：执行shell命令
 */
bool CopyFile(const std::string &from, const std::string &to) {
  std::ifstream src(from, std::ios::binary);
  /**
   * @brief 以二进制模式打开源文件
   */
  if (!src) {
    /**
     * @brief 源文件打开失败
     *
     * 可能是因为源是目录或其他特殊文件
     */
    AWARN << "Source path could not be normally opened: " << from;
    std::string command = "cp -r " + from + " " + to;
    /**
     * @brief 构建cp命令
     *
     * 使用系统cp命令进行复制
     */
    ADEBUG << command;
    const int ret = std::system(command.c_str());
    /**
     * @brief 执行shell命令
     *
     * std::system：
     * - 在shell中执行命令
     * - 返回命令的退出码
     */
    if (ret == 0) {
      ADEBUG << "Copy success, command returns " << ret;
      return true;
    } else {
      ADEBUG << "Copy error, command returns " << ret;
      return false;
    }
  }

  std::ofstream dst(to, std::ios::binary);
  /**
   * @brief 以二进制模式打开目标文件
   */
  if (!dst) {
    /**
     * @brief 目标文件打开失败
     */
    AERROR << "Target path is not writable: " << to;
    return false;
  }

  dst << src.rdbuf();
  /**
   * @brief 复制文件内容
   *
   * rdbuf()获取源流缓冲区
   * operator<<输出到目标流
   * 这会复制整个文件内容
   */

  return true;
}

/**
 * @brief 复制目录
 *
 * @param from 源目录路径
 * @param to 目标目录路径
 * @return bool 是否复制成功
 *
 * 功能说明：
 * 递归复制整个目录树
 *
 * 算法流程：
 * 1. 打开源目录
 * 2. 创建目标目录（EnsureDirectory）
 * 3. 遍历目录项
 * 4. 对每个子项，递归复制
 *
 * C++语法说明：
 * - DIR：目录流类型
 * - opendir/readdir/closedir：目录操作函数
 * - dirent->d_type：目录项类型
 * - DT_DIR：目录类型标志
 */
bool CopyDir(const std::string &from, const std::string &to) {
  DIR *directory = opendir(from.c_str());
  /**
   * @brief 打开源目录
   *
   * opendir：返回DIR*指针，失败返回nullptr
   */
  if (directory == nullptr) {
    /**
     * @brief 打开失败
     */
    AERROR << "Cannot open directory " << from;
    return false;
  }

  bool ret = true;
  /**
   * @brief 复制结果标记
   */
  if (EnsureDirectory(to)) {
    /**
     * @brief 确保目标目录存在
     */
    struct dirent *entry;
    /**
     * @brief 目录项结构体指针
     */
    while ((entry = readdir(directory)) != nullptr) {
      /**
       * @brief 遍历目录中的所有项
       *
       * readdir返回目录中的下一个条目
       * nullptr表示遍历结束
       */

      /**
       * @brief 跳过.和..目录
       *
       * .表示当前目录
       * ..表示父目录
       */
      if (!strcmp(entry->d_name, ".") || !strcmp(entry->d_name, "..")) {
        continue;
      }

      const std::string sub_path_from = from + "/" + entry->d_name;
      const std::string sub_path_to = to + "/" + entry->d_name;
      /**
       * @brief 构建子项的完整路径
       */

      if (entry->d_type == DT_DIR) {
        /**
         * @brief 如果是目录
         *
         * d_type：目录项类型
         * DT_DIR：目录类型标志
         */
        ret &= CopyDir(sub_path_from, sub_path_to);
        /**
         * @brief 递归复制子目录
         */
      } else {
        ret &= CopyFile(sub_path_from, sub_path_to);
        /**
         * @brief 复制文件
         */
      }
    }
  } else {
    AERROR << "Cannot create target directory " << to;
    ret = false;
  }

  closedir(directory);
  /**
   * @brief 关闭目录流
   *
   * 与opendir配对
   */
  return ret;
}

/**
 * @brief 复制文件或目录
 *
 * @param from 源路径
 * @param to 目标路径
 * @return bool 是否复制成功
 *
 * 功能说明：
 * 根据源路径类型选择复制文件或目录
 *
 * C++语法说明：
 * - DirectoryExists：判断是否是目录
 * - 条件运算符?: 选择复制函数
 */
bool Copy(const std::string &from, const std::string &to) {
  return DirectoryExists(from) ? CopyDir(from, to) : CopyFile(from, to);
  /**
   * @brief 三元运算符选择
   *
   * 如果from是目录，调用CopyDir
   * 否则调用CopyFile
   */
}

/**
 * @brief 确保目录存在（递归创建）
 *
 * @param directory_path 目录路径
 * @return bool 是否成功
 *
 * 功能说明：
 * 递归创建目录树，类似于mkdir -p
 *
 * 算法流程：
 * 1. 遍历路径字符串
 * 2. 遇到/时，截断并尝试创建该子目录
 * 3. 忽略EEXIST错误（目录已存在）
 * 4. 最后创建完整目录
 *
 * C++语法说明：
 * - mkdir(path, mode)：创建目录
 * - errno：系统错误码
 * - EEXIST：文件已存在错误
 * - path.c_str()：将string转为C字符串
 */
bool EnsureDirectory(const std::string &directory_path) {
  std::string path = directory_path;
  /**
   * @brief 复制路径字符串
   *
   * 需要修改，所以复制一份
   */
  for (size_t i = 1; i < directory_path.size(); ++i) {
    /**
     * @brief 从第二个字符开始遍历
     *
     * 跳过第一个字符（通常是/）
     */
    if (directory_path[i] == '/') {
      /**
       * @brief 遇到路径分隔符
       */

      /**
       * @brief 创建临时路径视图
       *
       * 将/替换为\0，创建C字符串
       * 这样path.c_str()只包含到该位置
       */
      path[i] = 0;

      if (mkdir(path.c_str(), S_IRWXU) != 0) {
        /**
         * @brief 尝试创建目录
         *
         * mkdir：创建目录
         * S_IRWXU：所有者读写执行权限
         *
         * 注意：如果目录已存在会返回EEXIST
         */
        if (errno != EEXIST) {
          /**
           * @brief 如果不是已存在错误
           *
           * errno：系统错误码变量
           * EEXIST：文件/目录已存在
           */
          return false;
        }
      }

      /**
       * @brief 恢复临时视图
       */
      path[i] = '/';
    }
  }

  /**
   * @brief 创建最后的完整目录
   */
  if (mkdir(path.c_str(), S_IRWXU) != 0) {
    if (errno != EEXIST) {
      return false;
    }
  }

  return true;
}

/**
 * @brief 删除目录中的所有文件
 *
 * @param directory_path 目录路径
 * @return bool 是否成功
 *
 * 功能说明：
 * 删除目录下所有文件，但保留子目录
 *
 * 算法流程：
 * 1. 打开目录
 * 2. 遍历所有目录项
 * 3. 跳过.和..
 * 4. 使用unlink删除文件
 *
 * C++语法说明：
 * - unlink：删除文件
 * - strerror：将errno转为错误消息字符串
 */
bool RemoveAllFiles(const std::string &directory_path) {
  DIR *directory = opendir(directory_path.c_str());
  /**
   * @brief 打开目录
   */
  if (directory == nullptr) {
    AERROR << "Cannot open directory " << directory_path;
    return false;
  }

  struct dirent *file;
  /**
   * @brief 目录项指针
   */
  while ((file = readdir(directory)) != nullptr) {
    /**
     * @brief 遍历目录项
     */

    /**
     * @brief 跳过.和..
     */
    if (!strcmp(file->d_name, ".") || !strcmp(file->d_name, "..")) {
      continue;
    }

    /**
     * @brief 构建文件完整路径
     */
    std::string file_path = directory_path + "/" + file->d_name;

    if (unlink(file_path.c_str()) < 0) {
      /**
       * @brief 删除文件
       *
       * unlink：删除文件（减少链接计数）
       * 返回-1表示失败
       */
      AERROR << "Fail to remove file " << file_path << ": " << strerror(errno);
      /**
       * @brief 输出错误信息
       *
       * strerror：将错误码转为可读字符串
       */
      closedir(directory);
      return false;
    }
  }

  closedir(directory);
  return true;
}

/**
 * @brief 列出目录中的子路径
 *
 * @param directory_path 目录路径
 * @param d_type 要筛选的目录项类型
 * @return std::vector<std::string> 匹配的子路径列表
 *
 * 功能说明：
 * 列出目录下指定类型的条目（文件或目录）
 *
 * 算法流程：
 * 1. 打开目录
 * 2. 遍历目录项
 * 3. 按类型筛选
 * 4. 跳过.和..
 *
 * C++语法说明：
 * - d_type：目录项类型
 * - DT_DIR/DT_REG/DT_UNKNOWN：目录/普通文件/未知
 */
std::vector<std::string> ListSubPaths(const std::string &directory_path,
                                      const unsigned char d_type) {
  std::vector<std::string> result;
  /**
   * @brief 结果列表
   */
  DIR *directory = opendir(directory_path.c_str());
  /**
   * @brief 打开目录
   */
  if (directory == nullptr) {
    AERROR << "Cannot open directory " << directory_path;
    return result;
  }

  struct dirent *entry;
  /**
   * @brief 目录项指针
   */
  while ((entry = readdir(directory)) != nullptr) {
    /**
     * @brief 遍历目录
     */

    /**
     * @brief 按类型筛选并跳过.和..
     */
    if (entry->d_type == d_type && strcmp(entry->d_name, ".") != 0 &&
        strcmp(entry->d_name, "..") != 0) {
      result.emplace_back(entry->d_name);
      /**
       * @brief 添加到结果列表
       */
    }
  }

  closedir(directory);
  return result;
}

/**
 * @brief 按模式查找路径
 *
 * @param base_path 基础路径
 * @param patt 匹配模式
 * @param d_type 目录项类型
 * @param recursive 是否递归
 * @param result_list 结果列表（输出参数）
 * @return size_t 找到的数量
 *
 * 功能说明：
 * 在目录树中递归查找匹配的文件/目录
 *
 * 算法流程：
 * 1. 打开目录
 * 2. 遍历目录项
 * 3. 如果匹配模式且类型正确，添加到结果
 * 4. 如果是递归且是目录，继续递归
 *
 * C++语法说明：
 * - recursive：递归标志
 * - result_cnt：计数器
 */
size_t FindPathByPattern(const std::string &base_path, const std::string &patt,
                         const unsigned char d_type, const bool recursive,
                         std::vector<std::string> *result_list) {
  DIR *directory = opendir(base_path.c_str());
  /**
   * @brief 打开目录
   */
  size_t result_cnt = 0;
  /**
   * @brief 结果计数
   */
  if (directory == nullptr) {
    AWARN << "cannot open directory " << base_path;
    return result_cnt;
  }

  struct dirent *entry;
  /**
   * @brief 目录项指针
   */
  for (entry = readdir(directory); entry != nullptr;
       entry = readdir(directory)) {
    /**
     * @brief for循环遍历目录
     */

    std::string entry_path = base_path + "/" + std::string(entry->d_name);
    /**
     * @brief 构建完整路径
     */

    /**
     * @brief 跳过.和..
     */
    if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0) {
      /**
       * @brief TODO：支持正则或glob模式
       */
      if ((patt == "" || strcmp(entry->d_name, patt.c_str()) == 0) &&
          entry->d_type == d_type) {
        /**
         * @brief 匹配模式
         *
         * 条件1：patt为空（匹配所有）
         * 条件2：patt与名称相等
         * 条件3：类型匹配
         */
        result_list->emplace_back(entry_path);
        /**
         * @brief 添加到结果列表
         */
        ++result_cnt;
      }

      if (recursive && (entry->d_type == DT_DIR)) {
        /**
         * @brief 递归处理子目录
         */
        result_cnt +=
            FindPathByPattern(entry_path, patt, d_type, recursive, result_list);
        /**
         * @brief 累加计数
         */
      }
    }
  }

  closedir(directory);
  return result_cnt;
}

/**
 * @brief 获取路径的目录部分
 *
 * @param path 文件路径
 * @return std::string 目录路径
 *
 * 功能说明：
 * 从完整路径中提取目录部分
 *
 * 算法流程：
 * 1. 从后向前查找最后一个/
 * 2. 如果没找到，返回"."
 * 3. 否则返回/前的部分
 *
 * C++语法说明：
 * - rfind：从后向前查找
 * - std::string::npos：未找到的返回值
 * - substr：字符串截取
 */
std::string GetDirName(const std::string &path) {
  std::string::size_type end = path.rfind('/');
  /**
   * @brief 从后向前查找/
   *
   * rfind：reverse find，从字符串末尾向前查找
   * 返回找到的位置，未找到返回npos
   */
  if (end == std::string::npos) {
    /**
     * @brief 如果没找到
     *
     * 返回当前目录
     */
    return ".";
  }

  return path.substr(0, end);
  /**
   * @brief 返回0到end之间的子串
   *
   * substr(pos, count)：从pos开始，长度为count
   */
}

/**
 * @brief 获取文件名
 *
 * @param path 文件路径
 * @param remove_extension 是否移除扩展名
 * @return std::string 文件名
 *
 * 功能说明：
 * 从路径中提取文件名，可选移除扩展名
 *
 * 算法流程：
 * 1. 找到最后一个/
 * 2. 文件名从/后开始
 * 3. 如果需要，移除最后一个.及其后的扩展名
 *
 * C++语法说明：
 * - rfind：从后向前查找
 * - start++：移动到/的下一个字符
 */
std::string GetFileName(const std::string &path, const bool remove_extension) {
  std::string::size_type start = path.rfind('/');
  /**
   * @brief 找最后一个/
   */
  if (start == std::string::npos) {
    /**
     * @brief 如果没有/
     *
     * 文件名就是整个path
     */
    start = 0;
  } else {
    /**
     * @brief 移动到/的下一个字符
     */
    ++start;
  }

  std::string::size_type end = std::string::npos;
  /**
   * @brief 扩展名结束位置
   */
  if (remove_extension) {
    /**
     * @brief 需要移除扩展名
     */
    end = path.rfind('.');
    /**
     * @brief 从后向前找最后一个.
     */
    if (end != std::string::npos && end < start) {
      /**
       * @brief .在/之前，不是扩展名
       *
       * 例如：/path/to.file/config
       * 最后一个.在config中，不是扩展名
       */
      end = std::string::npos;
    }
  }

  const auto len = (end != std::string::npos) ? end - start : end;
  /**
   * @brief 计算文件名长度
   *
   * 三元运算符：如果有扩展名，长度为end-start
   */
  return path.substr(start, len);
  /**
   * @brief 返回文件名
   *
   * 如果len是npos，substr会到字符串末尾
   */
}

/**
 * @brief 获取文件路径（支持环境变量）
 *
 * @param path 文件路径
 * @param env_var 环境变量名
 * @param file_path 输出：完整文件路径
 * @return bool 是否找到
 *
 * 功能说明：
 * 如果path是相对路径，从环境变量指定的目录列表中查找
 *
 * 算法流程：
 * 1. 如果path为空，返回false
 * 2. 如果path是绝对路径且存在，返回true
 * 3. 如果path是相对路径且存在，返回true
 * 4. 如果path以.开头但不存在，返回relative_path_exists
 * 5. 从环境变量中搜索
 *
 * C++语法说明：
 * - std::getenv：获取环境变量值
 * - path.front()：获取首字符
 */
bool GetFilePathWithEnv(const std::string &path, const std::string &env_var,
                        std::string *file_path) {
  if (path.empty()) {
    return false;
  }

  if (PathIsAbsolute(path)) {
    /**
     * @brief 如果是绝对路径
     */
    *file_path = path;
    return PathExists(path);
  }

  bool relative_path_exists = false;
  if (PathExists(path)) {
    /**
     * @brief 如果相对路径存在
     */
    *file_path = path;
    relative_path_exists = true;
  }

  if (path.front() == '.') {
    /**
     * @brief 如果是相对路径（以.开头）但不存在
     */
    return relative_path_exists;
  }

  const char *var = std::getenv(env_var.c_str());
  /**
   * @brief 获取环境变量值
   *
   * std::getenv：返回环境变量字符串指针
   * 失败返回nullptr
   */
  if (var == nullptr) {
    AWARN << "GetFilePathWithEnv: env " << env_var << " not found.";
    return relative_path_exists;
  }

  std::string env_path = std::string(var);
  /**
   * @brief 转换为string
   */

  /**
   * @brief 从环境变量指定的路径列表中搜索
   *
   * 路径列表用:分隔（Unix风格）
   */
  size_t begin = 0;
  size_t index;
  do {
    index = env_path.find(':', begin);
    /**
     * @brief 查找分隔符:
     */
    auto p = env_path.substr(begin, index - begin);
    /**
     * @brief 提取单个路径
     */
    if (p.empty()) {
      continue;
    }

    if (p.back() != '/') {
      /**
       * @brief 路径不以/结尾，添加/
       */
      p += '/' + path;
    } else {
      p += path;
    }

    if (PathExists(p)) {
      /**
       * @brief 如果路径存在
       */
      *file_path = p;
      return true;
    }

    begin = index + 1;
    /**
     * @brief 移动到下一个路径
     */
  } while (index != std::string::npos);

  return relative_path_exists;
}

/**
 * @brief 获取当前工作目录
 *
 * @return std::string 当前目录路径
 *
 * 功能说明：
 * 获取当前进程的工作目录
 *
 * C++语法说明：
 * - getcwd：获取当前工作目录的C函数
 * - PATH_MAX：系统定义的最大路径长度
 * - sizeof(tmp)：获取数组大小
 * - ternary operator：条件运算符
 */
std::string GetCurrentPath() {
  char tmp[PATH_MAX];
  /**
   * @brief 临时缓冲区
   *
   * PATH_MAX：系统支持的最大路径长度
   */
  return getcwd(tmp, sizeof(tmp)) ? std::string(tmp) : std::string("");
  /**
   * @brief 获取并返回当前目录
   *
   * getcwd：
   * - 成功返回tmp，条件为true
   * - 失败返回nullptr，条件为false
   * - 三元运算符选择返回的值
   */
}

/**
 * @brief 获取文件类型
 *
 * @param filename 文件名
 * @param type 类型输出参数
 * @return bool 是否成功
 *
 * 功能说明：
 * 判断文件是普通文件还是目录
 *
 * C++语法说明：
 * - lstat：获取文件状态（与stat类似，但不跟随符号链接）
 * - S_ISDIR/S_ISREG：判断文件类型的宏
 */
bool GetType(const string &filename, FileType *type) {
  struct stat stat_buf;
  /**
   * @brief 文件状态缓冲区
   */
  if (lstat(filename.c_str(), &stat_buf) != 0) {
    /**
     * @brief 获取文件状态失败
     */
    return false;
  }

  if (S_ISDIR(stat_buf.st_mode) != 0) {
    /**
     * @brief 判断是否是目录
     *
     * S_ISDIR：判断st_mode是否目录标志
     */
    *type = TYPE_DIR;
  } else if (S_ISREG(stat_buf.st_mode) != 0) {
    /**
     * @brief 判断是否是普通文件
     *
     * S_ISREG：判断st_mode是否普通文件标志
     */
    *type = TYPE_FILE;
  } else {
    AWARN << "failed to get type: " << filename;
    return false;
  }

  return true;
}

/**
 * @brief 删除文件或目录
 *
 * @param filename 文件名
 * @return bool 是否成功
 *
 * 功能说明：
 * 删除文件或递归删除目录树
 *
 * 算法流程：
 * 1. 检查文件是否存在
 * 2. 获取文件类型
 * 3. 如果是文件，直接删除
 * 4. 如果是目录，递归删除内容后删除自己
 *
 * C++语法说明：
 * - remove：删除文件或空目录
 * - DeleteFile：递归删除目录
 */
bool DeleteFile(const string &filename) {
  if (!PathExists(filename)) {
    /**
     * @brief 文件不存在，视为成功
     */
    return true;
  }

  FileType type;
  if (!GetType(filename, &type)) {
    /**
     * @brief 获取类型失败
     */
    return false;
  }

  if (type == TYPE_FILE) {
    /**
     * @brief 如果是文件
     */
    if (remove(filename.c_str()) != 0) {
      /**
       * @brief 删除文件
       *
       * remove：删除文件（非目录）
       */
      AERROR << "failed to remove file: " << filename;
      return false;
    }
    return true;
  }

  DIR *dir = opendir(filename.c_str());
  /**
   * @brief 打开目录
   */
  if (dir == nullptr) {
    AWARN << "failed to opendir: " << filename;
    return false;
  }

  dirent *dir_info = nullptr;
  /**
   * @brief 目录项指针
   */
  while ((dir_info = readdir(dir)) != nullptr) {
    /**
     * @brief 遍历目录项
     */
    if (strcmp(dir_info->d_name, ".") == 0 ||
        strcmp(dir_info->d_name, "..") == 0) {
      /**
       * @brief 跳过.和..
       */
      continue;
    }

    string temp_file = filename + "/" + string(dir_info->d_name);
    /**
     * @brief 构建子项路径
     */
    FileType temp_type;
    if (!GetType(temp_file, &temp_type)) {
      AWARN << "failed to get file type: " << temp_file;
      closedir(dir);
      return false;
    }

    if (temp_type == TYPE_DIR) {
      /**
       * @brief 如果是子目录，递归删除
       */
      DeleteFile(temp_file);
    }

    remove(temp_file.c_str());
    /**
     * @brief 删除子项
     */
  }

  closedir(dir);
  /**
   * @brief 关闭目录
   */
  remove(filename.c_str());
  /**
   * @brief 删除空目录
   */
  return true;
}

/**
 * @brief 创建目录
 *
 * @param dir 目录路径
 * @return bool 是否成功
 *
 * 功能说明：
 * 创建单个目录（不递归）
 *
 * C++语法说明：
 * - mkdir(path, mode)：创建目录
 * - S_IRWXU | S_IRWXG | S_IRWXO：所有权限
 */
bool CreateDir(const string &dir) {
  int ret = mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
  /**
   * @brief 创建目录
   *
   * mkdir：
   * - S_IRWXU：所有者读写执行
   * - S_IRWXG：组读写执行
   * - S_IRWXO：其他用户读写执行
   */
  if (ret != 0) {
    AWARN << "failed to create dir. [dir: " << dir
          << "] [err: " << strerror(errno) << "]";
    return false;
  }

  return true;
}

}  // namespace common
/**
 * @brief 命名空间结束标记
 */

}  // namespace cyber
/**
 * @brief Cyber RT命名空间结束标记
 */

}  // namespace apollo
/**
 * @brief Apollo命名空间结束标记
 */
