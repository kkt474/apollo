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
 * @file file.h
 * @brief Apollo Cyber RT文件系统工具头文件
 *
 * 功能说明：
 * 提供文件操作相关的工具函数，包括：
 * - Protobuf消息的读写（ASCII和Binary格式）
 * - 文件/目录的创建、复制、删除操作
 * - 路径处理（绝对路径、目录名、文件名等）
 * - 目录遍历和模式匹配
 *
 * 应用场景：
 * - 配置文件读写
 * - 日志文件操作
 * - 地图数据加载
 * - 模块间数据交换
 */

#ifndef CYBER_COMMON_FILE_H_  // 头文件保护宏，防止重复包含
#define CYBER_COMMON_FILE_H_

/**
 * @brief POSIX系统头文件
 *
 * C++语法说明：
 * 这些是Unix/Linux系统编程中常用的头文件
 * 提供文件操作、目录遍历等系统调用接口
 *
 * - <dirent.h>：目录操作，DIR结构体、readdir等
 * - <fcntl.h>：文件控制，open等函数
 * - <sys/stat.h>：文件状态，stat等函数
 * - <sys/types.h>：系统数据类型
 * - <unistd.h>：Unix标准函数，close等
 */
#include <dirent.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

/**
 * @brief C++标准库头文件
 *
 * C++语法说明：
 * - <cstdio>：C风格输入输出，FILE结构等
 * - <fstream>：C++文件流，ifstream/ofstream
 * - <string>：字符串类
 * - <vector>：动态数组容器
 */
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

/**
 * @brief Google Protobuf头文件
 *
 * 功能说明：
 * - google/protobuf/io/zero_copy_stream_impl.h：
 *   ZeroCopyStream实现，用于高效的protobuf序列化
 *   避免中间缓冲区，直接操作底层IO
 *
 * - google/protobuf/text_format.h：
 *   TextFormat工具，用于ASCII格式的protobuf解析和打印
 *
 * C++语法说明：
 * - google::protobuf::Message：
 *   Protobuf消息的基类，所有生成的protobuf消息都继承自此类
 */
#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/text_format.h"

/**
 * @brief Cyber RT日志系统头文件
 *
 * 功能说明：
 * - cyber/common/log.h：
 *   定义了AERROR、AINFO、ADEBUG等日志宏
 */
#include "cyber/common/log.h"

/**
 * @namespace apollo::cyber::common
 * @brief Apollo Cyber RT通用工具命名空间
 *
 * C++语法说明：
 * - namespace apollo：
 *   Apollo项目的最外层命名空间
 * - namespace cyber：
 *   Cyber RT框架的命名空间
 * - namespace common：
 *   通用工具的子命名空间
 *
 * 使用命名空间的好处：
 * 1. 避免命名冲突
 * 2. 组织代码结构
 * 3. 表达语义层级
 */
namespace apollo {
namespace cyber {
namespace common {

/**
 * @brief 文件类型枚举
 *
 * 功能说明：
 * 用于区分文件还是目录
 *
 * C++语法说明：
 * - enum：
 *   枚举类型，定义一组命名的整型常量
 * - enum FileType：
 *   枚举名，可以在作用域内直接使用TYPE_FILE、TYPE_DIR
 *
 * 枚举值：
 * - TYPE_FILE = 0：普通文件
 * - TYPE_DIR = 1：目录
 */
enum FileType { TYPE_FILE, TYPE_DIR };

/**
 * @brief 将Protobuf消息以ASCII格式写入文件描述符
 *
 * @param message 要写入的Protobuf消息
 * @param file_descriptor 已打开的文件描述符
 * @return bool 是否成功
 *
 * 功能说明：
 * 将Protobuf消息转换为ASCII文本格式并写入指定的文件描述符
 * 用于日志记录或人类可读的数据存储
 *
 * C++语法说明：
 * - const google::protobuf::Message &message：
 *   常量引用，传入Protobuf消息
 *   使用引用避免拷贝大型消息结构
 *   const保证消息不会被修改
 *
 * - int file_descriptor：
 *   POSIX文件描述符，是一个非负整数
 *   0=stdin, 1=stdout, 2=stderr, >=3=打开的文件
 */
bool SetProtoToASCIIFile(const google::protobuf::Message &message,
                         int file_descriptor);

/**
 * @brief 将Protobuf消息以ASCII格式写入文件
 *
 * @param message 要写入的Protobuf消息
 * @param file_name 目标文件名
 * @return bool 是否成功
 *
 * 功能说明：
 * 将Protobuf消息转换为人类可读的ASCII文本格式并写入文件
 * 文件内容可以用文本编辑器查看
 *
 * C++语法说明：
 * - const std::string &file_name：
 *   常量引用，传入文件名
 *   使用string而非char*，更安全方便
 *
 * 扩展名惯例：
 * - .pb.txt 或 .ascii：ASCII格式的Protobuf
 * - .pb：二进制格式的Protobuf
 */
bool SetProtoToASCIIFile(const google::protobuf::Message &message,
                         const std::string &file_name);

/**
 * @brief 将字符串内容写入文件
 *
 * @param content 要写入的字符串内容
 * @param file_name 目标文件名
 * @return bool 是否成功
 *
 * 功能说明：
 * 将字符串内容原样写入文件
 * 与SetProtoToASCIIFile的区别是不经过Protobuf转换
 *
 * 应用场景：
 * - 写入日志文本
 * - 写入配置文件
 * - 写入纯文本数据
 */
bool SetStringToASCIIFile(const std::string &content,
                          const std::string &file_name);

/**
 * @brief 从ASCII格式文件解析Protobuf消息
 *
 * @param file_name 要解析的文件名
 * @param message 输出：解析后的Protobuf消息
 * @return bool 是否成功
 *
 * 功能说明：
 * 读取ASCII格式的Protobuf文件并解析到message中
 * 与SetProtoToASCIIFile互为逆操作
 *
 * C++语法说明：
 * - google::protobuf::Message *message：
 *   指针参数，用于输出解析结果
 *   使用指针允许函数修改外部传入的消息对象
 *
 * - MergeFrom：
 *   Protobuf的合并方法，将新值合并到现有消息
 *   而不是完全替换
 */
bool GetProtoFromASCIIFile(const std::string &file_name,
                           google::protobuf::Message *message);

/**
 * @brief 将Protobuf消息以二进制格式写入文件
 *
 * @param message 要写入的Protobuf消息
 * @param file_name 目标文件名
 * @return bool 是否成功
 *
 * 功能说明：
 * 将Protobuf消息以二进制格式序列化并写入文件
 * 二进制格式比ASCII格式更紧凑、解析更快
 *
 * C++语法说明：
 * - Binary格式特点：
 *   - 文件更小（约为ASCII的1/3）
 *   - 解析更快（无需文本解析）
 *   - 人类不可读
 *
 * 扩展名惯例：.pb 或 .bin
 */
bool SetProtoToBinaryFile(const google::protobuf::Message &message,
                          const std::string &file_name);

/**
 * @brief 从二进制格式文件解析Protobuf消息
 *
 * @param file_name 要解析的文件名
 * @param message 输出：解析后的Protobuf消息
 * @return bool 是否成功
 *
 * 功能说明：
 * 读取二进制格式的Protobuf文件并解析到message中
 * 与SetProtoToBinaryFile互为逆操作
 *
 * 二进制解析比ASCII解析速度快数倍
 */
bool GetProtoFromBinaryFile(const std::string &file_name,
                            google::protobuf::Message *message);

/**
 * @brief 从文件解析Protobuf消息（自动判断格式）
 *
 * @param file_name 要解析的文件名
 * @param message 输出：解析后的Protobuf消息
 * @return bool 是否成功
 *
 * 功能说明：
 * 根据文件内容自动判断是ASCII还是二进制格式
 * 并进行相应解析
 *
 * 格式判断依据：
 * - 文件开头是否是Protobuf文本格式的标识字符
 * - 否则按二进制格式尝试解析
 *
 * 优先级：ASCII > 二进制
 */
bool GetProtoFromFile(const std::string &file_name,
                      google::protobuf::Message *message);

/**
 * @brief 从JSON格式文件解析Protobuf消息
 *
 * @param file_name 要解析的JSON文件名
 * @param message 输出：解析后的Protobuf消息
 * @return bool 是否成功
 *
 * 功能说明：
 * 读取JSON格式的文件并转换为Protobuf消息
 * JSON是Web和跨语言通信中常用的数据格式
 *
 * 注意：
 * 需要proto中定义JSON选项才能正确转换
 */
bool GetProtoFromJsonFile(const std::string &file_name,
                          google::protobuf::Message *message);

/**
 * @brief 读取文件内容作为字符串
 *
 * @param file_name 要读取的文件名
 * @param content 输出：文件内容字符串
 * @return bool 是否成功
 *
 * 功能说明：
 * 将整个文件内容读入字符串
 * 适用于小文件，大的二进制文件可能导致内存问题
 *
 * 算法流程：
 * 1. 打开文件
 * 2. 获取文件大小
 * 3. 分配字符串空间
 * 4. 读取内容
 * 5. 关闭文件
 *
 * C++语法说明：
 * - std::string *content：
 *   指针输出参数
 *   函数内部分配内存并赋值给*content
 */
bool GetContent(const std::string &file_name, std::string *content);

/**
 * @brief 拼接得到绝对路径
 *
 * @param prefix 前缀路径
 * @param relative_path 相对路径
 * @return std::string 拼接后的绝对路径
 *
 * 功能说明：
 * 将前缀和相对路径拼接为完整的绝对路径
 * 处理路径中的.和..等特殊目录
 *
 * 示例：
 * - GetAbsolutePath("/home/user", "config/file.txt")
 *   返回 "/home/user/config/file.txt"
 *
 * C++语法说明：
 * - std::string：
 *   返回值是字符串，直接返回
 *   无需指针或引用参数
 *
 * 路径处理：
 * - 自动处理多余的"/"
 * - 解析".."和"."等特殊路径
 */
std::string GetAbsolutePath(const std::string &prefix,
                            const std::string &relative_path);

/**
 * @brief 检查路径是否存在
 *
 * @param path 要检查的路径（文件或目录）
 * @return bool 如果存在返回true
 *
 * 功能说明：
 * 检查指定路径对应的文件或目录是否存在
 * 使用POSIX的stat函数实现
 *
 * C++语法说明：
 * - stat()系统调用：
 *   int stat(const char *path, struct stat *buf)
 *   获取文件/目录的状态信息
 *   成功返回0，失败返回-1
 *
 * 注意：
 * - 不区分文件还是目录
 * - 需要检查返回值判断成功/失败
 */
bool PathExists(const std::string &path);

/**
 * @brief 检查路径是否为绝对路径
 *
 * @param path 要检查的路径
 * @return bool 如果是绝对路径返回true
 *
 * 功能说明：
 * 判断路径是绝对路径还是相对路径
 *
 * 判断依据：
 * - Unix/Linux：以"/"开头的是绝对路径
 * - Windows：以盘符（如"C:"）开头的是绝对路径
 *
 * 注意：
 * 这里主要针对Unix/Linux系统
 */
bool PathIsAbsolute(const std::string &path);

/**
 * @brief 检查目录是否存在
 *
 * @param directory_path 目录路径
 * @return bool 如果存在且是目录返回true
 *
 * 功能说明：
 * 检查指定路径是否是一个存在的目录
 * 与PathExists的区别是会额外检查是否为目录类型
 *
 * 算法流程：
 * 1. 调用stat()获取路径状态
 * 2. 检查S_ISDIR宏判断是否为目录
 */
bool DirectoryExists(const std::string &directory_path);

/**
 * @brief 使用通配符模式匹配文件路径
 *
 * @param pattern 通配符模式，可以包含?和*
 *        ? 匹配任意单个字符
 *        * 匹配任意长度任意字符
 * @return std::vector<std::string> 匹配的文件路径列表
 *
 * 功能说明：
 * 根据通配符模式查找匹配的文件
 *
 * 示例：
 * - Glob("/path/to/*.txt") 返回所有.txt文件
 * - Glob("/path/to/file_?") 匹配file_1, file_a等
 * - Glob("/path/to/*") 匹配目录下所有文件
 *
 * C++语法说明：
 * - std::vector<std::string>：
 *   返回动态数组，包含所有匹配路径
 *   空vector表示没有匹配的文件
 *
 * 底层实现：
 * 使用glob()函数或自己实现目录遍历+模式匹配
 */
std::vector<std::string> Glob(const std::string &pattern);

/**
 * @brief 复制文件
 *
 * @param from 源文件路径
 * @param to 目标文件路径
 * @return bool 是否成功
 *
 * 功能说明：
 * 将源文件复制到目标路径
 * 目标路径可以包含新的文件名
 *
 * 算法流程：
 * 1. 打开源文件（读模式）
 * 2. 创建目标文件（写模式，权限0644）
 * 3. 循环读取源文件内容并写入目标文件
 * 4. 关闭两个文件
 *
 * 注意：
 * - 如果目标文件已存在，会被覆盖
 * - 不会复制文件元数据（如权限、时间戳）
 */
bool CopyFile(const std::string &from, const std::string &to);

/**
 * @brief 复制目录
 *
 * @param from 源目录路径
 * @param to 目标目录路径
 * @return bool 是否成功
 *
 * 功能说明：
 * 将源目录及其内容递归复制到目标路径
 *
 * 算法流程：
 * 1. 创建目标目录
 * 2. 遍历源目录中的所有项
 * 3. 对每个子项：
 *    - 如果是目录，递归调用CopyDir
 *    - 如果是文件，调用CopyFile
 * 4. 复制目录属性（权限、时间戳等）
 *
 * 注意：
 * - 目标目录必须不存在或为空目录
 * - 会递归复制所有子目录
 */
bool CopyDir(const std::string &from, const std::string &to);

/**
 * @brief 复制文件或目录
 *
 * @param from 源路径
 * @param to 目标路径
 * @return bool 是否成功
 *
 * 功能说明：
 * 自动判断源路径是文件还是目录
 * 调用相应的CopyFile或CopyDir
 *
 * C++语法说明：
 * - 根据源路径的FileType决定调用哪个函数
 * - 使用FileType枚举进行类型判断
 */
bool Copy(const std::string &from, const std::string &to);

/**
 * @brief 确保目录存在，不存在则创建
 *
 * @param directory_path 目录路径
 * @return bool 如果目录存在或创建成功返回true
 *
 * 功能说明：
 * 递归创建目录及其所有父目录
 * 类似于mkdir -p命令
 *
 * 算法流程：
 * 1. 检查目录是否已存在
 * 2. 如果不存在，递归创建父目录
 * 3. 创建目标目录本身
 *
 * C++语法说明：
 * - mkdir(path, mode)：
 *   POSIX创建目录函数
 *   mode通常为0755（rwxr-xr-x）
 *
 * 示例：
 * - EnsureDirectory("/a/b/c") 会创建 /a, /a/b, /a/b/c
 */
bool EnsureDirectory(const std::string &directory_path);

/**
 * @brief 删除目录下的所有文件
 *
 * @param directory_path 目录路径
 * @return bool 是否成功
 *
 * 功能说明：
 * 删除指定目录下的所有文件，但不影响子目录
 * 常用于日志清理等场景
 *
 * 注意：
 * - 子目录及其内容不受影响
 * - 隐藏文件（以.开头）也会被删除
 * - 目录本身不会被删除
 *
 * C++语法说明：
 * - remove(filename)：
 *   POSIX删除文件函数
 *   成功返回0，失败返回-1
 */
bool RemoveAllFiles(const std::string &directory_path);

/**
 * @brief 列出子路径
 *
 * @param directory_path 目录路径
 * @param d_type 子路径类型过滤
 *        DT_DIR：只返回子目录
 *        DT_REG：只返回普通文件
 *        默认只返回子目录
 * @return std::vector<std::string> 子路径列表（不含目录前缀）
 *
 * 功能说明：
 * 遍历指定目录，返回其直接子路径列表
 *
 * 算法流程：
 * 1. 打开目录（opendir）
 * 2. 循环读取目录项（readdir）
 * 3. 过滤.和..目录
 * 4. 根据d_type过滤类型
 * 5. 关闭目录（closedir）
 *
 * C++语法说明：
 * - DIR *dir：
 *   POSIX目录流结构
 *   通过opendir打开，closedir关闭
 *
 * - struct dirent *entry：
 *   目录项结构
 *   entry->d_name：目录项名称
 *   entry->d_type：目录项类型（DT_DIR/DT_REG等）
 *
 * - d_type默认值DT_DIR：
 *   只列出子目录，不列出文件
 */
std::vector<std::string> ListSubPaths(const std::string &directory_path,
                                      const unsigned char d_type = DT_DIR);

/**
 * @brief 使用模式匹配查找文件
 *
 * @param base_path 搜索的根目录
 * @param patt 要匹配的模式字符串
 * @param d_type 条目类型过滤
 * @param recursive 是否递归搜索子目录
 * @param result_list 输出：匹配结果列表
 * @return size_t 匹配结果的数量
 *
 * 功能说明：
 * 在指定目录中递归或非递归搜索匹配模式的文件/目录
 *
 * 参数说明：
 * @param base_path 搜索的起始目录
 * @param patt 用于过滤的d_name匹配模式
 * @param d_type 要搜索的类型DT_DIR/DT_REG/DT_UNKNOWN等
 * @param recursive true=递归搜索子目录，false=只搜索当前目录
 * @param result_list 存储匹配结果的输出参数
 *
 * C++语法说明：
 * - const unsigned char d_type：
 *   d_type是dirent中的文件类型字段
 *   使用unsigned char（1字节）存储
 *
 * - size_t：
 *   无符号整数类型，用于表示大小和索引
 *   返回匹配的条目数量
 *
 * - std::vector<std::string> *result_list：
 *   指针参数，用于输出匹配结果
 */
size_t FindPathByPattern(const std::string &base_path, const std::string &patt,
                         const unsigned char d_type, const bool recursive,
                         std::vector<std::string> *result_list);

/**
 * @brief 获取路径的目录名部分
 *
 * @param path 完整路径
 * @return std::string 目录名
 *
 * 功能说明：
 * 提取路径中的目录部分
 *
 * 示例：
 * - GetDirName("/home/user/file.txt") 返回 "/home/user"
 * - GetDirName("/file.txt") 返回 "/"
 * - GetDirName("file.txt") 返回 "."
 *
 * C++语法说明：
 * - std::string：
 *   直接返回字符串
 *
 * 实现方式：
 * 查找最后一个'/'的位置，提取其前面的部分
 */
std::string GetDirName(const std::string &path);

/**
 * @brief 获取路径的文件名部分
 *
 * @param path 完整路径
 * @param remove_extension 是否去除扩展名
 *        true=去除扩展名，false=保留完整文件名
 * @return std::string 文件名
 *
 * 功能说明：
 * 提取路径中的文件名部分
 *
 * 示例：
 * - GetFileName("/home/user/file.txt", false) 返回 "file.txt"
 * - GetFileName("/home/user/file.txt", true) 返回 "file"
 * - GetFileName("file.txt", true) 返回 "file"
 *
 * C++语法说明：
 * - bool remove_extension = false：
 *   默认参数，如果不指定则保留扩展名
 *   提高API的灵活性
 *
 * 算法流程：
 * 1. 查找最后一个'/'
 * 2. 提取文件名部分
 * 3. 如果需要去除扩展名，查找最后一个'.'
 * 4. 截取到扩展名之前
 */
std::string GetFileName(const std::string &path,
                        const bool remove_extension = false);

/**
 * @brief 根据优先级获取有效文件路径
 *
 * @param path 输入的文件路径字符串
 * @param env_var 环境变量名
 * @param file_path 输出：最终的有效文件路径
 * @return bool 是否找到有效路径
 *
 * 功能说明：
 * 按照优先级规则解析文件路径：
 * 1. 如果是绝对路径，直接使用
 * 2. 如果是相对路径（以"."开头），相对于当前目录
 * 3. 如果设置了环境变量，尝试在环境变量目录中查找
 * 4. 相对于当前工作目录
 *
 * @param path 输入路径
 * @param env_var 要查找的环境变量名，如"APOLLO_CONF_PATH"
 * @param file_path 输出参数，存储找到的有效路径
 *
 * C++语法说明：
 * - const std::string &env_var：
 *   常量引用，环境变量名
 *
 * - std::string *file_path：
 *   指针输出参数
 *
 * 示例：
 * - GetFilePathWithEnv("config.pb.txt", "APOLLO_CONF_PATH", &path)
 *   会在APOLLO_CONF_PATH目录中查找config.pb.txt
 */
bool GetFilePathWithEnv(const std::string &path, const std::string &env_var,
                        std::string *file_path);

/**
 * @brief 获取当前工作目录
 *
 * @return std::string 当前工作目录的绝对路径
 *
 * 功能说明：
 * 获取进程当前的工作目录
 *
 * C++语法说明：
 * - getcwd()：
 *   POSIX获取当前工作目录函数
 *   需要提供缓冲区存储路径
 */
std::string GetCurrentPath();

// 删除文件（文件或目录）
// 注意：这个函数名在注释中写为DeleteFile，但在代码中是DeleteFile
/**
 * @brief 删除文件或目录
 *
 * @param filename 要删除的文件/目录路径
 * @return bool 是否成功
 *
 * 功能说明：
 * 删除指定的文件或空目录
 *
 * 注意：
 * - 不能删除非空目录
 * - 删除后无法恢复
 *
 * C++语法说明：
 * - remove()：
 *   C标准库函数
 *   可以删除文件和空目录
 */
bool DeleteFile(const std::string &filename);

/**
 * @brief 获取文件/目录类型
 *
 * @param filename 要检查的路径
 * @param type 输出：文件类型
 * @return bool 是否成功获取类型
 *
 * 功能说明：
 * 查询指定路径的类型（文件还是目录）
 *
 * C++语法说明：
 * - FileType *type：
 *   指针输出参数
 *   FileType是前面定义的枚举类型
 */
bool GetType(const std::string &filename, FileType *type);

/**
 * @brief 创建目录
 *
 * @param dir 目录路径
 * @return bool 是否成功创建
 *
 * 功能说明：
 * 创建指定目录
 * 类似于mkdir命令（不递归创建父目录）
 *
 * 与EnsureDirectory的区别：
 * - CreateDir：只创建最后一级目录
 * - EnsureDirectory：递归创建所有父目录
 */
bool CreateDir(const std::string &dir);

/**
 * @brief 加载配置文件模板函数
 *
 * @tparam T Protobuf配置消息类型
 * @param relative_path 相对于配置目录的路径
 * @param config 输出：配置对象指针
 * @return bool 是否成功加载
 *
 * 功能说明：
 * 通用的配置文件加载模板函数
 * 封装了路径查找和Protobuf解析的流程
 *
 * 加载流程：
 * 1. 根据相对路径查找实际配置文件
 * 2. 解析文件内容到config对象
 *
 * C++语法说明：
 * - template <typename T>：
 *   函数模板，T是泛型类型参数
 *   允许加载任何Protobuf消息类型
 *
 * - template <typename T> bool LoadConfig(...)：
 *   模板函数定义
 *   可以在编译时确定T的具体类型
 *
 * - CHECK_NOTNULL(config)：
 *   断言宏，确保config指针非空
 *   防止空指针解引用
 *
 * - GetFilePathWithEnv：
 *   在环境变量指定目录中查找配置文件
 *   "APOLLO_CONF_PATH"是Apollo的标准配置目录环境变量
 *
 * - GetProtoFromFile：
 *   从文件解析Protobuf配置
 *
 * 示例：
 * @code
 *   PlanningConfig config;
 *   LoadConfig("planning_config.pb.txt", &config);
 * @endcode
 */
template <typename T>
bool LoadConfig(const std::string &relative_path, T *config) {
  // 断言检查config指针非空
  CHECK_NOTNULL(config);

  // TODO: get config base relative path
  // TODO: 应该获取配置的基础相对路径，而不是硬编码

  // 用于存储实际找到的配置文件路径
  std::string actual_config_path;

  // 在APOLLO_CONF_PATH环境变量指定的目录中查找配置文件
  if (!GetFilePathWithEnv(relative_path, "APOLLO_CONF_PATH",
                          &actual_config_path)) {
    // 查找失败，输出错误日志
    AERROR << "conf file [" << relative_path
           << "] is not found in APOLLO_CONF_PATH";
    return false;  // 返回失败
  }

  // 查找成功，记录信息日志
  AINFO << "load conf file: " << actual_config_path;

  // 解析配置文件到config对象
  return GetProtoFromFile(actual_config_path, config);
}

/**
 * @namespace命名空间结束标记
 *
 * C++语法说明：
 * - }  // namespace common：
 *   结束common命名空间
 * - }  // namespace cyber：
 *   结束cyber命名空间
 * - }  // namespace apollo：
 *   结束apollo命名空间
 *
 * 注释中的//是对应命名空间的说明，方便阅读
 */
}  // namespace common
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_COMMON_FILE_H_