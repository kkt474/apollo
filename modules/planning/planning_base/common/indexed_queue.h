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
 * @file indexed_queue.h
 * @brief 索引队列容器模板类头文件
 *
 * 功能说明：
 * 一种支持容量限制的FIFO（先进先出）队列容器
 * 同时通过ID提供O(1)时间复杂度的快速查找
 * 当容量满时，自动移除最旧的元素
 *
 * 设计模式：
 * - 模板类：支持任意类型的ID和对象
 * - 智能指针管理：使用unique_ptr自动管理内存
 * - 容量管理：支持设置队列最大容量
 */

#pragma once

#include <memory>
#include <queue>
#include <unordered_map>
#include <utility>

#include "modules/common/util/map_util.h"

namespace apollo {
/**
 * @brief Apollo外层命名空间
 */
namespace planning {

/**
 * @template IndexedQueue
 * @brief 索引队列模板类
 *
 * @tparam I ID的类型，通常是整数或字符串
 * @tparam T 存储的对象类型
 *
 * 功能说明：
 * 结合了queue和unordered_map的特性
 * - queue：维护元素的添加顺序，支持FIFO
 * - unordered_map：提供ID到元素的快速查找
 *
 * 设计特点：
 * - 容量限制：当元素数量超过容量时，自动移除最旧的元素
 * - 智能指针：使用unique_ptr管理对象内存，避免内存泄漏
 * - 唯一性：每个ID只能添加一次，不支持覆盖
 *
 * 数据结构：
 * - queue<pair<I, const T*>>：按添加顺序存储ID和指针
 * - unordered_map<I, unique_ptr<T>>：ID到对象的映射
 *
 * C++语法说明：
 * - std::queue：
 *   先进先出容器适配器
 *   底层可以由deque、list等容器支持
 * - std::unique_ptr<T>：
 *   独占所有权的智能指针
 *   - 不能拷贝，只能移动
 *   - 最后一个指针销毁时自动删除对象
 * - std::unordered_map<I, std::unique_ptr<T>>：
 *   值为unique_ptr的哈希表
 *   unique_ptr不能被拷贝，所以只能通过移动操作插入
 */
template <typename I, typename T>
class IndexedQueue {
 public:
  /**
   * @brief 构造函数
   *
   * @param capacity 队列的最大容量
   *        如果为0，表示无限容量
   *
   * 功能说明：
   * 创建IndexedQueue并设置容量限制
   *
   * C++语法说明：
   * - explicit：
   *   防止隐式类型转换
   *   IndexedQueue q = 10 会编译错误
   * - : capacity_(capacity)：
   *   构造函数初始化列表
   *   在构造函数体执行前初始化成员变量
   * - size_t：
   *   无符号整数类型
   *   用于表示大小和索引
   */
  explicit IndexedQueue(size_t capacity) : capacity_(capacity) {}

  /**
   * @brief 通过ID查找对象
   *
   * @param id 要查找的对象的ID
   * @return const T* 指向找到的对象的常量指针，如果未找到返回nullptr
   *
   * C++语法说明：
   * - const T*：返回常量指针
   *   不能通过返回的指针修改对象内容
   * - auto *result：
   *   auto自动推导为unique_ptr<T>*
   * - apollo::common::util::FindOrNull：
   *   Apollo提供的map查找辅助函数
   *   如果找到返回指向值的指针，未找到返回nullptr
   * - result->get()：
   *   unique_ptr的get方法返回原始指针
   * - result ? result->get() : nullptr：
   *   三元运算符
   *   如果result非空，返回原始指针；否则返回nullptr
   */
  const T *Find(const I id) const {
    // 查找ID对应的unique_ptr
    auto *result = apollo::common::util::FindOrNull(map_, id);
    // 如果找到，返回原始指针；否则返回nullptr
    return result ? result->get() : nullptr;
  }

  /**
   * @brief 获取最新添加的元素
   *
   * @return const T* 指向最新元素的常量指针，如果队列为空返回nullptr
   *
   * 功能说明：
   * 返回最后添加的元素，但不从队列中移除
   *
   * C++语法说明：
   * - queue_.empty()：
   *   判断队列是否为空
   *   返回true表示空，false表示非空
   * - queue_.back()：
   *   返回队列最后一个元素的引用
   *   - back().first 获取pair的第一个元素（ID）
   */
  const T *Latest() const {
    // 如果队列为空，返回nullptr
    if (queue_.empty()) {
      return nullptr;
    }
    // 获取最新元素的ID，然后查找并返回
    return Find(queue_.back().first);
  }

  /**
   * @brief 添加元素到队列
   *
   * @param id 对象的唯一标识符
   * @param ptr 对象的智能指针（所有权会被转移）
   * @return bool 添加是否成功
   *         如果ID已存在，返回false
   *
   * 功能说明：
   * 将对象添加到队列和映射表中
   * 如果队列已满，自动移除最旧的元素
   *
   * 算法流程：
   * 1. 检查ID是否已存在，如果存在则添加失败
   * 2. 如果队列已满，移除最旧的元素
   * 3. 将新元素添加到队列和映射表
   *
   * C++语法说明：
   * - std::unique_ptr<T> ptr：
   *   独占所有权的智能指针
   *   所有权会被移动到队列中
   * - Find(id)：
   *   调用上面的Find方法检查ID是否存在
   * - if (capacity_ > 0 && queue_.size() == capacity_)：
   *   检查是否需要移除元素
   *   - capacity_ > 0：容量已设置（非无限）
   *   - queue_.size() == capacity_：队列已满
   * - map_.erase(queue_.front().first)：
   *   从map中移除最旧元素的映射
   *   - queue_.front()：获取队列第一个元素
   *   - front().first：获取第一个元素的ID
   *   - erase：移除键值对
   * - queue_.pop()：
   *   移除队列第一个元素
   * - queue_.emplace(id, ptr.get())：
   *   在队列末尾构造新元素
   *   - emplace直接构造pair，避免拷贝
   *   - ptr.get()获取原始指针，但保留所有权
   * - map_[id] = std::move(ptr)：
   *   将智能指针移动到map中
   *   - std::move将左值转换为右值引用
   *   - 移动后ptr变为空
   */
  bool Add(const I id, std::unique_ptr<T> ptr) {
    // 检查ID是否已存在
    if (Find(id)) {
      return false;  // ID已存在，添加失败
    }

    // 如果队列已满，移除最旧的元素
    if (capacity_ > 0 && queue_.size() == capacity_) {
      // 获取最旧元素的ID
      auto oldest_id = queue_.front().first;
      // 从map中移除最旧元素（会触发unique_ptr的析构）
      map_.erase(oldest_id);
      // 从队列中移除最旧元素
      queue_.pop();
    }

    // 将新元素添加到队列
    queue_.emplace(id, ptr.get());
    // 将智能指针移动到map中
    map_[id] = std::move(ptr);
    return true;  // 添加成功
  }

  /**
   * @brief 清空队列
   *
   * 功能说明：
   * 移除所有元素，释放内存
   *
   * C++语法说明：
   * - while (!queue_.empty())：
   *   循环直到队列为空
   * - queue_.pop()：
   *   移除队列第一个元素
   *   循环结束时queue_已为空
   * - map_.clear()：
   *   清空map
   *   会销毁所有unique_ptr，释放内存
   */
  void Clear() {
    // 清空队列
    while (!queue_.empty()) {
      queue_.pop();
    }
    // 清空映射表
    map_.clear();
  }

 public:
  /**
   * @brief 队列容量
   *
   * size_t：无符号整数类型
   * 如果为0，表示无限容量
   */
  size_t capacity_ = 0;

  /**
   * @brief 存储元素顺序的队列
   *
   * std::queue<std::pair<I, const T*>>：
   * - queue：先进先出容器
   * - pair<I, const T*>：存储ID和指针的对
   *
   * C++语法说明：
   * - const T*：
   *   队列中存储的是原始指针
   *   实际对象由map中的unique_ptr拥有
   */
  std::queue<std::pair<I, const T*>> queue_;

  /**
   * @brief ID到对象的映射
   *
   * std::unordered_map<I, std::unique_ptr<T>>：
   * - I：键的类型（ID）
   * - std::unique_ptr<T>：值是对象的智能指针
   *
   * C++语法说明：
   * - std::unique_ptr：
   *   独占所有权的智能指针
   *   最后一个拥有者负责销毁对象
   */
  std::unordered_map<I, std::unique_ptr<T>> map_;
};

}  // namespace planning
}  // namespace apollo
