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
 * @file indexed_list.h
 * @brief 索引列表容器模板类头文件
 *
 * 功能说明：
 * 提供一种同时通过ID和顺序访问对象的容器
 * 支持O(1)时间复杂度的ID查找和O(1)时间的顺序遍历
 * 提供线程安全版本，支持多线程并发访问
 *
 * 设计模式：
 * - 模板类：支持任意类型的ID和对象
 * - 线程安全包装器：通过继承添加线程安全功能
 */

#pragma once

#include <unordered_map>
#include <vector>

#include <boost/thread/shared_mutex.hpp>

#include "cyber/common/log.h"
#include "modules/common/util/map_util.h"

namespace apollo {
/**
 * @brief Apollo外层命名空间
 */
namespace planning {

/**
 * @template IndexedList
 * @brief 索引列表模板类
 *
 * @tparam I ID的类型，通常是整数或字符串
 * @tparam T 存储的对象类型
 *
 * 功能说明：
 * 同时维护一个unordered_map和一个vector
 * - unordered_map<I, T>：提供ID到对象的快速查找
 * - vector<const T*>：提供对象的顺序访问
 *
 * 设计特点：
 * - 双数据结构：map用于快速查找，vector用于顺序遍历
 * - 指针存储：vector中存储指向实际对象的指针，避免拷贝
 * - 对象所有权的拥有：IndexedList拥有对象的所有权
 *
 * C++语法说明：
 * - template <typename I, typename T>：
 *   类模板声明
 *   I是ID类型，T是对象类型
 *   可以在实例化时指定具体类型
 * - std::unordered_map<I, T>：
 *   哈希表实现的关联容器
 *   - 查找时间复杂度：平均O(1)
 *   - 不保证元素顺序
 * - std::vector<const T*>：
 *   存储T类型常量指针的动态数组
 */
template <typename I, typename T>
class IndexedList {
 public:
  /**
   * @brief 添加对象到容器
   *
   * @param id 对象的唯一标识符
   * @param object 要添加的对象的常量引用
   * @return T* 指向容器中对象的指针
   *
   * 功能说明：
   * 将对象添加到容器中
   * 如果ID已存在，则覆盖原有对象
   *
   * 算法流程：
   * 1. 首先尝试查找该ID是否已存在
   * 2. 如果存在，直接覆盖并返回指针
   * 3. 如果不存在，插入新对象并返回指针
   *
   * C++语法说明：
   * - T*：返回指向T类型的指针
   * - const T& object：输入对象的常量引用，避免拷贝
   * - auto obs = Find(id)：
   *   auto自动推导类型为T*
   *   Find返回查找结果
   * - if (obs)：nullptr在if语句中为false，非nullptr为true
   * - *obs = object：
   *   解引用指针并赋值
   *   这是修改指针指向的对象内容
   * - object_dict_.insert({id, object})：
   *   unordered_map的插入方法
   *   使用花括号初始化pair
   * - object_dict_.at(id)：
   *   at方法返回键对应的值的引用
   *   如果键不存在，会抛出std::out_of_range异常
   * - object_list_.push_back(ptr)：
   *   vector的push_back方法
   *   将指针添加到vector末尾
   *
   * 示例：
   * @code
   *   IndexedList<int, Obstacle> obstacles;
   *   Obstacle obs1, obs2;
   *   obstacles.Add(1, obs1);  // 添加ID为1的障碍物
   *   obstacles.Add(2, obs2);  // 添加ID为2的障碍物
   * @endcode
   */
  T* Add(const I id, const T& object) {
    // 尝试查找该ID是否已存在
    auto obs = Find(id);
    if (obs) {
      // ID已存在，记录警告日志
      AWARN << "object " << id << " is already in container";
      // 覆盖原有对象
      *obs = object;
      return obs;
    } else {
      // ID不存在，插入新对象
      object_dict_.insert({id, object});
      // 获取插入对象的指针
      auto* ptr = &object_dict_.at(id);
      // 将指针添加到列表
      object_list_.push_back(ptr);
      return ptr;
    }
  }

  /**
   * @brief 通过ID查找对象（可修改版本）
   *
   * @param id 要查找的对象的ID
   * @return T* 指向找到的对象的指针，如果未找到返回nullptr
   *
   * C++语法说明：
   * - apollo::common::util::FindOrNull：
   *   Apollo提供的map查找辅助函数
   *   如果找到返回指向值的指针，未找到返回nullptr
   */
  T* Find(const I id) {
    return apollo::common::util::FindOrNull(object_dict_, id);
  }

  /**
   * @brief 通过ID查找对象（只读版本）
   *
   * @param id 要查找的对象的ID
   * @return const T* 指向找到的对象的常量指针，如果未找到返回nullptr
   *
   * C++语法说明：
   * - const T*：返回常量指针
   *   不能通过返回的指针修改对象内容
   * - const成员函数：
   *   函数声明后的const表示this指针是常量
   *   在const成员函数中不能修改成员变量
   */
  const T* Find(const I id) const {
    return apollo::common::util::FindOrNull(object_dict_, id);
  }

  /**
   * @brief 获取所有对象的列表
   *
   * @return const std::vector<const T*>& 所有对象指针的常量引用
   *
   * 功能说明：
   * 返回包含所有对象指针的vector
   * 用于顺序遍历所有对象
   *
   * C++语法说明：
   * - const std::vector<const T*>&：
   *   返回常量引用
   *   - 外层const：返回的vector不可修改
   *   - 内层const T*：vector中的指针指向的对象不可修改
   */
  const std::vector<const T*>& Items() const { return object_list_; }

  /**
   * @brief 获取所有对象的字典
   *
   * @return const std::unordered_map<I, T>& ID到对象的字典的常量引用
   *
   * 功能说明：
   * 返回底层的unordered_map
   * 用于需要直接访问map的场景
   */
  const std::unordered_map<I, T>& Dict() const { return object_dict_; }

  /**
   * @brief 拷贝赋值运算符
   *
   * @param other 要拷贝的另一个IndexedList
   * @return IndexedList& 返回引用支持链式赋值
   *
   * 功能说明：
   * 将另一个IndexedList的所有内容拷贝到当前对象
   * 实现深拷贝，所有对象都会被复制
   *
   * C++语法说明：
   * - IndexedList&：
   *   返回引用而非副本
   *   支持 a = b = c 这样的链式赋值
   * - operator=：
   *   赋值运算符重载
   * - *this：
   *   解引用this指针获取当前对象
   * - for (const auto& item : other.Dict())：
   *   范围for循环遍历map
   *   auto自动推导为std::pair<const I, T>
   */
  IndexedList& operator=(const IndexedList& other) {
    // 清空当前容器
    this->object_list_.clear();
    this->object_dict_.clear();
    // 从other拷贝所有对象
    for (const auto& item : other.Dict()) {
      Add(item.first, item.second);
    }
    return *this;
  }

 private:
  /**
   * @brief 存储所有对象的指针列表
   *
   * std::vector<const T*>：
   * - vector：动态数组，支持快速随机访问
   * - const T*：常量指针，不能修改指针指向的对象
   *
   * 用途：
   * - 支持O(1)时间的顺序遍历
   * - 保持对象添加的顺序
   */
  std::vector<const T*> object_list_;

  /**
   * @brief 存储ID到对象的映射
   *
   * std::unordered_map<I, T>：
   * - unordered_map：哈希表实现的关联容器
   * - I：键的类型（ID）
   * - T：值的类型（对象）
   *
   * 用途：
   * - 支持O(1)平均时间的ID查找
   * - 实际存储所有对象
   */
  std::unordered_map<I, T> object_dict_;
};

/**
 * @template ThreadSafeIndexedList
 * @brief 线程安全的索引列表模板类
 *
 * @tparam I ID的类型
 * @tparam T 存储的对象类型
 *
 * 功能说明：
 * 通过继承IndexedList并添加读写锁来实现线程安全
 * - 读操作：使用共享锁（shared_lock），允许多线程并发读
 * - 写操作：使用独占锁（unique_lock），保证写操作的原子性
 *
 * C++语法说明：
 * - class ThreadSafeIndexedList : public IndexedList<I, T>：
 *   公有继承自IndexedList
 *   子类可以调用父类的所有公有成员
 * - boost::shared_mutex：
 *   Boost库的读写锁
 *   - shared_lock：共享锁，多个线程可以同时持有
 *   - unique_lock：独占锁，同时只有一个线程可以持有
 * - mutable boost::shared_mutex mutex_：
 *   mutable允许在const成员函数中修改
 */
template <typename I, typename T>
class ThreadSafeIndexedList : public IndexedList<I, T> {
 public:
  /**
   * @brief 添加对象（线程安全版本）
   *
   * @param id 对象的唯一标识符
   * @param object 要添加的对象
   * @return T* 指向容器中对象的指针
   *
   * C++语法说明：
   * - boost::unique_lock<boost::shared_mutex>：
   *   独占锁，在构造时自动获取锁
   *   析构时自动释放锁（RAII手法）
   *   确保写操作的原子性
   * - mutex_：
   *   类成员变量，用于同步访问
   * - IndexedList<I, T>::Add：
   *   显式调用父类的Add方法
   */
  T* Add(const I id, const T& object) {
    boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
    return IndexedList<I, T>::Add(id, object);
  }

  /**
   * @brief 查找对象（线程安全版本）
   *
   * @param id 要查找的对象的ID
   * @return T* 指向找到的对象的指针，如果未找到返回nullptr
   *
   * C++语法说明：
   * - boost::shared_lock<boost::shared_mutex>：
   *   共享锁，允许多个线程同时持有
   *   用于读操作，提高并发性能
   */
  T* Find(const I id) {
    boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);
    return IndexedList<I, T>::Find(id);
  }

  /**
   * @brief 获取所有对象的列表（线程安全版本）
   *
   * @return std::vector<const T*> 所有对象指针的拷贝
   *
   * C++语法说明：
   * - 返回值不是引用而是拷贝
   *   因为返回后锁会被释放，不能返回内部数据的引用
   */
  std::vector<const T*> Items() const {
    boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);
    return IndexedList<I, T>::Items();
  }

 private:
  /**
   * @brief 读写锁
   *
   * mutable允许在const成员函数中使用
   * 用于保护对父类容器的同时访问
   *
   * C++语法说明：
   * - mutable：
   *   修饰成员变量
   *   允许在const成员函数中修改
   *   即使对象被声明为const
   * - boost::shared_mutex：
   *   Boost库提供的读写锁
   *   支持多个读者或单个写者
   */
  mutable boost::shared_mutex mutex_;
};

}  // namespace planning
}  // namespace apollo
