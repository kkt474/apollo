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
 * @file box2d.cc
 * @brief 二维包围盒实现文件
 *
 * 本文件实现Box2d类，表示二维空间中的轴对齐或旋转包围盒。
 * Box2d是Apollo中用于碰撞检测的核心数据结构。
 *
 * 主要功能：
 * 1. 包围盒构建（中心点+航向+长宽，或线段+宽度）
 * 2. 点与包围盒关系判断（内部/边界/外部）
 * 3. 包围盒重叠检测（与其他Box2d、线段）
 * 4. 距离计算（到点、到线段、到包围盒）
 * 5. 包围盒变换（旋转、平移、伸缩）
 *
 * C++语法说明：
 * - initializer list: 初始化列表用于成员变量初始化
 * - std::vector<T>: 动态数组容器
 * - std::abs/fabs: 绝对值函数（整数/浮点数）
 * - std::hypot: 计算sqrt(x²+y²)
 * - std::fmax/fmin: 最大/最小值函数
 * - std::swap: 交换两个变量的值
 * - CHECK_GT: 断言宏，检查条件是否满足
 */
#include "modules/common/math/box2d.h"

#include <algorithm>  /**< C++标准算法库，包含std::min, std::max, std::swap等 */
#include <cmath>      /**< C++标准数学库 */
#include <utility>    /**< C++标准实用工具库 */

#include "absl/strings/str_cat.h" /**< Abseil字符串拼接库 */

#include "cyber/common/log.h"     /**< Cyber RT日志系统 */
#include "modules/common/math/math_utils.h" /**< Apollo数学工具库 */
#include "modules/common/math/polygon2d.h"   /**< 二维多边形类 */

namespace apollo {
/**
 * apollo:: - Apollo最外层命名空间
 */
namespace common {
/**
 * common:: - 通用模块命名空间
 */
namespace math {

/**
 * 匿名命名空间
 * 定义文件作用域内的辅助函数
 * 这些函数只在当前翻译单元内可见
 */
namespace {

/**
 * @brief 计算点到线段的距离
 *
 * 计算查询点到线段的最小欧几里得距离。
 * 根据投影位置分为三种情况处理：
 * 1. 投影不在线段上：返回到最近端点的距离
 * 2. 投影在线段内部：返回到线段的垂直距离
 *
 * @param query_x/query_y 查询点坐标
 * @param start_x/start_y 线段起点坐标
 * @param end_x/end_y 线段终点坐标
 * @param length 线段长度
 * @return double 点到线段的距离
 *
 * 语法说明：
 * - 函数定义在匿名命名空间内，表示文件私有
 * - const double: 常量类型，函数内不可修改
 * - hypot(x, y): 标准库函数，计算sqrt(x²+y²)
 * - proj = x0*dx + y0*dy: 向量点积，表示投影量
 * - <= 0.0: 投影不在线段上（在起点之前）
 * - >= length*length: 投影不在线段上（在终点之后）
 * - std::abs(x0*dy - y0*dx) / length: 垂直距离 = 平行四边形面积 / 底边
 */
double PtSegDistance(double query_x, double query_y, double start_x,
                     double start_y, double end_x, double end_y,
                     double length) {
  /**
   * 计算从线段起点到查询点的向量
   * x0 = query_x - start_x
   * y0 = query_y - start_y
   */
  const double x0 = query_x - start_x;
  const double y0 = query_y - start_y;

  /**
   * 计算线段方向向量
   * dx = end_x - start_x
   * dy = end_y - start_y
   */
  const double dx = end_x - start_x;
  const double dy = end_y - start_y;

  /**
   * 计算点积（投影量）
   * proj = x0*dx + y0*dy = |x0|*|dx|*cos(θ)
   * 当proj<=0时，查询点投影不在线段上（在起点之前）
   */
  const double proj = x0 * dx + y0 * dy;

  /**
   * 情况1：投影在起点之前
   * 返回查询点到起点的距离
   */
  if (proj <= 0.0) {
    return hypot(x0, y0);  /**< hypot比sqrt(x*x+y*y)更数值稳定 */
  }

  /**
   * 情况2：投影在终点之后
   * proj >= length² 表示查询点在线段终点之外
   * 返回查询点到终点的距离
   */
  if (proj >= length * length) {
    return hypot(x0 - dx, y0 - dy);  /**< query到终点的距离 */
  }

  /**
   * 情况3：投影在线段上
   * 返回点到线段的垂直距离
   * 公式：|x0*dy - y0*dx| / length
   * 原理：平行四边形面积 = 底 * 高
   *       |x0*dy - y0*dx| = 底 * 高
   *       高 = |x0*dy - y0*dx| / length
   */
  return std::abs(x0 * dy - y0 * dx) / length;
}

}  // namespace

/**
 * @brief Box2d构造函数（中心点+航向+长宽）
 *
 * 使用中心点、航向角、长度和宽度构造旋转包围盒。
 *
 * @param center 包围盒中心点
 * @param heading 航向角（弧度），相对于X轴正方向逆时针
 * @param length 包围盒长度（沿航向方向）
 * @param width 包围盒宽度（垂直航向方向）
 *
 * 语法说明：
 * - : center_(center), length_(length), ... 初始化列表
 *   用于在构造函数体执行之前初始化成员变量
 * - center_(center): 将参数center赋值给成员变量center_
 * - half_length_(length / 2.0): 计算半长
 * - cos_heading_(cos(heading)): 计算航向角的余弦值，缓存以避免重复计算
 * - CHECK_GT(a, b): 断言宏，a > b，否则终止程序
 * - InitCorners(): 调用私有方法初始化四个角点
 */
Box2d::Box2d(const Vec2d &center, const double heading, const double length,
             const double width)
    : center_(center),             /**< 初始化中心点 */
      length_(length),               /**< 初始化长度 */
      width_(width),                 /**< 初始化宽度 */
      half_length_(length / 2.0),   /**< 初始化半长 */
      half_width_(width / 2.0),     /**< 初始化半宽 */
      heading_(heading),             /**< 初始化航向角 */
      cos_heading_(cos(heading)),   /**< 缓存cos(heading) */
      sin_heading_(sin(heading)) {  /**< 缓存sin(heading) */
  /**
   * CHECK_GT - 断言检查
   * 确保length和width大于-kMathEpsilon（允许非常小的值但不能为负）
   * GT = Greater Than
   */
  CHECK_GT(length_, -kMathEpsilon); /**< 断言：length > 0 */
  CHECK_GT(width_, -kMathEpsilon);  /**< 断言：width > 0 */
  InitCorners();  /**< 初始化四个角点坐标 */
}

/**
 * @brief Box2d构造函数（点+前后长度+宽度）
 *
 * 使用参考点、前后长度和宽度构造包围盒。
 * 常用于以前保险杠或后保险杠为参考点的场景。
 *
 * @param point 参考点（通常是车辆前后边缘的中心）
 * @param heading 航向角
 * @param front_length 前边缘到中心的长度
 * @param back_length 后边缘到中心的长度
 * @param width 包围盒宽度
 *
 * 语法说明：
 * - : length_(front_length + back_length) 计算总长度 = 前长 + 后长
 * - delta_length = (front_length - back_length) / 2.0
 *   计算从参考点到几何中心的偏移
 *   如果前后长度相等，偏移为0，参考点就是中心
 * - center_ = Vec2d(point.x() + cos_heading_ * delta_length, ...)
 *   将参考点沿航向方向偏移delta_length得到几何中心
 */
Box2d::Box2d(const Vec2d &point, double heading, double front_length,
             double back_length, double width)
    : length_(front_length + back_length),    /**< 总长度 = 前长 + 后长 */
      width_(width),                           /**< 宽度 */
      half_length_(length_ / 2.0),             /**< 半长 */
      half_width_(width / 2.0),                /**< 半宽 */
      heading_(heading),                        /**< 航向角 */
      cos_heading_(cos(heading)),              /**< cos缓存 */
      sin_heading_(sin(heading)) {             /**< sin缓存 */
  CHECK_GT(length_, -kMathEpsilon);   /**< 检查长度 */
  CHECK_GT(width_, -kMathEpsilon);    /**< 检查宽度 */

  /**
   * 计算从参考点到几何中心的偏移量
   * 例如：前长=2米，后长=1米
   *   delta_length = (2-1)/2 = 0.5米
   *   几何中心在参考点前方0.5米处
   */
  double delta_length = (front_length - back_length) / 2.0;

  /**
   * 计算几何中心
   * point + (cos_heading, sin_heading) * delta_length
   * 沿航向方向偏移delta_length
   */
  center_ = Vec2d(point.x() + cos_heading_ * delta_length,
                  point.y() + sin_heading_ * delta_length);
  InitCorners();  /**< 初始化角点 */
}

/**
 * @brief Box2d构造函数（线段+宽度）
 *
 * 以线段为轴线，线段中心为包围盒中心，构造包围盒。
 *
 * @param axis 作为轴线的线段
 * @param width 包围盒宽度
 *
 * 语法说明：
 * - : center_(axis.center()): 从线段获取中心点
 * - axis.length(): 获取线段长度
 * - axis.heading(): 获取线段航向角
 * - axis.cos_heading(): 获取航向角余弦
 * - axis.sin_heading(): 获取航向角正弦
 */
Box2d::Box2d(const LineSegment2d &axis, const double width)
    : center_(axis.center()),           /**< 线段中心作为包围盒中心 */
      length_(axis.length()),            /**< 包围盒长度 = 线段长度 */
      width_(width),                     /**< 包围盒宽度 */
      half_length_(axis.length() / 2.0),/**< 半长 */
      half_width_(width / 2.0),          /**< 半宽 */
      heading_(axis.heading()),           /**< 航向角 = 线段航向 */
      cos_heading_(axis.cos_heading()),  /**< cos缓存 */
      sin_heading_(axis.sin_heading()) { /**< sin缓存 */
  CHECK_GT(length_, -kMathEpsilon);   /**< 检查长度 */
  CHECK_GT(width_, -kMathEpsilon);    /**< 检查宽度 */
  InitCorners();  /**< 初始化角点 */
}

/**
 * @brief 初始化包围盒四个角点
 *
 * 根据中心点、航向角和尺寸计算四个角点的坐标。
 * 角点顺序：左上→右上→右下→左下（相对于局部坐标系）
 *
 * 局部坐标系：
 * - 以center_为原点
 * - X轴沿航向方向（heading方向）
 * - Y轴垂直于航向（左侧为正）
 *
 * 语法说明：
 * - dx1 = cos_heading_ * half_length_: X轴方向半长向量在全局X的分量
 * - dy1 = sin_heading_ * half_length_: X轴方向半长向量在全局Y的分量
 * - dx2 = sin_heading_ * half_width_: Y轴方向半宽向量在全局X的分量
 * - dy2 = -cos_heading_ * half_width_: Y轴方向半宽向量在全局Y的分量
 * - emplace_back(...): 在vector末尾就地构造元素，避免拷贝
 * - corners_[0] = center + (dx1,dy1) + (dx2,dy2): 左上角
 * - corners_[1] = center + (dx1,dy1) - (dx2,dy2): 右上角
 * - corners_[2] = center - (dx1,dy1) - (dx2,dy2): 右下角
 * - corners_[3] = center - (dx1,dy1) + (dx2,dy2): 左下角
 */
void Box2d::InitCorners() {
  /**
   * 计算半长和半宽在全局坐标系的分量
   */
  const double dx1 = cos_heading_ * half_length_;  /**< X轴方向半长 */
  const double dy1 = sin_heading_ * half_length_;  /**< X轴方向半长 */
  const double dx2 = sin_heading_ * half_width_;   /**< Y轴方向半宽 */
  const double dy2 = -cos_heading_ * half_width_;  /**< Y轴方向半宽 */

  corners_.clear();  /**< 清空角点向量 */

  /**
   * 四个角点坐标计算
   * 中心 + 半长向量 + 半宽向量
   */
  corners_.emplace_back(center_.x() + dx1 + dx2, center_.y() + dy1 + dy2);
  /**
   * 中心 + 半长向量 - 半宽向量
   */
  corners_.emplace_back(center_.x() + dx1 - dx2, center_.y() + dy1 - dy2);
  /**
   * 中心 - 半长向量 - 半宽向量
   */
  corners_.emplace_back(center_.x() - dx1 - dx2, center_.y() - dy1 - dy2);
  /**
   * 中心 - 半长向量 + 半宽向量
   */
  corners_.emplace_back(center_.x() - dx1 + dx2, center_.y() - dy1 + dy2);

  /**
   * 更新包围盒的轴对齐边界
   * for循环遍历所有角点
   * std::fmax/std::fmin: 浮点数最大/最小值
   */
  for (auto &corner : corners_) {
    max_x_ = std::fmax(corner.x(), max_x_);  /**< 更新X最大值 */
    min_x_ = std::fmin(corner.x(), min_x_);  /**< 更新X最小值 */
    max_y_ = std::fmax(corner.y(), max_y_);  /**< 更新Y最大值 */
    min_y_ = std::fmin(corner.y(), min_y_);  /**< 更新Y最小值 */
  }
}

/**
 * @brief Box2d构造函数（轴对齐包围盒）
 *
 * 从轴对齐包围盒(AABox2d)构造Box2d。
 * 轴对齐包围盒没有航向，转换为heading=0的Box2d。
 *
 * @param aabox 轴对齐包围盒
 *
 * 语法说明：
 * - heading_(0.0): 轴对齐盒子航向设为0
 * - cos_heading_(1.0): cos(0) = 1
 * - sin_heading_(0.0): sin(0) = 0
 */
Box2d::Box2d(const AABox2d &aabox)
    : center_(aabox.center()),             /**< 中心点 */
      length_(aabox.length()),               /**< 长度 */
      width_(aabox.width()),                 /**< 宽度 */
      half_length_(aabox.half_length()),    /**< 半长 */
      half_width_(aabox.half_width()),      /**< 半宽 */
      heading_(0.0),                          /**< 航向角为0（轴对齐） */
      cos_heading_(1.0),                     /**< cos(0) = 1 */
      sin_heading_(0.0) {                    /**< sin(0) = 0 */
  CHECK_GT(length_, -kMathEpsilon);   /**< 检查长度 */
  CHECK_GT(width_, -kMathEpsilon);    /**< 检查宽度 */
}

/**
 * @brief 从两个对角点创建轴对齐包围盒
 *
 * 根据一个角点和其对角点创建Box2d。
 *
 * @param one_corner 第一个角点
 * @param opposite_corner 对角点
 * @return Box2d 创建的轴对齐包围盒
 *
 * 语法说明：
 * - std::min(a, b): 返回较小值
 * - std::max(a, b): 返回较大值
 * - {(x1+x2)/2, (y1+y2)/2}: 列表初始化创建中心点
 * - Box2d({...}, 0.0, x2-x1, y2-y1): 创建轴对齐盒子（heading=0）
 */
Box2d Box2d::CreateAABox(const Vec2d &one_corner,
                         const Vec2d &opposite_corner) {
  /**
   * 计算边界
   * x1 = min(one_corner.x, opposite_corner.x) 左侧X
   * x2 = max(one_corner.x, opposite_corner.x) 右侧X
   */
  const double x1 = std::min(one_corner.x(), opposite_corner.x());
  const double x2 = std::max(one_corner.x(), opposite_corner.x());
  const double y1 = std::min(one_corner.y(), opposite_corner.y());
  const double y2 = std::max(one_corner.y(), opposite_corner.y());

  /**
   * 返回轴对齐包围盒
   * 中心点 = ((x1+x2)/2, (y1+y2)/2)
   * 长度 = x2 - x1
   * 宽度 = y2 - y1
   * 航向 = 0
   */
  return Box2d({(x1 + x2) / 2.0, (y1 + y2) / 2.0}, 0.0, x2 - x1, y2 - y1);
}

/**
 * @brief 获取所有角点（指针参数版本）
 *
 * @param corners 输出参数，存储角点的向量指针
 *
 * 语法说明：
 * - std::vector<Vec2d> *const corners: 指向vector的常量指针
 *   - 指针本身是const（不能修改指针）
 *   - 但可以通过指针修改vector内容
 * - if (corners == nullptr): 空指针检查
 * - *corners = corners_: 解引用赋值，拷贝所有角点到输出参数
 */
void Box2d::GetAllCorners(std::vector<Vec2d> *const corners) const {
  if (corners == nullptr) {  /**< 空指针检查 */
    return;  /**< 直接返回 */
  }
  *corners = corners_;  /**< 拷贝所有角点到输出参数 */
}

/**
 * @brief 获取所有角点（引用返回版本）
 *
 * @return const std::vector<Vec2d>& 角点向量的常量引用
 *
 * 语法说明：
 * - const std::vector<Vec2d> &: 常量引用返回
 *   - 避免拷贝，提高效率
 *   - 调用者不能修改返回的vector
 */
const std::vector<Vec2d> &Box2d::GetAllCorners() const { return corners_; }

/**
 * @brief 判断点是否在包围盒内部
 *
 * 使用局部坐标系判断点是否在旋转包围盒内部。
 *
 * 算法：
 * 1. 将点转换到包围盒的局部坐标系
 * 2. 检查局部坐标是否在[-half_length, half_length]和[-half_width, half_width]范围内
 *
 * @param point 要检查的点
 * @return bool 点是否在包围盒内部
 *
 * 语法说明：
 * - const double x0 = point.x() - center_.x(): 点相对于中心的坐标
 * - dx = |x0*cos_heading + y0*sin_heading|: 局部X坐标（投影到航向方向）
 * - dy = |-x0*sin_heading + y0*cos_heading|: 局部Y坐标（投影到垂直航向方向）
 * - std::abs(...): 取绝对值
 * - dx <= half_length_ + kMathEpsilon: 检查是否在范围内
 *   添加epsilon容差处理浮点数精度问题
 */
bool Box2d::IsPointIn(const Vec2d &point) const {
  /**
   * 计算点到中心的偏移
   */
  const double x0 = point.x() - center_.x();
  const double y0 = point.y() - center_.y();

  /**
   * 转换到局部坐标系
   * 局部X = 全局X * cos(heading) + 全局Y * sin(heading)
   * 局部Y = -全局X * sin(heading) + 全局Y * cos(heading)
   */
  const double dx = std::abs(x0 * cos_heading_ + y0 * sin_heading_);
  const double dy = std::abs(-x0 * sin_heading_ + y0 * cos_heading_);

  /**
   * 检查是否在范围内
   * 添加kMathEpsilon作为容差
   */
  return dx <= half_length_ + kMathEpsilon && dy <= half_width_ + kMathEpsilon;
}

/**
 * @brief 判断点是否在包围盒边界上
 *
 * @param point 要检查的点
 * @return bool 点是否在边界上
 *
 * 语法说明：
 * - (std::abs(dx - half_length_) <= kMathEpsilon && dy <= half_width_ + kMathEpsilon)
 *   检查是否在左或右边线上
 * - (std::abs(dy - half_width_) <= kMathEpsilon && dx <= half_length_ + kMathEpsilon)
 *   检查是否在上或下边线上
 * - ||: 逻辑或运算符
 */
bool Box2d::IsPointOnBoundary(const Vec2d &point) const {
  const double x0 = point.x() - center_.x();
  const double y0 = point.y() - center_.y();

  /**
   * 计算在局部坐标系的坐标
   */
  const double dx = std::abs(x0 * cos_heading_ + y0 * sin_heading_);
  const double dy = std::abs(-x0 * sin_heading_ + y0 * cos_heading_);

  /**
   * 判断是否在左/右边线 或 上/下边线
   * 使用epsilon容差判断是否"恰好"在边界上
   */
  return (std::abs(dx - half_length_) <= kMathEpsilon &&
          dy <= half_width_ + kMathEpsilon) ||
         (std::abs(dy - half_width_) <= kMathEpsilon &&
          dx <= half_length_ + kMathEpsilon);
}

/**
 * @brief 计算点到包围盒的距离
 *
 * 计算点到包围盒的最小欧几里得距离。
 *
 * @param point 查询点
 * @return double 点到包围盒的距离
 *
 * 语法说明：
 * - if (dx <= 0.0): 点在X方向已在内侧
 *   - return std::max(0.0, dy): 如果Y也在内侧返回0，否则返回dy
 * - if (dy <= 0.0): 点在Y方向已在内侧
 *   - return dx: 返回X方向的距离
 * - return hypot(dx, dy): 点在角点外侧，返回到最近边的距离
 */
double Box2d::DistanceTo(const Vec2d &point) const {
  const double x0 = point.x() - center_.x();
  const double y0 = point.y() - center_.y();

  /**
   * 计算到包围盒边缘的偏移
   * 如果dx<0表示点在内侧，dy<0表示点在内侧
   */
  const double dx =
      std::abs(x0 * cos_heading_ + y0 * sin_heading_) - half_length_;
  const double dy =
      std::abs(-x0 * sin_heading_ + y0 * cos_heading_) - half_width_;

  /**
   * 根据dx和dy的值分情况计算距离
   */
  if (dx <= 0.0) {
    /**
     * 点在X方向已包围盒内侧
     * 如果Y也在内侧，返回0
     * 否则返回dy（Y方向距离）
     */
    return std::max(0.0, dy);
  }
  if (dy <= 0.0) {
    /**
     * 点在Y方向已包围盒内侧
     * 返回dx（X方向距离）
     */
    return dx;
  }
  /**
   * 点在角点外侧
   * 返回到最近角的欧氏距离
   */
  return hypot(dx, dy);
}

/**
 * @brief 判断线段是否与包围盒重叠
 *
 * 碰撞检测的核心函数。
 * 使用分离轴定理（Separating Axis Theorem）判断。
 *
 * 算法：
 * 1. 快速排斥测试（轴对齐边界检查）
 * 2. 将线段端点转换到包围盒局部坐标系
 * 3. 检查端点是否在盒内
 * 4. 检查线段是否与盒子边相交
 *
 * @param line_segment 要检查的线段
 * @return bool 是否重叠
 *
 * 语法说明：
 * - line_segment.length(): 获取线段长度
 * - std::fmax/std::fmin: 浮点数最大/最小值
 * - corners_[2]: 左下角点（用于构建局部坐标系）
 * - Vec2d(x_axis, y_axis): 使用两个分量构造向量
 * - is_inside_rectangle(): 检查点是否在矩形内
 */
bool Box2d::HasOverlap(const LineSegment2d &line_segment) const {
  /**
   * 长度过小的线段视为点处理
   */
  if (line_segment.length() <= kMathEpsilon) {
    return IsPointIn(line_segment.start());  /**< 检查点是否在盒内 */
  }

  /**
   * 快速排斥测试
   * 如果线段的X坐标范围与包围盒完全不重叠，返回false
   * std::fmax: 返回两个浮点数的较大值
   */
  if (std::fmax(line_segment.start().x(), line_segment.end().x()) < min_x() ||
      std::fmin(line_segment.start().x(), line_segment.end().x()) > max_x() ||
      std::fmax(line_segment.start().y(), line_segment.end().y()) < min_y() ||
      std::fmin(line_segment.start().y(), line_segment.end().y()) > max_y()) {
    return false;  /**< 快速排斥，不重叠 */
  }

  /**
   * 构建以左下角为原点的局部坐标系
   * y轴沿heading方向
   * x轴垂直于heading方向
   */
  Vec2d x_axis(sin_heading_, -cos_heading_);   /**< X轴（垂直于heading） */
  Vec2d y_axis(cos_heading_, sin_heading_);   /**< Y轴（沿heading方向） */

  /**
   * corners_[2]是左下角点
   * 将线段起点转换到局部坐标系
   */
  Vec2d start_v = line_segment.start() - corners_[2];
  Vec2d start_point(start_v.InnerProd(x_axis), start_v.InnerProd(y_axis));

  /**
   * 检查起点是否在矩形内
   */
  if (is_inside_rectangle(start_point)) {
    return true;  /**< 起点在盒内，重叠 */
  }

  /**
   * 检查终点是否在矩形内
   */
  Vec2d end_v = line_segment.end() - corners_[2];
  Vec2d end_point(end_v.InnerProd(x_axis), end_v.InnerProd(y_axis));
  if (is_inside_rectangle(end_point)) {
    return true;  /**< 终点在盒内，重叠 */
  }

  /**
   * 检查线段两个端点是否都在矩形的同一侧
   * 如果都在左侧、右侧、上方或下方，则不相交
   */
  if ((start_point.x() < 0.0) && (end_point.x() < 0.0)) {
    return false;  /**< 都在左侧，不相交 */
  }
  if ((start_point.y() < 0.0) && (end_point.y() < 0.0)) {
    return false;  /**< 都在下方，不相交 */
  }
  if ((start_point.x() > width_) && (end_point.x() > width_)) {
    return false;  /**< 都在右侧，不相交 */
  }
  if ((start_point.y() > length_) && (end_point.y() > length_)) {
    return false;  /**< 都在上方，不相交 */
  }

  /**
   * 检查线段是否与盒子边相交
   */
  Vec2d line_direction = line_segment.end() - line_segment.start();  /**< 线段方向向量 */
  Vec2d normal_vec(line_direction.y(), -line_direction.x());  /**< 法向量（垂直于线段） */

  Vec2d p1 = center_ - line_segment.start();  /**< 中心到线段起点的向量 */
  Vec2d diagonal_vec = center_ - corners_[0]; /**< 中心到右上角的向量 */

  /**
   * 投影测试
   * 如果对角线在法向量上的投影小于p1的投影，则相交
   */
  double project_p1 = fabs(p1.InnerProd(normal_vec));  /**< p1在法向量上的投影 */
  if (fabs(diagonal_vec.InnerProd(normal_vec)) >= project_p1) {
    return true;  /**< 与右上角-中心对角线相交 */
  }

  diagonal_vec = center_ - corners_[1];  /**< 中心到左上角的向量 */
  if (fabs(diagonal_vec.InnerProd(normal_vec)) >= project_p1) {
    return true;  /**< 与左上角-中心对角线相交 */
  }

  return false;  /**< 不相交 */
}

/**
 * @brief 计算点到线段的距离
 *
 * 使用Cohen-Sutherland-like算法分类查询点位置，
 * 然后根据不同情况计算距离。
 *
 * @param line_segment 查询线段
 * @return double 点到线段的距离
 *
 * 语法说明：
 * - gx1/gy1: 点在X/Y方向的分区（-1=左侧/下方, 0=内部, 1=右侧/上方）
 * - int gx1 = (x1 >= box_x ? 1 : (x1 <= -box_x ? -1 : 0)):
 *   三元运算符，判断点相对于包围盒的位置
 * - switch-case: 根据gx2*3+gy2的组合决定计算方法
 * - std::swap(x1, y1): 交换两个变量的值
 */
double Box2d::DistanceTo(const LineSegment2d &line_segment) const {
  /**
   * 长度过小的线段视为点
   */
  if (line_segment.length() <= kMathEpsilon) {
    return DistanceTo(line_segment.start());  /**< 返回点到起点的距离 */
  }

  /**
   * 将线段端点转换到局部坐标系
   */
  const double ref_x1 = line_segment.start().x() - center_.x();
  const double ref_y1 = line_segment.start().y() - center_.y();

  /**
   * 旋转到局部坐标系
   */
  double x1 = ref_x1 * cos_heading_ + ref_y1 * sin_heading_;
  double y1 = ref_x1 * sin_heading_ - ref_y1 * cos_heading_;
  double box_x = half_length_;   /**< 包围盒半长 */
  double box_y = half_width_;    /**< 包围盒半宽 */

  /**
   * 计算分区码
   * gx = 1: x > box_x (右侧)
   * gx = 0: -box_x <= x <= box_x (内部)
   * gx = -1: x < -box_x (左侧)
   */
  int gx1 = (x1 >= box_x ? 1 : (x1 <= -box_x ? -1 : 0));
  int gy1 = (y1 >= box_y ? 1 : (y1 <= -box_y ? -1 : 0));

  /**
   * 如果起点在盒内，距离为0
   */
  if (gx1 == 0 && gy1 == 0) {
    return 0.0;
  }

  /**
   * 对终点做同样处理
   */
  const double ref_x2 = line_segment.end().x() - center_.x();
  const double ref_y2 = line_segment.end().y() - center_.y();
  double x2 = ref_x2 * cos_heading_ + ref_y2 * sin_heading_;
  double y2 = ref_x2 * sin_heading_ - ref_y2 * cos_heading_;
  int gx2 = (x2 >= box_x ? 1 : (x2 <= -box_x ? -1 : 0));
  int gy2 = (y2 >= box_y ? 1 : (y2 <= -box_y ? -1 : 0));

  if (gx2 == 0 && gy2 == 0) {
    return 0.0;
  }

  /**
   * 确保gx1和gy1为非负
   */
  if (gx1 < 0 || (gx1 == 0 && gx2 < 0)) {
    x1 = -x1;
    gx1 = -gx1;
    x2 = -x2;
    gx2 = -gx2;
  }
  if (gy1 < 0 || (gy1 == 0 && gy2 < 0)) {
    y1 = -y1;
    gy1 = -gy1;
    y2 = -y2;
    gy2 = -gy2;
  }

  /**
   * 交换使gx1 >= gy1
   */
  if (gx1 < gy1 || (gx1 == gy1 && gx2 < gy2)) {
    std::swap(x1, y1);
    std::swap(gx1, gy1);
    std::swap(x2, y2);
    std::swap(gx2, gy2);
    std::swap(box_x, box_y);
  }

  /**
   * 根据分区码计算距离
   * switch-case处理各种情况
   * PtSegDistance: 计算点到线段的距离
   * CrossProd: 计算叉积，判断位置关系
   */
  if (gx1 == 1 && gy1 == 1) {
    switch (gx2 * 3 + gy2) {
      case 4:  /**< gx2=1, gy2=1 */
        return PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                             line_segment.length());
      case 3:  /**< gx2=1, gy2=0 */
        return (x1 > x2) ? (x2 - box_x)
                         : PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                                         line_segment.length());
      case 2:  /**< gx2=1, gy2=-1 */
        return (x1 > x2) ? PtSegDistance(box_x, -box_y, x1, y1, x2, y2,
                                         line_segment.length())
                         : PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                                         line_segment.length());
      case -1: /**< gx2=0, gy2=-1 */
        return CrossProd({x1, y1}, {x2, y2}, {box_x, -box_y}) >= 0.0
                   ? 0.0
                   : PtSegDistance(box_x, -box_y, x1, y1, x2, y2,
                                   line_segment.length());
      case -4: /**< gx2=-1, gy2=-1 */
        return CrossProd({x1, y1}, {x2, y2}, {box_x, -box_y}) <= 0.0
                   ? PtSegDistance(box_x, -box_y, x1, y1, x2, y2,
                                   line_segment.length())
                   : (CrossProd({x1, y1}, {x2, y2}, {-box_x, box_y}) <= 0.0
                          ? 0.0
                          : PtSegDistance(-box_x, box_y, x1, y1, x2, y2,
                                          line_segment.length()));
    }
  } else {
    switch (gx2 * 3 + gy2) {
      case 4:  /**< gx2=1, gy2=1 */
        return (x1 < x2) ? (x1 - box_x)
                         : PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                                         line_segment.length());
      case 3:  /**< gx2=1, gy2=0 */
        return std::min(x1, x2) - box_x;
      case 1:  /**< gx2=0, gy2=1 */
      case -2: /**< gx2=-1, gy2=0 */
        return CrossProd({x1, y1}, {x2, y2}, {box_x, box_y}) <= 0.0
                   ? 0.0
                   : PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                                   line_segment.length());
      case -3: /**< gx2=-1, gy2=0 */
        return 0.0;
    }
  }

  /**
   * 未处理的状态，记录错误
   */
  ACHECK(0) << "unimplemented state: " << gx1 << " " << gy1 << " " << gx2 << " "
            << gy2;
  return 0.0;
}

/**
 * @brief 计算包围盒到另一个包围盒的距离
 *
 * 通过将Box2d转换为Polygon2d计算。
 *
 * @param box 另一个包围盒
 * @return double 到另一个包围盒的距离
 *
 * 语法说明：
 * - Polygon2d(box): 构造函数，从Box2d创建多边形
 * - .DistanceTo(*this): 调用多边形的距离计算方法
 */
double Box2d::DistanceTo(const Box2d &box) const {
  return Polygon2d(box).DistanceTo(*this);
}

/**
 * @brief 判断两个包围盒是否重叠
 *
 * 使用分离轴定理（SAT）判断。
 * 需要检查4个轴方向的投影重叠情况。
 *
 * @param box 另一个包围盒
 * @return bool 是否重叠
 *
 * 语法说明：
 * - 快速排斥测试：检查轴对齐边界
 * - 然后检查4个分离轴：
 *   1. this的航向轴
 *   2. this的垂直轴
 *   3. box的航向轴
 *   4. box的垂直轴
 * - shift_x/shift_y: 两个中心点的偏移
 * - dx1/dy1, dx2/dy2: this的半长和半宽向量
 * - dx3/dy3, dx4/dy4: box的半长和半宽向量
 */
bool Box2d::HasOverlap(const Box2d &box) const {
  /**
   * 快速排斥测试
   * 如果轴对齐边界完全不重叠，返回false
   */
  if (box.max_x() < min_x() || box.min_x() > max_x() || box.max_y() < min_y() ||
      box.min_y() > max_y()) {
    return false;  /**< 不重叠 */
  }

  /**
   * 计算中心点偏移
   */
  const double shift_x = box.center_x() - center_.x();
  const double shift_y = box.center_y() - center_.y();

  /**
   * 计算this的半向量
   */
  const double dx1 = cos_heading_ * half_length_;
  const double dy1 = sin_heading_ * half_length_;
  const double dx2 = sin_heading_ * half_width_;
  const double dy2 = -cos_heading_ * half_width_;

  /**
   * 计算box的半向量
   */
  const double dx3 = box.cos_heading() * box.half_length();
  const double dy3 = box.sin_heading() * box.half_length();
  const double dx4 = box.sin_heading() * box.half_width();
  const double dy4 = -box.cos_heading() * box.half_width();

  /**
   * 检查4个分离轴
   * 1. this的航向轴方向
   * 2. this的垂直航向轴方向
   * 3. box的航向轴方向
   * 4. box的垂直航向轴方向
   *
   * 使用绝对值确保正确处理负偏移
   */
  return std::abs(shift_x * cos_heading_ + shift_y * sin_heading_) <=
             std::abs(dx3 * cos_heading_ + dy3 * sin_heading_) +
                 std::abs(dx4 * cos_heading_ + dy4 * sin_heading_) +
                 half_length_ &&
         std::abs(shift_x * sin_heading_ - shift_y * cos_heading_) <=
             std::abs(dx3 * sin_heading_ - dy3 * cos_heading_) +
                 std::abs(dx4 * sin_heading_ - dy4 * cos_heading_) +
                 half_width_ &&
         std::abs(shift_x * box.cos_heading() + shift_y * box.sin_heading()) <=
             std::abs(dx1 * box.cos_heading() + dy1 * box.sin_heading()) +
                 std::abs(dx2 * box.cos_heading() + dy2 * box.sin_heading()) +
                 box.half_length() &&
         std::abs(shift_x * box.sin_heading() - shift_y * box.cos_heading()) <=
             std::abs(dx1 * box.sin_heading() - dy1 * box.cos_heading()) +
                 std::abs(dx2 * box.sin_heading() - dy2 * box.cos_heading()) +
                 box.half_width();
}

/**
 * @brief 获取包围盒的轴对齐包围盒
 *
 * @return AABox2d 轴对齐包围盒
 *
 * 语法说明：
 * - std::abs(...): 取绝对值
 * - dx1 = |cos_heading * half_length|: 长度方向对角线在X的投影
 * - AABox2d(center, length, width): 构造轴对齐包围盒
 */
AABox2d Box2d::GetAABox() const {
  const double dx1 = std::abs(cos_heading_ * half_length_);
  const double dy1 = std::abs(sin_heading_ * half_length_);
  const double dx2 = std::abs(sin_heading_ * half_width_);
  const double dy2 = std::abs(cos_heading_ * half_width_);

  /**
   * 返回轴对齐包围盒
   * 长度 = 2 * (|dx1| + |dx2|)
   * 宽度 = 2 * (|dy1| + |dy2|)
   */
  return AABox2d(center_, (dx1 + dx2) * 2.0, (dy1 + dy2) * 2.0);
}

/**
 * @brief 绕中心点旋转包围盒
 *
 * @param rotate_angle 旋转角度（弧度），逆时针为正
 *
 * 语法说明：
 * - NormalizeAngle(): 将角度归一化到[-π, π]范围
 * - heading_ = NormalizeAngle(heading_ + rotate_angle): 更新航向角
 * - cos_heading_ = std::cos(heading_): 重新计算cos缓存
 * - sin_heading_ = std::sin(heading_): 重新计算sin缓存
 * - InitCorners(): 重新初始化角点（角点位置会改变）
 */
void Box2d::RotateFromCenter(const double rotate_angle) {
  heading_ = NormalizeAngle(heading_ + rotate_angle);  /**< 更新航向角 */
  cos_heading_ = std::cos(heading_);   /**< 重新计算cos */
  sin_heading_ = std::sin(heading_);   /**< 重新计算sin */
  InitCorners();  /**< 重新初始化角点 */
}

/**
 * @brief 平移包围盒
 *
 * @param shift_vec 平移向量
 *
 * 语法说明：
 * - center_ += shift_vec: 中心点加上平移向量
 * - corners_[i] += shift_vec: 所有角点都加上平移向量
 * - for循环：遍历所有4个角点
 */
void Box2d::Shift(const Vec2d &shift_vec) {
  center_ += shift_vec;  /**< 移动中心点 */
  for (size_t i = 0; i < 4; ++i) {  /**< 遍历4个角点 */
    corners_[i] += shift_vec;  /**< 移动每个角点 */
  }
  for (auto &corner : corners_) {  /**< 重新计算边界 */
    max_x_ = std::fmax(corner.x(), max_x_);
    min_x_ = std::fmin(corner.x(), min_x_);
    max_y_ = std::fmax(corner.y(), max_y_);
    min_y_ = std::fmin(corner.y(), min_y_);
  }
}

/**
 * @brief 沿长度方向延伸包围盒
 *
 * @param extension_length 延伸长度，正值为向前延伸
 *
 * 语法说明：
 * - length_ += extension_length: 增加总长度
 * - half_length_ += extension_length / 2.0: 半长增加一半
 * - InitCorners(): 重新初始化角点
 */
void Box2d::LongitudinalExtend(const double extension_length) {
  length_ += extension_length;  /**< 增加总长度 */
  half_length_ += extension_length / 2.0;  /**< 半长增加 */
  InitCorners();  /**< 重新初始化角点 */
}

/**
 * @brief 沿宽度方向延伸包围盒
 *
 * @param extension_length 延伸宽度，正值为向两侧延伸
 *
 * 语法说明：
 * - width_ += extension_length: 增加总宽度
 * - half_width_ += extension_length / 2.0: 半宽增加一半
 * - InitCorners(): 重新初始化角点
 */
void Box2d::LateralExtend(const double extension_length) {
  width_ += extension_length;  /**< 增加总宽度 */
  half_width_ += extension_length / 2.0;  /**< 半宽增加 */
  InitCorners();  /**< 重新初始化角点 */
}

/**
 * @brief 获取调试字符串
 *
 * @return std::string 格式化的调试信息
 *
 * 语法说明：
 * - absl::StrCat(...): Abseil字符串拼接
 * - center_.DebugString(): 调用Vec2d的DebugString方法
 */
std::string Box2d::DebugString() const {
  return absl::StrCat("box2d ( center = ", center_.DebugString(),
                      "  heading = ", heading_, "  length = ", length_,
                      "  width = ", width_, " )");
}

}  // namespace math
}  // namespace common
}  // namespace apollo
