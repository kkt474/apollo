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
 * @file vec2d.cc
 * @brief 二维向量实现文件
 *
 * 本文件实现Vec2d类，表示二维欧几里得空间中的向量/点。
 * Vec2d是Apollo中广泛使用的基础数学类型，用于位置、速度、位移等计算。
 *
 * 主要功能：
 * 1. 向量基本运算（加减乘除、点积、叉积）
 * 2. 向量长度和归一化
 * 3. 向量旋转
 * 4. 距离计算
 * 5. 角度计算
 *
 * C++语法说明：
 * - const成员函数: 函数后加const，this指针是const的
 * - operator重载: 定义运算符行为
 * - std::hypot(): 标准库函数，计算sqrt(x² + y²)
 * - std::atan2(): 标准库函数，计算atan2(y, x)
 * - std::cos/sin(): 标准库三角函数
 * - std::abs(): 绝对值函数
 * - kMathEpsilon: 数学epsilon常量，用于浮点数比较
 */
#include "modules/common/math/vec2d.h"

#include <cmath>  /**< C++标准数学库，包含hypot, atan2, cos, sin等 */

#include "absl/strings/str_cat.h" /**< Abseil字符串拼接库 */

#include "cyber/common/log.h"  /**< Cyber RT日志系统 */

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
 * @brief 创建单位向量
 *
 * 根据给定角度创建一个单位向量（长度为1）。
 * 单位向量的方向由角度决定，长度为1。
 *
 * 数学公式：
 *   unit_vec = (cos(angle), sin(angle))
 *
 * @param angle 角度（弧度），从X轴正方向逆时针测量
 * @return Vec2d 单位向量
 *
 * 语法说明：
 * - static函数: 不依赖于对象实例的函数，可通过类名直接调用
 * - Vec2d(x, y): 构造函数，创建指定坐标的向量
 * - std::cos(angle): 标准库余弦函数
 * - std::sin(angle): 标准库正弦函数
 */
Vec2d Vec2d::CreateUnitVec2d(const double angle) {
  /**
   * 返回单位向量
   * (cos(angle), sin(angle)) 是单位圆的参数化表示
   * 当angle=0时，返回(1, 0)，即X轴正方向
   */
  return Vec2d(std::cos(angle), std::sin(angle));
}

/**
 * @brief 计算向量长度
 *
 * 返回向量的欧几里得长度（模）。
 *
 * 数学公式：
 *   |v| = sqrt(x² + y²)
 *
 * @return double 向量长度
 *
 * 语法说明：
 * - const成员函数: 函数后加const，表示不会修改成员变量
 * - std::hypot(x, y): 标准库函数，计算sqrt(x² + y²)
 *   比直接计算更数值稳定，避免溢出
 */
double Vec2d::Length() const { return std::hypot(x_, y_); }

/**
 * @brief 计算向量长度的平方
 *
 * 返回向量长度平方，避免开方运算，提高效率。
 * 比较距离大小时常用此方法。
 *
 * 数学公式：
 *   |v|² = x² + y²
 *
 * @return double 向量长度平方
 *
 * 语法说明：
 * - x_ * x_: 成员变量x_与自身相乘
 * - 返回值不需要开方，因此更高效
 */
double Vec2d::LengthSquare() const { return x_ * x_ + y_ * y_; }

/**
 * @brief 计算向量的角度
 *
 * 返回向量的极坐标角度（从X轴正方向逆时针测量）。
 * atan2比atan更安全，可以正确处理所有象限。
 *
 * 数学公式：
 *   angle = atan2(y, x)
 *
 * 返回值范围：[-π, π]
 *
 * @return double 角度（弧度）
 *
 * 语法说明：
 * - std::atan2(y, x): 标准库函数，计算y/x的反正切
 *   - 返回值范围是(-π, π]
 *   - 可以正确处理x=0或y=0的情况
 *   - 可以判断向量所在的象限
 */
double Vec2d::Angle() const { return std::atan2(y_, x_); }

/**
 * @brief 向量归一化
 *
 * 将向量缩放为单位长度（长度为1）。
 * 归一化后的向量保持原方向，但长度为1。
 *
 * 数学公式：
 *   v_normalized = v / |v|
 *
 * 注意：如果向量长度小于epsilon，不进行归一化（避免除零）。
 *
 * 语法说明：
 * - void函数: 不返回值的函数
 * - const double l = Length(): 在函数内创建局部常量
 * - if (l > kMathEpsilon): 检查长度是否足够大
 * - x_ /= l: 复合赋值运算符，等价于x_ = x_ / l
 */
void Vec2d::Normalize() {
  /**
   * 计算向量长度
   */
  const double l = Length();

  /**
   * 检查长度是否大于epsilon
   * kMathEpsilon是类中定义的数学epsilon常量
   * 如果长度太小，不进行归一化
   */
  if (l > kMathEpsilon) {
    x_ /= l;  /**< X分量除以长度 */
    y_ /= l;  /**< Y分量除以长度 */
  }
}

/**
 * @brief 计算到另一个点的距离
 *
 * 计算当前向量到另一个向量的欧几里得距离。
 *
 * 数学公式：
 *   distance = sqrt((x1-x2)² + (y1-y2)²)
 *
 * @param other 另一个向量
 * @return double 距离
 *
 * 语法说明：
 * - const Vec2d &other: 常量引用参数，函数内只读
 * - std::hypot(dx, dy): 计算sqrt(dx² + dy²)
 * - x_ - other.x(): 访问other的x分量，other.x()是getter方法
 */
double Vec2d::DistanceTo(const Vec2d &other) const {
  return std::hypot(x_ - other.x_, y_ - other.y_);
}

/**
 * @brief 计算到另一个点的距离平方
 *
 * 计算距离的平方，避免开方运算。
 * 比较距离大小时常用此方法，更高效。
 *
 * 数学公式：
 *   dist² = (x1-x2)² + (y1-y2)²
 *
 * @param other 另一个向量
 * @return double 距离平方
 *
 * 语法说明：
 * - const double dx = ...: 创建局部常量存储X差值
 * - return dx * dx + dy * dy: 计算距离平方
 */
double Vec2d::DistanceSquareTo(const Vec2d &other) const {
  const double dx = x_ - other.x();  /**< X分量差值 */
  const double dy = y_ - other.y();  /**< Y分量差值 */
  return dx * dx + dy * dy;  /**< 返回距离平方 */
}

/**
 * @brief 计算叉积（叉乘）
 *
 * 计算当前向量与另一个向量的二维叉积。
 * 叉积返回一个标量（不是向量）。
 *
 * 数学公式：
 *   cross = x1 * y2 - y1 * x2
 *        = |v1| * |v2| * sin(θ)
 *
 * 几何意义：
 * - cross > 0: other在当前向量的左侧（逆时针方向）
 * - cross < 0: other在当前向量的右侧（顺时针方向）
 * - cross = 0: 两向量平行（共线）
 *
 * @param other 另一个向量
 * @return double 叉积结果
 *
 * 语法说明：
 * - x_ * other.y(): 访问other的y分量
 * - other.x(): 调用getter方法获取x值
 */
double Vec2d::CrossProd(const Vec2d &other) const {
  return x_ * other.y() - y_ * other.x();
}

/**
 * @brief 计算点积（点乘、内积）
 *
 * 计算当前向量与另一个向量的点积。
 *
 * 数学公式：
 *   dot = x1 * x2 + y1 * y2
 *       = |v1| * |v2| * cos(θ)
 *
 * 几何意义：
 * - dot > 0: 两向量夹角小于90°
 * - dot < 0: 两向量夹角大于90°
 * - dot = 0: 两向量垂直
 *
 * @param other 另一个向量
 * @return double 点积结果
 *
 * 语法说明：
 * - x_ * other.x(): X分量相乘
 * - + y_ * other.y(): Y分量相乘后相加
 */
double Vec2d::InnerProd(const Vec2d &other) const {
  return x_ * other.x() + y_ * other.y();
}

/**
 * @brief 旋转向量
 *
 * 将当前向量绕原点逆时针旋转指定角度，返回旋转后的新向量。
 * 当前向量不变。
 *
 * 数学公式（旋转矩阵）：
 *   x' = x * cos(θ) - y * sin(θ)
 *   y' = x * sin(θ) + y * cos(θ)
 *
 * @param angle 旋转角度（弧度），逆时针为正
 * @return Vec2d 旋转后的新向量
 *
 * 语法说明：
 * - const函数: 不修改成员变量
 * - return Vec2d(...): 返回新创建的向量
 * - std::cos/std::sin: 标准库三角函数
 */
Vec2d Vec2d::rotate(const double angle) const {
  /**
   * 应用旋转矩阵公式
   * x' = x * cos(θ) - y * sin(θ)
   * y' = x * sin(θ) + y * cos(θ)
   */
  return Vec2d(x_ * cos(angle) - y_ * sin(angle),
               x_ * sin(angle) + y_ * cos(angle));
}

/**
 * @brief 原地旋转向量
 *
 * 将当前向量绕原点逆时针旋转指定角度。
 * 与rotate()不同，此函数会修改当前向量。
 *
 * @param angle 旋转角度（弧度），逆时针为正
 *
 * 语法说明：
 * - 非const函数: 会修改成员变量x_和y_
 * - double tmp_x = x_: 需要临时变量保存原x值
 *   因为y_的新值计算需要用到原来的x_
 */
void Vec2d::SelfRotate(const double angle) {
  double tmp_x = x_;  /**< 保存原X分量 */
  /**
   * 计算新X分量
   * x' = x * cos(θ) - y * sin(θ)
   */
  x_ = x_ * cos(angle) - y_ * sin(angle);

  /**
   * 计算新Y分量
   * y' = tmp_x * sin(θ) + y * cos(θ)
   * 使用tmp_x而不是x_，因为x_已被修改
   */
  y_ = tmp_x * sin(angle) + y_ * cos(angle);
}

/**
 * @brief 向量加法运算符重载
 *
 * 将当前向量与另一个向量相加。
 *
 * 数学公式：
 *   result = (x1 + x2, y1 + y2)
 *
 * @param other 要加的向量
 * @return Vec2d 相加后的新向量
 *
 * 语法说明：
 * - Vec2d operator+(...): 重载+运算符
 * - const Vec2d &other: 参数为常量引用
 * - const函数: 不修改成员变量
 */
Vec2d Vec2d::operator+(const Vec2d &other) const {
  return Vec2d(x_ + other.x(), y_ + other.y());
}

/**
 * @brief 向量减法运算符重载
 *
 * 将当前向量减去另一个向量。
 *
 * 数学公式：
 *   result = (x1 - x2, y1 - y2)
 *
 * @param other 要减去的向量
 * @return Vec2d 相减后的新向量
 *
 * 语法说明：
 * - Vec2d operator-(...): 重载-运算符
 */
Vec2d Vec2d::operator-(const Vec2d &other) const {
  return Vec2d(x_ - other.x(), y_ - other.y());
}

/**
 * @brief 向量数乘运算符重载
 *
 * 将向量与标量相乘。
 *
 * 数学公式：
 *   result = (x * ratio, y * ratio)
 *
 * @param ratio 标量乘数
 * @return Vec2d 缩放后的新向量
 *
 * 语法说明：
 * - Vec2d operator*(const double ratio): 成员函数形式的*
 */
Vec2d Vec2d::operator*(const double ratio) const {
  return Vec2d(x_ * ratio, y_ * ratio);
}

/**
 * @brief 向量数除运算符重载
 *
 * 将向量除以标量。
 *
 * 数学公式：
 *   result = (x / ratio, y / ratio)
 *
 * @param ratio 非零标量除数
 * @return Vec2d 缩放后的新向量
 *
 * 语法说明：
 * - CHECK_GT(...): 断言宏，检查ratio的绝对值大于epsilon
 * - std::abs(ratio): 取ratio的绝对值
 * - kMathEpsilon: 数学epsilon常量，防止除零
 */
Vec2d Vec2d::operator/(const double ratio) const {
  CHECK_GT(std::abs(ratio), kMathEpsilon);  /**< 断言：ratio ≠ 0 */
  return Vec2d(x_ / ratio, y_ / ratio);
}

/**
 * @brief 向量加法赋值运算符重载
 *
 * 将当前向量加上另一个向量，结果存回当前向量。
 *
 * @param other 要加的向量
 * @return Vec2d& 返回当前向量的引用（支持链式调用）
 *
 * 语法说明：
 * - Vec2d &operator+=(...): 返回引用，允许a += b += c这样的链式调用
 * - return *this: 返回当前对象的引用
 */
Vec2d &Vec2d::operator+=(const Vec2d &other) {
  x_ += other.x();  /**< X分量相加 */
  y_ += other.y();  /**< Y分量相加 */
  return *this;    /**< 返回当前对象引用 */
}

/**
 * @brief 向量减法赋值运算符重载
 *
 * 将当前向量减去另一个向量，结果存回当前向量。
 *
 * @param other 要减去的向量
 * @return Vec2d& 返回当前向量的引用
 */
Vec2d &Vec2d::operator-=(const Vec2d &other) {
  x_ -= other.x();  /**< X分量相减 */
  y_ -= other.y();  /**< Y分量相减 */
  return *this;    /**< 返回当前对象引用 */
}

/**
 * @brief 向量数乘赋值运算符重载
 *
 * 将当前向量乘以标量，结果存回当前向量。
 *
 * @param ratio 标量乘数
 * @return Vec2d& 返回当前向量的引用
 *
 * 语法说明：
 * - x_ *= ratio: 等价于x_ = x_ * ratio
 */
Vec2d &Vec2d::operator*=(const double ratio) {
  x_ *= ratio;  /**< X分量乘以ratio */
  y_ *= ratio;  /**< Y分量乘以ratio */
  return *this;  /**< 返回当前对象引用 */
}

/**
 * @brief 向量数除赋值运算符重载
 *
 * 将当前向量除以标量，结果存回当前向量。
 *
 * @param ratio 非零标量除数
 * @return Vec2d& 返回当前向量的引用
 */
Vec2d &Vec2d::operator/=(const double ratio) {
  CHECK_GT(std::abs(ratio), kMathEpsilon);  /**< 断言：ratio ≠ 0 */
  x_ /= ratio;  /**< X分量除以ratio */
  y_ /= ratio;  /**< Y分量除以ratio */
  return *this;  /**< 返回当前对象引用 */
}

/**
 * @brief 向量相等比较运算符重载
 *
 * 判断两个向量是否相等（考虑浮点数精度）。
 * 使用epsilon进行浮点数比较。
 *
 * @param other 要比较的向量
 * @return bool 是否相等
 *
 * 语法说明：
 * - bool operator==(...): 返回布尔值
 * - std::abs(x_ - other.x()) < kMathEpsilon: 判断两个浮点数是否足够接近
 *   使用绝对值比较而非直接==，避免浮点数精度问题
 * - &&: 逻辑与运算符
 */
bool Vec2d::operator==(const Vec2d &other) const {
  return (std::abs(x_ - other.x()) < kMathEpsilon &&
          std::abs(y_ - other.y()) < kMathEpsilon);
}

/**
 * @brief 友元函数：标量乘向量（左操作数）
 *
 * 支持 标量 * 向量 的运算顺序。
 * 这是非成员函数，允许左操作数为标量的情况。
 *
 * @param ratio 标量（左操作数）
 * @param vec 向量（右操作数）
 * @return Vec2d 缩放后的向量
 *
 * 语法说明：
 * - friend函数声明在类内，使非成员函数可以访问private成员
 * - 或者直接使用vec * ratio，因为乘法满足交换律
 * - 返回vec * ratio，调用成员函数operator*
 */
Vec2d operator*(const double ratio, const Vec2d &vec) { return vec * ratio; }

/**
 * @brief 调试字符串
 *
 * 返回向量的调试字符串表示。
 *
 * @return std::string 格式化的字符串
 *
 * 语法说明：
 * - std::string: C++标准库字符串类型
 * - absl::StrCat(...): Abseil字符串拼接函数
 *   - 将多个参数拼接成一个字符串
 *   - 自动处理类型转换
 */
std::string Vec2d::DebugString() const {
  return absl::StrCat("vec2d ( x = ", x_, "  y = ", y_, " )");
}

}  // namespace math
}  // namespace common
}  // namespace apollo
