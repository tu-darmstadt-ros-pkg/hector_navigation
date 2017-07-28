/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef COST_FUNCTOR_H
#define COST_FUNCTOR_H

#include <iostream>
#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"

template <typename FloatType>
class Rigid3 {
 public:
  using Affine = Eigen::Transform<FloatType, 3, Eigen::Affine>;
  using Vector = Eigen::Matrix<FloatType, 3, 1>;
  using Quaternion = Eigen::Quaternion<FloatType>;
  using AngleAxis = Eigen::AngleAxis<FloatType>;

  Rigid3()
      : translation_(Vector::Identity()), rotation_(Quaternion::Identity()) {}
  // TODO(damonkohler): Remove
  explicit Rigid3(const Affine& affine)
      : translation_(affine.translation()), rotation_(affine.rotation()) {}
  Rigid3(const Vector& translation, const Quaternion& rotation)
      : translation_(translation), rotation_(rotation) {}
  Rigid3(const Vector& translation, const AngleAxis& rotation)
      : translation_(translation), rotation_(rotation) {}

  static Rigid3 Rotation(const AngleAxis& angle_axis) {
    return Rigid3(Vector::Zero(), Quaternion(angle_axis));
  }

  static Rigid3 Rotation(const Quaternion& rotation) {
    return Rigid3(Vector::Zero(), rotation);
  }

  static Rigid3 Translation(const Vector& vector) {
    return Rigid3(vector, Quaternion::Identity());
  }

  static Rigid3<FloatType> Identity() {
    return Rigid3<FloatType>(Vector::Zero(), Quaternion::Identity());
  }

  template <typename OtherType>
  Rigid3<OtherType> cast() const {
    return Rigid3<OtherType>(translation_.template cast<OtherType>(),
                             rotation_.template cast<OtherType>());
  }

  const Vector& translation() const { return translation_; }
  const Quaternion& rotation() const { return rotation_; }

  Rigid3 inverse() const {
    const Quaternion rotation = rotation_.conjugate();
    const Vector translation = -(rotation * translation_);
    return Rigid3(translation, rotation);
  }

  std::string DebugString() const {
    std::string out;
    out.append("{ t: [");
    out.append(std::to_string(translation().x()));
    out.append(", ");
    out.append(std::to_string(translation().y()));
    out.append(", ");
    out.append(std::to_string(translation().z()));
    out.append("], q: [");
    out.append(std::to_string(rotation().w()));
    out.append(", ");
    out.append(std::to_string(rotation().x()));
    out.append(", ");
    out.append(std::to_string(rotation().y()));
    out.append(", ");
    out.append(std::to_string(rotation().z()));
    out.append("] }");
    return out;
  }

 private:
  Vector translation_;
  Quaternion rotation_;
};

template <typename FloatType>
Rigid3<FloatType> operator*(const Rigid3<FloatType>& lhs,
                            const Rigid3<FloatType>& rhs) {
  return Rigid3<FloatType>(
      lhs.rotation() * rhs.translation() + lhs.translation(),
      (lhs.rotation() * rhs.rotation()).normalized());
}

template <typename FloatType>
typename Rigid3<FloatType>::Vector operator*(
    const Rigid3<FloatType>& rigid,
    const typename Rigid3<FloatType>::Vector& point) {
  return rigid.rotation() * point + rigid.translation();
}

// This is needed for gmock.
template <typename T>
std::ostream& operator<<(std::ostream& os,
                         Rigid3<T>& rigid) {
  os << rigid.DebugString();
  return os;
}

using Rigid3d = Rigid3<double>;
using Rigid3f = Rigid3<float>;

class TranslationDeltaCostFunctor {
 public:
  // Creates an OccupiedSpaceCostFunctor using the specified grid, 'rotation' to
  // add to all poses, and point cloud.
  TranslationDeltaCostFunctor(Eigen::Matrix<double, 3, 1> pos_world,
                           Eigen::Matrix<double, 3, 1> pos_gps)
      : pos_world_(pos_world),
        pos_gps_(pos_gps) {}

  TranslationDeltaCostFunctor(const TranslationDeltaCostFunctor&) = delete;
  TranslationDeltaCostFunctor& operator=(const TranslationDeltaCostFunctor&) = delete;

  template <typename T>
  bool operator()(const T* const translation, const T* const rotation,
                  T* const residual) const {
    const Rigid3<T> transform(
        Eigen::Map<const Eigen::Matrix<T, 3, 1>>(translation),
        Eigen::Quaternion<T>(rotation[0], rotation[1], rotation[2],
                             rotation[3]));
    return Evaluate(transform, residual);
  }

  template <typename T>
  bool Evaluate(const Rigid3<T>& transform,
                T* const residual) const {
      const Eigen::Matrix<T, 3, 1> delta =
      pos_gps_.cast<T>() - transform * pos_world_.cast<T>();
      residual[0] = delta[0];
      residual[1] = delta[1];
      residual[2] = delta[2];
    return true;
  }

 private:
  Eigen::Matrix<double, 3, 1> pos_world_;
  Eigen::Matrix<double, 3, 1> pos_gps_;
};




#endif  // COST_FUNCTOR_H
