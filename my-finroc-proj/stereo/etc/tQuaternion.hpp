//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
//----------------------------------------------------------------------
/*!\file    rrlib/math/tQuaternion.cpp
 *
 * \author  Thorsten Ropertz
 *
 * \date    2013-10-15
 *
 */
//----------------------------------------------------------------------
#include "rrlib/math/tQuaternion.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <cmath>
//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace rrlib
{
namespace math
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
template<typename T>
const tQuaternion<T> tQuaternion<T>::IDENTITY(0, 0, 0, 1.0);
template<typename T>
const tQuaternion<T> tQuaternion<T>::ZERO(0, 0, 0, 0);
//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tQuaternion constructors
//----------------------------------------------------------------------
template<typename T>
tQuaternion<T>::tQuaternion() :
  q(0, 0, 0, 1.0)
{}

template<typename T>
tQuaternion<T>::tQuaternion(const tQuaternion<T>& other) :
  q(other.q)
{}

template<typename T>
tQuaternion<T>::tQuaternion(const T& x, const T& y,
                            const T& z, const T& w) :
  q(x, y, z, w)
{}

template<typename T>
tQuaternion<T>::tQuaternion(const tVector<4, T>& q):
  q(q)
{}

template<typename T>
tQuaternion<T>::tQuaternion(const tAngleRad& roll, const tAngleRad& pitch,
                            const tAngleRad& yaw)
{
  SetEuler(roll, pitch, yaw);
}

template<typename T>
tQuaternion<T>::tQuaternion(const tAngleRad& rad, const tVector<3, T>& unit_axis)
{
  T s = std::sin(rad / 2.0);
  q[3] = std::cos(rad / 2.0);
  q[0] = unit_axis.X() * s;
  q[1] = unit_axis.Y() * s;
  q[2] = unit_axis.Z() * s;
  q.Normalize();
}

template<typename T>
tQuaternion<T>::tQuaternion(const tMatrix<4, 4, T>& m)
{
  T t = 1 + m[0][0] + m[1][1] + m[2][2];
  if (t > 0.00000001)
  {
    T s = std::sqrt(t) * 2.0;
    q[3] = 0.25 * s;
    q[0] = (m[2][1] - m[1][2]) / s;
    q[1] = (m[0][2] - m[2][0]) / s;
    q[2] = (m[1][0] - m[0][1]) / s;
  }
  else if (m[0][0] > m[1][1] && m[0][0] > m[2][2])
  {
    T s = std::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0;
    q[3] = (m[2][1] - m[1][2]) / s;
    q[0] = 0.25 * s;
    q[1] = (m[1][0] + m[0][1]) / s;
    q[2] = (m[0][2] + m[2][0]) / s;
  }
  else if (m[1][1] > m[2][2])
  {
    T s = std::sqrt(1.0 - m[0][0] + m[1][1] - m[2][2]) * 2.0;
    q[3] = (m[0][2] - m[2][0]) / s;
    q[0] = (m[1][0] + m[0][1]) / s;
    q[1] = 0.25 * s;
    q[2] = (m[2][1] + m[1][2]) / s;
  }
  else
  {
    T s = std::sqrt(1.0 - m[0][0] - m[1][1] + m[2][2]) * 2.0;
    q[3] = (m[1][0] - m[0][1]) / s;
    q[0] = (m[0][2] + m[2][0]) / s;
    q[1] = (m[2][1] + m[1][2]) / s;
    q[2] = 0.25 * s;
  }
  Normalize();
}

template<typename T>
tQuaternion<T>::tQuaternion(const tMatrix<3, 3, T>& m)
{
  T t = 1 + m[0][0] + m[1][1] + m[2][2];
  if (t > 0.00000001)
  {
    T s = std::sqrt(t) * 2.0;
    q[3] = 0.25 * s;
    q[0] = (m[2][1] - m[1][2]) / s;
    q[1] = (m[0][2] - m[2][0]) / s;
    q[2] = (m[1][0] - m[0][1]) / s;
  }
  else if (m[0][0] > m[1][1] && m[0][0] > m[2][2])
  {
    T s = std::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0;
    q[3] = (m[2][1] - m[1][2]) / s;
    q[0] = 0.25 * s;
    q[1] = (m[1][0] + m[0][1]) / s;
    q[2] = (m[0][2] + m[2][0]) / s;
  }
  else if (m[1][1] > m[2][2])
  {
    T s = std::sqrt(1.0 - m[0][0] + m[1][1] - m[2][2]) * 2.0;
    q[3] = (m[0][2] - m[2][0]) / s;
    q[0] = (m[1][0] + m[0][1]) / s;
    q[1] = 0.25 * s;
    q[2] = (m[2][1] + m[1][2]) / s;
  }
  else
  {
    T s = std::sqrt(1.0 - m[0][0] - m[1][1] + m[2][2]) * 2.0;
    q[3] = (m[1][0] - m[0][1]) / s;
    q[0] = (m[0][2] + m[2][0]) / s;
    q[1] = (m[2][1] + m[1][2]) / s;
    q[2] = 0.25 * s;
  }
  Normalize();
}

template<typename T>
tQuaternion<T>::tQuaternion(const tPose3D& pose)
{
  SetEuler(pose.Roll(), pose.Pitch(), pose.Yaw());
}

//----------------------------------------------------------------------
// Conversions
//----------------------------------------------------------------------
template<typename T>
inline tVector<3, T> tQuaternion<T>::GetEulerAngles(const bool use_second_solution) const
{
  tVector<3, T> result;
  GetEulerAngles(result.X(), result.Y(), result.Z(), use_second_solution);
  return result;
}

template<typename T>
inline void tQuaternion<T>::GetEulerAngles(tAngleRad& roll, tAngleRad& pitch, tAngleRad& yaw, const bool use_second_solution) const
{
  T r, p, y;
  GetEulerAngles(r, p, y, use_second_solution);
  roll = tAngleRad(r);
  pitch = tAngleRad(p);
  yaw = tAngleRad(y);
}

template<typename T>
inline void tQuaternion<T>::GetEulerAngles(T& roll, T& pitch, T& yaw, const bool use_second_solution) const
{
  tMatrix<3, 3, T> m;
  GetRotationMatrix(m);

  if (!use_second_solution)
  {
    roll = std::atan2(m[2][1], m[2][2]);
  }
  else
  {
    roll = std::atan2(-m[2][1], -m[2][2]);
  }
  T sin_roll = std::sin(roll);
  T cos_roll = std::cos(roll);
  pitch = std::atan2(-m[2][0], sin_roll * m[2][1] + cos_roll * m[2][2]);
  yaw = std::atan2(sin_roll * m[0][2] - cos_roll * m[0][1], cos_roll * m[1][1] - sin_roll * m[1][2]);
}

template<typename T>
inline tMatrix<3, 3, T> tQuaternion<T>::GetRotationMatrix() const
{
  tMatrix<3, 3, T> result;
  GetRotationMatrix(result);
  return result;
}

template<typename T>
inline void tQuaternion<T>::GetRotationMatrix(tMatrix<3, 3, T>& m) const
{
  T w2 = q[3] * q[3];
  T x2 = q[0] * q[0];
  T y2 = q[1] * q[1];
  T z2 = q[2] * q[2];

  m[0][0] = w2 + x2 - y2 - z2;
  m[2][2] = w2 - x2 - y2 + z2;
  m[1][1] = w2 - x2 + y2 - z2;

  m[1][2] = 2.0 * (q[1] * q[2] - q[3] * q[0]);
  m[2][1] = 2.0 * (q[1] * q[2] + q[3] * q[0]);

  m[2][0] = 2.0 * (q[0] * q[2] - q[3] * q[1]);
  m[0][2] = 2.0 * (q[0] * q[2] + q[3] * q[1]);

  m[0][1] = 2.0 * (q[0] * q[1] - q[3] * q[2]);
  m[1][0] = 2.0 * (q[0] * q[1] + q[3] * q[2]);
}

template<typename T>
inline tAngleRad tQuaternion<T>::GetRoll(const bool use_second_solution) const
{
  if (!use_second_solution)
    return std::atan2(2.0 * (q[1] * q[2] + q[3] * q[0]), q[3] * q[3] - q[0] * q[0] - q[1] * q[1] + q[2] * q[2]);
  else
    return std::atan2(-2.0 * (q[1] * q[2] + q[3] * q[0]), q[0] * q[0] - q[3] * q[3] + q[1] * q[1] - q[2] * q[2]);
}

template<typename T>
inline tAngleRad tQuaternion<T>::GetPitch(const bool use_second_solution) const
{
  T roll = GetRoll(use_second_solution);
  return std::atan2(-2.0 * (q[0] * q[2] - q[3] * q[1]),
                    std::sin(roll) * (2.0 * (q[1] * q[2] + q[3] * q[0])) + std::cos(roll) *
                    (q[3] * q[3] - q[0] * q[0] - q[1] * q[1] + q[2] * q[2]));
}

template<typename T>
inline tAngleRad tQuaternion<T>::GetYaw(const bool use_second_solution) const
{
  T roll = GetRoll(use_second_solution);
  T sin_roll = std::sin(roll);
  T cos_roll = std::cos(roll);
  return std::atan2(sin_roll * 2.0 * (q[0] * q[2] + q[3] * q[1]) - cos_roll * 2.0 * (q[0] * q[1] - q[3] * q[2]),
                    cos_roll * (q[3] * q[3] - q[0] * q[0] + q[1] * q[1] - q[2] * q[2]) - sin_roll * 2.0 * (q[1] * q[2] - q[3] * q[0]));
}

template<typename T>
inline void tQuaternion<T>::GetAxisAndAngle(tVector<3, T> &rotation_axis, tAngleRad &rad) const
{
  T tmp = std::sqrt(1.0 - q[3] * q[3]);
  rad = std::acos(q[3]) * 2.0;
  if (std::fabs(tmp) < 0.0005)
    tmp = 0;
  rotation_axis[0] = q[0] / tmp;
  rotation_axis[1] = q[0] / tmp;
  rotation_axis[2] = q[0] / tmp;
}

template<typename T>
inline tAngleRad tQuaternion<T>::GetAngle() const
{
  return tAngleRad(std::acos(q[3]) * 2.0);
}

template<typename T>
inline tQuaternion<T> tQuaternion<T>::GetConjugate() const
{
  return tQuaternion(-q[0], -q[1], -q[2], q[3]);
}

template<typename T>
inline tVector<3, T> tQuaternion<T>::GetBasisVector0() const
{
  const T x2 = q[0] * 2.0f;
  const T w2 = q[3] * 2.0f;
  return tVector<3, T>((q[3] * w2) - 1.0 + q[0] * x2,
                       (q[2] * w2)        + q[1] * x2,
                       (-q[1] * w2)       + q[2] * x2);
}

template<typename T>
inline tVector<3, T> tQuaternion<T>::GetBasisVector1() const
{
  const T y2 = q[1] * 2.0f;
  const T w2 = q[3] * 2.0f;
  return tVector<3, T>((-q[2] * w2)       + q[0] * y2,
                       (q[3] * w2) - 1.0 + q[1] * y2,
                       (q[0] * w2)        + q[2] * y2);
}

template<typename T>
inline tVector<3, T> tQuaternion<T>::GetBasisVector2() const
{
  const T z2 = q[2] * 2.0f;
  const T w2 = q[3] * 2.0f;
  return tVector<3, T>((q[1] * w2)        + q[0] * z2,
                       (-q[0] * w2)       + q[1] * z2,
                       (q[3] * w2) - 1.0f + q[2] * z2);
}

template<typename T>
inline tQuaternion<T> tQuaternion<T>::GetNormalized() const
{
  return tQuaternion(*this).Normalize();
}

template<typename T>
inline tQuaternion<T> tQuaternion<T>::GetInverse() const
{
  return GetConjugate();
}
//----------------------------------------------------------------------
// Modifications
//----------------------------------------------------------------------

template<typename T>
inline tQuaternion<T>& tQuaternion<T>::Inverse()
{
  q *= GetNorm();
  return *this;
}

template<typename T>
inline void tQuaternion<T>::SetRealPart(const T& rad)
{
  q[3] = rad;
}

template<typename T>
inline void tQuaternion<T>::SetImaginaryPart(const tVector<3, T>& axis)
{
  memcpy(&this->q[0], &(axis), 3 * sizeof(T));
  /*q[0] = axis[0];
  q[1] = axis[1];
  q[2] = axis[3];*/
}

template<typename T>
inline tQuaternion<T>& tQuaternion<T>::Roll(const tAngleRad& rad)
{
  return this->operator *=(tQuaternion(std::sin(rad / 2.0), 0, 0, std::cos(rad / 2.0)));
}

template<typename T>
inline tQuaternion<T>& tQuaternion<T>::Pitch(const tAngleRad& rad)
{
  return this->operator *=(tQuaternion(0, std::sin(rad / 2.0), 0, std::cos(rad / 2.0)));
}

template<typename T>
inline tQuaternion<T>& tQuaternion<T>::Yaw(const tAngleRad& rad)
{
  return this->operator *=(tQuaternion(0, 0, std::sin(rad / 2.0), std::cos(rad / 2.0)));
}

template<typename T>
inline tQuaternion<T>& tQuaternion<T>::Normalize()
{
  //q.Normalize();
  T norm = GetNorm();
  q[0] /= norm;
  q[1] /= norm;
  q[2] /= norm;
  q[3] /= norm;
  return *this;
}

template<typename T>
inline tQuaternion<T>& tQuaternion<T>::SetEuler(const tAngleRad& roll, const tAngleRad& pitch, const tAngleRad& yaw)
{
  const T cr = std::cos(roll / 2.0);
  const T sr = std::sin(roll / 2.0);
  const T cp = std::cos(pitch / 2.0);
  const T sp = std::sin(pitch / 2.0);
  const T cy = std::cos(yaw / 2.0);
  const T sy = std::sin(yaw / 2.0);

  const T cpcy = cp * cy;
  const T spcy = sp * cy;
  const T cpsy = cp * sy;
  const T spsy = sp * sy;

  q[0] = sr * cpcy - cr * spsy;
  q[1] = cr * spcy + sr * cpsy;
  q[2] = cr * cpsy - sr * spcy;
  q[3] = cr * cpcy + sr * spsy;
  return Normalize();
}

//----------------------------------------------------------------------
// Properties
//----------------------------------------------------------------------
template<typename T>
inline const bool tQuaternion<T>::IsNaN() const
{
  return std::isnan(q[3]) || std::isnan(q[0]) || std::isnan(q[1]) || std::isnan(q[2]);
}

template<typename T>
inline const bool tQuaternion<T>::IsUnit() const
{
  return (std::sqrt(GetNorm()) == 1.0);
}

//----------------------------------------------------------------------
// Operators
//----------------------------------------------------------------------
template<typename T>
inline tVector<3, T> tQuaternion<T>::operator*(const tVector<3, T>& v) const
{
  return Rotate(v);
}

template<typename T>
inline tVector<3, T> tQuaternion<T>::Rotate(const tVector<3, T>& v) const
{
  tVector<3, T> uv, uuv;
  const tVector<3, T> axis(q[0], q[1], q[2]);
  uv = axis.CrossMultiplied(v);
  uuv = axis.CrossMultiplied(uv);
  uv *= (2.0f * q[3]);
  uuv *= 2.0f;

  return v + uv + uuv;
}

template<typename T>
inline tVector<3, T> tQuaternion<T>::RotateInverse(const tVector<3, T>& v) const
{
  return GetInverse() * v;
}

template<typename T>
inline tQuaternion<T> tQuaternion<T>::operator*(const tQuaternion& other) const
{
  tVector<4, T> result;
  result[3] = q[3] * other.q[3] - q[0] * other.q[0] - q[1] * other.q[1] - q[2] * other.q[2];
  result[0] = q[3] * other.q[0] + q[0] * other.q[3] + q[1] * other.q[2] - q[2] * other.q[1];
  result[1] = q[3] * other.q[1] - q[0] * other.q[2] + q[1] * other.q[3] + q[2] * other.q[0];
  result[2] = q[3] * other.q[2] + q[0] * other.q[1] - q[1] * other.q[0] + q[2] * other.q[3];
  return tQuaternion(result);
}

template<typename T>
inline tQuaternion<T> tQuaternion<T>::operator*(const T& scalar) const
{
  return tQuaternion(q * scalar);
}

template<typename T>
inline tQuaternion<T> tQuaternion<T>::operator+(const tQuaternion& other) const
{
  return tQuaternion(q + other.q);
}

template<typename T>
inline const bool tQuaternion<T>::operator==(const tQuaternion& other) const
{
  return q == other.q;
}

template<typename T>
inline tQuaternion<T>& tQuaternion<T>::operator = (const tQuaternion& other)
{
  q = other.q;
  return *this;
}

template<typename T>
inline tQuaternion<T>& tQuaternion<T>::operator *= (const tQuaternion& other)
{
  tVector<4, T> result;
  result[0] = q[3] * other.q[3] - q[0] * other.q[0] - q[1] * other.q[1] - q[2] * other.q[2];
  result[1] = q[3] * other.q[0] + q[0] * other.q[3] + q[1] * other.q[2] - q[2] * other.q[1];
  result[2] = q[3] * other.q[1] - q[0] * other.q[2] + q[1] * other.q[3] + q[2] * other.q[0];
  result[3] = q[3] * other.q[2] + q[0] * other.q[1] - q[1] * other.q[0] + q[2] * other.q[3];
  q = result;
  return *this;
}

template<typename T>
inline T tQuaternion<T>::dot(const tQuaternion<T>& other) const
{
  return q[3] * other.q[3] + q[0] * other.q[0] + q[1] * other.q[1] + q[2] * other.q[2];
}

//----------------------------------------------------------------------
// Getters
//----------------------------------------------------------------------
template<typename T>
inline T& tQuaternion<T>::X()
{
  return q[0];
}

template<typename T>
inline T& tQuaternion<T>::Y()
{
  return q[1];
}

template<typename T>
inline T& tQuaternion<T>::Z()
{
  return q[2];
}

template<typename T>
inline T& tQuaternion<T>::W()
{
  return q[3];
}

template<typename T>
inline const T& tQuaternion<T>::X() const
{
  return q[0];
}

template<typename T>
inline const T& tQuaternion<T>::Y() const
{
  return q[1];
}

template<typename T>
inline const T& tQuaternion<T>::Z() const
{
  return q[2];
}

template<typename T>
inline const T& tQuaternion<T>::W() const
{
  return q[3];
}

template<typename T>
inline tVector<3, T> tQuaternion<T>::GetImaginaryPart() const
{
  return tVector<3, T>(q[0], q[1], q[2]);
}

template<typename T>
inline const tVector<4, T>& tQuaternion<T>::ConstValue() const
{
  return q;
}

template<typename T>
inline tVector<4, T>& tQuaternion<T>::Value()
{
  return q;
}
//----------------------------------------------------------------------
// Comparison
//----------------------------------------------------------------------
template<typename T>
inline tAngleRad tQuaternion<T>::GetAngle(const tQuaternion<T>& other) const
{
  return tAngleRad(std::acos(dot(other)) * 2.0);
}

//----------------------------------------------------------------------
// Streaming Operators
//----------------------------------------------------------------------
template<typename T>
std::ostream &operator << (std::ostream& stream, const tQuaternion<T>& quat)
{
  return stream << quat.ConstValue();
}

template<typename T>
std::istream &operator >> (std::istream& stream, tQuaternion<T>& quat)
{
  char temp(0);
  stream >> temp;
  if (temp == '(')
  {
    stream >> quat.W() >> temp >> quat.X() >> temp >> quat.Y() >> temp >> quat.Z() >> temp ;
  }
  else
  {
    stream.putback(temp);
    stream >> quat.Value();
  }
  return stream;
}

//----------------------------------------------------------------------
// tQuaternion destructor
//----------------------------------------------------------------------
template<typename T>
tQuaternion<T>::~tQuaternion()
{}

template<typename T>
inline bool tQuaternion<T>::Equals(const T& a, const T& b, const T& tolerance) const
{
  return (a + tolerance >= b) && (a - tolerance <= b);
}

template<typename T>
inline T tQuaternion<T>::GetNorm() const
{
  return q[3] * q[3] + q[0] * q[0] + q[1] * q[1] + q[2] * q[2];
}
//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
