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
/*!\file    rrlib/math/tQuaternion.h
 *
 * \author  Thorsten Ropertz
 *
 * \date    2013-10-15
 *
 * \brief   Contains tQuaternion
 *
 * \b tQuaternion
 *
 * This class implements the functionality of quaternions for describing orientations.
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__math__tQuaternion_h__
#define __rrlib__math__tQuaternion_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <cmath>
//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/math/tAngle.h"
#include "rrlib/math/tVector.h"
#include "rrlib/math/tMatrix.h"
#include "rrlib/math/tPose3D.h"
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
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This class implements the functionality of quaternions for describing orientations.
 */
template<typename T>
class tQuaternion
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  static const tQuaternion<T> IDENTITY;
  static const tQuaternion<T> ZERO;
  //----------------------------------------------------------------------
  // Constructors
  //----------------------------------------------------------------------

  /**
   * Standard constructor that initializes the quaternion with the identity
   */
  tQuaternion();

  /**
   * Copy constructor
   */
  tQuaternion(const tQuaternion<T>& other);

  /**
   * Constructor with given real and imaginary part
   * @param[in] x the first imaginary value
   * @param[in] y the second imaginary value
   * @param[in] z the third imaginary value
   * @param[in] w the real value
   */
  tQuaternion(const T& x, const T& y, const T& z, const T& w);

  /**
   * Constructor with given real and imaginary part
   * @param[in] q a vector of dimension 4 containing the real value followed by the imaginary values
   */
  tQuaternion(const tVector<4, T>& q);

  /**
   * Constructor that initializes the quaternion using the given euler angles
   * @param[in] roll the rotation around the local x-axis
   * @param[in] pitch the rotation around the local y-axis
   * @param[in] yaw the rotation around the local z-axis
   */
  tQuaternion(const tAngleRad& roll, const tAngleRad& pitch, const tAngleRad& yaw);

  /**
   * Constructor that initializes the quaternion using the given rotation axis and rotation angle
   * @param[in] rad the rotation angle
   * @param[in] unit_axis the rotation axis as a 3-dimensional vector of length 1
   */
  tQuaternion(const tAngleRad& rad, const tVector<3, T>& unit_axis);

  /**
   * Constructor that initializes the quaternion using the given homogenous rotation matrix
   * @param[in] m the homogenous rotation matrix to extract the quaternion from
   */
  tQuaternion(const tMatrix<4, 4, T>& m);

  /**
   * Constructor that initializes the quaternion using the given rotation matrix
   * @param[in] m the rotation matrix to extract the quaternion from
   */
  tQuaternion(const tMatrix<3, 3, T>& m);

  /**
   * Constructor that initializes the quaternion using the given pose
   * @param[in] pose the pose to extract the quaternion from
   */
  tQuaternion(const tPose3D& pose);

  //----------------------------------------------------------------------
  // Conversions
  //----------------------------------------------------------------------

  /**
   * Returns the euler angle representation (1-2-3 / roll-pitch-yaw / x-y-z) of this quaternion
   * @param[in] use_second_solution flag to switch between the two solutions
   * @return 3-dimensional vector containing [roll, pitch, yaw] in radian
   */
  inline tVector<3, T> GetEulerAngles(const bool use_second_solution = false) const;

  /**
   * Computes the euler angle representation (1-2-3 / roll-pitch-yaw / x-y-z) of this quaternion
   * @param[out] roll the rotation angle around the local x-axis
   * @param[out] pitch the rotation angle around the local y-axis
   * @param[out] yaw the rotation angle around the local z-axis
   * @param[in] use_second_solution flag to switch between the two solutions
   */
  inline void GetEulerAngles(tAngleRad& roll, tAngleRad& pitch, tAngleRad& yaw, const bool use_second_solution = false) const;

  /**
   * Computes the euler angle representation (1-2-3 / roll-pitch-yaw / x-y-z) of this quaternion
   * @param[out] roll the rotation angle around the local x-axis
   * @param[out] pitch the rotation angle around the local y-axis
   * @param[out] yaw the rotation angle around the local z-axis
   * @param[in] use_second_solution flag to switch between the two solutions
   */
  inline void GetEulerAngles(T& roll, T& pitch, T& yaw, const bool use_second_solution = false) const;

  /**
   * Returns the rotation matrix representation of this quaternion
   * @return the rotation matrix representation of this quaternion
   */
  inline tMatrix<3, 3, T> GetRotationMatrix() const;

  /**
   * Computes the rotation matrix representation of this quaternion
   * @param[out] m the matrix to store the rotation matrix representation of this quaternion in
   */
  inline void GetRotationMatrix(tMatrix<3, 3, T>& m) const;

  /**
   * Returns the rotation angle around the local x-axis
   * @param[in] use_second_solution flag to switch between the two solutions
   * @return the rotation angle around the local x-axis in radian
   */
  inline tAngleRad GetRoll(const bool use_second_solution = false) const;

  /**
   * Returns the rotation angle around the local y-axis
   * @param[in] use_second_solution flag to switch between the two solutions
   * @return the rotation angle around the local y-axis in radian
   */
  inline tAngleRad GetPitch(const bool use_second_solution = false) const;

  /**
   * Returns the rotation angle around the local z-axis
   * @param[in] use_second_solution flag to switch between the two solutions
   * @return the rotation angle around the local z-axis in radian
   */
  inline tAngleRad GetYaw(const bool use_second_solution = false) const;

  /**
   * Computes the rotation axis and rotation angle representation of this quaternion
   * @param[out] rotation_axis the rotation axis
   * @param[out] rad the rotation angle in radian
   */
  inline void GetAxisAndAngle(tVector<3, T> &rotation_axis, tAngleRad &rad) const;

  /**
   * Returns the overall rotation angle in radian
   * @return the overall rotation angle in radian
   */
  inline tAngleRad GetAngle() const;

  /**
   * Returns the conjugate of the quaternion
   * Example: (a+bi+cj+dk)->(a-bi-cj-dk)
   * @return the conjugate of the quaternion
   */
  inline tQuaternion<T> GetConjugate() const;

  /**
   * Returns the rotated x-axis unit vector
   * @return the rotated x-axis unit vector
   */
  inline tVector<3, T> GetBasisVector0() const;

  /**
   * Returns the rotated y-axis unit vector
   * @return the rotated y-axis unit vector
   */
  inline tVector<3, T> GetBasisVector1() const;

  /**
   * Returns the rotated z-axis unit vector
   * @return the rotated z-axis unit vector
   */
  inline tVector<3, T> GetBasisVector2() const;

  /**
   * Returns a normalized copy of this quaternion
   * @return a normalized copy of this quaternion
   */
  inline tQuaternion<T> GetNormalized() const;

  /**
   * Returns an inverse copy of this quaternion
   * @return an inverse copy of this quaternion
   */
  inline tQuaternion<T> GetInverse() const;

  //----------------------------------------------------------------------
  // Modifications
  //----------------------------------------------------------------------

  /**
   * Computes the inverse of this quaternion
   * @return a reference to the resulting quaternion
   */
  inline tQuaternion<T>& Inverse();

  /**
   * Setter for the real part of the quaternion (W)
   * @param[in] w the value to copy
   */
  inline void SetRealPart(const T& w);

  /**
   * Setter for the imaginary part of the quaternion [x,y,z]
   * @param[in] axis the values to copy
   */
  inline void SetImaginaryPart(const tVector<3, T>& axis);

  /**
   * Adds the given roll angle to the current rotation
   * @param[in] rad the roll angle in radian
   * @return a reference to the rolled quaternion
   */
  inline tQuaternion<T>& Roll(const tAngleRad& rad);

  /**
   * Adds the given pitch angle to the current rotation
   * @param[in] rad the pitch angle in radian
   * @return a reference to the pitched quaternion
   */
  inline tQuaternion<T>& Pitch(const tAngleRad& rad);

  /**
   * Adds the given yaw angle to the current rotation
   * @param[in] rad the yaw angle in radian
   * @return a reference to the yawed quaternion
   */
  inline tQuaternion<T>& Yaw(const tAngleRad& rad);

  /**
   * Normalizes the quaternion
   * @return a reference to the normalized quaternion
   */
  inline tQuaternion<T>& Normalize();

  /**
   * Sets the quaternion based on the given euler-angles (x-y-z)
   * @param[in] roll the rotation angle around the x-axis in radian
   * @param[in] pitch the rotation angle around the y-axis in radian
   * @param[in] yaw the rotation angle around the z-axis in radian
   * @return a reference to the resulting quaternion
   */
  inline tQuaternion<T>& SetEuler(const tAngleRad& roll, const tAngleRad& pitch, const tAngleRad& yaw);

  //----------------------------------------------------------------------
  // Properties
  //----------------------------------------------------------------------

  /**
   * Checks whether one of the elements of the quaternion is NaN
   * @return true if one of the elements is NaN
   *     false otherwise
   */
  inline const bool IsNaN() const;

  /**
   * Checks whether the quaternion is a unit quaternion (normalized)
   * @return true if the quaternion is a unit quaternion
   *     false otherwise
   */
  inline const bool IsUnit() const;

  //----------------------------------------------------------------------
  // Operators
  //----------------------------------------------------------------------

  /**
   * Rotates the given 3-dimensional vector
   * @param[in] v the vector to rotate
   * @return the rotated vector
   */
  inline tVector<3, T> operator*(const tVector<3, T>& v) const;

  /**
   * Rotates the given 3-dimensional vector
   * @param[in] v the vector to rotate
   * @return the rotated vector
   */
  inline tVector<3, T> Rotate(const tVector<3, T>& v) const;

  /**
   * Rotates the given 3-dimensional vector
   * @param[in] v the vector to rotate
   * @return the rotated vector
   */
  inline tVector<3, T> RotateInverse(const tVector<3, T>& v) const;

  /**
   * Combines the current quaternion with the given one
   * @param[in] other the quaternion to append
   * @return the combined quaternion
   */
  inline tQuaternion<T> operator*(const tQuaternion& other) const;

  /**
   * Performs a scalar multiplication as defined in vector mathematics
   * @param[in] scalar the scalar value to multiply with
   * @return the resulting quaternion
   */
  inline tQuaternion<T> operator*(const T& scalar) const;

  /**
   * Quaternion addition as defined in complex number theory
   * @param[in] other the quaternion to add
   * @return the sum
   */
  inline tQuaternion<T> operator+(const tQuaternion& other) const;

  /**
   * Element-wise comparison of quaternions
   * @param[in] other the quaternion to compare with
   * @return true if they are euqal
   *     false otherwise
   */
  inline const bool operator==(const tQuaternion& other) const;

  /**
   * Sets the elements based on the given quaternion
   * @param[in] other the quaternion to copy from
   * @return the resulting quaternion
   */
  inline tQuaternion<T>& operator = (const tQuaternion& other);

  /**
   * Performs the quaternion multiplication and assigns the result to this quaternion
   * @praram[in] other the quaternion to multiply with
   * @return a reference to the product quaternion
   */
  inline tQuaternion<T>& operator *= (const tQuaternion& other);

  /**
   * Performs an element wise multiplication
   * @param[in] other the quaternion to multiply with
   * @return the resulting quaternion
   */
  inline T dot(const tQuaternion<T>& other) const;

  //----------------------------------------------------------------------
  // Getters
  //----------------------------------------------------------------------

  inline T& X();
  inline T& Y();
  inline T& Z();
  inline T& W();

  inline const T& X() const;
  inline const T& Y() const;
  inline const T& Z() const;
  inline const T& W() const;

  /**
   * Returns the imaginary part of the quaternion [x,y,z]
   * @return the imaginary part of the quaternion
   */
  inline tVector<3, T> GetImaginaryPart() const;

  /**
   * Returns the imaginary part of the quaternion [x,y,z]
   * @return the imaginary part of the quaternion
   */
  inline const tVector<4, T>& ConstValue() const;

  /**
   * Returns the raw values of the quaternion [x,y,z,w]
   * @return the raw values of the quaternion
   */
  inline tVector<4, T>& Value();

  //----------------------------------------------------------------------
  // Comparison
  //----------------------------------------------------------------------

  /**
   * Computes the angle between the given quaternions
   * @param[in] other the quaternion to compare with
   * @return the angle between the quaternions in radian
   */
  inline tAngleRad GetAngle(const tQuaternion<T>& other) const;

  //----------------------------------------------------------------------
  // Destructor
  //----------------------------------------------------------------------

  /**
   * Standard destructor
   */
  ~tQuaternion();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /**
   * acutal quaternion
   * layout: [x,y,z,w]
   */
  tVector<4, T> q;

  inline bool Equals(const T& a, const T& b, const T& tolerance) const;
  inline T GetNorm() const;

};

//----------------------------------------------------------------------
// Streaming Operators
//----------------------------------------------------------------------
template<typename T>
std::ostream &operator << (std::ostream& stream, const tQuaternion<T>& quat);
template<typename T>
std::istream &operator >> (std::istream& stream, tQuaternion<T>& quat);

typedef tQuaternion<double> tQuatd;
typedef tQuaternion<float> TQuatf;

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "rrlib/math/tQuaternion.hpp"

#endif
