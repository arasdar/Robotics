//
// You received this file as part of Finroc
// A framework for intelligent robot control
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//----------------------------------------------------------------------
/*!\file    projects/icarus/sensor_processing/libfeature/tIntegralImage2D.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-05-29
 *
 * \brief   Contains tIntegralImage2D
 *
 * \b tIntegralImage2D
 *
 * This class Determines an integral image representation for a given organized data array.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__stereo_traversability_experiments__aras__libfeature__tIntegralImage2D_h__
#define __projects__stereo_traversability_experiments__aras__libfeature__tIntegralImage2D_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <pcl/pcl_base.h>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace stereo_traversability_experiments
{
namespace aras
{
namespace libfeature
{

//using namespace pcl;
//using namespace eigen;

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Structures
//----------------------------------------------------------------------
template <typename DataType>
struct IntegralImageTypeTraits
{
  typedef DataType Type;
  typedef DataType IntegralType;
};

template <>
struct IntegralImageTypeTraits<float>
{
  typedef float Type;
  typedef double IntegralType;
};

template <>
struct IntegralImageTypeTraits<char>
{
  typedef char Type;
  typedef int IntegralType;
};

template <>
struct IntegralImageTypeTraits<short>
{
  typedef short Type;
  typedef long IntegralType;
};

template <>
struct IntegralImageTypeTraits<unsigned short>
{
  typedef unsigned short Type;
  typedef unsigned long IntegralType;
};

template <>
struct IntegralImageTypeTraits<unsigned char>
{
  typedef unsigned char Type;
  typedef unsigned int IntegralType;
};

template <>
struct IntegralImageTypeTraits<int>
{
  typedef int Type;
  typedef long IntegralType;
};

template <>
struct IntegralImageTypeTraits<unsigned int>
{
  typedef unsigned int Type;
  typedef unsigned long IntegralType;
};

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This class Determines an integral image representation for a given organized data array.
 */
template <class DataType, unsigned Dimension>
class tIntegralImage2D
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  /** \brief Constructor for an Integral Image
    * \param[in] compute_second_order_integral_images set to true if we want to compute a second order image
    */
  tIntegralImage2D(bool compute_second_order_integral_images) :
    first_order_integral_image_(),
    second_order_integral_image_(),
    finite_values_integral_image_(),
    width_(1),
    height_(1),
    compute_second_order_integral_images_(compute_second_order_integral_images)
  {
  }

  /** \brief Destructor */
  virtual
  ~tIntegralImage2D() { }  //You need this destructor if you allocated memory on the heap that must be free'd. Delete otherwise!

  /*Here is the right place for your public methods. Replace this line by your declarations!*/
  static const unsigned second_order_size = (Dimension * (Dimension + 1)) >> 1;
  typedef Eigen::Matrix<typename IntegralImageTypeTraits<DataType>::IntegralType, Dimension, 1> ElementType;
  typedef Eigen::Matrix<typename IntegralImageTypeTraits<DataType>::IntegralType, second_order_size, 1> SecondOrderType;

  /** \brief sets the computation for second order integral images on or off.
    * \param compute_second_order_integral_images
    */
  void
  setSecondOrderComputation(bool compute_second_order_integral_images);

  /** \brief Set the input data to compute the integral image for
    * \param[in] data the input data
    * \param[in] width the width of the data
    * \param[in] height the height of the data
    * \param[in] element_stride the element stride of the data
    * \param[in] row_stride the row stride of the data
    */
  void
  setInput(const DataType * data,
           unsigned width, unsigned height, unsigned element_stride, unsigned row_stride);

  /** \brief Compute the first order sum within a given rectangle
    * \param[in] start_x x position of rectangle
    * \param[in] start_y y position of rectangle
    * \param[in] width width of rectangle
    * \param[in] height height of rectangle
    */
  inline ElementType
  getFirstOrderSum(unsigned start_x, unsigned start_y, unsigned width, unsigned height) const;

  /** \brief Compute the first order sum within a given rectangle
    * \param[in] start_x x position of the start of the rectangle
    * \param[in] start_y x position of the start of the rectangle
    * \param[in] end_x x position of the end of the rectangle
    * \param[in] end_y x position of the end of the rectangle
    */
  inline ElementType
  getFirstOrderSumSE(unsigned start_x, unsigned start_y, unsigned end_x, unsigned end_y) const;

  /** \brief Compute the second order sum within a given rectangle
    * \param[in] start_x x position of rectangle
    * \param[in] start_y y position of rectangle
    * \param[in] width width of rectangle
    * \param[in] height height of rectangle
    */
  inline SecondOrderType
  getSecondOrderSum(unsigned start_x, unsigned start_y, unsigned width, unsigned height) const;

  /** \brief Compute the second order sum within a given rectangle
    * \param[in] start_x x position of the start of the rectangle
    * \param[in] start_y x position of the start of the rectangle
    * \param[in] end_x x position of the end of the rectangle
    * \param[in] end_y x position of the end of the rectangle
    */
  inline SecondOrderType
  getSecondOrderSumSE(unsigned start_x, unsigned start_y, unsigned end_x, unsigned end_y) const;

  /** \brief Compute the number of finite elements within a given rectangle
    * \param[in] start_x x position of rectangle
    * \param[in] start_y y position of rectangle
    * \param[in] width width of rectangle
    * \param[in] height height of rectangle
    */
  inline unsigned
  getFiniteElementsCount(unsigned start_x, unsigned start_y, unsigned width, unsigned height) const;

  /** \brief Compute the number of finite elements within a given rectangle
    * \param[in] start_x x position of the start of the rectangle
    * \param[in] start_y x position of the start of the rectangle
    * \param[in] end_x x position of the end of the rectangle
    * \param[in] end_y x position of the end of the rectangle
    */
  inline unsigned
  getFiniteElementsCountSE(unsigned start_x, unsigned start_y, unsigned end_x, unsigned end_y) const;


//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*Here is the right place for your variables. Replace this line by your declarations!*/
  typedef Eigen::Matrix<typename IntegralImageTypeTraits<DataType>::Type, Dimension, 1> InputType;

  /** \brief Compute the actual integral image data
    * \param[in] data the input data
    * \param[in] element_stride the element stride of the data
    * \param[in] row_stride the row stride of the data
    */
  void
  computeIntegralImages(const DataType * data, unsigned row_stride, unsigned element_stride);

  std::vector<ElementType, Eigen::aligned_allocator<ElementType>> first_order_integral_image_;
  std::vector<SecondOrderType, Eigen::aligned_allocator<SecondOrderType>> second_order_integral_image_;
  std::vector<unsigned> finite_values_integral_image_;

  /** \brief The width of the 2d input data array */
  unsigned width_;
  /** \brief The height of the 2d input data array */
  unsigned height_;

  /** \brief Indicates whether second order integral images are available **/
  bool compute_second_order_integral_images_;

};


/**
  * \brief partial template specialization for integral images with just one channel.
  */
template <class DataType>
class tIntegralImage2D <DataType, 1>
{
public:
  static const unsigned second_order_size = 1;
  typedef typename IntegralImageTypeTraits<DataType>::IntegralType ElementType;
  typedef typename IntegralImageTypeTraits<DataType>::IntegralType SecondOrderType;

  /** \brief Constructor for an Integral Image
    * \param[in] compute_second_order_integral_images set to true if we want to compute a second order image
    */
  tIntegralImage2D(bool compute_second_order_integral_images) :
    first_order_integral_image_(),
    second_order_integral_image_(),
    finite_values_integral_image_(),
    width_(1), height_(1),
    compute_second_order_integral_images_(compute_second_order_integral_images)
  {
  }

  /** \brief Destructor */
  virtual
  ~tIntegralImage2D() { }

  /** \brief Set the input data to compute the integral image for
    * \param[in] data the input data
    * \param[in] width the width of the data
    * \param[in] height the height of the data
    * \param[in] element_stride the element stride of the data
    * \param[in] row_stride the row stride of the data
    */
  void
  setInput(const DataType * data,
           unsigned width, unsigned height, unsigned element_stride, unsigned row_stride);

  /** \brief Compute the first order sum within a given rectangle
    * \param[in] start_x x position of rectangle
    * \param[in] start_y y position of rectangle
    * \param[in] width width of rectangle
    * \param[in] height height of rectangle
    */
  inline ElementType
  getFirstOrderSum(unsigned start_x, unsigned start_y, unsigned width, unsigned height) const;

  /** \brief Compute the first order sum within a given rectangle
    * \param[in] start_x x position of the start of the rectangle
    * \param[in] start_y x position of the start of the rectangle
    * \param[in] end_x x position of the end of the rectangle
    * \param[in] end_y x position of the end of the rectangle
    */
  inline ElementType
  getFirstOrderSumSE(unsigned start_x, unsigned start_y, unsigned end_x, unsigned end_y) const;

  /** \brief Compute the second order sum within a given rectangle
    * \param[in] start_x x position of rectangle
    * \param[in] start_y y position of rectangle
    * \param[in] width width of rectangle
    * \param[in] height height of rectangle
    */
  inline SecondOrderType
  getSecondOrderSum(unsigned start_x, unsigned start_y, unsigned width, unsigned height) const;

  /** \brief Compute the second order sum within a given rectangle
    * \param[in] start_x x position of the start of the rectangle
    * \param[in] start_y x position of the start of the rectangle
    * \param[in] end_x x position of the end of the rectangle
    * \param[in] end_y x position of the end of the rectangle
    */
  inline SecondOrderType
  getSecondOrderSumSE(unsigned start_x, unsigned start_y, unsigned end_x, unsigned end_y) const;

  /** \brief Compute the number of finite elements within a given rectangle
    * \param[in] start_x x position of rectangle
    * \param[in] start_y y position of rectangle
    * \param[in] width width of rectangle
    * \param[in] height height of rectangle
    */
  inline unsigned
  getFiniteElementsCount(unsigned start_x, unsigned start_y, unsigned width, unsigned height) const;

  /** \brief Compute the number of finite elements within a given rectangle
    * \param[in] start_x x position of the start of the rectangle
    * \param[in] start_y x position of the start of the rectangle
    * \param[in] end_x x position of the end of the rectangle
    * \param[in] end_y x position of the end of the rectangle
    */
  inline unsigned
  getFiniteElementsCountSE(unsigned start_x, unsigned start_y, unsigned end_x, unsigned end_y) const;

private:
//  typedef typename IntegralImageTypeTraits<DataType>::Type InputType;

  /** \brief Compute the actual integral image data
    * \param[in] data the input data
    * \param[in] element_stride the element stride of the data
    * \param[in] row_stride the row stride of the data
    */
  void
  computeIntegralImages(const DataType * data, unsigned row_stride, unsigned element_stride);

  std::vector<ElementType, Eigen::aligned_allocator<ElementType>> first_order_integral_image_;
  std::vector<SecondOrderType, Eigen::aligned_allocator<SecondOrderType>> second_order_integral_image_;
  std::vector<unsigned> finite_values_integral_image_;

  /** \brief The width of the 2d input data array */
  unsigned width_;
  /** \brief The height of the 2d input data array */
  unsigned height_;

  /** \brief Indicates whether second order integral images are available **/
  bool compute_second_order_integral_images_;
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}

#include "projects/stereo_traversability_experiments/aras/libfeature/tIntegralImage2D.hpp"

#endif
