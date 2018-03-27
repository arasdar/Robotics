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
/*!\file    projects/icarus/sensor_processing/libfeature/tIntegralImage2D.hpp
 *
 * \author  Aras Dargazany
 *
 * \date    2014-05-29
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

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

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

////----------------------------------------------------------------------
//// tIntegralImage2D constructors
////----------------------------------------------------------------------
//template <>
//tIntegralImage2D<>::tIntegralImage2D() //:
//  /*If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!*/
//{}
//
////----------------------------------------------------------------------
//// tIntegralImage2D destructor
////----------------------------------------------------------------------
//template <>
//tIntegralImage2D<>::~tIntegralImage2D()
//{}
//
////----------------------------------------------------------------------
//// tIntegralImage2D SomeExampleMethod
////----------------------------------------------------------------------
//template <>
//void tIntegralImage2D<>::SomeExampleMethod()
//{
//  This is an example for a method. Replace it by your own methods!
//}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename DataType, unsigned Dimension>
void tIntegralImage2D<DataType, Dimension>::setSecondOrderComputation(bool compute_second_order_integral_images)
{
  compute_second_order_integral_images_ = compute_second_order_integral_images;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename DataType, unsigned Dimension>
void tIntegralImage2D<DataType, Dimension>::setInput(const DataType * data, unsigned width, unsigned height, unsigned element_stride, unsigned row_stride)
{
  if ((width + 1) * (height + 1) > first_order_integral_image_.size())
  {
    width_  = width;
    height_ = height;
    first_order_integral_image_.resize((width_ + 1) * (height_ + 1));
    finite_values_integral_image_.resize((width_ + 1) * (height_ + 1));
    if (compute_second_order_integral_images_)
      second_order_integral_image_.resize((width_ + 1) * (height_ + 1));
  }
  computeIntegralImages(data, row_stride, element_stride);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename DataType, unsigned Dimension>
typename tIntegralImage2D<DataType, Dimension>::ElementType
tIntegralImage2D<DataType, Dimension>::getFirstOrderSum(
  unsigned start_x, unsigned start_y, unsigned width, unsigned height) const
{
  const unsigned upper_left_idx      = start_y * (width_ + 1) + start_x;
  const unsigned upper_right_idx     = upper_left_idx + width;
  const unsigned lower_left_idx      = (start_y + height) * (width_ + 1) + start_x;
  const unsigned lower_right_idx     = lower_left_idx + width;

  return (first_order_integral_image_[lower_right_idx] + first_order_integral_image_[upper_left_idx]  -
          first_order_integral_image_[upper_right_idx] - first_order_integral_image_[lower_left_idx]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename DataType, unsigned Dimension>
typename tIntegralImage2D<DataType, Dimension>::SecondOrderType
tIntegralImage2D<DataType, Dimension>::getSecondOrderSum(
  unsigned start_x, unsigned start_y, unsigned width, unsigned height) const
{
  const unsigned upper_left_idx      = start_y * (width_ + 1) + start_x;
  const unsigned upper_right_idx     = upper_left_idx + width;
  const unsigned lower_left_idx      = (start_y + height) * (width_ + 1) + start_x;
  const unsigned lower_right_idx     = lower_left_idx + width;

  return (second_order_integral_image_[lower_right_idx] + second_order_integral_image_[upper_left_idx]  -
          second_order_integral_image_[upper_right_idx] - second_order_integral_image_[lower_left_idx]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename DataType, unsigned Dimension>
unsigned tIntegralImage2D<DataType, Dimension>::getFiniteElementsCount(
  unsigned start_x, unsigned start_y, unsigned width, unsigned height) const
{
  const unsigned upper_left_idx      = start_y * (width_ + 1) + start_x;
  const unsigned upper_right_idx     = upper_left_idx + width;
  const unsigned lower_left_idx      = (start_y + height) * (width_ + 1) + start_x;
  const unsigned lower_right_idx     = lower_left_idx + width;

  return (finite_values_integral_image_[lower_right_idx] + finite_values_integral_image_[upper_left_idx]  -
          finite_values_integral_image_[upper_right_idx] - finite_values_integral_image_[lower_left_idx]);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename DataType, unsigned Dimension>
typename tIntegralImage2D<DataType, Dimension>::ElementType
tIntegralImage2D<DataType, Dimension>::getFirstOrderSumSE(
  unsigned start_x, unsigned start_y, unsigned end_x, unsigned end_y) const
{
  const unsigned upper_left_idx      = start_y * (width_ + 1) + start_x;
  const unsigned upper_right_idx     = start_y * (width_ + 1) + end_x;
  const unsigned lower_left_idx      = end_y * (width_ + 1) + start_x;
  const unsigned lower_right_idx     = end_y * (width_ + 1) + end_x;

  return (first_order_integral_image_[lower_right_idx] + first_order_integral_image_[upper_left_idx]  -
          first_order_integral_image_[upper_right_idx] - first_order_integral_image_[lower_left_idx]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename DataType, unsigned Dimension>
typename tIntegralImage2D<DataType, Dimension>::SecondOrderType
tIntegralImage2D<DataType, Dimension>::getSecondOrderSumSE(
  unsigned start_x, unsigned start_y, unsigned end_x, unsigned end_y) const
{
  const unsigned upper_left_idx      = start_y * (width_ + 1) + start_x;
  const unsigned upper_right_idx     = start_y * (width_ + 1) + end_x;
  const unsigned lower_left_idx      = end_y * (width_ + 1) + start_x;
  const unsigned lower_right_idx     = end_y * (width_ + 1) + end_x;

  return (second_order_integral_image_[lower_right_idx] + second_order_integral_image_[upper_left_idx]  -
          second_order_integral_image_[upper_right_idx] - second_order_integral_image_[lower_left_idx]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename DataType, unsigned Dimension>
unsigned tIntegralImage2D<DataType, Dimension>::getFiniteElementsCountSE(
  unsigned start_x, unsigned start_y, unsigned end_x, unsigned end_y) const
{
  const unsigned upper_left_idx      = start_y * (width_ + 1) + start_x;
  const unsigned upper_right_idx     = start_y * (width_ + 1) + end_x;
  const unsigned lower_left_idx      = end_y * (width_ + 1) + start_x;
  const unsigned lower_right_idx     = end_y * (width_ + 1) + end_x;

  return (finite_values_integral_image_[lower_right_idx] + finite_values_integral_image_[upper_left_idx]  -
          finite_values_integral_image_[upper_right_idx] - finite_values_integral_image_[lower_left_idx]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename DataType, unsigned Dimension> void
tIntegralImage2D<DataType, Dimension>::computeIntegralImages(
  const DataType *data, unsigned row_stride, unsigned element_stride)
{
  ElementType* previous_row = &first_order_integral_image_[0];
  ElementType* current_row  = previous_row + (width_ + 1);
  memset(previous_row, 0, sizeof(ElementType) * (width_ + 1));

  unsigned* count_previous_row = &finite_values_integral_image_[0];
  unsigned* count_current_row  = count_previous_row + (width_ + 1);
  memset(count_previous_row, 0, sizeof(unsigned) * (width_ + 1));

  if (!compute_second_order_integral_images_)
  {
    for (unsigned rowIdx = 0; rowIdx < height_; ++rowIdx, data += row_stride,
         previous_row = current_row, current_row += (width_ + 1),
         count_previous_row = count_current_row, count_current_row += (width_ + 1))
    {
      current_row [0].setZero();
      count_current_row [0] = 0;
      for (unsigned colIdx = 0, valIdx = 0; colIdx < width_; ++colIdx, valIdx += element_stride)
      {
        current_row [colIdx + 1] = previous_row [colIdx + 1] + current_row [colIdx] - previous_row [colIdx];
        count_current_row [colIdx + 1] = count_previous_row [colIdx + 1] + count_current_row [colIdx] - count_previous_row [colIdx];
        const InputType* element = reinterpret_cast <const InputType*>(&data [valIdx]);
        if (pcl_isfinite(element->sum()))
        {
          current_row [colIdx + 1] += element->template cast<typename IntegralImageTypeTraits<DataType>::IntegralType>();
          ++(count_current_row [colIdx + 1]);
        }
      }
    }
  }
  else
  {
    SecondOrderType* so_previous_row = &second_order_integral_image_[0];
    SecondOrderType* so_current_row  = so_previous_row + (width_ + 1);
    memset(so_previous_row, 0, sizeof(SecondOrderType) * (width_ + 1));

    SecondOrderType so_element;
    for (unsigned rowIdx = 0; rowIdx < height_; ++rowIdx, data += row_stride,
         previous_row = current_row, current_row += (width_ + 1),
         count_previous_row = count_current_row, count_current_row += (width_ + 1),
         so_previous_row = so_current_row, so_current_row += (width_ + 1))
    {
      current_row [0].setZero();
      so_current_row [0].setZero();
      count_current_row [0] = 0;
      for (unsigned colIdx = 0, valIdx = 0; colIdx < width_; ++colIdx, valIdx += element_stride)
      {
        current_row [colIdx + 1] = previous_row [colIdx + 1] + current_row [colIdx] - previous_row [colIdx];
        so_current_row [colIdx + 1] = so_previous_row [colIdx + 1] + so_current_row [colIdx] - so_previous_row [colIdx];
        count_current_row [colIdx + 1] = count_previous_row [colIdx + 1] + count_current_row [colIdx] - count_previous_row [colIdx];

        const InputType* element = reinterpret_cast <const InputType*>(&data [valIdx]);
        if (pcl_isfinite(element->sum()))
        {
          current_row [colIdx + 1] += element->template cast<typename IntegralImageTypeTraits<DataType>::IntegralType>();
          ++(count_current_row [colIdx + 1]);
          for (unsigned myIdx = 0, elIdx = 0; myIdx < Dimension; ++myIdx)
            for (unsigned mxIdx = myIdx; mxIdx < Dimension; ++mxIdx, ++elIdx)
              so_current_row [colIdx + 1][elIdx] += (*element)[myIdx] * (*element)[mxIdx];
        }
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename DataType>
void tIntegralImage2D<DataType, 1>::setInput(const DataType * data, unsigned width, unsigned height,
    unsigned element_stride, unsigned row_stride)
{
  if ((width + 1) * (height + 1) > first_order_integral_image_.size())
  {
    width_  = width;
    height_ = height;
    first_order_integral_image_.resize((width_ + 1) * (height_ + 1));
    finite_values_integral_image_.resize((width_ + 1) * (height_ + 1));
    if (compute_second_order_integral_images_)
      second_order_integral_image_.resize((width_ + 1) * (height_ + 1));
  }
  computeIntegralImages(data, row_stride, element_stride);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename DataType>
typename tIntegralImage2D<DataType, 1>::ElementType
tIntegralImage2D<DataType, 1>::getFirstOrderSum(
  unsigned start_x, unsigned start_y, unsigned width, unsigned height) const
{
  const unsigned upper_left_idx      = start_y * (width_ + 1) + start_x;
  const unsigned upper_right_idx     = upper_left_idx + width;
  const unsigned lower_left_idx      = (start_y + height) * (width_ + 1) + start_x;
  const unsigned lower_right_idx     = lower_left_idx + width;

  return (first_order_integral_image_[lower_right_idx] + first_order_integral_image_[upper_left_idx]  -
          first_order_integral_image_[upper_right_idx] - first_order_integral_image_[lower_left_idx]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename DataType>
typename tIntegralImage2D<DataType, 1>::SecondOrderType
tIntegralImage2D<DataType, 1>::getSecondOrderSum(
  unsigned start_x, unsigned start_y, unsigned width, unsigned height) const
{
  const unsigned upper_left_idx      = start_y * (width_ + 1) + start_x;
  const unsigned upper_right_idx     = upper_left_idx + width;
  const unsigned lower_left_idx      = (start_y + height) * (width_ + 1) + start_x;
  const unsigned lower_right_idx     = lower_left_idx + width;

  return (second_order_integral_image_[lower_right_idx] + second_order_integral_image_[upper_left_idx]  -
          second_order_integral_image_[upper_right_idx] - second_order_integral_image_[lower_left_idx]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename DataType> unsigned
tIntegralImage2D<DataType, 1>::getFiniteElementsCount(
  unsigned start_x, unsigned start_y, unsigned width, unsigned height) const
{
  const unsigned upper_left_idx      = start_y * (width_ + 1) + start_x;
  const unsigned upper_right_idx     = upper_left_idx + width;
  const unsigned lower_left_idx      = (start_y + height) * (width_ + 1) + start_x;
  const unsigned lower_right_idx     = lower_left_idx + width;

  return (finite_values_integral_image_[lower_right_idx] + finite_values_integral_image_[upper_left_idx]  -
          finite_values_integral_image_[upper_right_idx] - finite_values_integral_image_[lower_left_idx]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename DataType>
typename tIntegralImage2D<DataType, 1>::ElementType
tIntegralImage2D<DataType, 1>::getFirstOrderSumSE(
  unsigned start_x, unsigned start_y, unsigned end_x, unsigned end_y) const
{
  const unsigned upper_left_idx      = start_y * (width_ + 1) + start_x;
  const unsigned upper_right_idx     = start_y * (width_ + 1) + end_x;
  const unsigned lower_left_idx      = end_y * (width_ + 1) + start_x;
  const unsigned lower_right_idx     = end_y * (width_ + 1) + end_x;

  return (first_order_integral_image_[lower_right_idx] + first_order_integral_image_[upper_left_idx]  -
          first_order_integral_image_[upper_right_idx] - first_order_integral_image_[lower_left_idx]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename DataType>
typename tIntegralImage2D<DataType, 1>::SecondOrderType
tIntegralImage2D<DataType, 1>::getSecondOrderSumSE(
  unsigned start_x, unsigned start_y, unsigned end_x, unsigned end_y) const
{
  const unsigned upper_left_idx      = start_y * (width_ + 1) + start_x;
  const unsigned upper_right_idx     = start_y * (width_ + 1) + end_x;
  const unsigned lower_left_idx      = end_y * (width_ + 1) + start_x;
  const unsigned lower_right_idx     = end_y * (width_ + 1) + end_x;

  return (second_order_integral_image_[lower_right_idx] + second_order_integral_image_[upper_left_idx]  -
          second_order_integral_image_[upper_right_idx] - second_order_integral_image_[lower_left_idx]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename DataType>
unsigned tIntegralImage2D<DataType, 1>::getFiniteElementsCountSE(
  unsigned start_x, unsigned start_y, unsigned end_x, unsigned end_y) const
{
  const unsigned upper_left_idx      = start_y * (width_ + 1) + start_x;
  const unsigned upper_right_idx     = start_y * (width_ + 1) + end_x;
  const unsigned lower_left_idx      = end_y * (width_ + 1) + start_x;
  const unsigned lower_right_idx     = end_y * (width_ + 1) + end_x;

  return (finite_values_integral_image_[lower_right_idx] + finite_values_integral_image_[upper_left_idx]  -
          finite_values_integral_image_[upper_right_idx] - finite_values_integral_image_[lower_left_idx]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename DataType>
void tIntegralImage2D<DataType, 1>::computeIntegralImages(
  const DataType *data, unsigned row_stride, unsigned element_stride)
{
  ElementType* previous_row = &first_order_integral_image_[0];
  ElementType* current_row  = previous_row + (width_ + 1);
  memset(previous_row, 0, sizeof(ElementType) * (width_ + 1));

  unsigned* count_previous_row = &finite_values_integral_image_[0];
  unsigned* count_current_row  = count_previous_row + (width_ + 1);
  memset(count_previous_row, 0, sizeof(unsigned) * (width_ + 1));

  if (!compute_second_order_integral_images_)
  {
    for (unsigned rowIdx = 0; rowIdx < height_; ++rowIdx, data += row_stride,
         previous_row = current_row, current_row += (width_ + 1),
         count_previous_row = count_current_row, count_current_row += (width_ + 1))
    {
      current_row [0] = 0.0;
      count_current_row [0] = 0;
      for (unsigned colIdx = 0, valIdx = 0; colIdx < width_; ++colIdx, valIdx += element_stride)
      {
        current_row [colIdx + 1] = previous_row [colIdx + 1] + current_row [colIdx] - previous_row [colIdx];
        count_current_row [colIdx + 1] = count_previous_row [colIdx + 1] + count_current_row [colIdx] - count_previous_row [colIdx];
        if (pcl_isfinite(data [valIdx]))
        {
          current_row [colIdx + 1] += data [valIdx];
          ++(count_current_row [colIdx + 1]);
        }
      }
    }
  }
  else
  {
    SecondOrderType* so_previous_row = &second_order_integral_image_[0];
    SecondOrderType* so_current_row  = so_previous_row + (width_ + 1);
    memset(so_previous_row, 0, sizeof(SecondOrderType) * (width_ + 1));

    for (unsigned rowIdx = 0; rowIdx < height_; ++rowIdx, data += row_stride,
         previous_row = current_row, current_row += (width_ + 1),
         count_previous_row = count_current_row, count_current_row += (width_ + 1),
         so_previous_row = so_current_row, so_current_row += (width_ + 1))
    {
      current_row [0] = 0.0;
      so_current_row [0] = 0.0;
      count_current_row [0] = 0;
      for (unsigned colIdx = 0, valIdx = 0; colIdx < width_; ++colIdx, valIdx += element_stride)
      {
        current_row [colIdx + 1] = previous_row [colIdx + 1] + current_row [colIdx] - previous_row [colIdx];
        so_current_row [colIdx + 1] = so_previous_row [colIdx + 1] + so_current_row [colIdx] - so_previous_row [colIdx];
        count_current_row [colIdx + 1] = count_previous_row [colIdx + 1] + count_current_row [colIdx] - count_previous_row [colIdx];
        if (pcl_isfinite(data[valIdx]))
        {
          current_row [colIdx + 1] += data[valIdx];
          so_current_row [colIdx + 1] += data[valIdx] * data[valIdx];
          ++(count_current_row [colIdx + 1]);
        }
      }
    }
  }
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
