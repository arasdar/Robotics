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
/*!\file    projects/icarus/sensor_processing/libfeature/tIntegralImageNormalEstimation.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-05-29
 *
 * \brief   Contains tIntegralImageNormalEstimation
 *
 * \b tIntegralImageNormalEstimation
 *
 * This class is used for Surface normal estimation on organized data using integral images.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__stereo_traversability_experiments__aras__libfeature__tIntegralImageNormalEstimation_h__
#define __projects__stereo_traversability_experiments__aras__libfeature__tIntegralImageNormalEstimation_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/aras/libfeature/tFeature.h"
#include "projects/stereo_traversability_experiments/aras/libfeature/tIntegralImage2D.h"

//#if defined BUILD_Maintainer && defined __GNUC__ && __GNUC__ == 4 && __GNUC_MINOR__ > 3
//#pragma GCC diagnostic ignored "-Weffc++"
//#endif

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
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This class is used for Surface normal estimation on organized data using integral images.
    *
    *        For detailed information about this method see:
    *
    *        S. Holzer and R. B. Rusu and M. Dixon and S. Gedikli and N. Navab,
    *        Adaptive Neighborhood Selection for Real-Time Surface Normal Estimation
    *        from Organized Point Cloud Data Using Integral Images, IROS 2012.
    *
    *        D. Holz, S. Holzer, R. B. Rusu, and S. Behnke (2011, July).
    *        Real-Time Plane Segmentation using RGB-D Cameras. In Proceedings of
    *        the 15th RoboCup International Symposium, Istanbul, Turkey.
    *        http://www.ais.uni-bonn.de/~holz/papers/holz_2011_robocup.pdf
    *
 */
template <typename PointInT, typename PointOutT>
class tIntegralImageNormalEstimation : public tFeature<PointInT, PointOutT>
{

  using tFeature<PointInT, PointOutT>::input_;
  using tFeature<PointInT, PointOutT>::feature_name_;
  using tFeature<PointInT, PointOutT>::tree_;
  using tFeature<PointInT, PointOutT>::k_;
  using tFeature<PointInT, PointOutT>::indices_;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef boost::shared_ptr<tIntegralImageNormalEstimation<PointInT, PointOutT>> Ptr;
  typedef boost::shared_ptr<const tIntegralImageNormalEstimation<PointInT, PointOutT>> ConstPtr;

  /** \brief Different types of border handling. */
  enum tBorderPolicy
  {
    BORDER_POLICY_IGNORE,
    BORDER_POLICY_MIRROR
  };

  /** \brief Different normal estimation methods.
    * <ul>
    *   <li><b>COVARIANCE_MATRIX</b> - creates 9 integral images to compute the normal for a specific point
    *   from the covariance matrix of its local neighborhood.</li>
    *   <li><b>AVERAGE_3D_GRADIENT</b> - creates 6 integral images to compute smoothed versions of
    *   horizontal and vertical 3D gradients and computes the normals using the cross-product between these
    *   two gradients.
    *   <li><b>AVERAGE_DEPTH_CHANGE</b> -  creates only a single integral image and computes the normals
    *   from the average depth changes.
    * </ul>
    */
  enum tNormalEstimationMethod
  {
    COVARIANCE_MATRIX,
    AVERAGE_3D_GRADIENT,
    AVERAGE_DEPTH_CHANGE,
    SIMPLE_3D_GRADIENT
  };

  typedef typename tFeature<PointInT, PointOutT>::PointCloudIn  PointCloudIn;
  typedef typename tFeature<PointInT, PointOutT>::PointCloudOut PointCloudOut;

  /** \brief Constructor */
  tIntegralImageNormalEstimation()
    : normal_estimation_method_(AVERAGE_3D_GRADIENT)
    , border_policy_(BORDER_POLICY_IGNORE)
    , rect_width_(0), rect_width_2_(0), rect_width_4_(0)
    , rect_height_(0), rect_height_2_(0), rect_height_4_(0)
    , distance_threshold_(0)
    , integral_image_DX_(false)
    , integral_image_DY_(false)
    , integral_image_depth_(false)
    , integral_image_XYZ_(true)
    , diff_x_(NULL)
    , diff_y_(NULL)
    , depth_data_(NULL)
    , distance_map_(NULL)
    , use_depth_dependent_smoothing_(false)
    , max_depth_change_factor_(20.0f * 0.001f)
    , normal_smoothing_size_(10.0f)
    , init_covariance_matrix_(false)
    , init_average_3d_gradient_(false)
    , init_simple_3d_gradient_(false)
    , init_depth_change_(false)
    , vpx_(0.0f)
    , vpy_(0.0f)
    , vpz_(0.0f)
    , use_sensor_origin_(true)
  {
    feature_name_ = "IntegralImagesNormalEstimation";
    tree_.reset();
    k_ = 1;
  }

  /** \brief Destructor **/
  virtual ~tIntegralImageNormalEstimation();


  /** \brief Set the regions size which is considered for normal estimation.
    * \param[in] width the width of the search rectangle
    * \param[in] height the height of the search rectangle
    */
  void
  setRectSize(const int width, const int height);

  /** \brief Sets the policy for handling borders.
    * \param[in] border_policy the border policy.
    */
  void
  setBorderPolicy(const tBorderPolicy border_policy)
  {
    border_policy_ = border_policy;
  }

  /** \brief Computes the normal at the specified position.
    * \param[in] pos_x x position (pixel)
    * \param[in] pos_y y position (pixel)
    * \param[in] point_index the position index of the point
    * \param[out] normal the output estimated normal
    */
  void
  computePointNormal(const int pos_x, const int pos_y, const unsigned point_index, PointOutT &normal);

  /** \brief Computes the normal at the specified position with mirroring for border handling.
    * \param[in] pos_x x position (pixel)
    * \param[in] pos_y y position (pixel)
    * \param[in] point_index the position index of the point
    * \param[out] normal the output estimated normal
    */
  void
  computePointNormalMirror(const int pos_x, const int pos_y, const unsigned point_index, PointOutT &normal);

  /** \brief The depth change threshold for computing object borders
    * \param[in] max_depth_change_factor the depth change threshold for computing object borders based on
    * depth changes
    */
  void
  setMaxDepthChangeFactor(float max_depth_change_factor)
  {
    max_depth_change_factor_ = max_depth_change_factor;
  }

  /** \brief Set the normal smoothing size
    * \param[in] normal_smoothing_size factor which influences the size of the area used to smooth normals
    * (depth dependent if useDepthDependentSmoothing is true)
    */
  void
  setNormalSmoothingSize(float normal_smoothing_size)
  {
    if (normal_smoothing_size <= 0)
    {
      PCL_ERROR("[pcl::%s::setNormalSmoothingSize] Invalid normal smoothing size given! (%f). Allowed ranges are: 0 < N. Defaulting to %f.\n",
                feature_name_.c_str(), normal_smoothing_size, normal_smoothing_size_);
      return;
    }
    normal_smoothing_size_ = normal_smoothing_size;
  }

  /** \brief Set the normal estimation method. The current implemented algorithms are:
    * <ul>
    *   <li><b>COVARIANCE_MATRIX</b> - creates 9 integral images to compute the normal for a specific point
    *   from the covariance matrix of its local neighborhood.</li>
    *   <li><b>AVERAGE_3D_GRADIENT</b> - creates 6 integral images to compute smoothed versions of
    *   horizontal and vertical 3D gradients and computes the normals using the cross-product between these
    *   two gradients.
    *   <li><b>AVERAGE_DEPTH_CHANGE</b> -  creates only a single integral image and computes the normals
    *   from the average depth changes.
    * </ul>
    * \param[in] normal_estimation_method the method used for normal estimation
    */
  void
  setNormalEstimationMethod(tNormalEstimationMethod normal_estimation_method)
  {
    normal_estimation_method_ = normal_estimation_method;
  }

  /** \brief Set whether to use depth depending smoothing or not
    * \param[in] use_depth_dependent_smoothing decides whether the smoothing is depth dependent
    */
  void
  setDepthDependentSmoothing(bool use_depth_dependent_smoothing)
  {
    use_depth_dependent_smoothing_ = use_depth_dependent_smoothing;
  }

  /** \brief Provide a pointer to the input dataset (overwrites the PCLBase::setInputCloud method)
    * \param[in] cloud the const boost shared pointer to a PointCloud message
    */
  virtual inline void
  setInputCloud(const typename PointCloudIn::ConstPtr &cloud)
  {
    input_ = cloud;
    if (!cloud->isOrganized())
    {
      PCL_ERROR("[pcl::IntegralImageNormalEstimation::setInputCloud] Input dataset is not organized (height = 1).\n");
      return;
    }

    init_covariance_matrix_ = init_average_3d_gradient_ = init_depth_change_ = false;

    if (use_sensor_origin_)
    {
      vpx_ = input_->sensor_origin_.coeff(0);
      vpy_ = input_->sensor_origin_.coeff(1);
      vpz_ = input_->sensor_origin_.coeff(2);
    }

    // Initialize the correct data structure based on the normal estimation method chosen
    initData();
  }

  /** \brief Returns a pointer to the distance map which was computed internally
    */
  inline float*
  getDistanceMap()
  {
    return (distance_map_);
  }

  /** \brief Set the viewpoint.
    * \param vpx the X coordinate of the viewpoint
    * \param vpy the Y coordinate of the viewpoint
    * \param vpz the Z coordinate of the viewpoint
    */
  inline void
  setViewPoint(float vpx, float vpy, float vpz)
  {
    vpx_ = vpx;
    vpy_ = vpy;
    vpz_ = vpz;
    use_sensor_origin_ = false;
  }

  /** \brief Get the viewpoint.
    * \param [out] vpx x-coordinate of the view point
    * \param [out] vpy y-coordinate of the view point
    * \param [out] vpz z-coordinate of the view point
    * \note this method returns the currently used viewpoint for normal flipping.
    * If the viewpoint is set manually using the setViewPoint method, this method will return the set view point coordinates.
    * If an input cloud is set, it will return the sensor origin otherwise it will return the origin (0, 0, 0)
    */
  inline void
  getViewPoint(float &vpx, float &vpy, float &vpz)
  {
    vpx = vpx_;
    vpy = vpy_;
    vpz = vpz_;
  }

  /** \brief sets whether the sensor origin or a user given viewpoint should be used. After this method, the
    * normal estimation method uses the sensor origin of the input cloud.
    * to use a user defined view point, use the method setViewPoint
    */
  inline void
  useSensorOriginAsViewPoint()
  {
    use_sensor_origin_ = true;
    if (input_)
    {
      vpx_ = input_->sensor_origin_.coeff(0);
      vpy_ = input_->sensor_origin_.coeff(1);
      vpz_ = input_->sensor_origin_.coeff(2);
    }
    else
    {
      vpx_ = 0;
      vpy_ = 0;
      vpz_ = 0;
    }
  }

  /*Here is the right place for your public methods. Replace this line by your declarations!*/
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW


//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*Here is the right place for your variables. Replace this line by your declarations!*/
  /** \brief Flip (in place) the estimated normal of a point towards a given viewpoint
    * \param point a given point
    * \param vp_x the X coordinate of the viewpoint
    * \param vp_y the X coordinate of the viewpoint
    * \param vp_z the X coordinate of the viewpoint
    * \param nx the resultant X component of the plane normal
    * \param ny the resultant Y component of the plane normal
    * \param nz the resultant Z component of the plane normal
    * \ingroup features
    */
  inline void
  flipNormalTowardsViewpoint(const PointInT &point,
                             float vp_x, float vp_y, float vp_z,
                             float &nx, float &ny, float &nz)
  {
    // See if we need to flip any plane normals
    vp_x -= point.x;
    vp_y -= point.y;
    vp_z -= point.z;

    // Dot product between the (viewpoint - point) and the plane normal
    float cos_theta = (vp_x * nx + vp_y * ny + vp_z * nz);

    // Flip the plane normal
    if (cos_theta < 0)
    {
      nx *= -1;
      ny *= -1;
      nz *= -1;
    }
  }

  /** \brief The normal estimation method to use. Currently, 3 implementations are provided:
    *
    * - COVARIANCE_MATRIX
    * - AVERAGE_3D_GRADIENT
    * - AVERAGE_DEPTH_CHANGE
    */
  tNormalEstimationMethod normal_estimation_method_;

  /** \brief The policy for handling borders. */
  tBorderPolicy border_policy_;

  /** The width of the neighborhood region used for computing the normal. */
  int rect_width_;
  int rect_width_2_;
  int rect_width_4_;
  /** The height of the neighborhood region used for computing the normal. */
  int rect_height_;
  int rect_height_2_;
  int rect_height_4_;

  /** the threshold used to detect depth discontinuities */
  float distance_threshold_;

  /** integral image in x-direction */
  tIntegralImage2D<float, 3> integral_image_DX_;
  /** integral image in y-direction */
  tIntegralImage2D<float, 3> integral_image_DY_;
  /** integral image */
  tIntegralImage2D<float, 1> integral_image_depth_;
  /** integral image xyz */
  tIntegralImage2D<float, 3> integral_image_XYZ_;

  /** derivatives in x-direction */
  float *diff_x_;
  /** derivatives in y-direction */
  float *diff_y_;

  /** depth data */
  float *depth_data_;

  /** distance map */
  float *distance_map_;

  /** \brief Smooth data based on depth (true/false). */
  bool use_depth_dependent_smoothing_;

  /** \brief Threshold for detecting depth discontinuities */
  float max_depth_change_factor_;

  /** \brief */
  float normal_smoothing_size_;

  /** \brief True when a dataset has been received and the covariance_matrix data has been initialized. */
  bool init_covariance_matrix_;

  /** \brief True when a dataset has been received and the average 3d gradient data has been initialized. */
  bool init_average_3d_gradient_;

  /** \brief True when a dataset has been received and the simple 3d gradient data has been initialized. */
  bool init_simple_3d_gradient_;

  /** \brief True when a dataset has been received and the depth change data has been initialized. */
  bool init_depth_change_;

  /** \brief Values describing the viewpoint ("pinhole" camera model assumed). For per point viewpoints, inherit
    * from NormalEstimation and provide your own computeFeature (). By default, the viewpoint is set to 0,0,0. */
  float vpx_, vpy_, vpz_;

  /** whether the sensor origin of the input cloud or a user given viewpoint should be used.*/
  bool use_sensor_origin_;

  /** \brief This method should get called before starting the actual computation. */
  bool
  initCompute();

  /** \brief Internal initialization method for COVARIANCE_MATRIX estimation. */
  void
  initCovarianceMatrixMethod();

  /** \brief Internal initialization method for AVERAGE_3D_GRADIENT estimation. */
  void
  initAverage3DGradientMethod();

  /** \brief Internal initialization method for AVERAGE_DEPTH_CHANGE estimation. */
  void
  initAverageDepthChangeMethod();

  /** \brief Internal initialization method for SIMPLE_3D_GRADIENT estimation. */
  void
  initSimple3DGradientMethod();


//----------------------------------------------------------------------
// Protected fields and methods
//----------------------------------------------------------------------
protected:

  /** \brief Computes the normal for the complete cloud or only \a indices_ if provided.
    * \param[out] output the resultant normals
    */
  void
  computeFeature(PointCloudOut &output);

  /** \brief Computes the normal for the complete cloud.
    * \param[in] distance_map distance map
    * \param[in] bad_point constant given to invalid normal components
    * \param[out] output the resultant normals
    */
  void
  computeFeatureFull(const float* distance_map, const float& bad_point, PointCloudOut& output);

  /** \brief Computes the normal for part of the cloud specified by \a indices_
    * \param[in] distance_map distance map
    * \param[in] bad_point constant given to invalid normal components
    * \param[out] output the resultant normals
    */
  void
  computeFeaturePart(const float* distance_map, const float& bad_point, PointCloudOut& output);

  /** \brief Initialize the data structures, based on the normal estimation method chosen. */
  void
  initData();
};

//#if defined BUILD_Maintainer && defined __GNUC__ && __GNUC__ == 4 && __GNUC_MINOR__ > 3
//#pragma GCC diagnostic warning "-Weffc++"
//#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}

#include "projects/stereo_traversability_experiments/aras/libfeature/tIntegralImageNormalEstimation.hpp"

#endif
