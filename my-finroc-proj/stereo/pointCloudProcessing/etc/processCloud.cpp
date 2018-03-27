


//    /*display curvatures*/
//    CloudPtr curvature_cloud(new Cloud);
//    *curvature_cloud = *cloud;
//    //curvature_cloud = cloud_filtered;
//    int counter_green = 0, counter_red = 0;
//    for (size_t i  = 0; i < cloud->points.size(); i++)
//      //for ( size_t i; i < cloud_filtered->points.size(); i++ )
//    {
//      if (normal_cloud->points[i].curvature < 1) //0.04 original
//      {
//        curvature_cloud->points[i].r = 0;
//        curvature_cloud->points[i].g = 255;
//        curvature_cloud->points[i].b = 0;
//
//        //counting the green points
//        if (curvature_cloud->points[i].y <= dominant_plane_y_max && curvature_cloud->points[i].y >= dominant_plane_y_min )
//        {
//          counter_green ++;
//        }//if counter
//
//      }//if normal
//      else
//      {
//        curvature_cloud->points[i].r = 255;
//        curvature_cloud->points[i].g = 0;
//        curvature_cloud->points[i].b = 0;
//
//        //counting the red points which is curvature
//        if (curvature_cloud->points[i].y <= dominant_plane_y_max && curvature_cloud->points[i].y >= dominant_plane_y_min)
//        {
//          counter_red ++;
//        }//if counter red
//      }//esle
//    }//for
//
//    cout << "counter green which is NOT curvature: "<< counter_green << endl;
//    cout << "counter red points which is CURVATURE: "<< counter_red << endl;
//
//    /*! Draw the segmentation result testtttttt*/
//    CloudPtr ground_image_test(new Cloud);
//    *ground_image_test = *curvature_cloud;




//    /*
//     * VoxelGrid assembles a local 3D grid over a given PointCloud, and downsamples + filters the data.
//     *  The VoxelGrid class creates a 3D voxel grid (think about a voxel grid as a set of tiny 3D boxes in space) over the input point cloud data.
//     *  Then, in each voxel (i.e., 3D box), all the points present will be approximated (i.e., downsampled) with their centroid.
//     *  This approach is a bit slower than approximating them with the center of the voxel, but it represents the underlying surface more accurately.
//     */
//    // Create the filtering object
//    pcl::VoxelGrid<PointT> sor ; //(new VoxelGrid<PointT>)
//    //sor.setInputCloud(ground_image_test);
//    sor.setInputCloud(cloud_filtered);
//    /*Then, a pcl::VoxelGrid filter is created with a leaf size of 5cm*/
//    //sor.setLeafSize(0.05f, 0.05f, 0.05f);  //grid cell size
//    //sor.setLeafSize (0.1f, 0.1f, 0.1f); //grid cell size
//    //sor.setLeafSize(0.2f, 0.2f, 0.2f);
//    sor.setLeafSize(0.3f, 0.3f, 0.3f);
//    //sor.setLeafSize(0.5f, 0.5f, 0.5f);
//
//  //  /*      Set to true if leaf layout information needs to be saved for later access. */
//  //  bool save_leaf_layout = true;
//  //  sor.setSaveLeafLayout (save_leaf_layout);
//  //
//  //  CloudPtr ground_image_test(new Cloud);
//  //    sor.filter(*ground_image_test);
//  //    //sor.filter(*prev_ground_image_test);
//  //    //prev_ground_image_test = ground_image_test;  //testttttttttttttt
//  //
//  //    //for efficiency, user must make sure that the saving of the leaf layout is enabled and filtering performed
//  //
//  //    /*      Get the minimum coordinates of the bounding box (after filtering is performed). */
//  //  Eigen::Vector3i box_min = sor.getMinBoxCoordinates();
//  //  Eigen::Vector3i box_max = sor.getMaxBoxCoordinates();
//  //  //cout << "min" << box_min << endl;
//  //  //cout << "max" << box_max << endl;
//  //
//  //  int i_min = box_min[0];
//  //  int i_max = box_max[0];
//  //  //cout << i_min << " and " << i_max <<  endl;
//  //
//  //  int j_min = box_min[1];
//  //  int j_max = box_max[1];
//  //  //cout << j_min << " and " << j_max << endl;
//  //
//  //  int k_min = box_min[2];
//  //  int k_max = box_max[2];
//  //  //cout << k_min << " and " << k_max <<  endl;
//  //
//  //    //Returns the layout of the leafs for fast access to cells relative to current position.
//  //    //Note:
//  //    //position at (i-min_x) + (j-min_y)*div_x + (k-min_z)*div_x*div_y holds the index of the element at coordinates (i,j,k) in the grid (-1 if empty)
//  //    //std::vector<int> pcl::VoxelGrid< PointT >::getLeafLayout  (   )
//  //    vector<int> leaf_layout = sor.getLeafLayout();
//  //
//  ////    int test;
//  ////    for (int i = 0; cloud_filtered->size(); i++)
//  ////    {
//  ////
//  ////      test++;
//  ////    }
//  //
//  ////    //for (int i = 0; i < ground_image_test->points.size(); i++) //iterating over all the voxels centroids
//  //    for (int i = 0; i < leaf_layout.size(); i++) //leaf lay out of the voxels
//  //    {
//  //
//  //    vector <int> voxel_indices; //all indices of one voxel
//  //
//  //      for (int j = 0; j < cloud_filtered->points.size(); j++ )
//  //      {
//  //        float x = cloud->points[j].x ;
//  //        float y = cloud->points[j].y ;
//  //        float z = cloud->points[j].z ;
//  //        Eigen::Vector3i index_grid_coordinates = sor.getGridCoordinates(x, y, z);
//  //
//  //        int centroid_index = sor.getCentroidIndexAt (index_grid_coordinates); //const Eigen::Vector3i &ijk
//  //
//  //        //cout << ground_image_test->points[i] << "  compared to : "<< leaf_layout[i] << endl;
//  //
//  //
//  //        //int   centroid_index = sor.getCentroidIndex (cloud->points[j]); //const PointT &p
//  //
//  //        // condition for defining the points in one voxel
//  //        if (centroid_index == leaf_layout[i])
//  //        {
//  //          voxel_indices.push_back(centroid_index);
//  //        }
//  //
//  //      }// for j
//  //    }//for i



//   // Create the filtering object
//    pcl::VoxelGridCovariance<PointT> sor ; //(new VoxelGrid<PointT>)
//    //sor.setInputCloud(ground_image_test);
//    sor.setInputCloud(cloud_filtered);
//    /*Then, a pcl::VoxelGrid filter is created with a leaf size of 5cm*/
//    //sor.setLeafSize(0.05f, 0.05f, 0.05f);  //grid cell size
//    //sor.setLeafSize (0.1f, 0.1f, 0.1f); //grid cell size
//    //sor.setLeafSize(0.2f, 0.2f, 0.2f);
//    //sor.setLeafSize(0.3f, 0.3f, 0.3f);
//    //sor.setLeafSize(0.5f, 0.5f, 0.5f);
//  //CloudPtr ground_image_test(new Cloud);
//
//    /*
//     *        Set the minimum number of points required for a cell to be used (must be 3 or greater for covariance calculation).
//     *      */
//    int min_points_per_voxel = 3;
//    sor.setMinPointPerVoxel (min_points_per_voxel);
//
//
//    /*
//     *        Set the minimum allowable ratio between eigenvalues to prevent singular covariance matrices.
//     *
//     */
//    double min_covar_eigvalue_mult = 2;
//    sor.setCovEigValueInflationRatio (min_covar_eigvalue_mult);
//
//    CloudPtr ground_image_test(new Cloud);
//
//    bool searchable=true;
//    sor.filter(*ground_image_test, searchable);
//    //sor.filter(*prev_ground_image_test);
//    //prev_ground_image_test = ground_image_test
//
//
//  pcl::PointCloud< PointXYZ >::Ptr ground_image_test_display (new   pcl::PointCloud< PointXYZ >);
//    sor.getDisplayCloud(*ground_image_test_display);
//    copyPointCloud(*ground_image_test_display, *ground_image_test);
//








//    /*Segment Obstacles using OrganizedConnectedComponentSegmentation and EuclideanClusterComparator*/
//    /*Segment Obstacles using OrganizedConnectedComponentSegmentation and EuclideanClusterComparator*/
//    /*Segment Obstacles using OrganizedConnectedComponentSegmentation and EuclideanClusterComparator*/
//    /*Segment Obstacles using OrganizedConnectedComponentSegmentation and EuclideanClusterComparator*/
//    if (ground_cloud->points.size())
//    {    }// if obstacle for euclidean + cc
//
//    pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_compare(new pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label> ());
//    euclidean_cluster_compare->setInputCloud(cloud);
//    euclidean_cluster_compare->setInputNormals(normal_cloud);
//
//    float distance_threshold = 0.1f;  //the tolerance in meters (at 1m)
//    bool depth_dependent = false;   //whether to scale the threshold based on range from the sensor (default: false)
//    euclidean_cluster_compare->setDistanceThreshold(distance_threshold, depth_dependent);
//
//    float angular_threshold = pcl::deg2rad(10.0f); //the tolerance in radians
//    euclidean_cluster_compare->setAngularThreshold(angular_threshold); //30 degree //kinematic and vehicle capability
//
//    boost::shared_ptr<pcl::PointCloud<pcl::Label>> labels_ptr(new pcl::PointCloud<pcl::Label>());
//    *labels_ptr = labels;
//    euclidean_cluster_compare->setLabels(labels_ptr);
//
//    std::vector<bool> plane_labels;
//    plane_labels.resize(region_indices.size(), false);
//    for (size_t i = 0; i < region_indices.size(); i++)
//    {
//      if (region_indices[i].indices.size() > 1000)
//      {
//        plane_labels[i] = true;
//      }//if
//    }// for
//    euclidean_cluster_compare->setExcludeLabels(plane_labels);
//
//
//    //pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> organized_segmentation(road_comparator);
//    //pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> organized_cc_segmentation(rgb_compare);
//    //pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> organized_segmentation(edge_aware_compare); //compiling but not working - assertion error!!
//    //pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> organized_cc_segmentation(euclidean_compare);
//    pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> organized_cc_segmentation(euclidean_cluster_compare);
//    //pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> organized_segmentation(refinement_compare); //compiling and wroking but no result!!!
//    organized_cc_segmentation.setInputCloud(cloud);
//    pcl::PointCloud<pcl::Label> labels_test;
//    std::vector<pcl::PointIndices> inlier_indices_test;
//    organized_cc_segmentation.segment(labels_test, inlier_indices_test);
//
//    /*! Draw the segmentation result testtttttt*/
//    //CloudPtr ground_image_test(new Cloud);
//    //*ground_image_test = *cloud;
//    vector<PointIndices> region_indices_test(inlier_indices_test); //region_indices
//
//    /*colorizing the point cloud*/
//    for (int i = 0; i < region_indices_test.size(); i++)
//    {
//      if (region_indices_test[i].indices.size() >= 1000)
//      {
//
//        //Cloud cluster;
//        //pcl::copyPointCloud(*ground_image_test, region_indices_test[i].indices, cluster);
//        //clusters.push_back(cluster);
//
//        /*! Compute plane info*/
//        Eigen::Vector4f cluster_centroid = Eigen::Vector4f::Zero();
//        Eigen::Matrix3f cluster_covariance;
//        pcl::computeMeanAndCovarianceMatrix(*ground_image, region_indices_test[i].indices, cluster_covariance, cluster_centroid);
//
//        EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
//        EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
//        pcl::eigen33(cluster_covariance, eigen_value, eigen_vector);
//        Eigen::Vector3f plane_params;
//        plane_params[0] = eigen_vector[0];
//        plane_params[1] = eigen_vector[1];
//        plane_params[2] = eigen_vector[2];
//        //plane_params[3] = 0;
//
//        //float cos_theta = abs(plane_params.dot(ground_plane_params)); //tilt_road_normal
//        Eigen::Vector3f nominal_road_normal(0.0, -1.0, 0.0); //-1 default  x,y,z --> (z is depth here)
//        Eigen::Vector3f tilt_road_normal = Eigen::AngleAxisf(pcl::deg2rad(5.0f), Eigen::Vector3f::UnitX()) * nominal_road_normal;
//        //road_comparator->setExpectedGroundNormal(tilt_road_normal);
//        float cos_theta = abs(plane_params.dot(tilt_road_normal)); //tilt_road_normal
//
//        //#define PI 3.14159265
//        //float   ground_angular_threshold = 20.0f;
//        //float thresold = cos (ground_angular_threshold * PI / 180.0 ); //cos(20.0); //degree the angular_threshold for ground  //0.1 for prependicular surfaces
//        //float thresold_obstacle = cos ( 45.0 ); // 2*20 + 5 //cos(20.0); //degree the angular_threshold for ground  //0.1 for prependicular surfaces
//        //float threshold_semi_travers = cos (20.0);
//        //float threshold_semi_travers = cos (angular_threshold * PI / 180.0);
//        float thresold_obstacle = cos(70.0);    // 2*20 + 5 //cos(20.0); //degree the angular_threshold for ground  //0.1 for prependicular surfaces
//        float threshold_trav = cos(20.0);    // 2*20 + 5 //cos(20.0); //degree the angular_threshold for ground  //0.1 for prependicular surfaces
//
//
//        pcl::PointXYZ centroid_pt(cluster_centroid[0], cluster_centroid[1], cluster_centroid[2]);
//        double ptp_dist =  pcl::pointToPlaneDistanceSigned(centroid_pt, ground_plane_params[0], ground_plane_params[1], ground_plane_params[2], ground_plane_params[3]);
//
//        for (int j = 0; j < region_indices_test[i].indices.size(); j++)
//        {
//
////          if ((ptp_dist <= distance_threshold) && (cos_theta > threshold_trav))  //traversable and dominant plane
////          {
////            if (label_image->points[region_indices_test[i].indices[j]].g == 255)  //region_indices_test[i].indices.size() >= 1000
////            {
////              //ground_image->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
////              ground_image_test->points[region_indices_test[i].indices[j]].g = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
////            }//if
////            else
////            {
////              ground_image_test->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].b + 255) / 2);
////            }
////          }//else ptp
//
//          if ((ptp_dist > distance_threshold) && (cos_theta < thresold_obstacle))  //obstacle
//          {
//            //ground_image_test->points[region_indices_test[i].indices[j]].r = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].r + 255) / 2);
//            ground_image->points[region_indices_test[i].indices[j]].r = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].r + 255) / 2);
//
//          }// if ptp
//        }//for j indices
//      }// if for min inliers to 1000
//    }// for i the mainn one for regions


//    /*RGB by mps seg*/
//    /*RGB by mps seg*/
//    /*RGB by mps seg*/
//    /*RGB by mps seg*/
//    int counter_mps_rgb = 0;
//    if (ground_cloud->points.size())
//    {
//      pcl::RGBPlaneCoefficientComparator<PointT, pcl::Normal>::Ptr rgb_compare(new pcl::RGBPlaneCoefficientComparator<PointT, pcl::Normal> ());
//      rgb_compare->setInputCloud(cloud);
//      rgb_compare->setInputNormals(normal_cloud);
//
//      /*Provide a pointer to a vector of the d-coefficient of the planes' hessian normal form.
//       * Provide a pointer to a vector of the d-coefficient of the planes' hessian normal form.
//      a, b, and c are provided by the normal cloud.*/
//      //std::vector< float >  plane_coeff_d;
//      //rgb_compare->setPlaneCoeffD(plane_coeff_d);
//
//      /*    Set the tolerance in radians for difference in normal direction between neighboring points, to be considered part of the same plane.*/
//      float angular_threshold = pcl::deg2rad(10.0f); //the tolerance in radians
//      rgb_compare->setAngularThreshold(angular_threshold); //30 degree //kinematic and vehicle cpability
//
//      /*step threashold and Set the tolerance in meters for difference in perpendicular distance (d component of plane equation) to the plane between neighboring points, to be considered part of the same plane.*/
//      float distance_threshold = 0.1f;  //the tolerance in meters (at 1m)
//      bool depth_dependent = false;   //whether to scale the threshold based on range from the sensor (default: false)
//      rgb_compare->setDistanceThreshold(distance_threshold, depth_dependent);
//
//      /*Set the tolerance in color space between neighboring points, to be considered part of the same plane.*/
//      float color_threshold = 10.0f; //50.0f;
//      rgb_compare->setColorThreshold(color_threshold); //50.0f default  //1, 10 and 50 tested already
//
//      /*organized multi plane segmentation demo using different comparators*/
//      OrganizedMultiPlaneSegmentation<PointT, Normal, Label> mps_test;
//      mps_test.setInputCloud(cloud);
//
//      /*Provide a pointer to the vector of indices that represents the input data.*/
//      //mps_test.setIndices(const IndicesPtr &  indices); //not needed since we have the input
//
//      mps_test.setInputNormals(normal_cloud);
//
//      /*Set the minimum number of inliers required for a plane.
//       * image is 800* 600  = width*height */
//      unsigned min_inliers = 800; //500 first time
//      mps_test.setMinInliers(min_inliers);
//
//      //float angular_threshold = pcl::deg2rad(10.0f); //the tolerance in radians
//      mps_test.setAngularThreshold(angular_threshold);
//
//      /*this sis only for detecting segmenting plane without using comparator*/
//      float distance_threshold_mps = 0.1f;  //the tolerance in meters (at 1m)
//      mps_test.setDistanceThreshold(distance_threshold_mps);
//
//      /*Set the maximum curvature allowed for a planar region.
//       * The tolerance for maximum curvature after fitting a plane.
//      Used to remove smooth, but non-planar regions.*/
//      double maximum_curvature = 0.01;
//      mps_test.setMaximumCurvature(maximum_curvature);  // a small curvature
//
//      /*Provide a pointer to the comparator to be used for refinement.*/
//      //mps_test.setRefinementComparator(refinement_compare);
//
//      /*Provide a pointer to the comparator to be used for segmentation. */
//      mps_test.setComparator(rgb_compare);
//      //mps_test.setComparator(edge_aware_compare);
//      //mps_test.setComparator(euclidean_compare);
//      //mps_test.setComparator(euclidean_cluster_comparator_); //not working //only for organized connected component
//
//      /*Set whether or not to project boundary points to the plane, or leave them in the original 3D space.*/
//      bool project_points = true;
//      mps_test.setProjectPoints(project_points);
//
//      // Use one of the overloaded segmentAndRefine calls to get all the information that we want out
//      // Segment and refine out all planes
//      PointCloud<Label>::Ptr labels_test(new PointCloud<Label>);
//      vector<PointIndices> label_indices_test;
//      vector<ModelCoefficients> model_coefficients_test;
//      vector<PointIndices> inlier_indices_test, boundary_indices;
//      vector<PlanarRegion<PointT>, Eigen::aligned_allocator<PlanarRegion<PointT>>> regions_test;
//
//      /*Perform a segmentation, as well as additional refinement step.  Returns intermediate data structures for use in subsequent processing.*/
//      mps_test.segmentAndRefine(regions_test, model_coefficients_test, inlier_indices_test, labels_test, label_indices_test, boundary_indices);
//
//      /*Segmentation of all planes in a point cloud given by setInputCloud(), setIndices() -- a vector of inliers for each detected plane*/
//      //mps_test.segment(model_coefficients_test, inlier_indices_test);
//
//
//      /*! Draw the segmentation result testtttttt*/
//      CloudPtr ground_image_test(new Cloud);
//      *ground_image_test = *cloud;
//      vector<PointIndices> region_indices_test(inlier_indices_test); //region_indices
//
//      //      /*colorizing the point cloud*/
//      //      for (int i = 0; i < region_indices_test.size(); i++)
//      //      {
//      //        if (region_indices_test[i].indices.size() >= 1000)
//      //        {
//      //          for (int j = 0; j < region_indices_test[i].indices.size(); j++)
//      //          {
//      //            if (label_image->points[region_indices_test[i].indices[j]].g == 255)  //region_indices_test[i].indices.size() >= 1000
//      //            {
//      //              //ground_image->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
//      //              ground_image_test->points[region_indices_test[i].indices[j]].g = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
//      //            }//if
//      //            else
//      //            {
//      //              ground_image_test->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
//      //            }
//      //          }//for
//      //        }//if
//      //      }// for
//
//      /*colorizing the point cloud*/
//      for (int i = 0; i < region_indices_test.size(); i++)
//      {
//        if (region_indices_test[i].indices.size() >= 1000)
//        {
//
//          //Cloud cluster;
//          //pcl::copyPointCloud(*ground_image_test, region_indices_test[i].indices, cluster);
//          //clusters.push_back(cluster);
//
//          /*! Compute plane info*/
//          Eigen::Vector4f cluster_centroid = Eigen::Vector4f::Zero();
//          Eigen::Matrix3f cluster_covariance;
//          pcl::computeMeanAndCovarianceMatrix(*ground_image_test, region_indices_test[i].indices, cluster_covariance, cluster_centroid);
//
//          EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
//          EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
//          pcl::eigen33(cluster_covariance, eigen_value, eigen_vector);
//          Eigen::Vector3f plane_params;
//          plane_params[0] = eigen_vector[0];
//          plane_params[1] = eigen_vector[1];
//          plane_params[2] = eigen_vector[2];
//          //plane_params[3] = 0;
//
//          //float cos_theta = abs(plane_params.dot(ground_plane_params)); //tilt_road_normal
//          Eigen::Vector3f nominal_road_normal(0.0, -1.0, 0.0); //-1 default  x,y,z --> (z is depth here)
//          Eigen::Vector3f tilt_road_normal = Eigen::AngleAxisf(pcl::deg2rad(5.0f), Eigen::Vector3f::UnitX()) * nominal_road_normal;
//          //road_comparator->setExpectedGroundNormal(tilt_road_normal);
//          float cos_theta = abs(plane_params.dot(tilt_road_normal)); //tilt_road_normal
//
//          //#define PI 3.14159265
//          //float   ground_angular_threshold = 20.0f;
//          //float thresold = cos (ground_angular_threshold * PI / 180.0 ); //cos(20.0); //degree the angular_threshold for ground  //0.1 for prependicular surfaces
//          //float thresold_obstacle = cos ( 45.0 ); // 2*20 + 5 //cos(20.0); //degree the angular_threshold for ground  //0.1 for prependicular surfaces
//          //float threshold_semi_travers = cos (20.0);
//          //float threshold_semi_travers = cos (angular_threshold * PI / 180.0);
//          float thresold_obstacle = cos(70.0);    // 2*20 + 5 //cos(20.0); //degree the angular_threshold for ground  //0.1 for prependicular surfaces
//          float threshold_trav = cos(20.0);    // 2*20 + 5 //cos(20.0); //degree the angular_threshold for ground  //0.1 for prependicular surfaces
//
//
//          pcl::PointXYZ centroid_pt(cluster_centroid[0], cluster_centroid[1], cluster_centroid[2]);
//          double ptp_dist =  pcl::pointToPlaneDistanceSigned(centroid_pt, ground_plane_params[0], ground_plane_params[1], ground_plane_params[2], ground_plane_params[3]);
//
//          for (int j = 0; j < region_indices_test[i].indices.size(); j++)
//          {
//
//            if ((ptp_dist <= distance_threshold) && (cos_theta > threshold_trav))  //traversable and dominant plane
//            {
//              if (label_image->points[region_indices_test[i].indices[j]].g == 255)  //region_indices_test[i].indices.size() >= 1000
//              {
//                //ground_image->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
//                ground_image_test->points[region_indices_test[i].indices[j]].g = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
//                counter_mps_rgb ++;
//              }//if
//              else
//              {
//                ground_image_test->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].b + 255) / 2);
//              }
//            }//else ptp
//
//            if ((ptp_dist > distance_threshold) && (cos_theta < thresold_obstacle))  //obstacle
//            {
//              ground_image_test->points[region_indices_test[i].indices[j]].r = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].r + 255) / 2);
//              //ground_image->points[region_indices_test[i].indices[j]].r = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].r + 255) / 2);
//
//            }// if ptp
//          }//for j indices
//        }// if for min inliers to 1000
//      }// for i the mainn one for regions
//
//      cout << "counter_mps_rgb: " << counter_mps_rgb << endl;
//    }// if RGB mps




//    /*rgb by cc seg*/
//    /*rgb by cc seg*/
//    /*rgb by cc seg*/
//    /*rgb by cc seg*/
//    int counter_cc_rgb = 0;
//    if (ground_cloud->points.size())
//    {
//      pcl::RGBPlaneCoefficientComparator<PointT, pcl::Normal>::Ptr rgb_compare(new pcl::RGBPlaneCoefficientComparator<PointT, pcl::Normal> ());
//      rgb_compare->setInputCloud(cloud);
//      rgb_compare->setInputNormals(normal_cloud);
//
//      /*Provide a pointer to a vector of the d-coefficient of the planes' hessian normal form.
//      * Provide a pointer to a vector of the d-coefficient of the planes' hessian normal form.
//      a, b, and c are provided by the normal cloud.*/
//      //std::vector< float >  plane_coeff_d;
//      //rgb_compare->setPlaneCoeffD(plane_coeff_d);
//
//      /*    Set the tolerance in radians for difference in normal direction between neighboring points, to be considered part of the same plane.*/
//      float angular_threshold = pcl::deg2rad(10.0f); //the tolerance in radians
//      rgb_compare->setAngularThreshold(angular_threshold); //30 degree //kinematic and vehicle cpability
//
//      /*step threashold and Set the tolerance in meters for difference in perpendicular distance (d component of plane equation) to the plane between neighboring points, to be considered part of the same plane.*/
//      float distance_threshold = 0.1f;  //the tolerance in meters (at 1m)
//      bool depth_dependent = false;   //whether to scale the threshold based on range from the sensor (default: false)
//      rgb_compare->setDistanceThreshold(distance_threshold, depth_dependent);
//
//      /*Set the tolerance in color space between neighboring points, to be considered part of the same plane.*/
//      float color_threshold = 10.0f; //50.0f;
//      rgb_compare->setColorThreshold(color_threshold); //50.0f default  //1, 10 and 50 tested already
//
//
//      //pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> organized_segmentation(road_comparator);
//      pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> organized_cc_segmentation(rgb_compare);
//      //pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> organized_segmentation(edge_aware_compare); //compiling but not working - assertion error!!
//      //pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> organized_segmentation(euclidean_compare);
//      //pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> organized_segmentation(refinement_compare); //compiling and wroking but no result!!!
//      organized_cc_segmentation.setInputCloud(cloud);
//      pcl::PointCloud<pcl::Label> labels_test;
//      std::vector<pcl::PointIndices> inlier_indices_test;
//      organized_cc_segmentation.segment(labels_test, inlier_indices_test);
//
//      /*! Draw the segmentation result testtttttt*/
//      CloudPtr ground_image_test(new Cloud);
//      *ground_image_test = *cloud;
//      vector<PointIndices> region_indices_test(inlier_indices_test); //region_indices
//
//      //      /*colorizing the point cloud*/
//      //      for (int i = 0; i < region_indices_test.size(); i++)
//      //      {
//      //        if (region_indices_test[i].indices.size() >= 1000)
//      //        {
//      //          for (int j = 0; j < region_indices_test[i].indices.size(); j++)
//      //          {
//      //            if (label_image->points[region_indices_test[i].indices[j]].g == 255)  //region_indices_test[i].indices.size() >= 1000
//      //            {
//      //              //ground_image->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
//      //              ground_image_test->points[region_indices_test[i].indices[j]].g = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
//      //            }//if
//      //            else
//      //            {
//      //              ground_image_test->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
//      //            }
//      //          }//for
//      //        }//if
//      //      }   // for
//
//      /*colorizing the point cloud*/
//      for (int i = 0; i < region_indices_test.size(); i++)
//      {
//        if (region_indices_test[i].indices.size() >= 1000)
//        {
//
//          //Cloud cluster;
//          //pcl::copyPointCloud(*ground_image_test, region_indices_test[i].indices, cluster);
//          //clusters.push_back(cluster);
//
//          /*! Compute plane info*/
//          Eigen::Vector4f cluster_centroid = Eigen::Vector4f::Zero();
//          Eigen::Matrix3f cluster_covariance;
//          pcl::computeMeanAndCovarianceMatrix(*ground_image_test, region_indices_test[i].indices, cluster_covariance, cluster_centroid);
//
//          EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
//          EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
//          pcl::eigen33(cluster_covariance, eigen_value, eigen_vector);
//          Eigen::Vector3f plane_params;
//          plane_params[0] = eigen_vector[0];
//          plane_params[1] = eigen_vector[1];
//          plane_params[2] = eigen_vector[2];
//          //plane_params[3] = 0;
//
//          //float cos_theta = abs(plane_params.dot(ground_plane_params)); //tilt_road_normal
//          Eigen::Vector3f nominal_road_normal(0.0, -1.0, 0.0); //-1 default  x,y,z --> (z is depth here)
//          Eigen::Vector3f tilt_road_normal = Eigen::AngleAxisf(pcl::deg2rad(5.0f), Eigen::Vector3f::UnitX()) * nominal_road_normal;
//          //road_comparator->setExpectedGroundNormal(tilt_road_normal);
//          float cos_theta = abs(plane_params.dot(tilt_road_normal)); //tilt_road_normal
//
//          //#define PI 3.14159265
//          //float   ground_angular_threshold = 20.0f;
//          //float thresold = cos (ground_angular_threshold * PI / 180.0 ); //cos(20.0); //degree the angular_threshold for ground  //0.1 for prependicular surfaces
//          //float thresold_obstacle = cos ( 45.0 ); // 2*20 + 5 //cos(20.0); //degree the angular_threshold for ground  //0.1 for prependicular surfaces
//          //float threshold_semi_travers = cos (20.0);
//          //float threshold_semi_travers = cos (angular_threshold * PI / 180.0);
//          float thresold_obstacle = cos(70.0);    // 2*20 + 5 //cos(20.0); //degree the angular_threshold for ground  //0.1 for prependicular surfaces
//          float threshold_trav = cos(20.0);    // 2*20 + 5 //cos(20.0); //degree the angular_threshold for ground  //0.1 for prependicular surfaces
//
//
//          pcl::PointXYZ centroid_pt(cluster_centroid[0], cluster_centroid[1], cluster_centroid[2]);
//          double ptp_dist =  pcl::pointToPlaneDistanceSigned(centroid_pt, ground_plane_params[0], ground_plane_params[1], ground_plane_params[2], ground_plane_params[3]);
//
//          for (int j = 0; j < region_indices_test[i].indices.size(); j++)
//          {
//
//            if ((ptp_dist <= distance_threshold) && (cos_theta > threshold_trav))  //traversable and dominant plane
//            {
//              if (label_image->points[region_indices_test[i].indices[j]].g == 255)  //region_indices_test[i].indices.size() >= 1000
//              {
//                //ground_image->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
//                ground_image_test->points[region_indices_test[i].indices[j]].g = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
//                counter_cc_rgb ++;
//              }//if
//              else
//              {
//                ground_image_test->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].b + 255) / 2);
//              }
//            }//else ptp
//
//            if ((ptp_dist > distance_threshold) && (cos_theta < thresold_obstacle))  //obstacle
//            {
//              ground_image_test->points[region_indices_test[i].indices[j]].r = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].r + 255) / 2);
//              //ground_image->points[region_indices_test[i].indices[j]].r = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].r + 255) / 2);
//
//            }// if ptp
//          }//for j indices
//        }// if for min inliers to 1000
//      }// for i the mainn one for regions
//
//      cout << "counter_cc_rgb: " << counter_cc_rgb << endl;
//    } // rgb for cc seg






//    /*edge by mps*/
//    /*edge by mps*/
//    /*edge by mps*/
//    /*edge by mps*/
//    /*edge by mps*/
//    int counter_cc_edge = 0;
//    if (ground_cloud->points.size() > 0)
//    {
//      EdgeAwarePlaneComparator<PointT, Normal>::Ptr edge_aware_compare(new EdgeAwarePlaneComparator<PointT, Normal>());
//      edge_aware_compare->setInputCloud(cloud);
//      edge_aware_compare->setInputNormals(normal_cloud);
//
//      /*Provide a pointer to a vector of the d-coefficient of the planes' hessian normal form.
//       * Provide a pointer to a vector of the d-coefficient of the planes' hessian normal form.
//      a, b, and c are provided by the normal cloud.*/
//      //std::vector< float >  plane_coeff_d;
//      //edge_aware_compare->setPlaneCoeffD(plane_coeff_d);
//
//      /*    Set the tolerance in radians for difference in normal direction between neighboring points, to be considered part of the same plane.*/
//      float angular_threshold = pcl::deg2rad(10.0f); //the tolerance in radians
//      edge_aware_compare->setAngularThreshold(angular_threshold); //30 degree //kinematic and vehicle cpability
//
//      /*step threashold and Set the tolerance in meters for difference in perpendicular distance (d component of plane equation) to the plane between neighboring points, to be considered part of the same plane.*/
//      float distance_threshold = 0.1f;  //the tolerance in meters (at 1m)
//      bool depth_dependent = false;   //whether to scale the threshold based on range from the sensor (default: false)
//      edge_aware_compare->setDistanceThreshold(distance_threshold, depth_dependent);
//
//      /*Set a distance map to use. For an example of a valid distance map see OrganizedIntegralImageNormalEstimation*/
//      //std::vector<float> distance_map_; //    // Save the distance map for the plane comparator
//      //float *map = ne.getDistanceMap(); // This will be deallocated with the IntegralImageNormalEstimation object...
//      //distance_map_.assign(map, map + cloud->size()); //...so we must copy the data out
//      //edge_aware_compare->setDistanceMap(distance_map_.data());
//      float *distance_map = ne.getDistanceMap(); // This will be deallocated with the IntegralImageNormalEstimation object...
//      edge_aware_compare->setDistanceMap(distance_map);
//
//      /*Set the curvature threshold for creating a new segment.*/
//      float curvature_threshold = 10;
//      edge_aware_compare->setCurvatureThreshold(curvature_threshold);
//
//      /*Set the distance map threshold – the number of pixel away from a border / nan.
//       * this depends on the FOV of the common area in pixels and meter so that we provide a safty distance for robot as pix/meter
//       * if in this time is 800*600 and based on FOV of the common area it is 500 pixels/10meter =  50 pix/m  ==>
//       * 50 cm each side of robot is good which is 25 pixel on a border ~ 30 pixel*/
//      float distance_map_threshold = 30.0f;
//      edge_aware_compare->setDistanceMapThreshold(distance_map_threshold);
//
//      /*euclidean_distance_threshold  the euclidean distance threshold in meters
//       * this is based the kinmetic, wheel measurement and the robot size
//       * for ravon 10 cm is good*/
//      float euclidean_distance_threshold = 0.1f;
//      edge_aware_compare->setEuclideanDistanceThreshold(euclidean_distance_threshold);
//
//
//      /*organized multi plane segmentation demo using different comparators*/
//      OrganizedMultiPlaneSegmentation<PointT, Normal, Label> mps_test;
//      mps_test.setInputCloud(cloud);
//
//      /*Provide a pointer to the vector of indices that represents the input data.*/
//      //mps_test.setIndices(const IndicesPtr &  indices); //not needed since we have the input
//
//      mps_test.setInputNormals(normal_cloud);
//
//      /*Set the minimum number of inliers required for a plane.
//       * image is 800* 600  = width*height */
//      unsigned min_inliers = 800; //500 first time
//      mps_test.setMinInliers(min_inliers);
//
//      //float angular_threshold = pcl::deg2rad(10.0f); //the tolerance in radians
//      mps_test.setAngularThreshold(angular_threshold);
//
//      /*this sis only for detecting segmenting plane without using comparator*/
//      float distance_threshold_mps = 0.1f;  //the tolerance in meters (at 1m)
//      mps_test.setDistanceThreshold(distance_threshold_mps);
//
//      /*Set the maximum curvature allowed for a planar region.
//       * The tolerance for maximum curvature after fitting a plane.
//      Used to remove smooth, but non-planar regions.*/
//      double maximum_curvature = 0.01;
//      mps_test.setMaximumCurvature(maximum_curvature);  // a small curvature
//
//      /*Provide a pointer to the comparator to be used for refinement.*/
//      //mps_test.setRefinementComparator(refinement_compare);
//
//      /*Provide a pointer to the comparator to be used for segmentation. */
//      //mps_test.setComparator(rgb_compare);
//      mps_test.setComparator(edge_aware_compare);
//      //mps_test.setComparator(euclidean_compare);
//      //mps_test.setComparator(euclidean_cluster_comparator_); //not working //only for organized connected component
//
//      /*Set whether or not to project boundary points to the plane, or leave them in the original 3D space.*/
//      bool project_points = true;
//      mps_test.setProjectPoints(project_points);
//
//      // Use one of the overloaded segmentAndRefine calls to get all the information that we want out
//      // Segment and refine out all planes
//      PointCloud<Label>::Ptr labels_test(new PointCloud<Label>);
//      vector<PointIndices> label_indices_test;
//      vector<ModelCoefficients> model_coefficients_test;
//      vector<PointIndices> inlier_indices_test, boundary_indices;
//      vector<PlanarRegion<PointT>, Eigen::aligned_allocator<PlanarRegion<PointT>>> regions_test;
//
//      /*Perform a segmentation, as well as additional refinement step.  Returns intermediate data structures for use in subsequent processing.*/
//      mps_test.segmentAndRefine(regions_test, model_coefficients_test, inlier_indices_test, labels_test, label_indices_test, boundary_indices);
//
//      /*Segmentation of all planes in a point cloud given by setInputCloud(), setIndices() -- a vector of inliers for each detected plane*/
//      //mps_test.segment(model_coefficients_test, inlier_indices_test);
//
//
//      /*! Draw the segmentation result testtttttt*/
//      CloudPtr ground_image_test(new Cloud);
//      *ground_image_test = *cloud;
//      vector<PointIndices> region_indices_test(inlier_indices_test); //region_indices
//
//      //      /*colorizing the point cloud*/
//      //      for (int i = 0; i < region_indices_test.size(); i++)
//      //      {
//      //        if (region_indices_test[i].indices.size() >= 1000)
//      //        {
//      //          for (int j = 0; j < region_indices_test[i].indices.size(); j++)
//      //          {
//      //            if (label_image->points[region_indices_test[i].indices[j]].g == 255)  //region_indices_test[i].indices.size() >= 1000
//      //            {
//      //              //ground_image->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
//      //              ground_image_test->points[region_indices_test[i].indices[j]].g = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
//      //            }//if
//      //            else
//      //            {
//      //              ground_image_test->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
//      //            }
//      //          }//for
//      //        }//if
//      //      }// for
//
//      /*colorizing the point cloud*/
//      for (int i = 0; i < region_indices_test.size(); i++)
//      {
//        if (region_indices_test[i].indices.size() >= 1000)
//        {
//
//          //Cloud cluster;
//          //pcl::copyPointCloud(*ground_image_test, region_indices_test[i].indices, cluster);
//          //clusters.push_back(cluster);
//
//          /*! Compute plane info*/
//          Eigen::Vector4f cluster_centroid = Eigen::Vector4f::Zero();
//          Eigen::Matrix3f cluster_covariance;
//          pcl::computeMeanAndCovarianceMatrix(*ground_image_test, region_indices_test[i].indices, cluster_covariance, cluster_centroid);
//
//          EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
//          EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
//          pcl::eigen33(cluster_covariance, eigen_value, eigen_vector);
//          Eigen::Vector3f plane_params;
//          plane_params[0] = eigen_vector[0];
//          plane_params[1] = eigen_vector[1];
//          plane_params[2] = eigen_vector[2];
//          //plane_params[3] = 0;
//
//          //float cos_theta = abs(plane_params.dot(ground_plane_params)); //tilt_road_normal
//          Eigen::Vector3f nominal_road_normal(0.0, -1.0, 0.0); //-1 default  x,y,z --> (z is depth here)
//          Eigen::Vector3f tilt_road_normal = Eigen::AngleAxisf(pcl::deg2rad(5.0f), Eigen::Vector3f::UnitX()) * nominal_road_normal;
//          //road_comparator->setExpectedGroundNormal(tilt_road_normal);
//          float cos_theta = abs(plane_params.dot(tilt_road_normal)); //tilt_road_normal
//
//          //#define PI 3.14159265
//          //float   ground_angular_threshold = 20.0f;
//          //float thresold = cos (ground_angular_threshold * PI / 180.0 ); //cos(20.0); //degree the angular_threshold for ground  //0.1 for prependicular surfaces
//          //float thresold_obstacle = cos ( 45.0 ); // 2*20 + 5 //cos(20.0); //degree the angular_threshold for ground  //0.1 for prependicular surfaces
//          //float threshold_semi_travers = cos (20.0);
//          //float threshold_semi_travers = cos (angular_threshold * PI / 180.0);
//          float thresold_obstacle = cos(70.0);    // 2*20 + 5 //cos(20.0); //degree the angular_threshold for ground  //0.1 for prependicular surfaces
//          float threshold_trav = cos(20.0);    // 2*20 + 5 //cos(20.0); //degree the angular_threshold for ground  //0.1 for prependicular surfaces
//
//
//          pcl::PointXYZ centroid_pt(cluster_centroid[0], cluster_centroid[1], cluster_centroid[2]);
//          double ptp_dist =  pcl::pointToPlaneDistanceSigned(centroid_pt, ground_plane_params[0], ground_plane_params[1], ground_plane_params[2], ground_plane_params[3]);
//
//          for (int j = 0; j < region_indices_test[i].indices.size(); j++)
//          {
//
//            if ((ptp_dist <= distance_threshold) && (cos_theta > threshold_trav))  //traversable and dominant plane
//            {
//              if (label_image->points[region_indices_test[i].indices[j]].g == 255)  //region_indices_test[i].indices.size() >= 1000
//              {
//                //ground_image->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
//                ground_image_test->points[region_indices_test[i].indices[j]].g = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
//                counter_cc_edge ++;
//              }//if
//              else
//              {
//                ground_image_test->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].b + 255) / 2);
//              }
//            }//else ptp
//
//            if ((ptp_dist > distance_threshold) && (cos_theta < thresold_obstacle))  //obstacle
//            {
//              ground_image_test->points[region_indices_test[i].indices[j]].r = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].r + 255) / 2);
//              //ground_image->points[region_indices_test[i].indices[j]].r = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].r + 255) / 2);
//
//            }// if ptp
//          }//for j indices
//        }// if for min inliers to 1000
//      }// for i the mainn one for regions
//
//      counter_cc_edge ++;
//      cout << "counter_cc_edge: " << counter_cc_edge << endl;
//    }// if edge by mps



//    /*euclidean by mps*/
//    /*euclidean by mps*/
//    /*euclidean by mps*/
//    /*euclidean by mps*/
//    int counter_mps_euclidean = 0;
//    if (ground_cloud->points.size() > 0)
//    {
//      pcl::EuclideanPlaneCoefficientComparator<PointT, pcl::Normal>::Ptr euclidean_compare(new pcl::EuclideanPlaneCoefficientComparator<PointT, pcl::Normal> ());
//      euclidean_compare->setInputCloud(cloud);
//      euclidean_compare->setInputNormals(normal_cloud);
//
//      /*    Set the tolerance in radians for difference in normal direction between neighboring points, to be considered part of the same plane.*/
//      float angular_threshold = pcl::deg2rad(10.0f); //the tolerance in radians
//      euclidean_compare->setAngularThreshold(angular_threshold); //30 degree //kinematic and vehicle cpability
//
//      /*step threashold and Set the tolerance in meters for difference in perpendicular distance (d component of plane equation) to the plane between neighboring points, to be considered part of the same plane.*/
//      float distance_threshold = 0.1f;  //the tolerance in meters (at 1m)
//      bool depth_dependent = false;   //whether to scale the threshold based on range from the sensor (default: false)
//      euclidean_compare->setDistanceThreshold(distance_threshold, depth_dependent);
//
//      /*organized multi plane segmentation demo using different comparators*/
//      OrganizedMultiPlaneSegmentation<PointT, Normal, Label> mps_test;
//      mps_test.setInputCloud(cloud);
//
//      /*Provide a pointer to the vector of indices that represents the input data.*/
//      //mps_test.setIndices(const IndicesPtr &  indices); //not needed since we have the input
//
//      mps_test.setInputNormals(normal_cloud);
//
//      /*Set the minimum number of inliers required for a plane.
//       * image is 800* 600  = width*height */
//      unsigned min_inliers = 800; //500 first time
//      mps_test.setMinInliers(min_inliers);
//
//      //float angular_threshold = pcl::deg2rad(10.0f); //the tolerance in radians
//      mps_test.setAngularThreshold(angular_threshold);
//
//      /*this sis only for detecting segmenting plane without using comparator*/
//      float distance_threshold_mps = 0.1f;  //the tolerance in meters (at 1m)
//      mps_test.setDistanceThreshold(distance_threshold_mps);
//
//      /*Set the maximum curvature allowed for a planar region.
//       * The tolerance for maximum curvature after fitting a plane.
//      Used to remove smooth, but non-planar regions.*/
//      double maximum_curvature = 0.01;
//      mps_test.setMaximumCurvature(maximum_curvature);  // a small curvature
//
//      /*Provide a pointer to the comparator to be used for refinement.*/
//      //mps_test.setRefinementComparator(refinement_compare);
//
//      /*Provide a pointer to the comparator to be used for segmentation. */
//      //mps_test.setComparator(rgb_compare);
//      //mps_test.setComparator(edge_aware_compare);
//      mps_test.setComparator(euclidean_compare);
//      //mps_test.setComparator(euclidean_cluster_comparator_); //not working //only for organized connected component
//
//      /*Set whether or not to project boundary points to the plane, or leave them in the original 3D space.*/
//      bool project_points = true;
//      mps_test.setProjectPoints(project_points);
//
//      // Use one of the overloaded segmentAndRefine calls to get all the information that we want out
//      // Segment and refine out all planes
//      PointCloud<Label>::Ptr labels_test(new PointCloud<Label>);
//      vector<PointIndices> label_indices_test;
//      vector<ModelCoefficients> model_coefficients_test;
//      vector<PointIndices> inlier_indices_test, boundary_indices;
//      vector<PlanarRegion<PointT>, Eigen::aligned_allocator<PlanarRegion<PointT>>> regions_test;
//
//      /*Perform a segmentation, as well as additional refinement step.  Returns intermediate data structures for use in subsequent processing.*/
//      mps_test.segmentAndRefine(regions_test, model_coefficients_test, inlier_indices_test, labels_test, label_indices_test, boundary_indices);
//
//      /*Segmentation of all planes in a point cloud given by setInputCloud(), setIndices() -- a vector of inliers for each detected plane*/
//      //mps_test.segment(model_coefficients_test, inlier_indices_test);
//
//
//      /*! Draw the segmentation result testtttttt*/
//      CloudPtr ground_image_test(new Cloud);
//      *ground_image_test = *cloud;
//      vector<PointIndices> region_indices_test(inlier_indices_test); //region_indices
//
//      //      /*colorizing the point cloud*/
//      //      for (int i = 0; i < region_indices_test.size(); i++)
//      //      {
//      //        if (region_indices_test[i].indices.size() >= 1000)
//      //        {
//      //          for (int j = 0; j < region_indices_test[i].indices.size(); j++)
//      //          {
//      //            if (label_image->points[region_indices_test[i].indices[j]].g == 255)  //region_indices_test[i].indices.size() >= 1000
//      //            {
//      //              //ground_image->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
//      //              ground_image_test->points[region_indices_test[i].indices[j]].g = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
//      //            }//if
//      //            else
//      //            {
//      //              ground_image_test->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
//      //            }
//      //          }//for
//      //        }//if
//      //      }// for
//
//      /*colorizing the point cloud*/
//      for (int i = 0; i < region_indices_test.size(); i++)
//      {
//        if (region_indices_test[i].indices.size() >= 1000)
//        {
//
//          //Cloud cluster;
//          //pcl::copyPointCloud(*ground_image_test, region_indices_test[i].indices, cluster);
//          //clusters.push_back(cluster);
//
//          /*! Compute plane info*/
//          Eigen::Vector4f cluster_centroid = Eigen::Vector4f::Zero();
//          Eigen::Matrix3f cluster_covariance;
//          pcl::computeMeanAndCovarianceMatrix(*ground_image_test, region_indices_test[i].indices, cluster_covariance, cluster_centroid);
//
//          EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
//          EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
//          pcl::eigen33(cluster_covariance, eigen_value, eigen_vector);
//          Eigen::Vector3f plane_params;
//          plane_params[0] = eigen_vector[0];
//          plane_params[1] = eigen_vector[1];
//          plane_params[2] = eigen_vector[2];
//          //plane_params[3] = 0;
//
//          //float cos_theta = abs(plane_params.dot(ground_plane_params)); //tilt_road_normal
//          Eigen::Vector3f nominal_road_normal(0.0, -1.0, 0.0); //-1 default  x,y,z --> (z is depth here)
//          Eigen::Vector3f tilt_road_normal = Eigen::AngleAxisf(pcl::deg2rad(5.0f), Eigen::Vector3f::UnitX()) * nominal_road_normal;
//          //road_comparator->setExpectedGroundNormal(tilt_road_normal);
//          float cos_theta = abs(plane_params.dot(tilt_road_normal)); //tilt_road_normal
//
//          //#define PI 3.14159265
//          //float   ground_angular_threshold = 20.0f;
//          //float thresold = cos (ground_angular_threshold * PI / 180.0 ); //cos(20.0); //degree the angular_threshold for ground  //0.1 for prependicular surfaces
//          //float thresold_obstacle = cos ( 45.0 ); // 2*20 + 5 //cos(20.0); //degree the angular_threshold for ground  //0.1 for prependicular surfaces
//          //float threshold_semi_travers = cos (20.0);
//          //float threshold_semi_travers = cos (angular_threshold * PI / 180.0);
//          float thresold_obstacle = cos(70.0);    // 2*20 + 5 //cos(20.0); //degree the angular_threshold for ground  //0.1 for prependicular surfaces
//          float threshold_trav = cos(20.0);    // 2*20 + 5 //cos(20.0); //degree the angular_threshold for ground  //0.1 for prependicular surfaces
//
//
//          pcl::PointXYZ centroid_pt(cluster_centroid[0], cluster_centroid[1], cluster_centroid[2]);
//          double ptp_dist =  pcl::pointToPlaneDistanceSigned(centroid_pt, ground_plane_params[0], ground_plane_params[1], ground_plane_params[2], ground_plane_params[3]);
//
//          for (int j = 0; j < region_indices_test[i].indices.size(); j++)
//          {
//
//            if ((ptp_dist <= distance_threshold) && (cos_theta > threshold_trav))  //traversable and dominant plane
//            {
//              if (label_image->points[region_indices_test[i].indices[j]].g == 255)  //region_indices_test[i].indices.size() >= 1000
//              {
//                //ground_image->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
//                ground_image_test->points[region_indices_test[i].indices[j]].g = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
//                counter_mps_euclidean ++;
//              }//if
//              else
//              {
//                ground_image_test->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].b + 255) / 2);
//              }
//            }//else ptp
//
//            if ((ptp_dist > distance_threshold) && (cos_theta < thresold_obstacle))  //obstacle
//            {
//              ground_image_test->points[region_indices_test[i].indices[j]].r = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].r + 255) / 2);
//              //ground_image->points[region_indices_test[i].indices[j]].r = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].r + 255) / 2);
//
//            }// if ptp
//          }//for j indices
//        }// if for min inliers to 1000
//      }// for i the mainn one for regions
//
//      cout << "counter_mps_euclidean: " << counter_mps_euclidean << endl;
//
//    }// euclidean by mps





//    /*euclidean by cc seg*/
//    /*euclidean by cc seg*/
//    /*euclidean by cc seg*/
//    /*euclidean by cc seg*/
//    /*euclidean by cc seg*/
//    int counter_cc_euclidean = 0;
//    if (ground_cloud->points.size() > 0)
//    {
//      pcl::EuclideanPlaneCoefficientComparator<PointT, pcl::Normal>::Ptr euclidean_compare(new pcl::EuclideanPlaneCoefficientComparator<PointT, pcl::Normal> ());
//      euclidean_compare->setInputCloud(cloud);
//      euclidean_compare->setInputNormals(normal_cloud);
//
//      /*    Set the tolerance in radians for difference in normal direction between neighboring points, to be considered part of the same plane.*/
//      float angular_threshold = pcl::deg2rad(10.0f); //the tolerance in radians
//      euclidean_compare->setAngularThreshold(angular_threshold); //30 degree //kinematic and vehicle cpability
//
//      /*step threashold and Set the tolerance in meters for difference in perpendicular distance (d component of plane equation) to the plane between neighboring points, to be considered part of the same plane.*/
//      float distance_threshold = 0.1f;  //the tolerance in meters (at 1m)
//      bool depth_dependent = false;   //whether to scale the threshold based on range from the sensor (default: false)
//      euclidean_compare->setDistanceThreshold(distance_threshold, depth_dependent);
//
//
//      //pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> organized_segmentation(road_comparator);
//      //pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> organized_cc_segmentation(rgb_compare);
//      //pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> organized_segmentation(edge_aware_compare); //compiling but not working - assertion error!!
//      pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> organized_cc_segmentation(euclidean_compare);
//      //pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> organized_segmentation(refinement_compare); //compiling and wroking but no result!!!
//      organized_cc_segmentation.setInputCloud(cloud);
//      pcl::PointCloud<pcl::Label> labels_test;
//      std::vector<pcl::PointIndices> inlier_indices_test;
//      organized_cc_segmentation.segment(labels_test, inlier_indices_test);
//
//      /*! Draw the segmentation result testtttttt*/
//      CloudPtr ground_image_test(new Cloud);
//      *ground_image_test = *cloud;
//      vector<PointIndices> region_indices_test(inlier_indices_test); //region_indices
//
//      //      /*colorizing the point cloud*/
//      //      for (int i = 0; i < region_indices_test.size(); i++)
//      //      {
//      //        if (region_indices_test[i].indices.size() >= 1000)
//      //        {
//      //          for (int j = 0; j < region_indices_test[i].indices.size(); j++)
//      //          {
//      //            if (label_image->points[region_indices_test[i].indices[j]].g == 255)  //region_indices_test[i].indices.size() >= 1000
//      //            {
//      //              //ground_image->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
//      //              ground_image_test->points[region_indices_test[i].indices[j]].g = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
//      //            }//if
//      //            else
//      //            {
//      //              ground_image_test->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
//      //            }
//      //          }//for
//      //        }//if
//      //      }   // for
//
//      /*colorizing the point cloud*/
//      for (int i = 0; i < region_indices_test.size(); i++)
//      {
//        if (region_indices_test[i].indices.size() >= 1000)
//        {
//
//          //Cloud cluster;
//          //pcl::copyPointCloud(*ground_image_test, region_indices_test[i].indices, cluster);
//          //clusters.push_back(cluster);
//
//          /*! Compute plane info*/
//          Eigen::Vector4f cluster_centroid = Eigen::Vector4f::Zero();
//          Eigen::Matrix3f cluster_covariance;
//          pcl::computeMeanAndCovarianceMatrix(*ground_image_test, region_indices_test[i].indices, cluster_covariance, cluster_centroid);
//
//          EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
//          EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
//          pcl::eigen33(cluster_covariance, eigen_value, eigen_vector);
//          Eigen::Vector3f plane_params;
//          plane_params[0] = eigen_vector[0];
//          plane_params[1] = eigen_vector[1];
//          plane_params[2] = eigen_vector[2];
//          //plane_params[3] = 0;
//
//          //float cos_theta = abs(plane_params.dot(ground_plane_params)); //tilt_road_normal
//          Eigen::Vector3f nominal_road_normal(0.0, -1.0, 0.0); //-1 default  x,y,z --> (z is depth here)
//          Eigen::Vector3f tilt_road_normal = Eigen::AngleAxisf(pcl::deg2rad(5.0f), Eigen::Vector3f::UnitX()) * nominal_road_normal;
//          //road_comparator->setExpectedGroundNormal(tilt_road_normal);
//          float cos_theta = abs(plane_params.dot(tilt_road_normal)); //tilt_road_normal
//
//          //#define PI 3.14159265
//          //float   ground_angular_threshold = 20.0f;
//          //float thresold = cos (ground_angular_threshold * PI / 180.0 ); //cos(20.0); //degree the angular_threshold for ground  //0.1 for prependicular surfaces
//          //float thresold_obstacle = cos ( 45.0 ); // 2*20 + 5 //cos(20.0); //degree the angular_threshold for ground  //0.1 for prependicular surfaces
//          //float threshold_semi_travers = cos (20.0);
//          //float threshold_semi_travers = cos (angular_threshold * PI / 180.0);
//          float thresold_obstacle = cos(70.0);    // 2*20 + 5 //cos(20.0); //degree the angular_threshold for ground  //0.1 for prependicular surfaces
//          float threshold_trav = cos(20.0);    // 2*20 + 5 //cos(20.0); //degree the angular_threshold for ground  //0.1 for prependicular surfaces
//
//
//          pcl::PointXYZ centroid_pt(cluster_centroid[0], cluster_centroid[1], cluster_centroid[2]);
//          double ptp_dist =  pcl::pointToPlaneDistanceSigned(centroid_pt, ground_plane_params[0], ground_plane_params[1], ground_plane_params[2], ground_plane_params[3]);
//
//          for (int j = 0; j < region_indices_test[i].indices.size(); j++)
//          {
//
//            if ((ptp_dist <= distance_threshold) && (cos_theta > threshold_trav))  //traversable and dominant plane
//            {
//              if (label_image->points[region_indices_test[i].indices[j]].g == 255)  //region_indices_test[i].indices.size() >= 1000
//              {
//                //ground_image->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
//                ground_image_test->points[region_indices_test[i].indices[j]].g = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
//                counter_cc_euclidean++;
//              }//if
//              else
//              {
//                ground_image_test->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].b + 255) / 2);
//              }
//            }//else ptp
//
//            if ((ptp_dist > distance_threshold) && (cos_theta < thresold_obstacle))  //obstacle
//            {
//              ground_image_test->points[region_indices_test[i].indices[j]].r = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].r + 255) / 2);
//              //ground_image->points[region_indices_test[i].indices[j]].r = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].r + 255) / 2);
//
//            }// if ptp
//          }//for j indices
//        }// if for min inliers to 1000
//      }// for i the mainn one for regions
//
//      cout << "counter_cc_euclidean: " << counter_cc_euclidean << endl;
//    }// euclidean by cc seg
















//  /* region growing based on rgb compiled but not working*/
//  /* region growing based on rgb*/
//  /* region growing based on rgb*/
//  pcl::IndicesPtr indices_test(new std::vector <int>);
//  pcl::PassThrough<PointT> pass;
//  pass.setInputCloud(cloud);
////  pass.setFilterFieldName("z");
////  pass.setFilterLimits(-10.0, 100.0);
//  pass.setFilterFieldName("y");
//  pass.setFilterLimits(5, 15);
//  pass.filter(*indices_test);
//
//  pcl::RegionGrowingRGB<PointT> reg;
//  pcl::search::Search<PointT>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointT> > (new pcl::search::KdTree<PointT>);
//
//  reg.setInputCloud(cloud);
//  reg.setIndices(indices_test);
//  reg.setSearchMethod(tree);
//  reg.setDistanceThreshold(10);
//  reg.setPointColorThreshold(6);
//  reg.setRegionColorThreshold(5);
//  reg.setMinClusterSize(600);
//
//  std::vector <pcl::PointIndices> clusters;
//  reg.extract(clusters);
//
//  //pcl::PointCloud <PointT>::Ptr colored_cloud = reg.getColoredCloud();
//
//
//    /*! Draw the segmentation result region growing seg*/
//    CloudPtr ground_image_test(new Cloud);
//    *ground_image_test = *cloud;
//    vector<PointIndices> region_indices_test(clusters); //region_indices
//
//    //Note the regions that have been extended
//    for (int i = 0; i < region_indices_test.size(); i++)
//    {
//      if (region_indices_test[i].indices.size() >= 10) //almost all of them
//      {
//        for (int j = 0; j < region_indices_test[i].indices.size(); j++)
//        {
//
//          ground_image_test->points[region_indices_test[i].indices[j]].g = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
//
//        }
//      }
//    }







//    // Create the filtering object
//    //pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//    pcl::StatisticalOutlierRemoval<PointT> sor;
//    sor.setInputCloud(ground_image_test);
//    sor.setMeanK(50);
//    sor.setStddevMulThresh(1.0);
//    //CloudPtr ground_image_test (new Cloud);
//    sor.filter(*ground_image_test);


//    std::string default_method = "radius";
//    int default_mean_k = 50;
//    double default_std_dev_mul = 1.0;
//    int default_negative = 0;
//    double default_radius = 1.0;
//
//    // Command line parsing
//    std::string method = default_method;
//    int min_pts = default_min_pts;
//    double radius = default_radius;
//    int mean_k = default_mean_k;
//    double std_dev_mul = default_std_dev_mul;
//    int negative = default_negative;
//    bool keep_organized = true; //find_switch(argc, argv, "-keep_organized");
//
//
//    StatisticalOutlierRemoval<PointT> filter(true);
//    filter.setInputCloud(cloud);
//    filter.setMeanK(mean_k);
//    filter.setStddevMulThresh(std_dev_mul);
//    filter.setNegative(negative);
//    filter.setKeepOrganized(keep_organized);
//    //PCL_INFO("Computing filtered cloud from %zu points with mean_k %d, std_dev_mul %f, inliers %d ...", xyz_cloud->size(), filter.getMeanK(), filter.getStddevMulThresh(), filter.getNegative());
//    //filter.filter(*xyz_cloud_filtered);
//    CloudPtr ground_image_test;
//    filter.filter(*ground_image_test);
//    // Get the indices that have been explicitly removed
//    //filter.getRemovedIndices(*removed_indices);


//    // Create the filtering object
//    //pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//    pcl::StatisticalOutlierRemoval<PointT> sor;
//    sor.setInputCloud(cloud);
//    sor.setMeanK(50);
//    sor.setStddevMulThresh(1.0);
//    CloudPtr ground_image_test (new Cloud);
//    sor.filter(*ground_image_test);



//    // build the filter
//    //pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
//    RadiusOutlierRemoval<PointT> outrem;
//    outrem.setInputCloud(cloud);
//    outrem.setRadiusSearch(0.8);
//    outrem.setMinNeighborsInRadius(2);
//
//    // apply filter
//    CloudPtr ground_image_test (new Cloud);
//    outrem.filter(*ground_image_test);













//    /*display distance map*/
//    CloudPtr distance_map_cloud(new Cloud);
//    *distance_map_cloud = *cloud;
//    float* distance_map = ne.getDistanceMap();
//    for (size_t i  = 0; i < cloud->points.size(); i++)
//    {
//      if (distance_map[i] < 5.0) //0.04 original
//      {
//        distance_map_cloud->points[i].r = 0;
//        distance_map_cloud->points[i].g = 255;
//        distance_map_cloud->points[i].b = 0;
//      }
//      else
//      {
//        distance_map_cloud->points[i].r = 255;
//        distance_map_cloud->points[i].g = 0;
//        distance_map_cloud->points[i].b = 0;
//      }
//    }
//
//    /*! Draw the segmentation result testtttttt*/
//    CloudPtr ground_image_test(new Cloud);
//    *ground_image_test = *distance_map_cloud;



/*
 *     // Prefer a faster method if the cloud is organized, over RANSAC
  if (!cloud_->isOrganized()) //nmot organized cloud
 */
//    /*! planar segmentation using RANSAC */
//    /*! planar segmentation using RANSAC */
//    /*! planar segmentation using RANSAC */
//    /*! planar segmentation using RANSAC */
//    /*! segmentation*/
//    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //  std::vector<pcl::PointIndices> region_indices;
//
//    // Create the segmentation object
//    pcl::SACSegmentation<PointT> seg;
//    // Optional
//    seg.setOptimizeCoefficients(true);
//    // Mandatory
//    seg.setModelType(pcl::SACMODEL_PLANE);
//    seg.setMethodType(pcl::SAC_RANSAC);
//    seg.setDistanceThreshold(0.01);
//
//    seg.setInputCloud(cloud->makeShared());
//    seg.segment(*inliers, *coefficients);
//
//    /*! display the output*/
//    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
//              << coefficients->values[1] << " "
//              << coefficients->values[2] << " "
//              << coefficients->values[3] << std::endl;
//
//    /*! Draw the segmentation result*/
//    CloudPtr ground_image_test(new Cloud);
//    *ground_image_test = *cloud;
//
//    for (int j = 0; j < inliers->indices.size(); j++)
//    {
//      ground_image_test->points[inliers->indices[j]].b = static_cast<uint8_t>((cloud->points[inliers->indices[j]].g + 255) / 2);
//    } //for j






//    /*sac plane segmentation new test*/
//    /*sac plane segmentation new test*/
//    /*sac plane segmentation new test*/
//    /*sac plane segmentation new test*/
//    int max_iterations = 1000;
//    double threshold = 0.05;
//    bool negative = false;
//
//
//    // Refine the plane indices
//    typedef SampleConsensusModelPlane<PointT>::Ptr SampleConsensusModelPlanePtr;
//    SampleConsensusModelPlanePtr model(new SampleConsensusModelPlane<PointT> (cloud));
//    RandomSampleConsensus<PointT> sac(model, threshold);
//    sac.setMaxIterations(max_iterations);
//    bool res = sac.computeModel();
//
//    vector<int> inliers_sac;
//    sac.getInliers(inliers_sac);
//    Eigen::VectorXf coefficients_sac;
//    sac.getModelCoefficients(coefficients_sac);
//
//    if (!res || inliers_sac.empty())
//    {
//      PCL_ERROR("No planar model found. Relax thresholds and continue.\n");
//      return;
//    }
//    sac.refineModel(2, 50);
//    sac.getInliers(inliers_sac);
//    sac.getModelCoefficients(coefficients_sac);
//
//    print_info("[done, ");
//    //print_value("%g", tt.toc());
//    print_info(" ms, plane has : ");
//    print_value("%zu", inliers_sac.size());
//    print_info(" points]\n");
//
//    print_info("Model coefficients for sac segmentation: [");
//    print_value("%g %g %g %g", coefficients_sac[0], coefficients_sac[1], coefficients_sac[2], coefficients_sac[3]);
//    print_info("]\n");
//
//    // Instead of returning the planar model as a set of inliers, return the outliers, but perform a cluster segmentation first
//    // Remove the plane indices from the data
//    PointIndices::Ptr everything_but_the_plane(new PointIndices);
//    std::vector<int> indices_fullset(cloud->size());
//    for (int p_it = 0; p_it < static_cast<int>(indices_fullset.size()); ++p_it)
//      indices_fullset[p_it] = p_it;
//
//    std::sort(inliers_sac.begin(), inliers_sac.end());
//    set_difference(indices_fullset.begin(), indices_fullset.end(),
//                   inliers_sac.begin(), inliers_sac.end(),
//                   inserter(everything_but_the_plane->indices, everything_but_the_plane->indices.begin())
//                  );
//
//    // Extract largest cluster minus the plane
//    vector<PointIndices> cluster_indices;
//    EuclideanClusterExtraction<PointT> ec;
//    ec.setClusterTolerance(0.02);  // 2cm
//    ec.setMinClusterSize(100);
//    ec.setInputCloud(cloud);
//    ec.setIndices(everything_but_the_plane);
//    ec.extract(cluster_indices);
//
//    /*! Draw the segmentation result*/
//    CloudPtr ground_image_test(new Cloud);
//    *ground_image_test = *cloud;
//
//    for (int j = 0; j < inliers_sac.size(); j++)
//    {
//      //ground_image_test->points[inliers_sac.at[j]].g = static_cast<uint8_t>((cloud->points[inliers_sac.at[j]].g + 255) / 2);
//      //ground_image_test->points[inliers_sac.at[j]].g = 255;
//      ground_image_test->points[inliers_sac[j]].g = 255;
//    } //for j





//    /*don test*/
//    ///The smallest scale to use in the DoN filter.
//    double scale1 = 1;
//
//    ///The largest scale to use in the DoN filter.
//    double scale2 = 2;
//
//    ///The minimum DoN magnitude to threshold by
//    double threshold = 1;
//
//    ///segment scene into clusters with given distance tolerance using euclidean clustering
//    double segradius = 1;
//
//
//    // calculate normals with the small scale
//    cout << "Calculating normals for scale..." << scale1 << endl;
//    pcl::PointCloud<PointNormal>::Ptr normals_small_scale(new pcl::PointCloud<PointNormal>);
//
//    ne.setRadiusSearch(scale1);
//    ne.compute(*normals_small_scale);
//
//    // calculate normals with the large scale
//    cout << "Calculating normals for scale..." << scale2 << endl;
//    pcl::PointCloud<PointNormal>::Ptr normals_large_scale(new pcl::PointCloud<PointNormal>);
//
//    ne.setRadiusSearch(scale2);
//    ne.compute(*normals_large_scale);
//
//    // Create output cloud for DoN results
//    PointCloud<PointNormal>::Ptr doncloud(new pcl::PointCloud<PointNormal>);
//    copyPointCloud<PointXYZRGB, PointNormal>(*cloud, *doncloud);
//
//    cout << "Calculating DoN... " << endl;
//    // Create DoN operator
//    pcl::DifferenceOfNormalsEstimation<PointXYZRGB, PointNormal, PointNormal> don;
//    don.setInputCloud(cloud);
//    don.setNormalScaleLarge(normals_large_scale);
//    don.setNormalScaleSmall(normals_small_scale);
//
//    if (!don.initCompute())
//    {
//      std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
//      exit(EXIT_FAILURE);
//    }
//
//    // Compute DoN
//    don.computeFeature(*doncloud);
//
//    // Save DoN features
//    pcl::PCDWriter writer;
//    writer.write<pcl::PointNormal> ("don.pcd", *doncloud, false);
//
//  //  //  /*! displaying output - aras - working*/
//  //  pcl::visualization::CloudViewer viewer("Cluster viewer");
//  //  viewer.showCloud(cloud);
//  //  while (!viewer.wasStopped())
//  //  {
//  //    boost::this_thread::sleep(boost::posix_time::microseconds(100));
//  //  }
//
//    // Filter by magnitude
//    cout << "Filtering out DoN mag <= " << threshold << "..." << endl;
//
//    // Build the condition for filtering
//    pcl::ConditionOr<PointNormal>::Ptr range_cond(
//      new pcl::ConditionOr<PointNormal> ()
//    );
//    range_cond->addComparison(pcl::FieldComparison<PointNormal>::ConstPtr(
//                                new pcl::FieldComparison<PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold))
//                             );
//    // Build the filter
//    pcl::ConditionalRemoval<PointNormal> condrem(range_cond);
//    condrem.setInputCloud(doncloud);
//
//    pcl::PointCloud<PointNormal>::Ptr doncloud_filtered(new pcl::PointCloud<PointNormal>);
//
//    // Apply filter
//    condrem.filter(*doncloud_filtered);
//
//    doncloud = doncloud_filtered;
//
//    // Filter by magnitude
//    cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << endl;
//
//    pcl::search::KdTree<PointNormal>::Ptr segtree(new pcl::search::KdTree<PointNormal>);
//    segtree->setInputCloud(doncloud);
//
//    std::vector<pcl::PointIndices> cluster_indices;
//    pcl::EuclideanClusterExtraction<PointNormal> ec;
//
//    ec.setClusterTolerance(segradius);
//    ec.setMinClusterSize(50);
//    ec.setMaxClusterSize(100000);
//    ec.setSearchMethod(segtree);
//    ec.setInputCloud(doncloud);
//    ec.extract(cluster_indices);






//    /*sac seg looking for all plane in a pcd*/
//    CloudPtr ground_image_test(new Cloud);
//    if (cloud->isOrganized())
//    {
//      SACSegmentation<PointT> seg;
//      seg.setOptimizeCoefficients(true);
//      seg.setModelType(SACMODEL_PLANE);
//      seg.setMethodType(SAC_RANSAC);
//      seg.setMaxIterations(10); //10000
//      seg.setDistanceThreshold(0.005);
//
//      // Copy XYZ and Normals to a new cloud
//      PointCloud<PointT>::Ptr cloud_segmented(new PointCloud<PointT> (*cloud));
//      PointCloud<PointT>::Ptr cloud_remaining(new PointCloud<PointT>);
//
//      ModelCoefficients coefficients;
//      ExtractIndices<PointT> extract;
//      PointIndices::Ptr inliers(new PointIndices());
//
//      // Up until 30% of the original cloud is left
//      int i = 1;
//      while (double(cloud_segmented->size()) > 0.3 * double(cloud->size()))
//      {
//        seg.setInputCloud(cloud_segmented);
//
//        print_highlight(stderr, "Searching for the largest plane (%2.0d) ", i++);
//        TicToc tt;
//        tt.tic();
//        seg.segment(*inliers, coefficients);
//        print_info("[done, ");
//        print_value("%g", tt.toc());
//        print_info(" ms : ");
//        print_value("%zu", inliers->indices.size());
//        print_info(" points]\n");
//
//        // No datasets could be found anymore
//        if (inliers->indices.empty())
//          break;
//
//        // Segment out all planes
////    vector<ModelCoefficients> model_coefficients_test;
////    vector<PointIndices> inlier_indices_test, boundary_indices;
//        vector<PlanarRegion<PointT>, Eigen::aligned_allocator<PlanarRegion<PointT> > > regions;
//
//        // Save this plane
//        PlanarRegion<PointT> region;
//        region.setCoefficients(coefficients);
//        regions.push_back(region);
//
//        inlier_indices.push_back(*inliers);
//        model_coefficients.push_back(coefficients);
//
//        // Extract the outliers
//        extract.setInputCloud(cloud_segmented);
//        extract.setIndices(inliers);
//        extract.setNegative(true);
//        extract.filter(*cloud_remaining);
//        cloud_segmented.swap(cloud_remaining);
//      }//while
//
//
//      /*! Draw the segmentation result testtttttt*/
////      CloudPtr ground_image_test(new Cloud);
//      *ground_image_test = *cloud_remaining;
//      vector<PointIndices> region_indices_test(region_indices); //region_indices
//
//      //Note the regions that have been extended
//      for (int i = 0; i < region_indices_test.size(); i++)
//      {
//        if (region_indices_test[i].indices.size() >= 10)
//        {
//          for (int j = 0; j < region_indices_test[i].indices.size(); j++)
//          {
//
////              ground_image_test->points[region_indices_test[i].indices[j]].g = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
//
//          }
//        }
//      }
//
//    }// if





//    /*planar convex hull compiling but no results no seg fault or erro though:)*/
//    /*planar convex hull*/
//    /*planar convex hull*/
//    /*planar convex hull*/
////    pcl::visualization::CloudViewer viewer;
//    pcl::VoxelGrid<PointT> grid_;
//    pcl::SACSegmentation<PointT> seg_;
//    pcl::ProjectInliers<PointT> proj_;
//    pcl::ConvexHull<PointT> chull_;
//
//    grid_.setFilterFieldName("z");
//    grid_.setFilterLimits(0.0, 3.0);
//    grid_.setLeafSize(0.0001f, 0.0001f, 0.0001f);
//
//    seg_.setOptimizeCoefficients(true);
//    seg_.setModelType(pcl::SACMODEL_PLANE);
//    seg_.setMethodType(pcl::SAC_RANSAC);
//    seg_.setMaxIterations(1000);
//    double threshold = 0.005;
//    seg_.setDistanceThreshold(threshold);
//
//    proj_.setModelType(pcl::SACMODEL_PLANE);
//
//
//    CloudPtr temp_cloud(new Cloud);
//    CloudPtr temp_cloud2(new Cloud);
//
////    grid_.setInputCloud(cloud); //not working !!!
////    grid_.filter(*temp_cloud);
//    *temp_cloud = *cloud; //instead of above :(  //no filtering
//
//    pcl::ModelCoefficients::Ptr coefficients_seg(new pcl::ModelCoefficients());
//    pcl::PointIndices::Ptr inliers_seg(new pcl::PointIndices());
//
//    seg_.setInputCloud(temp_cloud); //temp_cloud
//    seg_.segment(*inliers_seg, *coefficients_seg);
//
//    // Project the model inliers
//    proj_.setInputCloud(temp_cloud);
//    proj_.setModelCoefficients(coefficients_seg);
//    proj_.filter(*temp_cloud2);
//
//    // Create a Convex Hull representation of the projected inliers
//    chull_.setInputCloud(temp_cloud2);
//    chull_.reconstruct(*temp_cloud);





//    /*region growing seg*/
//    /*region growing seg*/
//    /*region growing seg*/
//    /*region growing seg*/
//    pcl::search::Search<PointT>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointT> > (new pcl::search::KdTree<PointT>);
//    pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
//    pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
//    normal_estimator.setSearchMethod(tree);
//    normal_estimator.setInputCloud(cloud);
//    normal_estimator.setKSearch(50);
//    normal_estimator.compute(*normals);
//
//    pcl::IndicesPtr indices_rgs(new std::vector <int>);
//    pcl::PassThrough<PointT> pass;
//    pass.setInputCloud(cloud);
//    pass.setFilterFieldName("z");
//    pass.setFilterLimits(0.0, 1.0);
//    pass.filter(*indices_rgs);
//
//    pcl::RegionGrowing<PointT, pcl::Normal> reg;
//    reg.setMinClusterSize(100);
//    reg.setMaxClusterSize(10000);
//    reg.setSearchMethod(tree);
//    reg.setNumberOfNeighbours(30);
//    reg.setInputCloud(cloud);
//    //reg.setIndices (indices);
//    reg.setInputNormals(normals);
//    reg.setSmoothnessThreshold(7.0 / 180.0 * M_PI);
//    reg.setCurvatureThreshold(1.0);
//
//    std::vector <pcl::PointIndices> clusters;
//    reg.extract(clusters);
//
//    /*! Draw the segmentation result region growing seg*/
//    CloudPtr ground_image_test(new Cloud);
//    *ground_image_test = *cloud;
//    vector<PointIndices> region_indices_test(clusters); //region_indices
//
//    //Note the regions that have been extended
//    for (int i = 0; i < region_indices_test.size(); i++)
//    {
//      if (region_indices_test[i].indices.size() >= 10)
//      {
//        for (int j = 0; j < region_indices_test[i].indices.size(); j++)
//        {
//
//          ground_image_test->points[region_indices_test[i].indices[j]].g = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
//
//        }
//      }
//    }


