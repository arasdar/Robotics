#include "projects/stereo_traversability_experiments/daniel/stereo_color_hybrid/offline/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::daniel::stereo_color_hybrid::offline;

/* Method adapted from SLICSuperpixels
 * Draws boundaries around superpixels whose size exceeds threshold
 */
void tStereoProcessing::drawContoursAroundSegments(
  CloudPtr cloud,
  PointCloud<pcl::Label>& labels,
  std::vector<pcl::PointIndices>& segment_indices)
{
  unsigned invalid_label = std::numeric_limits<unsigned>::max();
  const int dx8[8] = { -1, -1,  0,  1, 1, 1, 0, -1};
  const int dy8[8] = { 0, -1, -1, -1, 0, 1, 1,  1};

  int width = cloud->width;
  int height = cloud->height;
  int sz = width * height;
  vector<bool> istaken(sz, false);
  vector<int> contourx(sz);
  vector<int> contoury(sz);
  int mainindex(0);
  int cind(0);
  for (int j = 0; j < height; j++)
  {
    for (int k = 0; k < width; k++)
    {
      if (labels[mainindex].label == invalid_label)
      {
        mainindex++;
        continue;
      }
      if (segment_indices[labels[mainindex].label].indices.size() > thresh_segments)// corresponding segment big enough
      {
        int np(0);
        for (int i = 0; i < 8; i++)
        {
          int x = k + dx8[i];
          int y = j + dy8[i];

          if ((x >= 0 && x < width) && (y >= 0 && y < height))
          {
            int index = y * width + x;

            //if( false == istaken[index] )//comment this to obtain internal contours
            {
              if (labels[mainindex].label != labels[index].label) np++;
            }
          }
        }
        if (np > 1)
        {
          contourx[cind] = k;
          contoury[cind] = j;
          istaken[mainindex] = true;
          //img[mainindex] = color;
          cind++;
        }
      }
      mainindex++;
    }
  }

  int numboundpix = cind;//int(contourx.size());
  for (int j = 0; j < numboundpix; j++)
  {
    int ii = contoury[j] * width + contourx[j];
    cloud->points[ii].r = 255;
    cloud->points[ii].g = 255;
    cloud->points[ii].b = 255;

    for (int n = 0; n < 8; n++)
    {
      int x = contourx[j] + dx8[n];
      int y = contoury[j] + dy8[n];
      if ((x >= 0 && x < width) && (y >= 0 && y < height))
      {
        int ind = y * width + x;
        if (!istaken[ind])
        {
          cloud->points[ind].r = 0;
          cloud->points[ind].g = 0;
          cloud->points[ind].b = 0;
        }
      }
    }
  }
}

/*
 * Draw surface normals as red arrows for dedicated viewer
 */
void tStereoProcessing::drawSurfaceNormals(visualization::PCLVisualizer::Ptr viewer,
    vector<pcl::PointIndices> segment_indices,
    vector<Eigen::VectorXf> planes,
    vector<PointXYZ> centroid,
    unsigned& max_id)
{
  Eigen::Vector3f gravity(0, 1, 0); // points "downwards"
  char str[50];
  //cleaning the previous segment normals first
  for (unsigned idx = 0; idx < max_id; idx++)
  {
    sprintf(str, "segm_norm_%01d", idx);
    viewer->removeShape(str);
  }

  unsigned index = 0;
  for (unsigned i = 0; i < planes.size(); i++)
  {
    if (segment_indices[i].indices.size() > thresh_segments)
    {
      PointXYZ np1 = centroid[i];
      PointXYZ np2(centroid[i].x + planes[i][0],
                   centroid[i].y + planes[i][1],
                   centroid[i].z + planes[i][2]);

      PointXYZ avgnp1 = np1;
      Eigen::Vector3f avg_normal = Eigen::Vector3f::Zero();
      unsigned num_infinite = 0;
      for (unsigned j = 0; j < segment_indices[i].indices.size(); j++)
      {
        // only consider valid normals
        if (pcl_isfinite(prev_normal_cloud->points[segment_indices[i].indices[j]].data_c[0]) &&
            pcl_isfinite(prev_normal_cloud->points[segment_indices[i].indices[j]].data_c[1]) &&
            pcl_isfinite(prev_normal_cloud->points[segment_indices[i].indices[j]].data_c[2]))
        {
          avg_normal = avg_normal + prev_normal_cloud->points[segment_indices[i].indices[j]].getNormalVector3fMap();
        }
        else
        {
          num_infinite ++;
        }
      }
      avg_normal = avg_normal / (segment_indices[i].indices.size() - num_infinite);
      //std::cout << avg_normal[0] << " " <<avg_normal[1] << " " << avg_normal[2] << " " << segment_indices[i].indices.size() << " "<< num_infinite << std::endl ;
      PointXYZ avgnp2(avgnp1.x + avg_normal[0],
                      avgnp1.y + avg_normal[1],
                      avgnp1.z + avg_normal[2]);

      PointXYZ gp1 = np1;
      PointXYZ gp2(np1.x + gravity[0],
                   np1.y + gravity[1],
                   np1.z + gravity[2]);

      sprintf(str, "segm_norm_%01d", index);
      viewer->addArrow(np2, np1, 1.0, 0.0, 0, false, str);
      index ++;
      sprintf(str, "segm_norm_%01d", index);
      viewer->addArrow(gp2, gp1, 0.0, 0.0, 1.0, false, str);
      index++;
      sprintf(str, "segm_norm_%01d", index);
      viewer->addArrow(avgnp2, avgnp1, 1.0, 1.0, 1.0, false, str);
      index ++;
    }

  }
  max_id = index;
}

/*
 * Remove surface normals as red arrows for dedicated viewer
 */
void tStereoProcessing::removeSurfaceNormals(visualization::PCLVisualizer::Ptr viewer,
    unsigned& max_id)
{
  char str[50];
  //cleaning the previous segment normals first
  for (unsigned idx = 0; idx < max_id; idx++)
  {
    sprintf(str, "segm_norm_%01d", idx);
    viewer->removeShape(str);
  }
  max_id = 0;
}

