/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <vector>


using namespace pcl;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

//testing template
//template <typename PointType>
//typedef pcl::PointCloud<PointType> allCloud;
//typedef typename Cloud::ConstPtr CloudConstPtr;
//typedef typename Cloud::Ptr CloudPtr;



  /** \brief this is a demonstration application for using PCL's openni grabber with pcd output suitable for real-time point cloud processing.
    *
    * \revisor Aras
    */


class SimpleOpenNIViewer
{
private:
   pcl::visualization::CloudViewer viewer;
   boost::mutex cloud_mutex;
   CloudConstPtr prev_cloud;



  public:
   SimpleOpenNIViewer () :
	   prev_cloud (new Cloud),
	   viewer ("PCL OpenNI Viewer")
   {}//constructor

    void
    cloud_cb_ (const CloudConstPtr& cloud)
    {
      if (!viewer.wasStopped ())
      {
        cloud_mutex.lock ();
        prev_cloud = cloud;
        cloud_mutex.unlock ();
      }
    }

    void run ()
    {

     	 //////////openni grabber
      	pcl::Grabber* interface = new pcl::OpenNIGrabber();
         boost::function<void (const CloudConstPtr&)> f =
           boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);
         interface->registerCallback (f);
      interface->start ();

		while (!viewer.wasStopped())
		{
			viewer.showCloud (prev_cloud);
		}

		 //////stop openni grabber interface
      interface->stop ();

    }//run
};//class




int
main (int argc, char** argv)
{



	  ////testing kinect
	  SimpleOpenNIViewer v;
	     v.run ();

//
//	// Process and display
//	HRCSSegmentation hrcs ( v.getLatestCloud() );
//	hrcs.run ();

 return 0;
}
