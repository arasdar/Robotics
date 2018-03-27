#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/vtk_lib_io.h>

#include <stdio.h>

using namespace cv;

static void print_help()
{
    printf("\nDemo stereo matching converting L and R images into disparity and point clouds\n");
    printf("\nUsage: stereo_match <left_image> <right_image> [--algorithm=bm|sgbm|hh|var] [--blocksize=<block_size>]\n"
           "[--max-disparity=<max_disparity>] [--scale=scale_factor>] [-i <intrinsic_filename>] [-e <extrinsic_filename>]\n"
           "[--no-display] [-o <disparity_image>]\n");
}

int main(int argc, char** argv)
{

	if(argc < 3)
	{
		print_help();
		return 0;
	}

	int img_number_left = 0, img_number_right = 0 ;
	int img_index = 0;

  // Get list of input png images left
  vector<string> left_images;
  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator itr (argv[1]); itr != end_itr; ++itr)
  {
	left_images.push_back (itr->path ().string ());
	img_number_left++;
  }
  sort (left_images.begin (), left_images.end ());

  // Get list of input png images right
  vector<string> right_images;
//  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator itr (argv[2]); itr != end_itr; ++itr)
  {
	right_images.push_back (itr->path ().string ());
	img_number_right++;
  }
  sort (right_images.begin (), right_images.end ());


  for (;;)
    {

		// exit criteria
		if (img_number_right == img_number_left){
		  cout << "start reading the stereo images ...................................................." << endl;
		  cout << "equal number of image pairs" << endl;
		}
		else{
		  cout << "img_number_left" << img_number_left << "-----vs-----" << "img_number_right" << img_number_right << endl;
		  cout << "uneven pair of images!!!!!!!!!!!!!!!!!" << img_number_left << img_number_right << endl;  // add quotation
		  break;
		}
		// exit criteria
		cout << "img_index:" << img_index << "  img_number: " << img_number_left << endl;
		if ( img_index == img_number_left){
			cout << "end............................................." << endl;
			break;
		}

		// showing the input images
		cout << "left_images[img_index]: " << left_images[img_index] << std::endl;
		cout << "right_images[img_index]: " << right_images[img_index] << std::endl;

		//options
		const char* algorithm_opt = "--algorithm=";
		const char* maxdisp_opt = "--max-disparity=";
		const char* blocksize_opt = "--blocksize=";
		const char* nodisplay_opt = "--no-display=";
		const char* scale_opt = "--scale=";

		//variable initialization
		const char* img1_filename = (char*) left_images[img_index].c_str();
		const char* img2_filename = (char*) right_images[img_index].c_str();
		const char* intrinsic_filename = 0;
		const char* extrinsic_filename = 0;
		const char* disparity_filename = 0;
		const char* point_cloud_filename = 0;

		enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3 };
		int alg = STEREO_SGBM;
		int SADWindowSize = 0, numberOfDisparities = 0;
		bool no_display = false;
		float scale = 1.f;

		StereoBM bm;
		StereoSGBM sgbm;
		StereoVar var;

		// rectification module for loop
		for( int i = 9; i < argc; i++ )
		{
			if( strncmp(argv[i], algorithm_opt, strlen(algorithm_opt)) == 0 )
			{
				char* _alg = argv[i] + strlen(algorithm_opt);
				alg = strcmp(_alg, "bm") == 0 ? STEREO_BM :
					  strcmp(_alg, "sgbm") == 0 ? STEREO_SGBM :
					  strcmp(_alg, "hh") == 0 ? STEREO_HH :
					  strcmp(_alg, "var") == 0 ? STEREO_VAR : -1;
				if( alg < 0 )
				{
					printf("Command-line parameter error: Unknown stereo algorithm\n\n");
					print_help();
					return -1;
				}
			}
			else if( strncmp(argv[i], maxdisp_opt, strlen(maxdisp_opt)) == 0 )
			{
				if( sscanf( argv[i] + strlen(maxdisp_opt), "%d", &numberOfDisparities ) != 1 ||
					numberOfDisparities < 1 || numberOfDisparities % 16 != 0 )
				{
					printf("Command-line parameter error: The max disparity (--maxdisparity=<...>) must be a positive integer divisible by 16\n");
					print_help();
					return -1;
				}
			}
			else if( strncmp(argv[i], blocksize_opt, strlen(blocksize_opt)) == 0 )
			{
				if( sscanf( argv[i] + strlen(blocksize_opt), "%d", &SADWindowSize ) != 1 ||
					SADWindowSize < 1 || SADWindowSize % 2 != 1 )
				{
					printf("Command-line parameter error: The block size (--blocksize=<...>) must be a positive odd number\n");
					return -1;
				}
			}
			else if( strncmp(argv[i], scale_opt, strlen(scale_opt)) == 0 )
			{
				if( sscanf( argv[i] + strlen(scale_opt), "%f", &scale ) != 1 || scale < 0 )
				{
					printf("Command-line parameter error: The scale factor (--scale=<...>) must be a positive floating-point number\n");
					return -1;
				}
			}
			else if( strcmp(argv[i], nodisplay_opt) == 0 )
				no_display = true;
			else if( strcmp(argv[i], "-i" ) == 0 )
				intrinsic_filename = argv[++i];
			else if( strcmp(argv[i], "-e" ) == 0 )
				extrinsic_filename = argv[++i];
			else if( strcmp(argv[i], "-o" ) == 0 )
				disparity_filename = argv[++i];
			else if( strcmp(argv[i], "-p" ) == 0 )
				point_cloud_filename = argv[++i];
			else
			{
				printf("Command-line parameter error: unknown option %s\n", argv[i]);
				return -1;
			}
		} //for rectification



//		if( !img1_filename || !img2_filename )
//		{
//			printf("Command-line parameter error: both left and right images must be specified\n");
//			return -1;
//		}
//
//		if( (intrinsic_filename != 0) ^ (extrinsic_filename != 0) )
//		{
//			printf("Command-line parameter error: either both intrinsic and extrinsic parameters must be specified, or none of them (when the stereo pair is already rectified)\n");
//			return -1;
//		}
//
//		if( extrinsic_filename == 0 && point_cloud_filename )
//		{
//			printf("Command-line parameter error: extrinsic and intrinsic parameters must be specified to compute the point cloud\n");
//			return -1;
//		}

		int color_mode = alg == STEREO_BM ? 0 : -1;
		Mat img1 = imread(img1_filename, color_mode);
		Mat img2 = imread(img2_filename, color_mode);

		if( scale != 1.f )
		{
			Mat temp1, temp2;
			int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
			resize(img1, temp1, Size(), scale, scale, method);
			img1 = temp1;
			resize(img2, temp2, Size(), scale, scale, method);
			img2 = temp2;
		}

		Size img_size = img1.size();

		Rect roi1, roi2;
		Mat Q;

		if( intrinsic_filename )
		{
			// reading intrinsic parameters
			FileStorage fs(intrinsic_filename, CV_STORAGE_READ);
			if(!fs.isOpened())
			{
				printf("Failed to open file %s\n", intrinsic_filename);
				return -1;
			}

			Mat M1, D1, M2, D2;
			fs["M1"] >> M1;
			fs["D1"] >> D1;
			fs["M2"] >> M2;
			fs["D2"] >> D2;

			fs.open(extrinsic_filename, CV_STORAGE_READ);
			if(!fs.isOpened())
			{
				printf("Failed to open file %s\n", extrinsic_filename);
				return -1;
			}

			Mat R, T, R1, P1, R2, P2;
			fs["R"] >> R;
			fs["T"] >> T;

			stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );

			Mat map11, map12, map21, map22;
			initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
			initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

			Mat img1r, img2r;
			remap(img1, img1r, map11, map12, INTER_LINEAR);
			remap(img2, img2r, map21, map22, INTER_LINEAR);

			img1 = img1r;
			img2 = img2r;
		}

		numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;

		bm.state->roi1 = roi1;
		bm.state->roi2 = roi2;
		bm.state->preFilterCap = 31;
		bm.state->SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 9;
		bm.state->minDisparity = 0;
		bm.state->numberOfDisparities = numberOfDisparities;
		bm.state->textureThreshold = 10;
		bm.state->uniquenessRatio = 15;
		bm.state->speckleWindowSize = 100;
		bm.state->speckleRange = 32;
		bm.state->disp12MaxDiff = 1;

		sgbm.preFilterCap = 63;
		sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;

		int cn = img1.channels();

		sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
		sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
		sgbm.minDisparity = 0;
		sgbm.numberOfDisparities = numberOfDisparities;
		sgbm.uniquenessRatio = 10;
		sgbm.speckleWindowSize = bm.state->speckleWindowSize;
		sgbm.speckleRange = bm.state->speckleRange;
		sgbm.disp12MaxDiff = 1;
		sgbm.fullDP = alg == STEREO_HH;

		var.levels = 3;                                 // ignored with USE_AUTO_PARAMS
		var.pyrScale = 0.5;                             // ignored with USE_AUTO_PARAMS
		var.nIt = 25;
		var.minDisp = -numberOfDisparities;
		var.maxDisp = 0;
		var.poly_n = 3;
		var.poly_sigma = 0.0;
		var.fi = 15.0f;
		var.lambda = 0.03f;
		var.penalization = var.PENALIZATION_TICHONOV;   // ignored with USE_AUTO_PARAMS
		var.cycle = var.CYCLE_V;                        // ignored with USE_AUTO_PARAMS
		var.flags = var.USE_SMART_ID | var.USE_AUTO_PARAMS | var.USE_INITIAL_DISPARITY | var.USE_MEDIAN_FILTERING ;

		Mat disp, disp8;
		Mat img1p, img2p, dispp;
		copyMakeBorder(img1, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
		copyMakeBorder(img2, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);

		int64 t = getTickCount();
		if( alg == STEREO_BM )
			bm(img1, img2, disp);
		else if( alg == STEREO_VAR ) {
			var(img1, img2, disp);
		}
		else if( alg == STEREO_SGBM || alg == STEREO_HH )
			sgbm(img1, img2, disp);
		t = getTickCount() - t;
		printf("Time elapsed: %fms\n", t*1000/getTickFrequency());

		//disp = dispp.colRange(numberOfDisparities, img1p.cols);
		if( alg != STEREO_VAR )
			disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
		else
			disp.convertTo(disp8, CV_8U);

		//save output disparity image
	    if(disparity_filename){
			std::string dir_name_disparity = argv [3];
			std::string file_name_disparity = argv[4];
			std::stringstream ss_disparity;
			ss_disparity << dir_name_disparity << "/" << file_name_disparity << "_rect_" << img_index << ".png";
			cout << "disparity map for rectified stereo images output: " << ss_disparity.str()  << endl;
			imwrite( ss_disparity.str(), disp8 );
	    }

		//save output left rectified image
	    if(disparity_filename){
			std::string dir_name_disparity = argv [5];
			std::string file_name_disparity = argv[6];
			std::stringstream ss_disparity;
			ss_disparity << dir_name_disparity << "/" << file_name_disparity << "_rect_" << img_index << ".png";
			cout << "left rectified image output: " << ss_disparity.str()  << endl;
			imwrite( ss_disparity.str(), disp8 );
	    }

		//save output right rectified image
	    if(disparity_filename){
			std::string dir_name_disparity = argv [7];
			std::string file_name_disparity = argv[8];
			std::stringstream ss_disparity;
			ss_disparity << dir_name_disparity << "/" << file_name_disparity << "_rect_" << img_index << ".png";
			cout << "right rectified image output: " << ss_disparity.str()  << endl;
			imwrite( ss_disparity.str(), disp8 );
	    }



		if( !no_display )
		{
			namedWindow("left", 1);
			imshow("left", img1);
			namedWindow("right", 1);
			imshow("right", img2);
			namedWindow("disparity", 0);
			imshow("disparity", disp8);
			printf("press any key to continue...");
			fflush(stdout);
			waitKey(10);
			printf("\n");
		}

		img_index++;
    }//for

    return 0;
}
