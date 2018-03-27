#include <iostream>
#include <fstream>
#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace cv;
using namespace std;


///////////////////////
void learning(const vector<Mat>& images, Size& size, vector<Mat>& images_conv, unsigned int& num_patches_per_img)
{

  // Reshape and stack images into a rowMatrix
  Mat data = /*formatImagesForPCA*/convertImageMatToColVector(images);
  cout << "  Mat data = /*formatImagesForPCA*/convertImageMatToColVector(images): (input data to PCA for learning).............." << endl;
  LOG_PRINT(data);

  // perform PCA
  PCA pca(data, cv::Mat(), PCA::DATA_AS_COL, /*0.95*/1.00); // trackbar is initially set here, also this is a common value for retainedVariance

  // copy the PCA results
  Mat mean = pca.mean; //.reshape(1, 1); // store the mean vector
  cout << "mean before reshaping:::" << endl;
  LOG_PRINT(mean);
  mean = mean.reshape(images[0].channels(), images[0].rows);
  cout << "mean (after reshaping)::::::::::::::::::::::::::::::::::" << endl;
  cout << " mean = mean.reshape(images[0].channels(), images[0].rows)::::::::::::::::::::::::::::::::::" << endl;
  LOG_PRINT(mean);
  cout << " images[0]::::::::::::::::::::::::::::::::::" << endl;
  Mat img_0(images[0]);
  LOG_PRINT(img_0);

  mean = toGrayscale(mean);
  cout << "mean (after converted toGrayscale):::::::::" << endl;
  LOG_PRINT(mean);
  namedWindow("mean", WINDOW_NORMAL);
  imshow("mean", mean);

  Mat eigenvalues = pca.eigenvalues.clone(); // eigenvalues by row
  cout << "eigenvalues::::::::::::::::::::::::::::::::::" << endl;
  LOG_PRINT(eigenvalues);

  Mat eigenvectors;
  transpose(pca.eigenvectors, eigenvectors); // eigenvectors by column
  cout << "eigenvectors.transposed::::::::::::::::::::::::::::::::::" << endl;
  LOG_PRINT(eigenvectors);

  waitKey(0); // make sure we saw these

  // waitKey control keys
  char key /*= 's'*/; // default is stop
  int delay = 0;


  // Display EigenFeatures or Principle Components
  for (/*unsigned*/ int i = 0; i < eigenvectors.cols; i++)
  {
    if (eigenvalues.at<double>(i) > 1.00)
    {
      // show the eigenvalues
      string msg = format("Eigenvalues #%d = %0.5f", i, eigenvalues.at<double>(i));
      cout << msg << endl;

      // Reading the eigenvectors
      Mat eigenFeature = eigenvectors.col(i).clone();
      cout << "eigenFeature reshaping:::" << endl;
      LOG_PRINT(eigenFeature);

      // Reshape from a vector to an image (original size) -> rows and channels should be added to it
      eigenFeature = eigenFeature.reshape(images[0].channels(), images[0].rows); // now it s only a colVector -> needs rows & channels for original shape
      cout << "eigenFeature (after reshaping)::::::::::::::::::::::::::::::::::" << endl;
      LOG_PRINT(eigenFeature);
      cout << " eigenFeature.reshape(images[0].channels(), images[0].rows)::::::::::::::::::::::::::::::::::" << endl;
      Mat img_0(images[0]);
      LOG_PRINT(img_0);

      // normalize to 0-255 for display and grayscale conversion one channel
      eigenFeature = toGrayscale(eigenFeature);
      cout << "eigenFeature (after converted toGrayscale):::::::::" << endl;
      LOG_PRINT(eigenFeature);




      // Displaying it with its order with respect to WRT its eigenvalues
      string name = "eigenFeatures";//format("EigenFeature_%d", i);
      //string name = format("EigenFeature_%d", i);
      namedWindow(name, WINDOW_NORMAL);
      imshow(name, eigenFeature);

      /*int*/ key = waitKey(delay);
      if (key == 'q' || key == 27)
      {
        //exit(1); // completely exit the program
        break; //only exiting the loop by breaking the loop
      }
      if (key == 's')
      {
        /*stop or pause*/ delay = 0;
      }
      if (key == 'c')
      {
        /*continue;*/ delay = 1;
      }
    }
  }



  // reset delay to default
  delay = 0;


  vector<vector<double>> vec_V;

  Mat point = pca.project(data./*row*/col(0)); // project into the eigenspace, thus the image becomes a "point"
  vec_V.resize(point.rows);

  for (/*unsigned */int i = 0 ; i < data.cols; i++) //pool of patches
  {

    cout << "the INPUT patch data::::::::::::::::::::::::::::::::::" << endl;
    LOG_PRINT(data);

    // Demonstration of the effect of retainedVariance on the first image
    Mat point = pca.project(data./*row*/col(i)); // project into the eigenspace, thus the image becomes a "point"
    cout << "point = pca.project(data./*row*/col(i))::::::::::::::::::::::::::::::::::" << endl;
    LOG_PRINT(point);
    Mat reconstruction = pca.backProject(point); // re-create the image from the "point"
    cout << "reconstruction = pca.backProject(point)::::::::::::::::::::::::::::::::::" << endl;
    LOG_PRINT(reconstruction);
    reconstruction = reconstruction.reshape(images[0].channels(), images[0].rows); // only had one row before and nw reshape to this
    // reshape from a row vector into image shape
    cout << "reshaped::::::::::::::::::::::::::::::::::" << endl;
    LOG_PRINT(reconstruction);
    reconstruction = toGrayscale(reconstruction); // re-scale for displaying purposes
    cout << "converted toGrayscale or toGrayscaled ;)::::::::::::::::::::::::::::::::::" << endl;
    LOG_PRINT(reconstruction);

    for (/*unsigned */int row = 0; row < point.rows; ++row /*row*/)
    {
      double W = point.at<double>(row /*this is const number row */, 0);   // point is a rowMajorMatrix // this is almost DEFAULT for MAT as an img
      vec_V[row].push_back(W); // here FI = I - I_mean // I also represents all the patches in our pool extracted from the images
    }

    // init highgui window
    string winName = "Reconstruction | press 'q' to quit";
    namedWindow(winName, WINDOW_NORMAL);

    // display until user presses q
    imshow(winName, reconstruction);

    /*int*/ key = waitKey(delay);
    if (key == 'q' || key == 27)
    {
      break;
    }
    if (key == 's')
    {
      /*stop or pause*/ delay = 0;
    }
    if (key == 'c')
    {
      /*continue;*/ delay = 1;
    }
  }

  // reset delay to default
  delay = 0;
  cout << "vec_V for all projected patches:" << vec_V.size() << endl;
  cout << "vec_V[0] :" << endl;
  Mat mat(vec_V[0]);
  LOG_PRINT(mat);

  for (vector<vector<double>>::iterator iter = vec_V.begin(); iter < vec_V.end(); ++iter)
  {

    vector<double> V_dot_FI = *iter;

    /*! This is the convolution part */
    Mat convolved_images_same_V(V_dot_FI);// /*convolved_images_same_V.row(1)*/ = Mat(vec_V1_dot_FI); //.reshape(1);
    cout << "convolved image transposed before reshape::::::::::::::::::::::::::::::::::" << endl;
    LOG_PRINT(convolved_images_same_V);

    // to devide every images patches
    cout << "number of patches per image: " << num_patches_per_img << endl;
    // reshaping convolved images to the connvolved images
    convolved_images_same_V = convolved_images_same_V.reshape(0 /*no change for channels*/, num_patches_per_img /*num patches is defining number of W for each image*/);
    cout << "convolved image transposed reshaped for each image using total number of patches per image:::::::::::::" << endl;
    LOG_PRINT(convolved_images_same_V);

    //now reading each column of the Mat for each convolved images
    for (/*unsigned */int j = 0; j < convolved_images_same_V.cols; ++j) // j --> w, col, x
    {

      Mat convolved_img_same_V(convolved_images_same_V.col(j));  // width, w, j, X, x, columns, col
      cout << "convolved image for same V should be one column:::::::::::::" << endl;
      LOG_PRINT(convolved_img_same_V);

//      double minVal, maxVal;
//      Point minLoc, maxLoc;
//      minMaxLoc(convolved_img_same_V, &minVal, &maxVal, &minLoc, &maxLoc);
//      //Rect rect;
//      //rect = Rect(maxLoc, Size(cPATCH_WINDOW_SIZE_RECT, cPATCH_WINDOW_SIZE_RECT));
//      //rectangle(convolved_img_same_V, rect, Scalar(0, 255, 0), 2);
//      cout << "minVal: " << minVal << endl;
//      cout << "maxVal: " << maxVal << endl;

      convolved_img_same_V = toGrayscale(convolved_img_same_V); // re-scale for displaying purposes
      cout << "converted toGrayscale or toGrayscaled ;)::::::::::::::::::::::::::::::::::" << endl;
      LOG_PRINT(convolved_img_same_V);

      // reshape from a row vector into image shape
      cout << "size: " << size << endl;
      convolved_img_same_V = convolved_img_same_V.reshape(images[0].channels() /*no change in channels*/ /*channels should be same as the input images*/, size.height);
      cout << "reshaped::::::::::::::::::::::::::::::::::" << endl;
      LOG_PRINT(convolved_img_same_V);

      images_conv.push_back(convolved_img_same_V);


      // Displaying it with its order with respect to WRT its eigenvalues
      string name = "convolved_img_same_V";//format("EigenFeature_%d", i);
      //string name = format("EigenFeature_%d", i);
      namedWindow(name, WINDOW_NORMAL);
      imshow(name, convolved_img_same_V);
      namedWindow("convolved_img_same_V 2");
      imshow("convolved_img_same_V 2", convolved_img_same_V);


      /*int*/ key = waitKey(delay);
      if (key == 'q' || key == 27)
      {
        //exit(1); // completely exit the program
        break; //only exiting the loop by breaking the loop
      }
      if (key == 's')
      {
        /*stop or pause*/ delay = 0;
      }
      if (key == 'c')
      {
        /*continue;*/ delay = 1;
      }
    }

  }

}
