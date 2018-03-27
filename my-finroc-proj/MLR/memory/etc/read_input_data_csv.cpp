#include "projects/stereo_traversability_experiments/def/tMLR.h"


using namespace cv;

//int main(int, char** argv)
void finroc::stereo_traversability_experiments::def::tMLR::read_input_data_test(int argc, const char** argv)
{
  //! [pre-process]
  // Load image
//    Mat src = imread("/home/aras/finroc_/database/facerec/yale/yalefaces/subject01.normal", CV_LOAD_IMAGE_GRAYSCALE);
  Mat src = imread("/home/aras/finroc_/database/facerec/yale/test.jpg", CV_LOAD_IMAGE_GRAYSCALE);
  //Mat src = imread(argv[1]);

  // Check if image is loaded successfully
  if (!src.data || src.empty())
  {
    cout << "Problem loading image!!!" << endl;
    //return EXIT_FAILURE;
  }

  imshow("src", src);

//  // Convert image to grayscale
//  Mat gray;
//  cvtColor(src, gray, COLOR_BGR2GRAY);
//
//  // Convert image to binary
//  //Mat bw;
//  //threshold(gray, bw, 50, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
//
//  imshow("output", src);
  waitKey(0);

}

/*static*/ void finroc::stereo_traversability_experiments::def::tMLR::read_imgList(const string& filename, vector<Mat>& images)
{
  std::ifstream file(filename.c_str(), ifstream::in);
  if (!file)
  {
    string error_message = "No valid input file was given, please check the given filename.";
    CV_Error(/*Error::*/CV_StsBadArg, error_message);
  }
  string line;
  while (getline(file, line))
  {
    images.push_back(imread(line, 0));
  }
}

void finroc::stereo_traversability_experiments::def::tMLR::read_csv_2(const string& filename, vector<Mat>& images, vector<int>& labels)
{
  std::ifstream file(filename.c_str(), ifstream::in);
  if (!file)
    throw std::exception();
  std::string line, path, classlabel;
  // For each line in the given file:
  while (std::getline(file, line))
  {
    // Get the current line:
    std::stringstream liness(line);
    // Split it at the semicolon:
    std::getline(liness, path, ';');
    std::getline(liness, classlabel);
    // And push back the data into the result vectors:
    images.push_back(imread(path, IMREAD_GRAYSCALE));
    labels.push_back(atoi(classlabel.c_str()));
  }
  /* This is how to use it
  vector<int> labels;
  read_csv("/home/philipp/facerec/data/at.txt", db, labels);
  */

}

/*static*/ void finroc::stereo_traversability_experiments::def::tMLR::read_csv(const string& filename, vector<Mat>& input_data, vector<int>& labels, char separator = ';')
{
  std::ifstream file(filename.c_str(), ifstream::in);
  if (!file)
  {
    string error_message = "No valid input file was given, please check the given filename.";
    CV_Error(/*Error::*/CV_StsBadArg, error_message);
  }
  string line, path, classlabel;
  while (getline(file, line))
  {
    stringstream liness(line);
    getline(liness, path, separator);
    getline(liness, classlabel);
    if (!path.empty() && !classlabel.empty())
    {
      input_data.push_back(imread(path, 0));
      labels.push_back(atoi(classlabel.c_str()));
    }
  }
}



/*int main(int argc, const char *argv[])*/
vector<Mat> finroc::stereo_traversability_experiments::def::tMLR::read_input_data_csv(int argc, const char** argv)
{
  // Check for valid command line arguments, print usage
  // if no arguments were given.
  if (argc < 3)
  {
    cout << "usage: " << argv[0] << " I<csv_file>  I_new<csv_file>" << endl;
    exit(1);
  }

  // Get the path to your CSV.
  string fn_csv("");
  if (!already_learnt/*not for recognition*/)
  {
    /*string */fn_csv = string(argv[1]);
  }
  else /*this data is for recognition*/
  {
    fn_csv = string(argv[2]);
  }

  // These vectors hold the input_data and corresponding labels.
  vector<Mat> input_data;
  vector<int> labels;


  // Read in the data. This can fail if no valid
  // input filename is given.
  try
  {
    read_csv(fn_csv, input_data, labels); // read and raw input data and its labels in the csv file
  }
  catch (cv::Exception& e)
  {
    cerr << "Error opening file \"" << fn_csv << "\". Reason: " << e.msg << endl;
    // nothing more we can do
    exit(1);
  }

  // Quit if there are not enough input_data for this demo.
  if (input_data.size() < 1)
  {
    string error_message = "There are no input_data to work with.";
    CV_Error(CV_StsError, error_message);
  }

  // assert there are as much samples as labels
  if (static_cast<unsigned int>(labels.size()) !=  input_data.size())
  {
    String error_message = format("The number of samples (src) must equal the number of labels (labels)! len(src)=%d, len(labels)=%d.", input_data.size(), labels.size());
    CV_Error(CV_StsBadArg, error_message);
  }


  return input_data;
}
