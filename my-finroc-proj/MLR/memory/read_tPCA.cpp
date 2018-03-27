

#include "projects/stereo_traversability_experiments/mlr/tMLR.h"


using namespace finroc::stereo_traversability_experiments::mlr;

void tMLR::read_tPCA(/*INPUT*/const char** argv, /*OUTPUT*/mlr::learn_PCA::tPCA& pca)
{

  boost::filesystem::path p(argv[1]);    // p reads clearer than argv[1] in the following code

  // Created directory ALREADY for all learned Eigen Space details
  const string dir_name = p.string() + "/" + "learned_PCA"; // write the director at that  specific spot which the program is being executed
  cout << "dir_name: " << dir_name << endl;
  boost::filesystem::path dir(dir_name);

  // More clear and simpler one
  if (
    (exists(dir))     // does p actually exist?
    && (is_directory(dir))      // is p a directory?
  )
  {
    cout << dir.string() << " is existing and is a directory. now ready to read the learned PCA ..............................." << endl;
  }// end if directory exist
  else
  {
    cout << dir.string() << " is not existing, Please LEARN the database first!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
  }

  // Then fisrt write mean into its own file
  const string filename_mean = dir_name + "/" + "mean.xml";
  cv::FileStorage fs;
  fs.open(filename_mean, cv::FileStorage::READ);
  fs["mean"] >> pca.mean; //.mean;
  cout << filename_mean << " readdddddddddddddd & loaded" << endl;

  //Mat eigenvalues (eigenvalues.rows, 1, mean.type);
  // This is used to createMat header for eigenvalues matrix before loaing them
  const string filename_SV = dir_name + "/" + "SV_matrix_info.xml";
  fs.open(filename_SV, cv::FileStorage::READ);
  int V_rows, V_cols, V_type;
  fs["V_rows"] >> V_rows;
  fs["V_cols"] >> V_cols;
  fs["V_type"] >> V_type; // this is DOUBLE or CV64FC1 or 6
  pca.S.create(1 /*only one row*/, V_cols, V_type);
  pca.V.create(V_rows, V_cols, V_type);
  cout << filename_SV << " readddddddddddddd & loaded" << endl;

  // Loading the Principle Components
  for (int i = 0; i < pca.V.cols; i++) // i is either ITERATION or ROWS(HEIGHT, Y) -- in this case ROWS
  {
    const string index = format("_%.4d", i);
    const string filename_PC = dir_name + "/" + "PC" + index + ".xml";
    fs.open(filename_PC, cv::FileStorage::READ);
    cv::Mat V_col_i;
    fs["V"] >> V_col_i;
    V_col_i.convertTo(pca.V.col(i), pca.V.type()); // i == row == y == height and also ITERATING THROUGH ROWS & also INDEX == i
    cv::Mat S_col_i;
    fs["S"] >> S_col_i;
    S_col_i.convertTo(pca.S.col(i), pca.S.type());
    cout << filename_PC << " readdddddddddddddddd & loaded" << endl;
  }// for iterating through eigenvalues

  cout << "learned PCA and PCs are read and loaded from: " << dir_name << endl;

  fs.release();
} // end of writing learned ES into memory
