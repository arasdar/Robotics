

#include "projects/stereo_traversability_experiments/mlr/tMLR.h"


void finroc::stereo_traversability_experiments::mlr::tMLR::write_PCA(const cv::PCA pca_ /*constant because it is an INPUT*/, const char** argv)
{


  ///*
  //  bool create_directory(const path& p);
  //  bool create_directory(const path& p, system::error_code& ec);
  //  Effects: Attempts to create the directory p resolves to, as if by POSIX mkdir() with a second argument of S_IRWXU|S_IRWXG|S_IRWXO.
  //
  //  Postcondition: is_directory(p)
  //*/

  /*
    //  if (exists(p))    // does p actually exist?
    //  {
    //    if (is_regular_file(p))        // is p a regular file?
    //    {
    //      cout << p << " size is " << file_size(p) << '\n';
    //    }
    //
    //    else if (is_directory(p))      // is p a directory?
    //    {
    //      cout << p << " is a directory containing:\n";
    //    }
    //    else
    //    {
    //      cout << p << " exists, but is neither a regular file nor a directory\n";
    //    }
    //  }
    //  else
    //  {
    //    cout << p << " exists, but is neither a regular file nor a directory\n";
    //  }
  */

  /*! bool remove(const path& p);
  bool remove(const path& p, system::error_code& ec);
  Effects:  If exists(symlink_status(p,ec)), it is removed as if by POSIX remove().

  [Note: A symbolic link is itself removed, rather than the file it resolves to being removed. -- end note]

  Postcondition: !exists(symlink_status(p)).

  Returns:  false if p did not exist in the first place, otherwise true.

  Throws: As specified in Error reporting.

  uintmax_t remove_all(const path& p);
  uintmax_t remove_all(const path& p, system::error_code& ec);
  Effects:  Recursively deletes the contents of p if it exists, then deletes file p itself, as if by POSIX remove().

  [Note: A symbolic link is itself removed, rather than the file it resolves to being removed. -- end note]

  Postcondition: !exists(p)

  Returns: The number of files removed.

  Throws: As specified in Error reporting.
  */

  // I want to know if this path exist or not -- meaning have already been learned or created
  // In this case, I do not care what is in it -- remove the whole folder and its content and create a new one
  // To write the whole learned PCA files
  boost::filesystem::path p(argv[1]);    // p reads clearer than argv[1] in the following code

  // Create a directory path and name for all learned PCA details
  const string dir_name = p.string() + "/" + "learned_PCA"; // write the director at that  specific spot which the program is being executed
  cout << "dir_name: " << dir_name << endl;

  boost::filesystem::path dir(dir_name);

  // if it already exist - remove it first with all its content
  if ((exists(dir))     // does p actually exist?
      || (is_regular_file(dir))        // is p a regular file?
      || (is_directory(dir))      // is p a directory?
     )
  {
    boost::filesystem::remove_all(dir);
    cout << dir.string() << " was already existing and now it is successfully and completely removed..............................." << endl;
  }// end if directory exist

  // Create it again but this time EMPTY
  if (boost::filesystem::create_directories(dir))
  {
    std::cout << dir_name << " is new directory and is now successfully created........................." << "\n";
  }

  // Now fill it up with the learned PCA files
  // Then fisrt write mean into its own file
  const string filename_mean = dir_name + "/" + "mean.xml";
  cv::FileStorage fs;
  fs.open(filename_mean, cv::FileStorage::WRITE);
//  fs << "mean.size" << pca_.mean.size; // they all have the same type
//  fs << "mean.type()" << pca_.mean.type();
  fs << "mean" << pca_.mean;
  cout << filename_mean << " writtennnnnnnnnnnn" << endl;

  //Mat eigenvalues (eigenvalues.rows, 1, mean.type);
  // This is used to createMat header for eigenvalues matrix before loaing them
  const string filename_eigenvalues = dir_name + "/" + "eigenvalues_matrix_info.xml";
  fs.open(filename_eigenvalues, cv::FileStorage::WRITE);
  fs << "eigenvalues_rows" << pca_.eigenvalues.rows;
  //fs << "eigenvalues.type()" << pca_.eigenvalues.type();
  cout << filename_eigenvalues << " writtennnnnnnnnnnn" << endl;

  //Mat eigenvalues (eigenvalues.rows, eigenvectors.cols, mean.type);
  // This is used to createMat header for eigenvectors matrix before loading it
  const string filename_eigenvectors = dir_name + "/" + "eigenvectors_matrix_info.xml";
  fs.open(filename_eigenvectors, cv::FileStorage::WRITE);
  fs << "eigenvectors_cols" << pca_.eigenvectors.cols;
  //fs << "eigenvectors.type()" << pca_.eigenvectors.type();
  cout << filename_eigenvectors << " writtennnnnnnnnnnn" << endl;

  // eigenvectors matrix

  // Iterating the Principle Components
  for (int i = 0; i < pca_.eigenvalues.rows; i++) // i is either ITERATION or ROWS(HEIGHT, Y) -- in this case ROWS
  {
    // eigenvalues and eigenvectors
    const string index = format("_%d", i);
    const string filenam_eigenspace = dir_name + "/" + "EigenSpace" + index + ".xml";
    fs.open(filenam_eigenspace, cv::FileStorage::WRITE);
    fs << "eigenvectors" << pca_.eigenvectors.row(i); // i == row == y == height and also ITERATING THROUGH ROWS & also INDEX == i
    fs << "eigenvalues" << pca_.eigenvalues.row(i);
    cout << filenam_eigenspace << " writtennnnnnnnnnnn" << endl;
  }// for iterating through eigenvalues

  cout << "learned PCA is written to: " << dir_name << endl;

  fs.release();
} // end of writing learned ES into memory
