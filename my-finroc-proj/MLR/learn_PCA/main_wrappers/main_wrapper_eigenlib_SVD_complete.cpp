#include <Eigen/Dense>
#include <iostream>

using namespace std;
using namespace Eigen;

/*! Be careful about number of feature vectors (DataSamples) vs
 *          number of feature values (features or DataDimension)
 *          number of data samples should be infinite
 *          number of features is finite
*/
const unsigned int cNUMBER_OF_COLUMNS_INPUT_DATA = 10;  // this is the number of samples of the data which is recorded during time // SAME FOR INPUT AND OUTPUT
const unsigned int cNUMBER_OF_FEATURES_INPUT_DATA = 7; //cNUMBER_OF_FEATURES_INPUT_DATA //this is number of rows

int main(int argc, char** argv)
{
  /*! IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII -- INPUT
   * Reading the DataSet or FeatureVectors or DataSamples
   * Data Acquistion
   *
   * */
  MatrixXd matrix_input_data(cNUMBER_OF_FEATURES_INPUT_DATA, cNUMBER_OF_COLUMNS_INPUT_DATA);
  matrix_input_data.setRandom();
  cout << "matrix_input_data Input:\n" << matrix_input_data << endl;
//  cout << "matrix_input_data Input.transpose():\n" << matrix_input_data.transpose() << endl;

  //test data new
  VectorXd vector_input_data_new(cNUMBER_OF_FEATURES_INPUT_DATA);
  vector_input_data_new << matrix_input_data.col(0); //1st sample or first column
//  vector_input_data_new << matrix_input_data.col(1); //2nd sample
//  vector_input_data_new << matrix_input_data.col(2); //3rd sample
//  vector_input_data_new << matrix_input_data.col(3); //4th sample
//  vector_input_data_new << matrix_input_data.col(4); //5th sample
//  vector_input_data_new << matrix_input_data.col(5); //6th sample
//  vector_input_data_new << matrix_input_data.col(6); //7th sample //challenging
//  vector_input_data_new << matrix_input_data.col(7); //8th sample
//  vector_input_data_new << matrix_input_data.col(8); //9th sample
//  vector_input_data_new << matrix_input_data.col(cNUMBER_OF_COLUMNS_INPUT_DATA - 1); //10th or last column or data sample or feature vector
//  vector_input_data_new.setRandom();
  cout << "vector_input_data_new:\n" << vector_input_data_new << endl;



  /*! 111111111111111111111111111111111111111111111111111111111111 FIRST
   * N*T = DataGeneratorFeatures resampling or bottom to top sampling data original feature space using
   * N feature values(Dimension) & T feature vectors (data samples)
   * */
  /*! mean=(I1+..............+It)/t & FI=I-mean={I1-mean,...................., It-mean}*/
  VectorXd vector_input_data_mean(cNUMBER_OF_FEATURES_INPUT_DATA);
  vector_input_data_mean.setZero();
  for (unsigned int i = 0; i < cNUMBER_OF_COLUMNS_INPUT_DATA; ++i)
  {
    vector_input_data_mean += matrix_input_data.col(i);
  }
  vector_input_data_mean = vector_input_data_mean / cNUMBER_OF_COLUMNS_INPUT_DATA;
  cout << "vector_input_data_mean:\n" << vector_input_data_mean << endl;

  //FI= I-mean= {I1-mean,...................., It-mean}
  //original input samples
  MatrixXd m_FI(cNUMBER_OF_FEATURES_INPUT_DATA, cNUMBER_OF_COLUMNS_INPUT_DATA);
  for (unsigned /*because the default is signed*/ int i = 0; i < cNUMBER_OF_COLUMNS_INPUT_DATA; ++i)
  {
    m_FI.col(i) << matrix_input_data.col(i) - vector_input_data_mean;
  }
  cout << "m_FI:\n" << m_FI << endl;

  // unittest to make sure thing working right
  VectorXd vector_input_data_mean_test(cNUMBER_OF_FEATURES_INPUT_DATA);
  vector_input_data_mean_test.setZero();
  for (unsigned /*because the default is signed*/ int i = 0; i < cNUMBER_OF_COLUMNS_INPUT_DATA; ++i)
  {
    vector_input_data_mean_test += m_FI.col(i);
  }
  cout << "vector_input_data_mean_test:  ALL should 0 the data samples are all centered \n" << vector_input_data_mean_test << endl;


  // new input sample should be examined
  VectorXd v_FI_new(cNUMBER_OF_FEATURES_INPUT_DATA);
  v_FI_new = vector_input_data_new - vector_input_data_mean;
  cout << "v_FI_new:\n" << v_FI_new << endl;


  /*! 22222222222222222222222222222222222222222222222222222222222222222 SECOND
   * 22222222222222222222222222222222222222222222222222222222222222222
   * 22222222222222222222222222222222222222222222222222222222222222222
   * EigenFeature values & vectors extraction using SVD or EVD
   *
   * Eigen feature values of every single input feature vectors (DataSamples)
   * In other word project all the feature vectors to  the eigenfeature space and
   * measure their eigenfeature values for every single input feature vector
   *
   * SVD decomposition consists in decomposing any n-by-p matrix A as a product
   * \[ A = U S V^* \]
   * where U is a n-by-n unitary,
   * V is a p-by-p unitary,
   * and S is a n-by-p real positive matrix which is zero outside of its main diagonal;
   * and the columns of U and V are known as the left and right singular vectors of A respectively.
   * */
  /*! FI.U = S.V_transposed & FI=V.S.U_transposed */
  Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::FullPivHouseholderQRPreconditioner> svd(m_FI, Eigen::ComputeFullU | Eigen:: ComputeFullV);
  MatrixXd m_V(svd.matrixU()), m_S_vector(svd.singularValues()), m_U_transposed(svd.matrixV().transpose());

  /*! normalizing singular values to use them as weights(percentage) of feature vectors in eigenfeatures space */
  cout       << "m_S_vector:\n" << m_S_vector << endl;
  MatrixXd m_S(m_S_vector.size() , m_S_vector.size());
  m_S.setZero();
  for (unsigned int i = 0; i < m_S_vector.size(); ++i)
  {
    m_S(i, i) = m_S_vector(i, 0);
  }
  cout        << "m_S:\n" << m_S << endl
              << "m_V:\n" << m_V << endl;
//              << "m_V * m_S:\n" << m_V * m_S << endl
//              << "m_V * m_S_vector:\n" << m_V * m_S_vector << endl
//          << "m_U:\n" << svd.matrixV() << endl;

//  // testing the SVD  // they were all as expected and correct
//  MatrixXd m_FI_reconst(m_V * m_S * m_U_transposed);
//  cout << "m_FI_reconst: should be equal to m_FI\n" << m_FI_reconst << endl;

  MatrixXd m_V_dot_S(m_V * m_S);
  MatrixXd m_VS_inversed(m_V_dot_S.inverse());
//  MatrixXd m_VS_inversed_dot_FI(m_VS_inversed * m_FI);
//  cout << "m_V_dot_S:\n" << m_V_dot_S << endl
//      << "m_VS_inversed:\n"<< m_VS_inversed << endl
//      << "m_VS_inversed_dot_FI: THIS IS SHOULD BE EQUAL TO m_U_transposed\n"<< m_VS_inversed_dot_FI << endl
//      << "m_U_transposed:\n"<< m_U_transposed << endl;
//
// MatrixXd m_V_inverse(m_V.inverse());
// cout << "m_V_inverse:\n"<< m_V_inverse << endl
//     << "m_V.transpose():\n"<< m_V.transpose() << endl;

  //  m_V_transposed_dot_FI = m_V_transposed_dot_FI.normalized();
  //  cout    << "m_V_transposed_dot_FI.normalized():\n" << m_V_transposed_dot_FI << endl;
  //  v_V_transposed_dot_FI_new = v_V_transposed_dot_FI_new.normalized();
  //  cout << "v_V_transposed_dot_FI_new.normalized():\n" << v_V_transposed_dot_FI_new << endl;
  //  m_S_vector = m_S_vector.normalized();
  //  cout << "m_S_vector.normalized():\n"<< m_S_vector << endl;

//  MatrixXd m_S_inversed_dot_V_transposed_dot_FI(m_S.inverse() * m_V.transpose() * m_FI);
//  cout << "m_S_inversed_dot_V_transposed_dot_FI: MUST be equal to m_VS_inversed_dot_FI\n" << m_S_inversed_dot_V_transposed_dot_FI << endl
//      << "m_VS_inversed_dot_FI:\n" << m_VS_inversed * m_FI << endl;


  /////// V.FI  without S -- projection to eigenfeatures space
  MatrixXd m_IN(matrix_input_data); // this is the projected FI
  cout  << "m_IN:\n" << m_IN << endl;
  VectorXd v_IN_new(vector_input_data_new);
  cout  << "v_IN_new:\n" << v_IN_new << endl;

  cout  << "m_FI:\n" << m_FI << endl;
  cout  << "v_FI_new:\n" << v_FI_new << endl;

  MatrixXd m_V_dot_FI(m_V * m_FI); // this is the projected FI
  cout  << "m_V_dot_FI:\n" << m_V_dot_FI << endl;
  VectorXd v_V_dot_FI_new(m_V * v_FI_new);
  cout  << "v_V_dot_FI_new:\n" << v_V_dot_FI_new << endl;

  MatrixXd m_V_transposed_dot_FI(m_V.transpose() * m_FI); // this is the projected FI
  cout  << "m_V_transposed_dot_FI:\n" << m_V_transposed_dot_FI << endl;
  VectorXd v_V_transposed_dot_FI_new(m_V.transpose() * v_FI_new);
  cout << "v_V_transposed_dot_FI_new:\n" << v_V_transposed_dot_FI_new << endl;

  MatrixXd m_VS_inversed_dot_FI(m_VS_inversed * m_FI); // this is the projected FI
  VectorXd v_VS_inversed_dot_FI_new(m_VS_inversed * v_FI_new);

  MatrixXd m_S_inversed_dot_V_transposed_dot_FI(m_S.inverse() * m_V.transpose() * m_FI);
  VectorXd v_S_inversed_dot_V_transposed_dot_FI_new(m_S.inverse() * m_V.transpose() * v_FI_new);
  cout << "m_S_inversed_dot_V_transposed_dot_FI: MUST be equal to m_VS_inversed_dot_FI\n" <<
       m_S_inversed_dot_V_transposed_dot_FI << endl
       << "m_VS_inversed_dot_FI:\n" << m_VS_inversed * m_FI << endl;
  cout << "v_VS_inversed_dot_FI_new: MUST be equal to v_S_inversed_dot_V_transposed_dot_FI_new\n" << v_VS_inversed_dot_FI_new << endl;
  cout << "v_S_inversed_dot_V_transposed_dot_FI_new:\n" << v_S_inversed_dot_V_transposed_dot_FI_new << endl;





  /*! 333333333333333333333333333333333333333333333333333333333333 THIRD
   * Recognition part starts from this line on
   * This is the part for similarity measurements
   * Similarity between two inputs are measured based on their eigen feature
   * distance & angle measure are two key method of similarity measurement
   * */

  //vector_similarity_eigenfeatures_new_vs_originals
  VectorXd v_cos_IN(cNUMBER_OF_COLUMNS_INPUT_DATA);
  VectorXd v_cos_FI(cNUMBER_OF_COLUMNS_INPUT_DATA);
  VectorXd v_cos_V(cNUMBER_OF_COLUMNS_INPUT_DATA);
  VectorXd v_cos_V_transposed(cNUMBER_OF_COLUMNS_INPUT_DATA);
  VectorXd v_cos(cNUMBER_OF_COLUMNS_INPUT_DATA);
  double sum_cos_IN = 0;
  double sum_cos_FI = 0;
  double sum_cos_V = 0;
  double sum_cos_V_transposed = 0;
  double sum_cos = 0;
  for (unsigned int i = 0; i < cNUMBER_OF_COLUMNS_INPUT_DATA; ++i)
  {
    v_cos_IN[i] = abs(v_IN_new.dot(m_IN.col(i))); // only the absolute unsigned angle between two vectors is important
    v_cos_FI[i] = abs(v_FI_new.dot(m_FI.col(i))); // only the absolute unsigned angle between two vectors is important
    v_cos_V[i] = abs(v_V_dot_FI_new.dot(m_V_dot_FI.col(i))); // only the absolute unsigned angle between two vectors is important
    v_cos_V_transposed[i] = abs(v_V_transposed_dot_FI_new.dot(m_V_transposed_dot_FI.col(i))); // only the absolute unsigned angle between two vectors is important
    v_cos[i] = abs(v_S_inversed_dot_V_transposed_dot_FI_new.dot(m_S_inversed_dot_V_transposed_dot_FI.col(i))); // only the absolute unsigned angle between two vectors is important
    sum_cos_IN += v_cos_IN[i];
    sum_cos_FI += v_cos_FI[i];
    sum_cos_V += v_cos_V[i];
    sum_cos_V_transposed += v_cos_V_transposed[i];
    sum_cos += v_cos[i];
  }
  VectorXd v_cos_percentage_IN(cNUMBER_OF_COLUMNS_INPUT_DATA);
  VectorXd v_cos_percentage_FI(cNUMBER_OF_COLUMNS_INPUT_DATA);
  VectorXd v_cos_percentage_V(cNUMBER_OF_COLUMNS_INPUT_DATA);
  VectorXd v_cos_percentage_V_transposed(cNUMBER_OF_COLUMNS_INPUT_DATA);
  VectorXd v_cos_percentage(cNUMBER_OF_COLUMNS_INPUT_DATA);
  for (unsigned int i = 0; i < cNUMBER_OF_COLUMNS_INPUT_DATA; ++i)
  {
    v_cos_percentage_IN[i] = v_cos_IN[i] / sum_cos_IN;
    v_cos_percentage_FI[i] = v_cos_FI[i] / sum_cos_FI;
    v_cos_percentage_V[i] = v_cos_V[i] / sum_cos_V;
    v_cos_percentage_V_transposed[i] = v_cos_V_transposed[i] / sum_cos_V_transposed;
    v_cos_percentage[i] = v_cos[i] / sum_cos;
  }

  // Euclidean distance to measure the similarity between two projected samples
  VectorXd dist_IN, v_dist_IN(cNUMBER_OF_COLUMNS_INPUT_DATA);
  VectorXd dist_FI, v_dist_FI(cNUMBER_OF_COLUMNS_INPUT_DATA);
  VectorXd dist_V, v_dist_V(cNUMBER_OF_COLUMNS_INPUT_DATA);
  VectorXd dist_V_transposed, v_dist_V_transposed(cNUMBER_OF_COLUMNS_INPUT_DATA);
  VectorXd dist, v_dist(cNUMBER_OF_COLUMNS_INPUT_DATA);
  for (unsigned int i = 0; i < cNUMBER_OF_COLUMNS_INPUT_DATA; ++i)
  {
    dist_IN = v_IN_new - m_IN.col(i);
    v_dist_IN[i] = dist_IN.norm();
    dist_FI = v_FI_new - m_FI.col(i);
    v_dist_FI[i] = dist_FI.norm();
    dist_V = v_V_dot_FI_new - m_V_dot_FI.col(i);
    v_dist_V[i] = dist_V.norm();
    dist_V_transposed = v_V_transposed_dot_FI_new - m_V_transposed_dot_FI.col(i);
    v_dist_V_transposed[i] = dist_V_transposed.norm();
    dist = v_S_inversed_dot_V_transposed_dot_FI_new - m_S_inversed_dot_V_transposed_dot_FI.col(i);
    v_dist[i] = dist.norm();

  }

  /*! OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO --- OUTPUT
   * Displaying the similarity measurement and final recognition results
   * Data Generation either output or similarity score
   * */

  for (unsigned int i = 0; i < cNUMBER_OF_COLUMNS_INPUT_DATA; ++i)
  {
    cout << i << "th Columnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn: cos(angle)  "
         << v_cos_percentage_IN.row(i)  << "% vs "
         << v_cos_percentage_FI.row(i)  << "% vs "
         << v_cos_percentage_V.row(i)  << "% vs "
         << v_cos_percentage_V_transposed.row(i)  << "% vs "
         << v_cos_percentage.row(i) << "%" << endl;
    cout << i << "th Columnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn: norm(distance)  "
         << v_dist_IN.row(i) <<  " vs "
         << v_dist_FI.row(i) << " vs "
         << v_dist_V.row(i)  << " vs "
         << v_dist_V_transposed.row(i)  << " vs "
         << v_dist.row(i) << "\n" << endl;

  }



  return 0; // output of main function
}
