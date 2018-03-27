#include <Eigen/Dense>
#include <iostream>
#include <opencv2/core/eigen.hpp> //cv2eigen and reverse

using namespace std;
using namespace Eigen;


int main(int argc, char** argv)
{

  {
    double a[5][5] =
    {
      { 1.96 , -6.49, -0.47, -7.20, -0.65},
      { -6.49,  3.80, -6.39,  1.50, -6.34},
      { -0.47, -6.39,  4.17, -1.51,  2.67},
      { -7.20,  1.50, -1.51,  5.70,  1.80},
      { -0.65, -6.34,  2.67,  1.80, -7.10}
    };
    // convert into OpenCV representation
    cv::Mat I = cv::Mat(5 /*rows*/, 5 /*cols*/, CV_64FC1 /*DOUBLE precision*/, a).clone(); // this is transposed

    // eigen2cv
    MatrixXd m_FI;
    cv::cv2eigen(I, m_FI);

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
    cout        << "m_FI:\n" << m_FI << endl
                << "m_S:\n" << m_S << endl
                << "m_V:\n" << m_V << endl;
  }

  return 0; // output of main function
}
