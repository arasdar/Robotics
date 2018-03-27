//The following program computes the eigenvalues and eigenvectors of the 4-th order Hilbert matrix, H(i,j) = 1/(i + j + 1).

#include <stdio.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_eigen.h>

int
main(void)
{
//  double data[] = { 1.0  , 1 / 2.0, 1 / 3.0, 1 / 4.0,
//                    1 / 2.0, 1 / 3.0, 1 / 4.0, 1 / 5.0,
//                    1 / 3.0, 1 / 4.0, 1 / 5.0, 1 / 6.0,
//                    1 / 4.0, 1 / 5.0, 1 / 6.0, 1 / 7.0
//                  };
//
//  gsl_matrix_view m
//    = gsl_matrix_view_array(data, 4, 4);

  ///////////////////////////////////////////////////////////// DOUBLE
  double a[] =
  {
    1.96 , -6.49 , -0.47 , -7.20 , -0.65,
    -6.49 ,  3.80  , -6.39  , 1.50 , -6.34,
    -0.47 , -6.39 ,  4.17 , -1.51 ,  2.67,
    -7.20 ,  1.50 , -1.51 ,  5.70 ,  1.80,
    -0.65 , -6.34 ,  2.67 ,  1.80 , -7.10
  };
  gsl_matrix_view m
    = gsl_matrix_view_array(a, 5, 5);

  gsl_vector *eval = gsl_vector_alloc(5);
  gsl_matrix *evec = gsl_matrix_alloc(5, 5);

  gsl_eigen_symmv_workspace * w =
    gsl_eigen_symmv_alloc(5);

  gsl_eigen_symmv(&m.matrix, eval, evec, w);

  gsl_eigen_symmv_free(w);

  gsl_eigen_symmv_sort(eval, evec,
                       GSL_EIGEN_SORT_ABS_ASC);

  {
    int i;

    for (i = 0; i < 5; i++)
    {
      double eval_i
        = gsl_vector_get(eval, i);
      gsl_vector_view evec_i
        = gsl_matrix_column(evec, i);

      printf("eigenvalue = %g\n", eval_i);
      printf("eigenvector = \n");
      gsl_vector_fprintf(stdout,
                         &evec_i.vector, "%g");
    }
  }

  gsl_vector_free(eval);
  gsl_matrix_free(evec);

  return 0;
}
//  Here is the beginning of the output from the program,
//
//  $ ./a.out
//  eigenvalue = 9.67023e-05
//  eigenvector =
//  -0.0291933
//  0.328712
//  -0.791411
//  0.514553
//  ...
//  This can be compared with the corresponding output from GNU OCTAVE,
//
//  octave> [v,d] = eig(hilb(4));
//  octave> diag(d)
//  ans =
//
//     9.6702e-05
//     6.7383e-03
//     1.6914e-01
//     1.5002e+00
//
//  octave> v
//  v =
//
//     0.029193   0.179186  -0.582076   0.792608
//    -0.328712  -0.741918   0.370502   0.451923
//     0.791411   0.100228   0.509579   0.322416
//    -0.514553   0.638283   0.514048   0.252161
//  Note that the eigenvectors can differ by a change of sign, since the sign of an eigenvector is arbitrary.
