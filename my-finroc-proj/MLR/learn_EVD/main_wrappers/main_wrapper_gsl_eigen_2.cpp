//The following program illustrates the use of the nonsymmetric eigensolver, by computing the eigenvalues and eigenvectors of the Vandermonde matrix V(x;i,j) = x_i^{n - j} with x = (-1,-2,3,4).

#include <stdio.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_eigen.h>

int
main(void)
{
//  double data[] = { -1.0, 1.0, -1.0, 1.0,
//                    -8.0, 4.0, -2.0, 1.0,
//                    27.0, 9.0, 3.0, 1.0,
//                    64.0, 16.0, 4.0, 1.0
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

  gsl_vector_complex *eval = gsl_vector_complex_alloc(5);
  gsl_matrix_complex *evec = gsl_matrix_complex_alloc(5, 5);

  gsl_eigen_nonsymmv_workspace * w =
    gsl_eigen_nonsymmv_alloc(5);

  gsl_eigen_nonsymmv(&m.matrix, eval, evec, w);

  gsl_eigen_nonsymmv_free(w);

  gsl_eigen_nonsymmv_sort(eval, evec,
                          GSL_EIGEN_SORT_ABS_DESC);

  {
    int i, j;

    for (i = 0; i < 5; i++)
    {
      gsl_complex eval_i
        = gsl_vector_complex_get(eval, i);
      gsl_vector_complex_view evec_i
        = gsl_matrix_complex_column(evec, i);

      printf("eigenvalue = %g + %gi\n",
             GSL_REAL(eval_i), GSL_IMAG(eval_i));
      printf("eigenvector = \n");
      for (j = 0; j < 5; ++j)
      {
        gsl_complex z =
          gsl_vector_complex_get(&evec_i.vector, j);
        printf("%g + %gi\n", GSL_REAL(z), GSL_IMAG(z));
      }
    }
  }

  gsl_vector_complex_free(eval);
  gsl_matrix_complex_free(evec);

  return 0;
}
//  Here is the beginning of the output from the program,
//
//  $ ./a.out
//  eigenvalue = -6.41391 + 0i
//  eigenvector =
//  -0.0998822 + 0i
//  -0.111251 + 0i
//  0.292501 + 0i
//  0.944505 + 0i
//  eigenvalue = 5.54555 + 3.08545i
//  eigenvector =
//  -0.043487 + -0.0076308i
//  0.0642377 + -0.142127i
//  -0.515253 + 0.0405118i
//  -0.840592 + -0.00148565i
//  ...
//  This can be compared with the corresponding output from GNU OCTAVE,
//
//  octave> [v,d] = eig(vander([-1 -2 3 4]));
//  octave> diag(d)
//  ans =
//
//    -6.4139 + 0.0000i
//     5.5456 + 3.0854i
//     5.5456 - 3.0854i
//     2.3228 + 0.0000i
//
//  octave> v
//  v =
//
//   Columns 1 through 3:
//
//    -0.09988 + 0.00000i  -0.04350 - 0.00755i  -0.04350 + 0.00755i
//    -0.11125 + 0.00000i   0.06399 - 0.14224i   0.06399 + 0.14224i
//     0.29250 + 0.00000i  -0.51518 + 0.04142i  -0.51518 - 0.04142i
//     0.94451 + 0.00000i  -0.84059 + 0.00000i  -0.84059 - 0.00000i
//
//   Column 4:
//
//    -0.14493 + 0.00000i
//     0.35660 + 0.00000i
//     0.91937 + 0.00000i
//     0.08118 + 0.00000i

//Note that the eigenvectors corresponding to the eigenvalue 5.54555 + 3.08545i differ by the multiplicative constant 0.9999984 + 0.0017674i which is an arbitrary phase factor of magnitude 1.
