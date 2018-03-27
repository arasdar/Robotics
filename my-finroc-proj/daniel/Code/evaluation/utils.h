// renamed it utils.h


//------------------------------------------------------
// module  : FunctionMDSCCT.h
// auteur  : Mignotte Max
// date    :
// version : 1.0
// langage : C++
// labo    : DIRO
// note    :
//------------------------------------------------------ 

//------------------------------------------------
// FICHIERS INCLUS -------------------------------
//------------------------------------------------
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <new>
#include <time.h>

//------------------------------------------------
// CONSTANTES ------------------------------------
//------------------------------------------------
#define PI  3.14159
#define TROIS 3
#define GREY_LEVEL 255

#define MILLION        1000000
#define BIGNUMBER    100000000

#define CARRE(X)   ((X)*(X))
#define MAX(a,b)   (((a)>(b))?(a):(b))
#define MIN(a,b)   (((a)>(b))?(b):(a))

//from jseg -> ioutil.h
#define I_YUV  1
#define I_RGB  2
#define I_GRAY 3
#define I_PGM  4
#define I_PPM  5
#define I_JPG  6
#define I_GIF  9
#define V_YUV  21
#define V_RGB  22

#define P_SEG 1
#define P_QUA 2
#define P_BW  3

//------------------------------------------------
// PROTOTYPE -------------------------------------
//------------------------------------------------

//>Matrix Allocation
float*         fmatrix_allocate_1d(int);
float**        fmatrix_allocate_2d(int,int);
float***       fmatrix_allocate_3d(int,int,int);
float****      fmatrix_allocate_4d(int,int,int,int);
int*           imatrix_allocate_1d(int);
int**          imatrix_allocate_2d(int,int);
int***         imatrix_allocate_3d(int,int,int);
int****        imatrix_allocate_4d(int,int,int,int);
short int*     simatrix_allocate_1d(int);
short int**    simatrix_allocate_2d(int,int);
short int***   simatrix_allocate_3d(int,int,int);
double*        dmatrix_allocate_1d(int);
double**       dmatrix_allocate_2d(int,int);

void  free_fmatrix_1d(float*);
void  free_fmatrix_2d(float**);
void  free_fmatrix_3d(float***,int);
void  free_fmatrix_4d(float****,int,int);
void  free_imatrix_1d(int*);
void  free_imatrix_2d(int**);
void  free_imatrix_3d(int***,int);
void  free_imatrix_4d(int****,int,int);
void  free_simatrix_1d(short int*);
void  free_simatrix_2d(short int**);
void  free_simatrix_3d(short int***,int);
void  free_dmatrix_1d(double*);
void  free_dmatrix_2d(double**);

//>Matrix Gestion
void  CopyMat(float***,float***,int,int,int); 
void  CopyMat(float**,float**,int,int); 
void  Recal(float**,int,int); 
void  Recal(float***,int,int);
float ComputeDiff(float**,float**,int,int);
void  AddMatrix(float**,float**,float,float**,int,int);
void  MultMatrix(float**,float,float**,int,int);
void  InitMatrix(float**,int,int);  

//>File PGM/PPM
void GetLengthWidth(char*,int*,int*);
void load_image_ppm(char*,float***,int,int); 
void load_image_pgm(char*,float**,int,int); 
void SaveImagePpm(char*,float***,int,int);
void SaveImagePgm(char*,float**,int,int);

//>Gestion Couleur
void RGBToLAB(float,float,float,float*,float*,float*);
void RGBToXYZ(float,float,float,float*,float*,float*);
void XYZToLAB(float,float,float,float*,float*,float*);
void ConvertImgRGBToLAB(float***,float***,int,int);

//>Random
float randomize(void);

//>ContourValue
float EstimPotCont(int,int,int,int,float**);

//>Complexity
float Complexity(float***,int,int,int,int);


//from jseg -> ioutil.h
void inputimgpm(char *fname,unsigned char **img,int *ny,int *nx);
void outputimgpm(char *fname,unsigned char *img,int ny,int nx,int dim);



