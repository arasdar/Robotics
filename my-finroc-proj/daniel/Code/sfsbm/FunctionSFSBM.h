//------------------------------------------------------
// module  : FunctionPRIF.h
// auteur  : Mignotte Max
// date    :
// version : 1.0
// langage : C++
// labo    : DIRO
// note    :
//------------------------------------------------------
// quelques fonctions  

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
#include <sys/resource.h>


//------------------------------------------------
// CONSTANTES & DEFINITIONS ----------------------
//------------------------------------------------
#define PI  3.14159
#define GREY_LEVEL 255
#define BIGNUMBER 100000000

#define SQUARE(a)  ((a)*(a))
#define CARRE(X) ((X)*(X))
#define MAX(a,b)   (((a)>(b))?(a):(b))
#define MIN(a,b)   (((a)>(b))?(b):(a))

//------------------------------------------------
// PROTOTYPE -------------------------------------
//------------------------------------------------

//>Matrix Allocation
float*    fmatrix_allocate_1d(int);
float**   fmatrix_allocate_2d(int,int);
float***  fmatrix_allocate_3d(int,int,int);

int*      imatrix_allocate_1d(int);
int**     imatrix_allocate_2d(int,int);
int***    imatrix_allocate_3d(int,int,int);

double*   dmatrix_allocate_1d(int);
double**  dmatrix_allocate_2d(int,int);

void  free_fmatrix_1d(float*);
void  free_fmatrix_2d(float**);
void  free_fmatrix_3d(float***,int);

void  free_imatrix_1d(int*);
void  free_imatrix_2d(int**);
void  free_imatrix_3d(int***,int);

void free_dmatrix_1d(double*);
void free_dmatrix_2d(double**);

//>Matrix Gestion
void copy_mat(float**,float**,int,int);
void Recal(float**,int,int); 
void Zeros(float**,int,int);
int  DiffMat(float**,float**,int,int);
void AddTwoMatrix(float**,float**,float**,int,int,float);

//>Random
float randomize(void);
float random_flo(float);

//>Load/Save File
void GetLengthWidth(char*,int*,int*);
void load_image_ppm(char*,int***,int,int);
void SaveImagePgm(char*,char*,float**,int,int);
void SaveImagePpm(char*,char*,int***,int,int); 

//>Space Color
void RGBToHSV(float,float,float,float*,float*,float*);
void RGBToYIQ(float,float,float,float*,float*,float*);
void RGBToXYZ(float,float,float,float*,float*,float*);
void XYZToRGB(float,float,float,float*,float*,float*);
void RGBToLAB(float,float,float,float*,float*,float*);
void RGBToLUV(float,float,float,float*,float*,float*);
void RGBToI1I2I3(float,float,float,float*,float*,float*);
void RGBToH1H2H3(float,float,float,float*,float*,float*);
void RGBToHSL(float,float,float,float*,float*,float*);
void RGBToTSL(float,float,float,float*,float*,float*);
void RGBToYCbCr(float,float,float,float*,float*,float*);
void RGBToP1P2(float,float,float,float*,float*,float*);

void ColorBase(int***,int***,int,int,int);

//>Kmean
int Identical(int*,int**,int,int);
void KmeanFast(int**,int,int,int,int*,int,int);

//>FusionRegions
void Paint(float**,float**,int,int,int,int,int);
void Paint2(float**,float**,int,int,int,int,int);
void ConvertClassRegion(float**,int,int);
void ConvertClassRegionInf(float**,int,int);
int FuseSmallRegions(float**,int,int,int*,int);

//>VisuContour
void AddContourImg(float**,int***,int,int,int);

//> ContourSeg. & SoftContour
void ComputeContour(float**,float**,int,int);
void GradFoncText(int***,float**,float**,int,int,int);
void GradFoncText(int***,float**,float**,int,int,int,int);
void Cany(int***,float**,int,int,float,float,int,int,int);
void follow(float**,int,int,float**,float**,int,int,float);
void GradFoncTextCS(int***,float**,float**,int,int,int,int);
void Grad(int***,float**,float**,int,int,int);
void AddContour(float**,float**,int,int,int);
void Thicken(int***,float**,float**,int,int);

//> SFSBM Fonctions
float EstimGrad(int,int,int,int,float**);
void multiresMax(float**,int,int,int,float**);
void multires(int***,int,int,int,int***);
void MultGrad2(int***,float**,int,int,float);
void ContrainWithSeg(float**,int,int,float**);
void Convert320x214(float**,float**,int*,int*);
void Convert481x321(float**,float**,int*,int*);
float Complexity(int***,int,int,int,int);

//> ICM SFSBM
void MarkovSFSBM(float**,float**,int,float**,int,int,int,int,float,float,float**,float**, int***, char*);

//>> Fuse Big Regions
int  FuseBigRegion(float**,float**,int,int,int,float);

// --------- My additions ------------
//
//
void store_contour_image(int ***ImgMyCont, int ***ImgC, float **ImgSeg, int length, int width);
void draw_edges(int *RGB0,int *rmap, int *RGB,
    int ny,int nx,int dim,float displayintensity);
//
//
// ------ My additions ---------


