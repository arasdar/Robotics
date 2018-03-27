//------------------------------------------------------
// module  : FunctionClusterSeg.h
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
float**** fmatrix_allocate_4d(int,int,int,int);

int*      imatrix_allocate_1d(int);
int**     imatrix_allocate_2d(int,int);
int***    imatrix_allocate_3d(int,int,int);
int****   imatrix_allocate_4d(int,int,int,int);

void  free_fmatrix_1d(float*);
void  free_fmatrix_2d(float**);
void  free_fmatrix_3d(float***,int);
void  free_fmatrix_4d(float****,int,int);

void  free_imatrix_1d(int*);
void  free_imatrix_2d(int**);
void  free_imatrix_3d(int***,int);
void  free_imatrix_4d(int****,int,int);

//>Matrix Gestion
void  copy_mat(int***,int***,int,int,int); 
void  copy_matrix(float**,float**,int);
void  Recal(float**,int,int); 

//>Random
float randomize(void);

//>File
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
void ColorBase(int***,int***,int,int,int);

//>Kmean
int Identical(float*,float**,int,int);
void Kmean(float**,int,int,int,int*,int,int);;

//>Fusion
void Paint(float**,int**,int,int,int,int,int);
void Paint(float**,float**,int,int,int,int,int);
void ConvertClassRegion(float**,int,int);
void FuseSmallRegions(float**,int,int,int);
int FuseRegionBhattaharyaMIN(float**,int***,int,int,float,int,float);
float DistMinHist_(int***,int,int,float**,int,float*,int,int,float*);

//>VisuContour
void AddContourImg(float**,int***,int,int,int);

// --------- My additions ------------
//
//
void store_contour_image(int ***ImgMyCont, int ***ImgC, float **ImgSeg, int length, int width);
void draw_edges(int *RGB0,int *rmap, int *RGB,
    int ny,int nx,int dim,float displayintensity);
//
//
// ------ My additions ---------

