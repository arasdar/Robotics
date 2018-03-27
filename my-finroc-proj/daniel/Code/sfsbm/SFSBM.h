//---------------------------------------------------
// module   : SFSBM.h                         
// author   : Max Mignotte                            
// date     :                                              
// language : C++                                      
// lab      :                                  
//---------------------------------------------------

#ifndef SFSBM_H
#define SFSBM_H

//------------------------------------------------
// INCLUDED FILES --------------------------------
//------------------------------------------------
#include "FunctionSFSBM.h"

//------------------------------------------------
// DEFINITIONS -----------------------------------
//------------------------------------------------
#define NCHAR 200
#define TROIS   3

#define SIZE_WINDOW         7
#define SIZE_BIGWINDOW     80
#define SIZE_SMALLREGIONS 130
#define NBDIV               5
#define SCALE               4

//------------------------------------------------
// CLASSE SFSBM ----------------------------------
//------------------------------------------------
class SFSBM
{
  //--Data-----
  //-----------
  public:

  //Matrix Img
  int***    mat_img;
  float**   mat_seg; 
  float**   mat_rest;

  float**  mat1;
  float**  mat2;
  float**  mat3;
  float**  mat4;
  float**  mat_tmp0;
  float**  mat_tmp1;

  //Par. Img
  int width,length;

  //Parameters
  float Beta;
  float Xi;

  int   NbCl0;
  int   Size1,Size2;
  int   NbBins;
  int   Seed;

  //Par. Segmentation
  int   NbRegions;
 
  //Flag
  int  flag_save;
  int  flag_visu;

  //Util
  char Name_Sauve[NCHAR];
  char Name_Img[NCHAR];
  char Extension[NCHAR];  

  //--Methods---
  //------------
  public:
  SFSBM(int***,int,int);
  ~SFSBM();
       
  void SegmentFromSoftBoundaryMap();
  void Segment();
 
  void  SegmentationHistoCOLORKmeanFast(int***,float**,int,int,int,int,int,int,int); 
};

#endif 
