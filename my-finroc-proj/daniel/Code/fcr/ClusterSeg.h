//---------------------------------------------------
// module  : ClusterSeg.h                         
// auteur  : Max Mignotte                            
// date    :                                              
// langage : C++                                      
// labo    :                                  
//---------------------------------------------------

#ifndef CLUSTERSEG_H
#define CLUSTERSEG_H

//------------------------------------------------
// FICHIERS INCLUS -------------------------------
//------------------------------------------------
#include "FunctionClusterSeg.h"

//------------------------------------------------
// DEFINITIONS -----------------------------------
//------------------------------------------------
#define NB_ESPACE_COUL 6 
#define NCHAR 200

//------------------------------------------------
// CLASSE CLUSTERSEG -----------------------------
//------------------------------------------------
class ClusterSeg
{
  //--Donnees--
  //-----------
  public:

  //Matrices Img
  int***    mat_img;
  int***    mat_imgc;
  float***  smat_seg;
  float**   mat_seg; 
  
  //Windows
  int** win_tabseg;
  int SizeS;

  //ParamIMG
  int width,length;

  //Parametre.
  int   NbCl1,NbCl2;
  float DistBhatta;

  int   Seed;
  int   Size1,Size2;
  int   NbBins;
  int   SizeMin;
 
  //Flag
  int  flag_save;
  int  flag_silent;
  int  flag_visu;

  //Utile
  char Name_Sauve[NCHAR];
  char Name_Img[NCHAR];
  char Extension[NCHAR];  

  //--Methodes--
  //------------
  public:
  ClusterSeg(int***,int,int);
  ~ClusterSeg();

  //Function
  void ComputeWindows(int,int,int);
       
  void Segmente();
 
  //-------------------------------------------------
  //Algorithm
  //-------------------------------------------------
  //Segmentation <Histo><COLOR-BASE>
  void  SegmentationHistoCOLORKmean(int***,float**,int,int,int,int,int,int,int); 

  //SegmentationFusionKmean <Vect><Label-SzWVoisin->
  void  SegmentationVectLabel1Kmean(float**,float***,int,int,int,int,int,int,int);
};

#endif 
