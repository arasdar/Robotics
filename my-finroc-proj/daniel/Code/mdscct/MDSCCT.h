//---------------------------------------------------
// module  : MDSCCT.h                         
// auteur  : Max Mignotte                            
// date    :                                              
// langage : C++                                      
// labo    :                                  
//---------------------------------------------------

#ifndef MDSCCT_H
#define MDScct_H

//------------------------------------------------
// FICHIERS INCLUS -------------------------------
//------------------------------------------------
#include "FunctionMDSCCT.h"

//------------------------------------------------
// DEFINITIONS -----------------------------------
//------------------------------------------------

//>CONSTANTS
//==========

#define NCHAR 200

#define UN     1
#define DEUX   2
#define TROIS  3
#define QUATRE 4
#define CINQ   5

#define DIM1   1
#define DIM2   2
  
#define NBBIN_IMG 256 


//>PARAMETERS_MODEL
//=================
#define NB_GRADBINS      10   
#define SIZEW_HIGRD      7
#define NB_DIVCOLHISTO   4  
#define SIZEW_HISTO      7
#define DIST_GRAD        3

#define SIZEW_GRAPH     15      
#define SCALE_SEG       16      

#define NBITSEG_GRAD   100 
#define NBITSEG_METR   150             
#define STEPGRADSEG      0.00002 
#define TEMPSEG_INIT   200.0      
#define TEMPSEG_FINAL    1.0
#define RADIUS_SEG      10        

#define SZW_INTSEG       5
#define H_INTSEG         0.1
#define FACT_CPOT        0.25 



//>PARAMETERS_SEGMENTATION
//========================
#define Tbh   0.1  
#define KM_  10    
 
  
//------------------------------------------------
// CLASSE MD2S -----------------------------------
//------------------------------------------------
class MD2S
{
  //--Donnees--
  //-----------
  public:

  //>MtxResult
  float***  Img;
  float**   ImgSeg; 

  float***  ImgC;
  float**   ImgCP;

  float***  Imag1;
  float***  Imag2;
  float***  Imag3;
  float***  Imag4;
  
  //>Gradient
  float**** MatxGrd;

  //>Parameter
  //==========
  char  Name_Img[NCHAR];
  char  Extension[NCHAR]; 
  int   length,width; 
  
 //>Gradient        
  float StGr;                
  float NrjImg,NrjImgInit;

  //>Flag
  int  flag_save;
  int  flag_visu; 

  //>ParameterSEG
  //float RegulSeg;
  int   PAR_KM; 
  
  public:
  MD2S(int,int);    
  ~MD2S();

  void MD2S_Segment();
  void MD2S_SegmentCont();
  

 //=================================
 // File>f_MDSCCT_2.cc
 //=================================
 //-------
 //GESTION
 //-------
 void  SEG_CreateImgCoul(float**,float**,float**,int,int,float***);
 void  SEG_InitAleat(float**,int,int);
 float SEG_DiffMat(float**,float**,int,int,float);

 //------
 //>MODEL 
 //------
 void SEG_ComputeTextFeat(float***,int,int,float***,int,int,int);
 void SEG_ComputeTextFeat_(float***,int,int,float***,int,int,int);
 void SEG_ComputeMultiresNonLocalGraphWindow(float***,int,int,int,int,short int***,int,int,int,float**);

 //--------------
 //>INTERPOLATION 
 //--------------
 void  SEG_Interpolate(float***,int,int,int,int,float**,int,int,float**,int,int,int,float);
 void  SEG_Interpolate(float***,int,int,int,int,float***,int,int,float***,int,int,int,float,int);

 //-------------
 //>OPTIMISATION 
 //-------------
 void  SEG_GGMRFDerive(float**,float**,int,int,short int***,int);
 float SEG_ComputeNrjDim(float***,int,int,int,short int***,int);
 float SEG_ComputeBetaDim(float***,float***,int,int,int);
 void  SEG_ConjGradDescDim(float***,float***,int,int,int,short int***,int,int);
 float SEG_ComputeLocNrjDim(float***,int,int,int,float*,int,short int***,int);
 void  SEG_SimulatedAnnealingDim(float***,float***,int,int,int,short int***,int,int,float,float);
 void  SEG_MultiResOptimizationContTex(float***,float***,int,int,int,int,int,int,float***,int,int,int,int,float**);
 void  SEG_MultiResOptimizContTex(float***,float***,int,int,int,int,int,int,float***,int,int,int,float**);

 //-----------
 //>CLUSTERING
 //-----------
 void SEG_Paint(float**,float**,int,int,int,int,int);
 void SEG_Paint(float**,int**,int,int,int,int,int);
 int  SEG_Identical(int*,int**,int,int); 
 void SEG_SegmentationCOLORKmeanFast(float***,float**,int,int,int,int,int);
 void SEG_KmeanFast(int**,int,int,int,int*,int,int);

 void SEG_ComputeMapRegFromSoftCont(float**,float**,int,int,int,int);
 void SEG_ContrainWithSeg(float**,int,int,float**);
 void SEG_SegmentationCOLORKmeanFast_SpatConst(float***,float**,int,int,int,int,int,float**);
 void SEG_KmeanFast_SpatConst(int**,int,int,int,int*,int,int,float**,int,int);

 void SEG_ConvertClassRegion(float**,int,int);
 void SEG_ConvertCertainClassRegionInf(float**,int,int,int*,int);
 
 //---------
 //>CLEANING 
 //---------
 int  SEG_FuseSmallRegionsMax(float**,int,int,int*,int,float,float**,float);
 int  SEG_FuseSmallRegions(float**,int,int,int*,int);
 void SEG_MedianFilter(float**,int,int,int);

 //--------
 //>CONTOUR
 //--------
 void SEG_ComputeColGradMap(float***,float**,int,int,int);
 void SEG_GradMCS(float***,float**,int,int,int,int,int);

 //------------
 //>SPACE COLOR
 //------------
 void RGBToHSV(float,float,float,float*,float*,float*);
 void RGBToYIQ(float,float,float,float*,float*,float*);
 void RGBToXYZ(float,float,float,float*,float*,float*);
 void RGBToLAB(float,float,float,float*,float*,float*);
 void RGBToLUV(float,float,float,float*,float*,float*);
 void RGBToI1I2I3(float,float,float,float*,float*,float*);
 void RGBToH1H2H3(float,float,float,float*,float*,float*);
 void RGBToHSL(float,float,float,float*,float*,float*);
 void RGBToTSL(float,float,float,float*,float*,float*);
 void RGBToYCbCr(float,float,float,float*,float*,float*);
 void RGBToP1P2(float,float,float,float*,float*,float*);
 void ColorBase(int***,float***,int,int,int);

 //------------
 //>SEG TEXTURE
 //------------
 void SEG_SegHistoTEXKmeanFast(float***,float**,int,int,int,int,int,int,int);

 //------------
 //>CONTOUR
 //------------
 void AddContourImg(float**,float***,int,int,int); 
 void EstimContPot(float**,float**,float**,int,int); 
 void EstimCont(float**,float**,int,int);

 //------------
 //>ESSAI
 //------------
 void MoySegForContPot(float***,float**,int,int,int);
 
};

#endif 
