//---------------------------------------------------
// module  : f_MDSCCT.cc                       
// auteur  : Max Mignotte                            
// date    :                                              
// langage : C++                                      
// labo    :                                  
//---------------------------------------------------

//-----------------------------------------------
// FICHIERS INCLUS ------------------------------
//-----------------------------------------------
#include "MDSCCT.h"

//------------------------------------------------
// METHODES DE LA CLASSE MD2S  -------------------
//------------------------------------------------
//----------------------------------------------------------
// Constructeur de la classe MD2S
//----------------------------------------------------------
MD2S::MD2S(int lgth,int wdth)           
 {
  //Initialisation 
  length=lgth;
  width=wdth;

  //Allocation Memoire Matrice
  Img=fmatrix_allocate_3d(TROIS,length,width);
  ImgSeg=fmatrix_allocate_2d(length,width);

  ImgC=fmatrix_allocate_3d(TROIS,length,width);
  ImgCP=fmatrix_allocate_2d(length,width);

  //Allocation Memoire Matrice Tmp 
  Imag1=fmatrix_allocate_3d(TROIS,length,width);
  Imag2=fmatrix_allocate_3d(TROIS,length,width);
  Imag3=fmatrix_allocate_3d(TROIS,length,width);
  Imag4=fmatrix_allocate_3d(TROIS,length,width);
  
  //>Gradient
  MatxGrd=fmatrix_allocate_4d(10,TROIS,length,width);

  //>StepGradient
  StGr=STEPGRADSEG;
 }

//----------------------------------------------------------
// Destructeur de la classe MD2S
//----------------------------------------------------------
MD2S::~MD2S()
{ 
  //Libere Memoire Matrice 
  if (Img)    free_fmatrix_3d(Imag1,TROIS);
  if (ImgSeg) free_fmatrix_2d(ImgSeg);

  if (ImgC)   free_fmatrix_3d(Imag1,TROIS);
  if (ImgCP)  free_fmatrix_2d(ImgSeg); 

  if (Imag1)  free_fmatrix_3d(Imag1,TROIS);
  if (Imag2)  free_fmatrix_3d(Imag2,TROIS);
  if (Imag3)  free_fmatrix_3d(Imag3,TROIS); 
  if (Imag4)  free_fmatrix_3d(Imag4,TROIS);  
}

//___________________________________
//___________________________________
//___________________________________
//___________________________________
//----------------------------------------------------------
// Segmentation
//----------------------------------------------------------    
void MD2S::MD2S_Segment()
{
  int k,l;
  char MyBuff[200];
  char MyBufb[200];
  int NbRegions;
  float complexity;
  int   SzWndHisto;
  int   NbClKm;
  
  //Record
  //======
  strcpy(MyBuff,Name_Img);
  strcat(MyBuff,Extension);  

  //>Cst
  //====     
  #define NB_FEATCOL (NB_DIVCOLHISTO-1)*CARRE(NB_DIVCOLHISTO)  
  #define NB_FEATURES NB_FEATCOL+(4*NB_GRADBINS)
  StGr=STEPGRADSEG;
  SzWndHisto=SIZEW_HISTO;
      
  //>Information
  //============
  printf(" ---------------------------------------------");
  printf("\n Image : [%s]",Name_Img);
  printf("\n Ext   : [%s]",Extension);
  printf("\n Size  : [lgth:%d | wdth:%d]",length,width);
  printf("\n ---------------------------------------------");
  printf("\n KM   : [%d]",PAR_KM);
  printf("\n ---------------------------------------------");
  fflush(stdout);

  //>Load
  //=====
  printf("\n Load Image...");
  load_image_ppm(Name_Img,Imag1,length,width);
  CopyMat(Img,Imag1,length,width,TROIS);
  CopyMat(ImgC,Imag1,length,width,TROIS);
  
  //>AllocationMemoire
  //==================
  float**   ImgSoftCont=fmatrix_allocate_2d(length,width);
  float***  MatxText=fmatrix_allocate_3d(NB_FEATURES,length,width);
  float***  ImgRed1=fmatrix_allocate_3d(TROIS,length/SCALE_SEG,width/SCALE_SEG);
  float***  ImgRed2=fmatrix_allocate_3d(TROIS,length/SCALE_SEG,width/SCALE_SEG);
  float***  ImgRed3=fmatrix_allocate_3d(TROIS,length/SCALE_SEG,width/SCALE_SEG);
  float***  Img1=fmatrix_allocate_3d(TROIS,length,width);
  float***  Img2=fmatrix_allocate_3d(TROIS,length,width);
  float***  Img3=fmatrix_allocate_3d(TROIS,length,width);
  float**   ImgPot=fmatrix_allocate_2d(length,width);
  float**   ImgPot2=fmatrix_allocate_2d(length,width);
  float**   ImgPot3=fmatrix_allocate_2d(length,width);
  float**   ImgSegCoul=fmatrix_allocate_2d(length,width);

  float ***ImgMyCont=fmatrix_allocate_3d(3,length,width);
	float** display_mat=fmatrix_allocate_2d(length,width);
	bool verbose = true;


  //>Complexity
  //===========
  complexity=Complexity(Imag1,length,width,5,7);
  printf(" ::COMPLEXITY:: [%f]\n",complexity);
  fflush(stdout);


  //======================================
  //======================================
  // [A] TEXTURE FEATURE EXTRACTION STEP    
  //======================================
  //======================================
  printf("\n\n\n===========");
  printf("\n=== [A] ===");
  printf("\n===========");
  
  //>Compute Matrice Texture
  //========================
  SEG_ComputeTextFeat(Imag1,length,width,MatxText,NB_DIVCOLHISTO,SzWndHisto,SzWndHisto);



  //>Compute SoftContours 
  //=====================
  SEG_GradMCS(Imag1,ImgSoftCont,length,width,DIST_GRAD,NB_DIVCOLHISTO+1,SzWndHisto);
  CopyMat(ImgPot3,ImgSoftCont,length,width);

  if (verbose) {
    strcpy(MyBuff,Name_Img);
    char ext [100] = ".01_SoftContMap";
    strcat(MyBuff,ext);
		SaveImagePgm(MyBuff,ImgSoftCont,length,width);
	}

   //>DEBUG
    if (0)
      { SaveImagePgm((char*)"ImgSoftCont",ImgSoftCont,length,width); 
         exit(-1); }
 

  //==============================================
  //==============================================
  // [B] MDS-BASED DIMENSIONALITY REDUCTION STEP
  //==============================================
  //==============================================
  printf("\n\n\n===========");
  printf("\n=== [B] ===");
  printf("\n===========");

  //----------------------------------
  //>COLOR-GRADIENT/CONTOUR <LowScale>
  //---------------------------------- 
    for(l=0;l<DEUX;l++) SEG_InitAleat(ImgRed1[l],length/SCALE_SEG,width/SCALE_SEG); 
    SEG_MultiResOptimizationContTex(ImgRed1,MatxText,length,width,CARRE(SIZEW_GRAPH),SIZEW_GRAPH,0,NB_FEATCOL,ImgRed1,DIM2,SCALE_SEG,NBITSEG_GRAD,NBITSEG_METR,ImgSoftCont);
    SEG_Interpolate(MatxText,length,width,0,NB_FEATCOL,ImgRed1,length/SCALE_SEG,width/SCALE_SEG,Img1,length,width,SZW_INTSEG,H_INTSEG,DIM2);

    for(l=0;l<1;l++) SEG_InitAleat(ImgRed2[l],length/SCALE_SEG,width/SCALE_SEG); 
    SEG_MultiResOptimizationContTex(ImgRed2,MatxText,length,width,CARRE(SIZEW_GRAPH),SIZEW_GRAPH,NB_FEATCOL,NB_FEATURES,ImgRed2,DIM1,SCALE_SEG,NBITSEG_GRAD,NBITSEG_METR,ImgSoftCont);
    SEG_Interpolate(MatxText,length,width,NB_FEATCOL,NB_FEATURES,ImgRed2,length/SCALE_SEG,width/SCALE_SEG,Img2,length,width,SZW_INTSEG,H_INTSEG,DIM1);

    SEG_CreateImgCoul(ImgRed1[0],ImgRed1[1],ImgRed2[0],length/SCALE_SEG,width/SCALE_SEG,ImgRed3);
    SEG_CreateImgCoul(Img1[0],Img1[1],Img2[0],length,width,Img3);


    if (verbose) {
      strcpy(MyBuff,Name_Img);
      char ext [100] = ".02_MDS_LowScaleResult";
      strcat(MyBuff,ext);
		  CopyMat(ImgMyCont, Img3,length,width, 3); 
		  //Recal(ImgMyCont,length,width);	
		  SaveImagePpm(MyBuff ,ImgMyCont,length,width);
  	}
   //=End===SCALE_SEG=====================================

   //------------------------------------
   //>COLOR-GRADIENT/CONTOUR  <FullScale>
   //------------------------------------
    SEG_MultiResOptimizContTex(Img1,MatxText,length,width,100,16,0,NB_FEATCOL,Img1,DIM2,1,110,ImgSoftCont);
    SEG_MultiResOptimizContTex(Img2,MatxText,length,width,100,16,NB_FEATCOL,NB_FEATURES,Img2,DIM1,1,110,ImgSoftCont);
    SEG_CreateImgCoul(Img1[0],Img1[1],Img2[0],length,width,Img3);
    Recal(Img3,length,width);


    if (verbose) {
      strcpy(MyBuff,Name_Img);
      char ext [100] = ".03_MDS_ResultHighScale";
      strcat(MyBuff,ext);
		  CopyMat(ImgMyCont, Img3,length,width, 3); 
		  //Recal(ImgMyCont,length,width);	
		  SaveImagePpm(MyBuff ,ImgMyCont,length,width);
  	}

     //>DEBUG
    if (0)
      { CopyMat(Imag3,Img3,length,width,TROIS);
        SaveImagePpm((char*)"ImgDeTex",Imag3,length,width); 
        Recal(Img2[0],length,width);
        SaveImagePgm((char*)"ImgDeTexGrad",Img2[0],length,width); 
        exit(-1); }
   //=End=================================================


   //=========================
   //=========================
   // [C] SEGMENTATION STEP 
   //=========================
   //=========================
   printf("\n\n\n===========");
   printf("\n=== [C] ===");
   printf("\n===========");

      //Calcul Carte SoftContour
      //========================
      SEG_ComputeColGradMap(Img3,ImgPot,length,width,DIST_GRAD);
      Recal(ImgPot,length,width);

      if (verbose) {
        strcpy(MyBuff,Name_Img);
        char ext [100] = ".04_New_SoftContMap";
        strcat(MyBuff,ext);
    		SaveImagePgm(MyBuff ,ImgPot,length,width);
    	}

      AddMatrix(ImgSoftCont,ImgPot,0.50,ImgSoftCont,length,width);
      Recal(ImgSoftCont,length,width); 

      if (verbose) {
        strcpy(MyBuff,Name_Img);
        char ext [100] = ".05_New_SoftContMap_Avgd";
        strcat(MyBuff,ext);
    		SaveImagePgm(MyBuff ,ImgSoftCont,length,width);
    	}

      //>DEBUG
      if (0)
         { SaveImagePgm((char*)"ImgSoftCont",ImgSoftCont,length,width); 
           exit(-1); }
      

      //K-mean SpatialyConstraint
      //=========================
      SEG_ComputeMapRegFromSoftCont(ImgSoftCont,ImgPot2,length,width,-1,160);

      if (verbose) {
        strcpy(MyBuff,Name_Img);
        char ext [100] = ".06_ThreshMap";
        strcat(MyBuff,ext);
    		SaveImagePgm(MyBuff ,ImgPot2,length,width);
    	}

      NbClKm=2+(int)((complexity*PAR_KM));
      if (NbClKm<2)   NbClKm=2;
      if ((NbClKm>(PAR_KM-2))) NbClKm=(PAR_KM-2);
      SEG_SegmentationCOLORKmeanFast_SpatConst(Img3,ImgSeg,length,width,NbClKm,5,0,ImgPot2);
   

      if (verbose) {
        strcpy(MyBuff,Name_Img);
        char ext [100] = ".07_SpatConstrKMeans_Result";
        strcat(MyBuff,ext);
		    CopyMat(display_mat, ImgSeg, length,width); 
		    Recal(display_mat,length,width);	
		    SaveImagePgm(MyBuff,display_mat,length,width);
		    store_contour_image(ImgMyCont, Imag1, ImgSeg, length, width); 
		    SaveImagePpm(MyBuff,ImgMyCont,length,width);
	    }

      //Cleaning
      //========
      for(k=0;k<3;k++) if (!SEG_FuseSmallRegionsMax(ImgSeg,length,width,&NbRegions,150,1.0,ImgSoftCont,150))  break;

      if (verbose) {
        strcpy(MyBuff,Name_Img);
        char ext [100] = ".08_Merged_1";
        strcat(MyBuff,ext);
		    CopyMat(display_mat, ImgSeg, length,width); 
		    Recal(display_mat,length,width);	
		    SaveImagePgm(MyBuff,display_mat,length,width);
		    store_contour_image(ImgMyCont, Imag1, ImgSeg, length, width); 
		    SaveImagePpm(MyBuff,ImgMyCont,length,width);
	    }
   
      //Class>Regions
      //============= 
      SEG_ConvertClassRegion(ImgSeg,length,width);
      Recal(ImgSeg,length,width);

   //=======================
   //=======================
   // [D] REFINEMENT STEP
   //=======================
   //=======================
   printf("\n\n\n===========");
   printf("\n=== [D] ===");
   printf("\n===========");

    //>Segmentation Texture
    //=====================
    SEG_SegHistoTEXKmeanFast(Imag1,ImgSegCoul,length,width,1+PAR_KM,NB_DIVCOLHISTO+3,SIZEW_HISTO,0,0);

    if (verbose) {
      strcpy(MyBuff,Name_Img);
      char ext [100] = ".09_Ref_OversegKMeans";
      strcat(MyBuff,ext);
	    CopyMat(display_mat, ImgSegCoul, length,width); 
	    Recal(display_mat,length,width);	
	    SaveImagePgm(MyBuff ,display_mat,length,width);
	    store_contour_image(ImgMyCont, Imag1, ImgSegCoul, length, width); 
	    SaveImagePpm(MyBuff,ImgMyCont,length,width);
	  }

    for(k=0;k<3;k++) if (!SEG_FuseSmallRegions(ImgSegCoul,length,width,&NbRegions,60))  break; 

    if (verbose) {
      strcpy(MyBuff,Name_Img);
      char ext [100] = ".10_Ref_Merged_2";
      strcat(MyBuff,ext);
	    CopyMat(display_mat, ImgSegCoul, length,width); 
	    Recal(display_mat,length,width);	
	    SaveImagePgm(MyBuff,display_mat,length,width);
	    store_contour_image(ImgMyCont, Imag1, ImgSegCoul, length, width); 
	    SaveImagePpm(MyBuff,ImgMyCont,length,width);
	  }

    SEG_ConvertClassRegion(ImgSegCoul,length,width);


    //>Contrainte-Fusion
    //==================
    SEG_ContrainWithSeg(ImgSegCoul,length,width,ImgSeg);

    if (verbose) {
      strcpy(MyBuff,Name_Img);
      char ext [100] = ".11_Ref_Fused";
      strcat(MyBuff,ext);
	    SaveImagePgm(MyBuff,ImgSeg,length,width);
	    store_contour_image(ImgMyCont, Imag1, ImgSeg, length, width); 
	    SaveImagePpm(MyBuff,ImgMyCont,length,width);
	  }

    //>Cleaning
    //=========
    for(k=0;k<3;k++) if (!SEG_FuseSmallRegionsMax(ImgSeg,length,width,&NbRegions,200,0,ImgSoftCont,130))  break; 
    printf("\n  [Regions:%d]\n\n",NbRegions);

    if (verbose) {
      strcpy(MyBuff,Name_Img);
      char ext [100] = ".12_Ref_Merged_3";
      strcat(MyBuff,ext);
	    SaveImagePgm(MyBuff,ImgSeg,length,width);
	    store_contour_image(ImgMyCont, Imag1, ImgSeg, length, width); 
	    SaveImagePpm(MyBuff,ImgMyCont,length,width);
	  }

    for(k=0;k<1;k++) SEG_MedianFilter(ImgSeg,length,width,5);    
 

    //>Save
    //======
    strcpy(MyBuff,Name_Img);
    char ext [100] = ".13_Final";
    strcat(MyBuff,ext);
    SaveImagePgm(MyBuff,ImgSeg,length,width);
    store_contour_image(ImgMyCont, Imag1, ImgSeg, length, width);
		SaveImagePpm(MyBuff,ImgMyCont,length,width);
		

   //=======================
   //=======================
   // FOR CONTOUR
   //=======================
   //=======================

    //>Binary Contour
    if (0) 
       { AddContourImg(ImgSeg,ImgC,length,width,3);
         strcat(MyBuff,"_CONT");
         SaveImagePpm(MyBuff,ImgC,length,width); }

    //>Soft Potential [TextCont+DeTextCont::ImgSoftCont]
    if (0) 
       { Recal(ImgSoftCont,length,width);
         EstimContPot(ImgSeg,ImgSoftCont,ImgCP,length,width);
         Recal(ImgCP,length,width); 
         //CopyMat(ImgCP,ImgSoftCont,length,width);
         strcpy(MyBufb,MyBuff);
         strcat(MyBufb,"_SOFTCONT");
         SaveImagePgm(MyBufb,ImgCP,length,width); }

  //>Liberation Memoire
  //===================
  if (ImgSoftCont) free_fmatrix_2d(ImgSoftCont);
  if (MatxText)    free_fmatrix_3d(MatxText,NB_FEATURES);
  if (ImgRed1)     free_fmatrix_3d(ImgRed1,TROIS); 
  if (ImgRed2)     free_fmatrix_3d(ImgRed2,TROIS); 
  if (ImgRed3)     free_fmatrix_3d(ImgRed3,TROIS);
  if (Img1)        free_fmatrix_3d(Img1,TROIS);
  if (Img2)        free_fmatrix_3d(Img2,TROIS);
  if (Img3)        free_fmatrix_3d(Img3,TROIS);
  if (ImgPot)      free_fmatrix_2d(ImgPot);
  if (ImgPot2)     free_fmatrix_2d(ImgPot2);
  if (ImgPot3)     free_fmatrix_2d(ImgPot3);
  if (ImgSegCoul)  free_fmatrix_2d(ImgSegCoul);

  free_fmatrix_3d(ImgMyCont,3);
	free_fmatrix_2d(display_mat);
}

//===============
//===============
// FOR F MEASURE
//===============
//===============
//----------------------------------------------------------
// SegmentationContour
//----------------------------------------------------------    
void MD2S::MD2S_SegmentCont()
{
  int i,j,k,l;
  char MyBuff[200];
  char MyBufb[200];
  int NbRegions;
  float complexity;
  int   SzWndHisto;
  int   NbClKm;
  int   cpt;
  
  //Record
  //======
  strcpy(MyBuff,Name_Img);
  strcat(MyBuff,Extension);  

  //>Cst
  //====     
  #define NB_FEATCOL (NB_DIVCOLHISTO-1)*CARRE(NB_DIVCOLHISTO)  
  #define NB_FEATURES NB_FEATCOL+(4*NB_GRADBINS)
  StGr=STEPGRADSEG;
  SzWndHisto=SIZEW_HISTO;
      
  //>Information
  //============
  printf(" ---------------------------------------------");
  printf("\n Image : [%s]",Name_Img);
  printf("\n Ext   : [%s]",Extension);
  printf("\n Size  : [lgth:%d | wdth:%d]",length,width);
  printf("\n ---------------------------------------------");
  printf("\n KM   : [%d]",PAR_KM);
  printf("\n ---------------------------------------------");
  fflush(stdout);

  //>Load
  //=====
  printf("\n Load Image...");
  load_image_ppm(Name_Img,Imag1,length,width);
  CopyMat(Img,Imag1,length,width,TROIS);
  CopyMat(ImgC,Imag1,length,width,TROIS);
  
  //>AllocationMemoire
  //==================
  float**   ImgSoftCont=fmatrix_allocate_2d(length,width);
  float***  MatxText=fmatrix_allocate_3d(NB_FEATURES,length,width);
  float***  ImgRed1=fmatrix_allocate_3d(TROIS,length/SCALE_SEG,width/SCALE_SEG);
  float***  ImgRed2=fmatrix_allocate_3d(TROIS,length/SCALE_SEG,width/SCALE_SEG);
  float***  ImgRed3=fmatrix_allocate_3d(TROIS,length/SCALE_SEG,width/SCALE_SEG);
  float***  Img1=fmatrix_allocate_3d(TROIS,length,width);
  float***  Img2=fmatrix_allocate_3d(TROIS,length,width);
  float***  Img3=fmatrix_allocate_3d(TROIS,length,width);
  float**   ImgPot=fmatrix_allocate_2d(length,width);
  float**   ImgPot2=fmatrix_allocate_2d(length,width);
  float**   ImgPot3=fmatrix_allocate_2d(length,width);
  float**   ImgSegCoul=fmatrix_allocate_2d(length,width);


  //>Complexity
  //===========
  complexity=Complexity(Imag1,length,width,5,7);
  printf(" ::COMPLEXITY:: [%f]\n",complexity);
  fflush(stdout);


  //======================================
  //======================================
  // [A] TEXTURE FEATURE EXTRACTION STEP    
  //======================================
  //======================================
  printf("\n\n\n===========");
  printf("\n=== [A] ===");
  printf("\n===========");
  
  //>Compute Matrice Texture
  //========================
  SEG_ComputeTextFeat(Imag1,length,width,MatxText,NB_DIVCOLHISTO,SzWndHisto,SzWndHisto);

  //>Compute SoftContours 
  //=====================
  SEG_GradMCS(Imag1,ImgSoftCont,length,width,DIST_GRAD,NB_DIVCOLHISTO+1,SzWndHisto);
  CopyMat(ImgPot3,ImgSoftCont,length,width);
 

  //==============================================
  //==============================================
  // [B] MDS-BASED DIMENSIONALITY REDUCTION STEP
  //==============================================
  //==============================================
  printf("\n\n\n===========");
  printf("\n=== [B] ===");
  printf("\n===========");

  //----------------------------------
  //>COLOR-GRADIENT/CONTOUR <LowScale>
  //---------------------------------- 
    for(l=0;l<DEUX;l++) SEG_InitAleat(ImgRed1[l],length/SCALE_SEG,width/SCALE_SEG); 
    SEG_MultiResOptimizationContTex(ImgRed1,MatxText,length,width,CARRE(SIZEW_GRAPH),SIZEW_GRAPH,0,NB_FEATCOL,ImgRed1,DIM2,SCALE_SEG,NBITSEG_GRAD,NBITSEG_METR,ImgSoftCont);
    SEG_Interpolate(MatxText,length,width,0,NB_FEATCOL,ImgRed1,length/SCALE_SEG,width/SCALE_SEG,Img1,length,width,SZW_INTSEG,H_INTSEG,DIM2);

    for(l=0;l<1;l++) SEG_InitAleat(ImgRed2[l],length/SCALE_SEG,width/SCALE_SEG); 
    SEG_MultiResOptimizationContTex(ImgRed2,MatxText,length,width,CARRE(SIZEW_GRAPH),SIZEW_GRAPH,NB_FEATCOL,NB_FEATURES,ImgRed2,DIM1,SCALE_SEG,NBITSEG_GRAD,NBITSEG_METR,ImgSoftCont);
    SEG_Interpolate(MatxText,length,width,NB_FEATCOL,NB_FEATURES,ImgRed2,length/SCALE_SEG,width/SCALE_SEG,Img2,length,width,SZW_INTSEG,H_INTSEG,DIM1);

    SEG_CreateImgCoul(ImgRed1[0],ImgRed1[1],ImgRed2[0],length/SCALE_SEG,width/SCALE_SEG,ImgRed3);
    SEG_CreateImgCoul(Img1[0],Img1[1],Img2[0],length,width,Img3);
   //=End===SCALE_SEG=====================================

   //------------------------------------
   //>COLOR-GRADIENT/CONTOUR  <FullScale>
   //------------------------------------
    SEG_MultiResOptimizContTex(Img1,MatxText,length,width,100,16,0,NB_FEATCOL,Img1,DIM2,1,110,ImgSoftCont);
    SEG_MultiResOptimizContTex(Img2,MatxText,length,width,100,16,NB_FEATCOL,NB_FEATURES,Img2,DIM1,1,110,ImgSoftCont);
    SEG_CreateImgCoul(Img1[0],Img1[1],Img2[0],length,width,Img3);
    Recal(Img3,length,width);
   //=End=================================================


   //=========================
   //=========================
   // [C] SEGMENTATION STEP 
   //=========================
   //=========================
   printf("\n\n\n===========");
   printf("\n=== [C] ===");
   printf("\n===========");

    //>Calcul Carte SoftContour
    SEG_ComputeColGradMap(Img3,ImgPot,length,width,DIST_GRAD);
    Recal(ImgPot,length,width);
    AddMatrix(ImgSoftCont,ImgPot,0.50,ImgSoftCont,length,width);
    Recal(ImgSoftCont,length,width); 

   //------------------------
   // Img3: Img a segmenter
   // ImgSoftCont : SoftContour
   // ImgSeg : Resultat
   // PAR_KM : NbClass 6-24
   //------------------------
   int CL_INIT=6; 
   int CL_END=16; 
   int NBSEG=CL_END-CL_INIT;
   float***  Vct_MatSeg=fmatrix_allocate_3d(NBSEG,length,width);
   for(k=0;k<NBSEG;k++) for(i=0;i<length;i++) for(j=0;j<width;j++) Vct_MatSeg[k][i][j]=0.0;

   int nbcla;
   for(cpt=0,nbcla=CL_INIT;nbcla<CL_END;nbcla++,cpt++)
    {
      printf("\n <<<[%d] (%d)>>>\n",cpt,nbcla);
     
      //>K-mean SpatialyConstraint
      SEG_ComputeMapRegFromSoftCont(ImgSoftCont,ImgPot2,length,width,-1,160);
      NbClKm=2+(int)((complexity*nbcla));
      if (NbClKm<2)   NbClKm=2;
      if ((NbClKm>(nbcla-2))) NbClKm=(nbcla-2);
      SEG_SegmentationCOLORKmeanFast_SpatConst(Img3,ImgSeg,length,width,NbClKm,5,0,ImgPot2);
      for(k=0;k<3;k++) if (!SEG_FuseSmallRegionsMax(ImgSeg,length,width,&NbRegions,150,1.0,ImgSoftCont,150))  break;
      SEG_ConvertClassRegion(ImgSeg,length,width);
      Recal(ImgSeg,length,width);

      //>Segmentation Texture
      SEG_SegHistoTEXKmeanFast(Imag1,ImgSegCoul,length,width,1+nbcla,NB_DIVCOLHISTO+3,SIZEW_HISTO,0,0);
      for(k=0;k<3;k++) if (!SEG_FuseSmallRegions(ImgSegCoul,length,width,&NbRegions,60))  break; 
      SEG_ConvertClassRegion(ImgSegCoul,length,width);
      SEG_ContrainWithSeg(ImgSegCoul,length,width,ImgSeg);
      for(k=0;k<3;k++) if (!SEG_FuseSmallRegionsMax(ImgSeg,length,width,&NbRegions,200,0,ImgSoftCont,130))  break; 
      printf("\n  [Regions:%d]\n\n",NbRegions);
      for(k=0;k<1;k++) SEG_MedianFilter(ImgSeg,length,width,5);  

      //>Save Contour
      EstimCont(ImgSeg,Vct_MatSeg[nbcla-CL_INIT],length,width);      
    }

   //>CONTOUR POTENTIEL
   MoySegForContPot(Vct_MatSeg,ImgCP,NBSEG,length,width);
   Recal(ImgCP,length,width); 
   strcpy(MyBufb,MyBuff);
   strcat(MyBufb,"_SOFTCONT");
   SaveImagePgm(MyBufb,ImgCP,length,width); 

   //>Segmentation
  //SaveImagePgm(MyBuff,ImgSeg,length,width);


  //>Liberation Memoire
  //===================
  if (ImgSoftCont) free_fmatrix_2d(ImgSoftCont);
  if (MatxText)    free_fmatrix_3d(MatxText,NB_FEATURES);
  if (ImgRed1)     free_fmatrix_3d(ImgRed1,TROIS); 
  if (ImgRed2)     free_fmatrix_3d(ImgRed2,TROIS); 
  if (ImgRed3)     free_fmatrix_3d(ImgRed3,TROIS);
  if (Img1)        free_fmatrix_3d(Img1,TROIS);
  if (Img2)        free_fmatrix_3d(Img2,TROIS);
  if (Img3)        free_fmatrix_3d(Img3,TROIS);
  if (ImgPot)      free_fmatrix_2d(ImgPot);
  if (ImgPot2)     free_fmatrix_2d(ImgPot2);
  if (ImgPot3)     free_fmatrix_2d(ImgPot3);
  if (ImgSegCoul)  free_fmatrix_2d(ImgSegCoul);
}

