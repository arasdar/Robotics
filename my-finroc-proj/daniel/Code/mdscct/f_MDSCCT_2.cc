//---------------------------------------------------
// module  : f_MD2SCCT_2.cc                       
// auteur  : Max Mignotte                            
// date    :                                              
// langage : C++                                      
// labo    :                                  
//---------------------------------------------------
//
//-----------------------------------------------
// FICHIERS INCLUS ------------------------------
//-----------------------------------------------
#include "MDSCCT.h"


//-----------------------------------------------
//-----------------------------------------------
// GESTION --------------------------------------
//-----------------------------------------------
//-----------------------------------------------
//----------------------------------------------------------
// SEG >> SEG_CreateImgCoul
//---------------------------------------------------------- 
void MD2S::SEG_CreateImgCoul(float** Img0,float** Img1,float** Img2,int lgth,int wdth,float*** ImgOut)
{
 int i,j;

 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) ImgOut[0][i][j]=Img0[i][j];
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) ImgOut[1][i][j]=Img1[i][j];
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) ImgOut[2][i][j]=Img2[i][j];
}

//----------------------------------------------------------
// SEG >> SEG_InitAleat
//---------------------------------------------------------- 
void MD2S::SEG_InitAleat(float** Img,int lgth,int wdth)
{
 int i,j;

 //>Boucle
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  
    Img[i][j]=(int)(randomize()*(NBBIN_IMG-1)); 
}

//----------------------------------------------------------
// calcule le nombre de differences a chaque iteration
//----------------------------------------------------------
float MD2S::SEG_DiffMat(float** pmatl1,float** pmatl2,int lgth,int wdth,float precs)
{
 int i,j;
 float chg;

 //>Compte NbChgt
 chg=0.0;
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) if (fabs(pmatl1[i][j]-pmatl2[i][j])>precs) chg++; 
 chg/=(lgth*wdth);
 chg*=100.0;
       
 return chg;
}

//=========================================================================
//=========================================================================
//>MODELE       ===========================================================
//=========================================================================
//----------------------------------------------------------
// SEG >> SEG_ComputeTextFeat
//----------------------------------------------------------
//
//  Output>MatxText[NB_FEAT][lgth][wdth]
//                  NB_FEAT=(div*div*div)+(25*4)
//
//----------------------------------------------------------     
void MD2S::SEG_ComputeTextFeat(float*** img,int lgth,int wdth,float*** MatxText,int div,int SzW,int SzWH)
{
  int i,j,k,l;
  int posr,posc;
  int NbParamColorHisto,NbParamFin;
  float cpt; 
  float lab;
  float divis;
  float divisL; 

  //Init. Parametres
  NbParamColorHisto=div*div*(div-1); 
  NbParamFin=NbParamColorHisto+(4*NB_GRADBINS);; 
  divis=(256.0/(float)div);
  divisL=(256.0/(float)(div-1)); 

  //Presentation
  printf("\n >> Calcul Texture Features [Length:Width:%d:%d]",lgth,wdth);
  printf("[Div=%d][SzW=%d][SzWH=%d]",div,SzW,SzWH);
  fflush(stdout);
  
  //Init. Memoire
  float***  SquWin=fmatrix_allocate_3d(QUATRE,SzW,SzW);
  float***  SquWinH=fmatrix_allocate_3d(QUATRE,SzWH,SzWH);
  float**   MatxNB=fmatrix_allocate_2d(lgth,wdth);
  float**   MatxHor=fmatrix_allocate_2d(lgth,wdth);
  float**   MatxVer=fmatrix_allocate_2d(lgth,wdth);
  float**   MatxDgd=fmatrix_allocate_2d(lgth,wdth);
  float**   MatxDgg=fmatrix_allocate_2d(lgth,wdth);
  float***  ImgLAB=fmatrix_allocate_3d(TROIS,lgth,wdth);

  //Init. Matrix    
  for(k=0;k<NbParamFin;k++) for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) MatxText[k][i][j]=0.0;
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) 
     MatxNB[i][j]=(0.33*img[0][i][j])+(0.33*img[1][i][j])+(0.33*img[2][i][j]);

  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)
    { MatxHor[i][j]=0.0;
      MatxVer[i][j]=0.0;
      MatxDgd[i][j]=0.0; 
      MatxDgg[i][j]=0.0; }

  for(i=1;i<(lgth-1);i++) for(j=1;j<(wdth-1);j++) MatxHor[i][j]=fabs(MatxNB[i][j-1]-MatxNB[i][j+1]);
  for(i=1;i<(lgth-1);i++) for(j=1;j<(wdth-1);j++) MatxVer[i][j]=fabs(MatxNB[i-1][j]-MatxNB[i+1][j]);
  for(i=1;i<(lgth-1);i++) for(j=1;j<(wdth-1);j++) MatxDgd[i][j]=0.0; //fabs(MatxNB[i-1][j-1]-MatxNB[i+1][j+1]);
  for(i=1;i<(lgth-1);i++) for(j=1;j<(wdth-1);j++) MatxDgg[i][j]=0.0; //fabs(MatxNB[i-1][j+1]-MatxNB[i+1][j-1]);

  //Conversion RGB=>LAB
  ConvertImgRGBToLAB(img,ImgLAB,lgth,wdth);
  for(k=0;k<TROIS;k++) for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) img[k][i][j]=ImgLAB[k][i][j];
  for(k=0;k<TROIS;k++) Recal(img[k],lgth,wdth);

 //=============================================================

      //------------------------------------
      //--- FEATURES COLOR HISTO -----------
      //------------------------------------
      for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  
        {
         for(k=0;k<SzW;k++) for(l=0;l<SzW;l++) 
            { posr=i-(SzW/2)+k;
              posc=j-(SzW/2)+l;

              if (posr<0)           posr+=lgth;
              if (posr>(lgth-1))    posr-=lgth;  
              if (posc<0)           posc+=wdth;
              if (posc>(wdth-1))    posc-=wdth;

              SquWin[0][k][l]=img[0][posr][posc];
              SquWin[1][k][l]=img[1][posr][posc];
              SquWin[2][k][l]=img[2][posr][posc]; }
   

         //LAB
         for(k=0;k<SzW;k++) for(l=0;l<SzW;l++)  
	    { lab=((int)(SquWin[0][k][l]/divisL))*(div*div);
              lab+=((int)(SquWin[1][k][l]/divis))*(div);  
              lab+=((int)(SquWin[2][k][l]/divis));
   
              MatxText[(int)(lab)][i][j]++; }

	}//-End-Features-Color-Histo--------
           

      //------------------------------------
      //--- FEATURES GRAD HISTO ------------
      //------------------------------------
      cpt=0.0;
       for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  
        {
         for(k=0;k<SzWH;k++) for(l=0;l<SzWH;l++) 
            { posr=i-(SzWH/2)+k;
              posc=j-(SzWH/2)+l;

              if (posr<0)           posr+=lgth;
              if (posr>(lgth-1))    posr-=lgth;  
              if (posc<0)           posc+=wdth;
              if (posc>(wdth-1))    posc-=wdth;

              SquWinH[0][k][l]=MatxHor[posr][posc]; 
              SquWinH[1][k][l]=MatxVer[posr][posc];        
              SquWinH[2][k][l]=MatxDgd[posr][posc];       
              SquWinH[3][k][l]=MatxDgg[posr][posc];  }

              for(k=0;k<SzWH;k++) for(l=0;l<SzWH;l++)  
	        { lab=SquWinH[0][k][l]+0.5;
                  if (lab>NB_GRADBINS) { lab=NB_GRADBINS; cpt++; }
                  MatxText[NbParamColorHisto+(int)(lab)][i][j]++; }

              for(k=0;k<SzWH;k++) for(l=0;l<SzWH;l++)  
	        { lab=SquWinH[1][k][l]+0.5;
                  if (lab>NB_GRADBINS) {  lab=NB_GRADBINS; cpt++; } 
                  MatxText[NbParamColorHisto+NB_GRADBINS+(int)(lab)][i][j]++; }

              for(k=0;k<SzWH;k++) for(l=0;l<SzWH;l++)  
	        { lab=SquWinH[2][k][l]+0.5;
                  if (lab>NB_GRADBINS) { lab=NB_GRADBINS; cpt++; } 
                  MatxText[NbParamColorHisto+(2*NB_GRADBINS)+(int)(lab)][i][j]++; }

              for(k=0;k<SzWH;k++) for(l=0;l<SzWH;l++)  
	        { lab=SquWinH[3][k][l]+0.5;
                  if (lab>NB_GRADBINS) { lab=NB_GRADBINS-1; cpt++; } 
                  MatxText[NbParamColorHisto+(3*NB_GRADBINS)+(int)(lab)][i][j]++; }

        }//-End-Features-Grad-Histo-------- 

  //---------
  //__INFO___
  //---------
  cpt=100*cpt/(4*CARRE(SzWH)*lgth*wdth);
  printf("\n    [%.2f/100 Gradient>%d]\n",cpt,NB_GRADBINS);
  fflush(stdout);

  //===========================================================

  //Liberation Memoire
  if (SquWin)  free_fmatrix_3d(SquWin,QUATRE);
  if (SquWinH) free_fmatrix_3d(SquWinH,QUATRE);
  if (MatxNB)  free_fmatrix_2d(MatxNB);
  if (MatxHor) free_fmatrix_2d(MatxHor);
  if (MatxVer) free_fmatrix_2d(MatxVer);
  if (MatxDgd) free_fmatrix_2d(MatxDgd);
  if (MatxDgg) free_fmatrix_2d(MatxDgg);
  if (ImgLAB)  free_fmatrix_3d(ImgLAB,TROIS);
 }

//----------------------------------------------------------
// SEG >> SEG_ComputeTextFeat
//----------------------------------------------------------
//
//  Output>MatxText[NB_FEAT][lgth][wdth]
//                  NB_FEAT=(div*div*div)+(25*4)
//
//----------------------------------------------------------     
void MD2S::SEG_ComputeTextFeat_(float*** img,int lgth,int wdth,float*** MatxText,int div,int SzW,int SzWH)
{
  int i,j,k,l;
  int posr,posc;
  int NbParamColorHisto,NbParamFin;
  float cpt; 
  float lab;
  float divis;
  float divisL; 

  //Init. Parametres
  NbParamColorHisto=div*div*(div-1); 
  NbParamFin=NbParamColorHisto+(4*NB_GRADBINS);
  divis=(256.0/(float)div);
  divisL=(256.0/(float)(div-1)); 

  //Presentation
  printf("\n >> Calcul Texture Features [Length:Width:%d:%d]",lgth,wdth);
  printf("[Div=%d][SzW=%d][SzWH=%d]",div,SzW,SzWH);
  fflush(stdout);
  
  //Init. Memoire
  float***  SquWin=fmatrix_allocate_3d(QUATRE,SzW,SzW);
  float***  SquWinH=fmatrix_allocate_3d(QUATRE,SzWH,SzWH);
  float**   MatxNB=fmatrix_allocate_2d(lgth,wdth);
  float**   MatxHor=fmatrix_allocate_2d(lgth,wdth);
  float**   MatxVer=fmatrix_allocate_2d(lgth,wdth);
  float**   MatxDgd=fmatrix_allocate_2d(lgth,wdth);
  float**   MatxDgg=fmatrix_allocate_2d(lgth,wdth);
  float***  ImgLAB=fmatrix_allocate_3d(TROIS,lgth,wdth);

  //Init. Matrix    
  for(k=0;k<NbParamFin;k++) for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) MatxText[k][i][j]=0.0;
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) 
     MatxNB[i][j]=(0.33*img[0][i][j])+(0.33*img[1][i][j])+(0.33*img[2][i][j]);

  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)
    { MatxHor[i][j]=0.0;
      MatxVer[i][j]=0.0;
      MatxDgd[i][j]=0.0; 
      MatxDgg[i][j]=0.0; }

  for(i=1;i<(lgth-1);i++) for(j=1;j<(wdth-1);j++) MatxHor[i][j]=fabs(MatxNB[i][j-1]-MatxNB[i][j+1]);
  for(i=1;i<(lgth-1);i++) for(j=1;j<(wdth-1);j++) MatxVer[i][j]=fabs(MatxNB[i-1][j]-MatxNB[i+1][j]);
  for(i=1;i<(lgth-1);i++) for(j=1;j<(wdth-1);j++) MatxDgd[i][j]=fabs(MatxNB[i-1][j-1]-MatxNB[i+1][j+1]);
  for(i=1;i<(lgth-1);i++) for(j=1;j<(wdth-1);j++) MatxDgg[i][j]=fabs(MatxNB[i-1][j+1]-MatxNB[i+1][j-1]);
  

  //Conversion RGB=>LAB
  ConvertImgRGBToLAB(img,ImgLAB,lgth,wdth);
  for(k=0;k<TROIS;k++) for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) img[k][i][j]=ImgLAB[k][i][j];
  for(k=0;k<TROIS;k++) Recal(img[k],lgth,wdth);

 //=============================================================

      //------------------------------------
      //--- FEATURES COLOR HISTO -----------
      //------------------------------------
      for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  
        {
         for(k=0;k<SzW;k++) for(l=0;l<SzW;l++) 
            { posr=i-(SzW/2)+k;
              posc=j-(SzW/2)+l;

              if (posr<0)           posr+=lgth;
              if (posr>(lgth-1))    posr-=lgth;  
              if (posc<0)           posc+=wdth;
              if (posc>(wdth-1))    posc-=wdth;

              SquWin[0][k][l]=img[0][posr][posc];
              SquWin[1][k][l]=img[1][posr][posc];
              SquWin[2][k][l]=img[2][posr][posc]; }
   

         //LAB
         for(k=0;k<SzW;k++) for(l=0;l<SzW;l++)  
	    { lab=((int)(SquWin[0][k][l]/divisL))*(div*div);
              lab+=((int)(SquWin[1][k][l]/divis))*(div);  
              lab+=((int)(SquWin[2][k][l]/divis));
   
              MatxText[(int)(lab)][i][j]++; }

	}//-End-Features-Color-Histo--------
           
      //------------------------------------
      //--- FEATURES GRAD HISTO ------------
      //------------------------------------
      cpt=0.0;
       for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  
        {
         for(k=0;k<SzWH;k++) for(l=0;l<SzWH;l++) 
            { posr=i-(SzWH/2)+k;
              posc=j-(SzWH/2)+l;

              if (posr<0)           posr+=lgth;
              if (posr>(lgth-1))    posr-=lgth;  
              if (posc<0)           posc+=wdth;
              if (posc>(wdth-1))    posc-=wdth;

              SquWinH[0][k][l]=MatxHor[posr][posc]; 
              SquWinH[1][k][l]=MatxVer[posr][posc];        
              SquWinH[2][k][l]=MatxDgd[posr][posc];       
              SquWinH[3][k][l]=MatxDgg[posr][posc];  }

              for(k=0;k<SzWH;k++) for(l=0;l<SzWH;l++)  
	        { lab=SquWinH[0][k][l]+0.5;
                  if (lab>NB_GRADBINS) { lab=NB_GRADBINS-1; cpt++; }
                  MatxText[NbParamColorHisto+(int)(lab)][i][j]++; }

              for(k=0;k<SzWH;k++) for(l=0;l<SzWH;l++)  
	        { lab=SquWinH[1][k][l]+0.5;
                  if (lab>NB_GRADBINS) {  lab=NB_GRADBINS-1; cpt++; } 
                  MatxText[NbParamColorHisto+NB_GRADBINS+(int)(lab)][i][j]++; }

              for(k=0;k<SzWH;k++) for(l=0;l<SzWH;l++)  
	        { lab=SquWinH[2][k][l]+0.5;
                  if (lab>NB_GRADBINS) { lab=NB_GRADBINS-1; cpt++; } 
                  MatxText[NbParamColorHisto+(2*NB_GRADBINS)+(int)(lab)][i][j]++; }

              for(k=0;k<SzWH;k++) for(l=0;l<SzWH;l++)  
	        { lab=SquWinH[3][k][l]+0.5;
                  if (lab>NB_GRADBINS) { lab=NB_GRADBINS-1; cpt++; } 
                  MatxText[NbParamColorHisto+(3*NB_GRADBINS)+(int)(lab)][i][j]++; }

        }//-End-Features-Grad-Histo-------- 

  //---------
  //__INFO___
  //---------
  cpt=100*cpt/(4*CARRE(SzWH)*lgth*wdth);
  printf("\n    [%.2f/100 Gradient>%d]\n",cpt,NB_GRADBINS);
  fflush(stdout);

  //===========================================================

  //Liberation Memoire
  if (SquWin)  free_fmatrix_3d(SquWin,QUATRE);
  if (SquWinH) free_fmatrix_3d(SquWinH,QUATRE);
  if (MatxNB)  free_fmatrix_2d(MatxNB);
  if (MatxHor) free_fmatrix_2d(MatxHor);
  if (MatxVer) free_fmatrix_2d(MatxVer);
  if (MatxDgd) free_fmatrix_2d(MatxDgd);
  if (MatxDgg) free_fmatrix_2d(MatxDgg);
  if (ImgLAB)  free_fmatrix_3d(ImgLAB,TROIS);
 }

//--------------------------------------------------------------
// SEG >> ComputeMultiresNonLocalGraphWindow
//--------------------------------------------------------------
//
//  In :: float*** MatxText[NbFeat][lgth][wdth]
// Out :: float*** NonLocGrph[NbCnx][lgth/scale][wdth/scale]
//
//  <<<<NRJ>>>>
//  NonLocGrph[2][][] :: dist :: Beta^2
//                    :: SQUARE (Distance Euclid.) DISTANCE=1
//                    :: SQUARE (Distance Bhatta)  DISTANCE=0
//  <<<<NRJ>>>>
//
//--------------------------------------------------------------     
void MD2S::SEG_ComputeMultiresNonLocalGraphWindow(float*** MatxText,int BegFeat,int EndFeat,\
int lgth,int wdth,short int*** NonLocGrph,int SzW,int NbCnx,int Scale,float** MatxCont)
{
 short int i,j,k,l,m;
 short int posc,posr;
 short int SzWdiv2;
 float echdiv;
 int Eechdiv;
 float dist;
 int cpt,nb;
 float NbFeat;
 float dmin,dmax;
 float Sum1,Sum2;
 float ValPotCont;
 float distclq;

 //Constante[0::Bhatta 1::Euclidean]
 const int DISTANCE=0;
 const int NBPPV=8;

 //Connexion Graph
 echdiv=CARRE(SzW)/(float)NbCnx;
 echdiv=(int)(echdiv+0.4999);
 Eechdiv=(int)echdiv;
 SzWdiv2=(SzW/2); 
 NbFeat=EndFeat-BegFeat;

 //Presentation
 printf("\n");  
 if (DISTANCE)  printf("\n  > Compute NonLocalGraph Window (NormeL2) ");
 if (!DISTANCE) printf("\n  > Compute NonLocalGraph Window (Bhatta) ");
 printf("\n  >>> Scale_    ::[%d]",Scale);
 printf("\n  >>> Matx_Text ::[Begin_%d::End_%d][lgthR:%d-wdthR:%d]",BegFeat,EndFeat,lgth,wdth);
 printf("\n  >>> Window_Sz ::[%d]  Connect.::[%d] <Ecart:%d>",SzW,NbCnx,(int)echdiv);
 fflush(stdout);

 //Init_NonLocGrph 
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) for(k=0;k<NbCnx;k++)
   { NonLocGrph[0][k][(i*wdth)+j]=i;
     NonLocGrph[1][k][(i*wdth)+j]=j;
     NonLocGrph[2][k][(i*wdth)+j]=0; }

  //-----------------------------------------------
  //--NonLocal Graph ------------------------------
  //-----------------------------------------------
 dmax=0.0;
 dmin=BIGNUMBER;
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)
     {
      //>SearchInBigWindow[posr,posc]
      cpt=0; nb=0;
      for(k=0;k<SzW;k++) for(l=0;l<SzW;l++) 
        { 
          cpt++;
          if (cpt%Eechdiv) continue;

          posr=i-(SzWdiv2)+k;
          posc=j-(SzWdiv2)+l;
          if (posr<0)         posr+=lgth;
          if (posr>(lgth-1))  posr-=lgth; 
          if (posc<0)         posc+=wdth;
          if (posc>(wdth-1))  posc-=wdth;
 
          //>Euclidean Distance 
          //===================         
          if (DISTANCE)
	     { for(dist=0.0,m=BegFeat;m<EndFeat;m++) 
	       dist+=CARRE(MatxText[m][i*Scale][j*Scale]-MatxText[m][posr*Scale][posc*Scale]); 
               dist=sqrt(dist); }

          //>Bhattacharta Distance
          //======================
          if (!DISTANCE)
	     { for(Sum1=0.0,Sum2=0.0,m=BegFeat;m<EndFeat;m++) 
	          { Sum1+=MatxText[m][i*Scale][j*Scale];
                    Sum2+=MatxText[m][posr*Scale][posc*Scale]; }

               for(dist=0.0,m=BegFeat;m<EndFeat;m++) 
	       dist+=sqrt(((float)MatxText[m][i*Scale][j*Scale]/Sum1)*((float)MatxText[m][posr*Scale][posc*Scale]/Sum2));

               if (dist>1.0) dist=1.0;
               dist=10000*(1-dist); }    

           //>Distance Contour 
           //=================
	   distclq=fabs(i*Scale-posr*Scale)+fabs(j*Scale-posc*Scale);
           ValPotCont=2*EstimPotCont(i*Scale,j*Scale,posr*Scale,posc*Scale,MatxCont); 
           dist*=ValPotCont;  
     
           //>Prior
           //======
           if (dist<(int)(Tbh*10000)) dist=0.0; 
                      
          //Stat
          if (dist>dmax) dmax=dist;
          if (dist<dmin) dmin=dist;
                        
          //>RecordInGraph
          if (nb>=NbCnx) break;
          NonLocGrph[0][nb][(i*wdth)+j]=posr;
          NonLocGrph[1][nb][(i*wdth)+j]=posc;
	  NonLocGrph[2][nb][(i*wdth)+j]=(int)dist;
          nb++;       
	}


      //>NBPPV
      //============================
      nb=(NbCnx-NBPPV);
      for(k=-1;k<=1;k++) for(l=-1;l<=1;l++) 
      if (((abs(k)+abs(l))==1)||((abs(k)+abs(l))==2))
        {
          posr=i+k;
          posc=j+l;
          if (posr<0)         posr+=lgth;
          if (posr>(lgth-1))  posr-=lgth; 
          if (posc<0)         posc+=wdth;
          if (posc>(wdth-1))  posc-=wdth;

          //>RecordInGraph
          NonLocGrph[0][nb][(i*wdth)+j]=posr;
          NonLocGrph[1][nb][(i*wdth)+j]=posc;
	  NonLocGrph[2][nb][(i*wdth)+j]=0;
          nb++;               
        } 

     }//-End-NonLocal-Grph---------------------------

  //---------
  //__INFO___
  //---------
  printf(" [ValMin=%.1f::ValMax=%.1f]",dmin,dmax);
  fflush(stdout);
}


//=========================================================================
//=========================================================================
//>INTERPOLATION       ====================================================
//=========================================================================
//----------------------------------------------------------
// SEG_Interpolate 
//----------------------------------------------------------
//
//  ImgUp [lgthUp*wdthUp] >>  ImgBl [lgthBl*wdthBl]
//
//  Using> float*** TextCub [DpthBeg::DpthEnd][lgth][wdth] (CubeTexture)
//  Using> Buades
//
//  Nota1: ImgUp in [0.0::255] ou in [0.0::1.0]
//              
//----------------------------------------------------------
void MD2S::SEG_Interpolate(float*** TextCub,int lgth,int wdth,int DpthBeg,int DpthEnd,\
float** ImgUp,int lgthUp,int wdthUp,float** ImgBl,int lgthBl,int wdthBl,int SizeW,float h)
{ 
 int i,j,k,l,m;
 int posr,posc,pposr,pposc; 
 int rowEnt,colEnt; 
 int Scale,Scale3,Scale2;
 int cpt;
 double dist,den,min;
 int Dpth;
 
 //parametre
 Scale=lgthBl/lgthUp;
 Scale3=lgth/lgthBl;
 Scale2=lgth/lgthUp;
 Dpth=DpthEnd-DpthBeg;

 //Presentation
 printf("\n    SEG_Interpolation [%d::%d]",lgth,wdth);
 printf(" <Sc1=%d :: Sc3=%d>",Scale,Scale3);
 printf(" <SzW=%d>",SizeW);
 fflush(stdout);

 //AllocationMemoire
 float*  VctIn=fmatrix_allocate_1d(Dpth);
 float*  VctOu=fmatrix_allocate_1d(Dpth);
 double** Val=dmatrix_allocate_2d(CINQ,SizeW*SizeW);

 //Init ImgBl (below) from ImgUp
 for(i=0;i<lgthBl;i++) for(j=0;j<wdthBl;j++) ImgBl[i][j]=0.0;

 for(i=0;i<lgthBl;i++) for(j=0;j<wdthBl;j++)
 if ((!(i%Scale))&&(!(j%Scale))) if (((i/Scale)<lgthUp)&&((j/Scale)<wdthUp))  
 ImgBl[i][j]=ImgUp[i/Scale][j/Scale];

 //=======================================================    
 //Boucle
 //=======================================================
 for(i=0;i<lgthBl;i++) for(j=0;j<wdthBl;j++) if (!ImgBl[i][j])
   {
    //>Recup VctIn
    for(m=DpthBeg;m<DpthEnd;m++) VctIn[m-DpthBeg]=TextCub[m][i*Scale3][j*Scale3]; //>Img[lgth,wdth]

     //>Recup[LevelGrey_Pixels\in.Spat.Vois.]_ImgUp
     rowEnt=(int)i/Scale; 
     colEnt=(int)j/Scale; //>ImgUp
     cpt=0;
     for(k=0;k<SizeW;k++) for(l=0;l<SizeW;l++) 
        { 
         posr=rowEnt-(SizeW/2)+k;
         posc=colEnt-(SizeW/2)+l;
          if (posr<0)           posr+=lgthUp;
          if (posr>(lgthUp-1))  posr-=lgthUp; 
          if (posc<0)           posc+=wdthUp;
          if (posc>(wdthUp-1))  posc-=wdthUp;

          Val[2][cpt]=ImgUp[posr][posc]; 
           
	  pposr=(posr*Scale2);
	  pposc=(posc*Scale2); //>Img[lgth,wdth]
          for(m=DpthBeg;m<DpthEnd;m++) VctOu[m-DpthBeg]=TextCub[m][pposr][pposc];

          //>CompareWith[Vector\in.Spat.Vois.]_ImgUp
          dist=0.0;
          for(m=0;m<Dpth;m++) dist+=CARRE(VctIn[m]-VctOu[m]);
          dist/=Dpth;
          Val[3][cpt]=exp(-(dist/h));
          Val[4][cpt]=dist; 
          cpt++;                      
	}        

    //>Interpolation <Buades ou ppv>
    //------------------------------
    for(dist=0.0,m=0;m<SizeW*SizeW;m++) dist+=Val[3][m]*Val[2][m];
    for(den=0.0,m=0;m<SizeW*SizeW;m++)  den+=Val[3][m];
    if (den) ImgBl[i][j]=(dist/den);
    else { 
          ImgBl[i][j]=Val[2][0];
          for(min=Val[4][0],m=0;m<SizeW*SizeW;m++) if (Val[4][m]<min) 
                 {  min=Val[4][m];  
		    ImgBl[i][j]=Val[2][m]; } }
   }
 //======================================================

 //LiberationMemoire
 if (VctIn)  free_fmatrix_1d(VctIn);
 if (VctOu)  free_fmatrix_1d(VctOu); 
 if (Val)    free_dmatrix_2d(Val);
}

//----------------------------------------------------------
// SEG_Interpolate 
//----------------------------------------------------------
//              
//----------------------------------------------------------
void MD2S::SEG_Interpolate(float*** TextCub,int lgth,int wdth,int DpthBeg,int DpthEnd,\
float*** ImgUp,int lgthUp,int wdthUp,float*** ImgBl,int lgthBl,int wdthBl,int SizeW,float h,int nbdpth)
{ 
 int k;

 for(k=0;k<nbdpth;k++) 
   SEG_Interpolate(TextCub,lgth,wdth,DpthBeg,DpthEnd,ImgUp[k],lgthUp,wdthUp,ImgBl[k],lgthBl,wdthBl,SizeW,h);
}


//=========================================================================
//=========================================================================
//>OPTIMISATION ===========================================================
//=========================================================================
//=========================================================================
//-------------------------------------------------------------
// SEG >> SEG_GGMRFDerive
//-------------------------------------------------------------
//
//  <<<<NRJ>>>>
//  DELTA(NRJ) = -4 (Xs-Xt)) * SUM_<st> ( Beta - (Xs-Xt)^2 )
//  <<<<NRJ>>>>
//  
// In  :: short int*** NonLocGrph[TROIS][NbCnx][lgth*wdth]
//     :: Img ...>> Derivate(Img)=ImgGrad
// Out :: ImgGrad[lgth][wdth]
//
//-------------------------------------------------------------     
void MD2S::SEG_GGMRFDerive(float** Img,float** ImgGrad,int lgth,int wdth,short int*** NonLocGrph,int NbCnx)
{
 int i,j,k;
 int posr,posc;
 float tmp;
 float xs,xt;
 float difx;
 int nbp;
 
 //Boucle
 //==================================
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)
   {
     nbp=(i*wdth)+j;
     xs=Img[i][j];

     tmp=0.0;
     //>ForAllConnexion
     for(k=0;k<NbCnx;k++)
       { posr=NonLocGrph[0][k][nbp];
         posc=NonLocGrph[1][k][nbp];
         xt=Img[posr][posc];
         difx=(xs-xt);
         tmp+=-4*(NonLocGrph[2][k][nbp]-CARRE(difx))*difx; 
       }

     //>Record
     ImgGrad[i][j]=tmp/(float)NbCnx;
   } 
}

//-----------------------------------------------
//-----------------------------------------------
// GRADIENT/SA <DIM> ----------------------------
//-----------------------------------------------
//-----------------------------------------------
//----------------------------------------------------------
// SEG >> ComputeNrj <Norme L2> 
//----------------------------------------------------------
//
// In  :: Img[NbDpth][lgth][wdth]
//     :: NonLocGrph[TROIS][NbCnx][lgth*wdth]
// Out :: Nrj
//
//  <<<<NRJ>>>>
//  NRJ = SUM_<st> ( Beta - (Xs-Xt)^2 )^2
//  <<<<NRJ>>>>
//
//----------------------------------------------------------     
float MD2S::SEG_ComputeNrjDim(float*** Img,int lgth,int wdth,int NbDpth,short int*** NonLocGrph,int NbCnx)
{
 int i,j,k,l;
 float totvr,TotalVar;
 int posr,posc;
 int nbp;
 float dist;
 float distImg;

 //Variation Totale
 TotalVar=0.0;
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) 
     {
      totvr=0.0;
      for(k=0;k<NbCnx;k++)
        { nbp=(i*wdth)+j;
          posr=(int)NonLocGrph[0][k][nbp];
          posc=(int)NonLocGrph[1][k][nbp];
          dist=NonLocGrph[2][k][nbp];

          for(distImg=0.0,l=0;l<NbDpth;l++) distImg+=CARRE(Img[l][i][j]-Img[l][posr][posc]);
          totvr+=CARRE(dist-distImg); }

      TotalVar+=totvr/(float)NbCnx; 
     }

 //Retour
 TotalVar/=(float)(NbDpth);
 return TotalVar/(float)(wdth*lgth);
}

//----------------------------------------------------------
// SEG >> SEG_ComputeBeta
//----------------------------------------------------------
//
//----------------------------------------------------------     
float MD2S::SEG_ComputeBetaDim(float*** ImGrd,float*** ImGrdp,int nbdpth,int lgth,int wdth)
{
 int i,j,l;
 float Num,Denum;

 // Numerateur & Denumerateur
 for(Num=0.0,l=0;l<nbdpth;l++)   for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) Num+=(ImGrd[l][i][j]-ImGrdp[l][i][j])*ImGrd[l][i][j];
 for(Denum=0.0,l=0;l<nbdpth;l++) for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) Denum+=CARRE(ImGrdp[l][i][j]);

 //Retour
 return (Num/Denum);
}

//----------------------------------------------------------
//  SEG >> SEG_ConjGradDescDim
//----------------------------------------------------------
// Descente Gradient Conjugué ([NBDPTH] Dimensions) 
// ---------------------------------------------------------
//
// x[n+1]=x[n]+StGr*d[n]
// 
//  d[n]=- Grad_J(x[n])
//       + <Grad_J(x[n])-Grad_J(x[n-1]),Grad_J(x[n])>/<Grad_J(x[n-1]),Grad_J(x[n-1])> d[n-1]
//
// Img                 : Img Initiale :: Image_Nrj_init
// MatxGrd[8][x]       : Img Initiale :: Image_Nrj_init
// MatxGrd[1][x]       : x[n] puis ...x[n+1]
// MatxGrd[2][x]       : Grad_J(x[n])
// MatxGrd[3][x]       : Grad_J(x[n-1])
// MatxGrd[4][x]       : d[n]
// MatxGrd[5][x]       : d[n-1]
// MatxGrd[9][x]       : x[n+1]
// 
// MatxGrd[7][x]       : ImgInit
//
// ------------------------------
// Img[NbDpth] :: MAX(NbDpth)=3
// ------------------------------
//----------------------------------------------------------    
void MD2S::SEG_ConjGradDescDim(float*** Img,float*** ImgRes,int NBDPTH,int lgth,int wdth, \
short int*** NonLocGrph,int NbCnx,int NbIter)
{
 int k,l;
 float nrj,nrj_prec,nrj_init;
 float Beta;
 int NbDiv;

 //>Verif
 if (NBDPTH>3) { printf("\n NbDpth>3"); exit(-1); }

 //>Init
 for(l=0;l<NBDPTH;l++) CopyMat(MatxGrd[1][l],Img[l],lgth,wdth);
 for(l=0;l<NBDPTH;l++) CopyMat(MatxGrd[8][l],Img[l],lgth,wdth);
 for(l=0;l<NBDPTH;l++) CopyMat(MatxGrd[7][l],Img[l],lgth,wdth);
 nrj=SEG_ComputeNrjDim(MatxGrd[8],lgth,wdth,NBDPTH,NonLocGrph,NbCnx); 
 nrj_prec=nrj;
 nrj_init=nrj;
 printf("\n    >  ConjGradDescenteDim<%d> [L:W:%d:%d][NbCx=%d][StGr=%.3f*10-6]\n",NBDPTH,lgth,wdth,NbCnx,StGr*MILLION);  
 printf("\n    >  <NrjInit=%.1f>",nrj);
 fflush(stdout);
 NrjImgInit=nrj_init;
 Beta=0.0;
 NbDiv=0;

 //>Flag_stop
 int FlagStop=0;
 
 //>Conjugate Gradient Descent <DIM=NBDPTH>
 //========================================
 for(k=0;((k<=NbIter)||(!FlagStop));k++)
   {
     //>Info
     if ((k<20)||(!(k%10))) printf("\n [%d]>  ",k);

     // MatxGrd[2][] <-- Grad_J(x[k])
     for(l=0;l<NBDPTH;l++)  SEG_GGMRFDerive(MatxGrd[1][l],MatxGrd[2][l],lgth,wdth,NonLocGrph,NbCnx);

     //-----------------------
     //> [k==0] SimpleGradient
     //-----------------------
     //      > d[k] <-- Grad_J(x[k])    
     //      > x[k+1]=x[n]-StrGr*Grad_J(x[k]) 
     if (k==0) 
        { for(l=0;l<NBDPTH;l++) CopyMat(MatxGrd[4][l],MatxGrd[2][l],lgth,wdth);
          for(l=0;l<NBDPTH;l++) AddMatrix(MatxGrd[9][l],MatxGrd[1][l],-1.0*StGr,MatxGrd[2][l],lgth,wdth); }
    
     //--------------------------------------------
     // [k!=0] Gradient Conjugue
     //-------------------------------------------- 
     //      > x[k+1]=x[k]-StGr*d[k]
     if (k) 
	{  Beta=SEG_ComputeBetaDim(MatxGrd[2],MatxGrd[3],NBDPTH,lgth,wdth);            
           for(l=0;l<NBDPTH;l++) MultMatrix(MatxGrd[6][l],Beta,MatxGrd[5][l],lgth,wdth);
           for(l=0;l<NBDPTH;l++) AddMatrix(MatxGrd[4][l],MatxGrd[2][l],-1.0,MatxGrd[6][l],lgth,wdth);
           for(l=0;l<NBDPTH;l++) AddMatrix(MatxGrd[9][l],MatxGrd[1][l],-1.0*StGr,MatxGrd[4][l],lgth,wdth); }

      //>Calcul Nrj & Stab(x[k],x[k-1])
      if ((k<20)||(!(k%10)))
         {  nrj_prec=nrj;
	    nrj=SEG_ComputeNrjDim(MatxGrd[9],lgth,wdth,NBDPTH,NonLocGrph,NbCnx); 
            printf("<Nrj=%.1f> <Beta=%.1f*10^-6>",nrj,Beta*MILLION);
            fflush(stdout); }
    
      //>RecordBest
      if ((nrj<nrj_prec)&&(nrj<nrj_init)) for(l=0;l<NBDPTH;l++) CopyMat(MatxGrd[8][l],MatxGrd[9][l],lgth,wdth);
       
      //>Verif si StGr trop grand
      if (k<30)
          if ((nrj>nrj_prec)||(nrj>nrj_init)||(isnan(nrj)))
	     { for(l=0;l<NBDPTH;l++) CopyMat(MatxGrd[9][l],MatxGrd[8][l],lgth,wdth);
               StGr/=2.0;
               NbDiv++;
               for(l=0;l<NBDPTH;l++) InitMatrix(MatxGrd[5][l],lgth,wdth);
               printf(" ! New StGr=%.1f*10^-6",StGr*MILLION);
               k--;
               fflush(stdout); }
      
       // x[k+1] --> x[k]  
       // Img3 <-- Grad_J(x[k-1]) 
       // Img5 <-- d[k-1]
       for(l=0;l<NBDPTH;l++) CopyMat(MatxGrd[1][l],MatxGrd[9][l],lgth,wdth);
       for(l=0;l<NBDPTH;l++) CopyMat(MatxGrd[3][l],MatxGrd[2][l],lgth,wdth);
       for(l=0;l<NBDPTH;l++) CopyMat(MatxGrd[5][l],MatxGrd[4][l],lgth,wdth);

       //End
       if ((k>NbIter)||((NbDiv>6)&&k>10)) break;  

   }//FIN_ALGO======================================
      
 //Enregistrement Nrj_finale[ MatxGrd[9] ] 
 NrjImg=nrj;
 printf("\n      >> [Nrj:%.5f]",NrjImg);
 for(l=0;l<NBDPTH;l++) printf(" [Diff=%.1f]",ComputeDiff(MatxGrd[9][l],MatxGrd[7][l],lgth,wdth));       

 //Transfert>ImgRes
 printf("\n");
 for(l=0;l<NBDPTH;l++) CopyMat(ImgRes[l],MatxGrd[9][l],lgth,wdth);  
}

//----------------------------------------------------------
// SEG >> Compute Local NRJ  
//----------------------------------------------------------
//
// <<<<NRJ>>>>
// NRJ = SUM_<st> ( Beta - (Xs-Xt)^2 )^2
// <<<<NRJ>>>>
//
//---------------------------------------------------------- 
float MD2S::SEG_ComputeLocNrjDim(float*** Img,int nbdpth,int row,int col,float* val,int wdth,short int*** NonLocGrph,int NbCnx)
{
 int k,l;
 float totvr;
 int posr,posc;
 int nbp;
 float dist,distImg;

 //Calcul Local NRJ
 //-------------------------
 totvr=0.0;
 nbp=(row*wdth)+col;
 
 for(k=0;k<NbCnx;k++)
    { posr=NonLocGrph[0][k][nbp];
      posc=NonLocGrph[1][k][nbp];
      dist=NonLocGrph[2][k][nbp];
      
      for(distImg=0.0,l=0;l<nbdpth;l++) distImg+=CARRE(val[l]-Img[l][posr][posc]);
      totvr+=CARRE(dist-distImg); }

 totvr/=(float)NbCnx;

 //Retour
 totvr/=(float)nbdpth;
 return totvr;
}

//---------------------------------------------------------
//----------------------------------------------------------
// SEG >> SimulatedAnnealing <DIM>
//----------------------------------------------------------
//
//----------------------------------------------------------    
void MD2S::SEG_SimulatedAnnealingDim(float*** Img,float*** ImgRes,int NBDPTH,int lgth,int wdth,\
short int*** NonLocGrph,int NbCnx,int NbIter,float Temp_Init,float Radius)
{
 int i,j,k,l,m;
 float nrj,nrj1,nrj2;
 float ValImg;
 float rd,rd2; 
 float DeltaNrj;
 float ProbAcc;
 float RadiusDiv2; 
 float nbchgt;

 //>Simulated Annealing
 float Temp_Fina=TEMPSEG_FINAL;
 float Temp,TempInv;
 int   Kmax=NbIter;

 //AllocationMemory
 float* TabVal1=fmatrix_allocate_1d(NBDPTH);
 float* TabVal2=fmatrix_allocate_1d(NBDPTH);

 //>Init
 RadiusDiv2=Radius/2.0;
 for(l=0;l<NBDPTH;l++) CopyMat(MatxGrd[1][l],Img[l],lgth,wdth);
 for(l=0;l<NBDPTH;l++) CopyMat(MatxGrd[2][l],Img[l],lgth,wdth);
 nrj=SEG_ComputeNrjDim(MatxGrd[1],lgth,wdth,NBDPTH,NonLocGrph,NbCnx);
 printf("\n\n    >  Simulated Annealing [DIM=%d][NbIterMax=%d][Radius=%.1f]",NBDPTH,NbIter,Radius);  
 printf("\n                             [Temp_Init=%.1f][Temp_Final=%.1f]",Temp_Init,Temp_Fina);
 printf("\n\n    >  <NrjInit=%.2f>",nrj);

  //====================================================================
  //Simulated Annealing -->sur Img 
  //====================================================================
  for(k=0;;k++)
     {
      Temp=Temp_Init*powf((Temp_Fina/Temp_Init),((float)k/Kmax));
      TempInv=1.0/Temp;
      if (Temp<Temp_Fina) break;

      if (!(k%10)) 
      for(l=0;l<NBDPTH;l++) CopyMat(MatxGrd[2][l],MatxGrd[1][l],lgth,wdth);

      //-----------------------------------------------------------
      // Loop Img > Loop Pixels
      //------------------------------------------------------------
          for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) for(l=0;l<NBDPTH;l++)
            {
               //Gibbs Sampling Record
	       ValImg=MatxGrd[1][l][i][j];
  	       rd=ValImg-RadiusDiv2+(Radius*randomize());
 
               if (rd<0.0)   rd=0.0;
               if (rd>255.0) rd=255.0;

	        for(m=0;m<NBDPTH;m++) { TabVal1[m]=MatxGrd[1][m][i][j]; TabVal2[m]=MatxGrd[1][m][i][j]; }
                TabVal2[l]=rd;
 
                nrj1=SEG_ComputeLocNrjDim(MatxGrd[1],NBDPTH,i,j,TabVal1,wdth,NonLocGrph,NbCnx);
                nrj2=SEG_ComputeLocNrjDim(MatxGrd[1],NBDPTH,i,j,TabVal2,wdth,NonLocGrph,NbCnx);               
                DeltaNrj=nrj2-nrj1;

                if (DeltaNrj<0) MatxGrd[1][l][i][j]=rd;

                else { ProbAcc=exp(-TempInv*DeltaNrj); 
                       rd2=randomize();
                       if (rd2<ProbAcc)  MatxGrd[1][l][i][j]=rd; }    
           
             }//End_Loop_Pixel------------------------------------------  

          //>Info
          if (!(k%10))
	     { nrj=SEG_ComputeNrjDim(MatxGrd[1],lgth,wdth,NBDPTH,NonLocGrph,NbCnx); 
               nbchgt=SEG_DiffMat(MatxGrd[2][0],MatxGrd[1][0],lgth,wdth,1.0);
               printf("\n  [%d]>%.1f [Temp:%.2f][NbChgt/100=%.2f]",k,nrj,Temp,nbchgt); }

         }//End_Simulated_Annealing==================================================================
  
 //Nrj_finale[Img1] 
 for(l=0;l<NBDPTH;l++) CopyMat(ImgRes[l],MatxGrd[1][l],lgth,wdth);
 nrj=SEG_ComputeNrjDim(ImgRes,lgth,wdth,NBDPTH,NonLocGrph,NbCnx);
 NrjImg=nrj;
 printf("\n   >> [Nrj:%.3f]",nrj); 
 fflush(stdout);

 //FreeMemory
 if (TabVal1) free_fmatrix_1d(TabVal1);
 if (TabVal2) free_fmatrix_1d(TabVal2);
}

//------------------------------------------------------------------
// SEG >> MultiResOptimization<DIM> <GRADIENT+SA>
//------------------------------------------------------------------
//
// In  :: float*** Img[lgth][wdth]
// Out :: float** ImgRes[lgth/scale][wdth/scale]
//
//------------------------------------------------------------------     
void MD2S::SEG_MultiResOptimizationContTex(float*** ImgIn,float*** MatxText,int lgth,int wdth,int NbCnxGrph,int SzWGrph, \
int dpthBeg,int dpthEnd,float*** ImgRes,int NBDPTH,int Scale,int NbItGrad,int NbItMetr,float** MatxCont)
{
 int wdth_r,lgth_r;

 //>Parametre
 lgth_r=(int)(lgth/Scale);
 wdth_r=(int)(wdth/Scale);

  //>Presentation
  printf("\n\n ____MultiResolution OptimizationContTex__[DIM::%d]___",NBDPTH);
  printf("\n ___________________________________________________");
  printf("\n  >> Scale:::::[%d]",Scale);
  printf("\n  >> Img:::::::[L=%d::W=%d]>>[LRed=%d::WRed=%d]",lgth,wdth,lgth_r,wdth_r);
  printf("\n  >> NbFeat::::[%d::%d]",dpthBeg,dpthEnd);
  printf("\n  >> NbCnxGrph:[%d]",NbCnxGrph);
  printf("\n  >> SzWGrph:::[%d]",SzWGrph);
  fflush(stdout);

  //>AllocationMemoire
  short int*** NonLocGrphRed=simatrix_allocate_3d(TROIS,NbCnxGrph,(wdth_r*lgth_r));
  float*** ImgRed=fmatrix_allocate_3d(NBDPTH,lgth_r,wdth_r);

  //>NonLocalMultiResGraph
  SEG_ComputeMultiresNonLocalGraphWindow(MatxText,dpthBeg,dpthEnd,lgth_r,wdth_r,NonLocGrphRed,SzWGrph,NbCnxGrph,Scale,MatxCont);

  //>ImgIn > ImgRed      
  SEG_ConjGradDescDim(ImgIn,ImgRed,NBDPTH,lgth_r,wdth_r,NonLocGrphRed,NbCnxGrph,NbItGrad);           
  SEG_SimulatedAnnealingDim(ImgRed,ImgRes,NBDPTH,lgth_r,wdth_r,NonLocGrphRed,NbCnxGrph,NbItMetr,TEMPSEG_INIT,RADIUS_SEG);     

  //>Liberation Memoire
 if (NonLocGrphRed) free_simatrix_3d(NonLocGrphRed,TROIS);
 if (ImgRed)        free_fmatrix_3d(ImgRed,NBDPTH);
}

//------------------------------------------------------------------
// SEG >> MultiResOptimiz<DIM> <SA> 
//------------------------------------------------------------------
//
// In  :: float*** Img[lgth][wdth]
// Out :: float** ImgRes[lgth/scale][wdth/scale]
//
//------------------------------------------------------------------     
void MD2S::SEG_MultiResOptimizContTex(float*** ImgIn,float*** MatxText,int lgth,int wdth,int NbCnxGrph,int SzWGrph, \
int dpthBeg,int dpthEnd,float*** ImgRes,int NBDPTH,int Scale,int NbItMetr,float** MatxCont)
{
 int wdth_r,lgth_r;

 //>Parametre
 lgth_r=(int)(lgth/Scale);
 wdth_r=(int)(wdth/Scale);

  //>Presentation
  printf("\n\n ____MultiResolution OptimizContTex__[DIM::%d]___",NBDPTH);
  printf("\n ____________________________________________");
  printf("\n  >> Scale:::::[%d]",Scale);
  printf("\n  >> Img:::::::[L=%d::W=%d]>>[LRed=%d::WRed=%d]",lgth,wdth,lgth_r,wdth_r);
  printf("\n  >> NbFeat::::[%d::%d]",dpthBeg,dpthEnd);
  printf("\n  >> NbCnxGrph:[%d]",NbCnxGrph);
  printf("\n  >> SzWGrph:::[%d]",SzWGrph);
  printf("\n  >> NbItMetr::[%d]",NbItMetr);
  fflush(stdout);

  //>AllocationMemoire
  short int*** NonLocGrphRed=simatrix_allocate_3d(TROIS,NbCnxGrph,(wdth_r*lgth_r));
  float*** ImgRed=fmatrix_allocate_3d(NBDPTH,lgth_r,wdth_r);

  //>NonLocalMultiResGraph
  SEG_ComputeMultiresNonLocalGraphWindow(MatxText,dpthBeg,dpthEnd,lgth_r,wdth_r,NonLocGrphRed,SzWGrph,NbCnxGrph,Scale,MatxCont);

  //>ImgIn > ImgRes            
  SEG_SimulatedAnnealingDim(ImgIn,ImgRes,NBDPTH,lgth_r,wdth_r,NonLocGrphRed,NbCnxGrph,NbItMetr,TEMPSEG_INIT,RADIUS_SEG);     

  //>Liberation Memoire
 if (NonLocGrphRed) free_simatrix_3d(NonLocGrphRed,TROIS);
 if (ImgRed)        free_fmatrix_3d(ImgRed,NBDPTH);
}



//=========================================================================
//=========================================================================
//>CLUSTERING =============================================================
//=========================================================================
//----------------------------------------------------------
// SEG >> SEG_Paint
//----------------------------------------------------------
// Paint
//----------------------------------------------------------
void MD2S::SEG_Paint(float** mat,float** mattmp,int row,int col,int lgth,int wdth,int lab)
{
   if ((mattmp[row][col]==0)&&(mat[row][col]==lab))
    {
     mattmp[row][col]=-1;

     if ((col+1)<=(wdth-1))   SEG_Paint(mat,mattmp,row,col+1,lgth,wdth,lab);
     if ((col-1)>=0)          SEG_Paint(mat,mattmp,row,col-1,lgth,wdth,lab);
     if ((row+1)<=(lgth-1))   SEG_Paint(mat,mattmp,row+1,col,lgth,wdth,lab);
     if ((row-1)>=0)          SEG_Paint(mat,mattmp,row-1,col,lgth,wdth,lab);
     }

 else return;
}

//----------------------------------------------------------
// SEG >> SEG_Paint
//----------------------------------------------------------
// Paint
//----------------------------------------------------------
void MD2S::SEG_Paint(float** mat,int** mattmp,int row,int col,int lgth,int wdth,int lab)
{
 if ((mattmp[row][col]==0)&&(mat[row][col]==lab))
    {
     mattmp[row][col]=-1;

     if ((col+1)<=(wdth-1))  SEG_Paint(mat,mattmp,row,col+1,lgth,wdth,lab);
     if ((col-1)>=0)         SEG_Paint(mat,mattmp,row,col-1,lgth,wdth,lab);
     if ((row+1)<=(lgth-1))  SEG_Paint(mat,mattmp,row+1,col,lgth,wdth,lab);
     if ((row-1)>=0)         SEG_Paint(mat,mattmp,row-1,col,lgth,wdth,lab);
     }

 else return;
}

//----------------------------------------------------------
// SEG >> SEG_Identical
//----------------------------------------------------------
// Identical                      
//----------------------------------------------------------
int MD2S::SEG_Identical(int* VctPts,int** MtxCtrs,int nbcl,int nbpar)
{
 int k,l;
 int value=0;

 for(k=0;k<nbcl;k++) 
   { value=1;
     for(l=0;l<nbpar;l++) if (VctPts[l]!=MtxCtrs[k][l]) value=0;
     if (value) return value; }
    
 return value;
}

//----------------------------------------------------------
// SEG >> SEG_SegmentationMCOLORKmeanFast
//----------------------------------------------------------     
// SegmentationCOLORKmean Avec Entier 
//----------------------------------------------------------     
void MD2S::SEG_SegmentationCOLORKmeanFast(float*** img,float** matout,int lgth,int wdth,int NbClass,int SzW,int Seed)
{
 int i,j,k,l;
 int nbpts; 
 int posr,posc;
 int nb;

 //Init
 int NbParam=3*SzW*SzW;   
 
 //Initialisatio memoire
 int** PtsTab=imatrix_allocate_2d(lgth*wdth,NbParam);
 float*** SquWin=fmatrix_allocate_3d(TROIS,SzW,SzW);
 int*  TabLabel=imatrix_allocate_1d(NbParam);
 int* VctLabel=imatrix_allocate_1d(lgth*wdth);  

 //Presentation
 printf("\n   >>> Segmentation-COLOR-Kmean");
 printf("[NbCl:%d][Ws:%d][Par:%d]",NbClass,SzW,NbParam); 
 fflush(stdout);

 //---------------------------------------------------------------
 //Data   
 //----------------------------------------------------------------                       
 nbpts=0;
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  
    {
     for(k=0;k<SzW;k++) for(l=0;l<SzW;l++) 
        { 
	 posr=i-(SzW/2)+k;
         posc=j-(SzW/2)+l;

         if (posr<0)           posr+=lgth;
         if (posr>(lgth-1))    posr-=lgth;  
         if (posc<0)           posc+=wdth;
         if (posc>(wdth-1))    posc-=wdth;
        
         SquWin[0][k][l]=img[0][posr][posc];
         SquWin[1][k][l]=img[1][posr][posc];
         SquWin[2][k][l]=img[2][posr][posc]; 
        }

     //DataStructure
     for(k=0;k<NbParam;k++)  TabLabel[k]=0;
      
     nb=0;
     for(k=0;k<SzW;k++) for(l=0;l<SzW;l++)  
       { TabLabel[(3*nb)+0]=(int)SquWin[0][k][l];
         TabLabel[(3*nb)+1]=(int)SquWin[1][k][l];
         TabLabel[(3*nb)+2]=(int)SquWin[2][k][l]; 
         nb++; }
    
     for(k=0;k<NbParam;k++)    PtsTab[nbpts][k]=TabLabel[k];

     nbpts++;     
    }//----------------------------------------------------------------

 //K-Mean    
 SEG_KmeanFast(PtsTab,NbClass,(lgth*wdth),NbParam,VctLabel,Seed,1);  
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  matout[i][j]=VctLabel[(i*wdth)+j];    

 //Liberation Memoire
 if (PtsTab)         free_imatrix_2d(PtsTab);
 if (SquWin)         free_fmatrix_3d(SquWin,TROIS);
 if (TabLabel)       free_imatrix_1d(TabLabel);
 if (VctLabel)       free_imatrix_1d(VctLabel);
}

//----------------------------------------------------------
// SEG >> SEG_KmeanFast
//----------------------------------------------------------
//  K-Mean  <Avec Entiers>
//----------------------------------------------------------
void MD2S::SEG_KmeanFast(int** MatrixPts,int NbClusters,int NbPts,int NbParam,int* VctLabel,int Seed,int FlgMan)
{
 int i,j,k,l;
 int classe;
 float aleat;
 int nb;
 float  dist,distmin;
 int flag;
 
 //Allocation memoire
 int** MatrixCtrs=imatrix_allocate_2d(NbClusters,NbParam);
 
 //Presentation
 printf("\n  <Seed:%d> -> Funct.Kmean",Seed);
 srand(Seed);
 if (FlgMan) printf("\n  K-Moy > [Dist. Manhattan]"); 


 //--------------------------------------------------------------
 // -[0]- Initialize Centroid
 // --------------------------------------------------------------
  if (0)                    
  for(i=0;i<NbClusters;i++) 
     { aleat=(i+1)*(NbPts/(NbClusters+2));
       for(k=0;k<NbParam;k++) MatrixCtrs[i][k]=MatrixPts[(int)aleat][k]; } 

  if (1)
  for(i=0;i<NbClusters;i++) 
   { nb=0;
     do { aleat=(int)((randomize())*NbPts-1); nb++; }
     while((i)&&(nb<100)&&(SEG_Identical(MatrixPts[(int)aleat],MatrixCtrs,NbClusters,NbParam)));

     if (nb>1) printf(" <|%d Essais|>",nb);

     for(k=0;k<NbParam;k++) MatrixCtrs[i][k]=MatrixPts[(int)aleat][k]; } 
  //------------------------------------------------------------------
  //------------------------------------------------------------------
    
 
  printf("\n    ");
 //================================================================
 //BOUCLE =========================================================
 //================================================================
 for(l=0;l<50;l++)
   {  
       //-[1]- Select Cluster Membership
       //-------------------------------
       flag=0; classe=0;

       for(j=0;j<NbPts;j++)
          { distmin=BIGNUMBER;  

           for(i=0;i<NbClusters;i++)
              { dist=0.0; 
                
                //Euclidean distance
                if (!FlgMan)
                for(k=0;k<NbParam;k++) dist+=CARRE(MatrixCtrs[i][k]-MatrixPts[j][k]);

                //Manhattan distance
		if (FlgMan)
                for(k=0;k<NbParam;k++) dist+=abs(MatrixCtrs[i][k]-MatrixPts[j][k]);
		     
                if (dist<distmin) { distmin=dist; classe=i; }
	      }            
           if (VctLabel[j]!=classe) { VctLabel[j]=classe; flag++; }
	  }

       //-[2]- Stop Criterion & Info
       //---------------------------
       if ((!flag)&&(l>2))  break;
       printf("[%d>%d]",l,flag); fflush(stdout);
          
       
       //-[3]- Update Centroids For Each Clusters
       //----------------------------------------
        for(i=0;i<NbClusters;i++) 
	  {  
           nb=0;    
	   for(k=0;k<NbParam;k++) MatrixCtrs[i][k]=0;
                      
	   for(j=0;j<NbPts;j++) if (VctLabel[j]==i)
	      { nb++;
                for(k=0;k<NbParam;k++) MatrixCtrs[i][k]+=MatrixPts[j][k]; }

           for(k=0;k<NbParam;k++) if (nb) MatrixCtrs[i][k]/=nb;
          }

   }//==============================================================
 //================================================================= 

 //FreeMemory
 if (MatrixCtrs) free_imatrix_2d(MatrixCtrs);
}

//----------------------------------------------------------
// SEG >> SEG_ComputeMapRegFromSoftCont
//----------------------------------------------------------     
//  
//----------------------------------------------------------     
void MD2S::SEG_ComputeMapRegFromSoftCont(float** SoftCont,float** MapReg,int lgth,int wdth,int NbRegMin,int SizeMin)
{
 int i,j;
 float thr;
 int NbRegCur;
 int MaxNbReg;
 float EstThr;

 //Allocation memoire
 float** MatTmp1=fmatrix_allocate_2d(lgth,wdth);
 float** MatTmp2=fmatrix_allocate_2d(lgth,wdth);
 float** MatTmp3=fmatrix_allocate_2d(lgth,wdth);
 float** MatTmp4=fmatrix_allocate_2d(lgth,wdth);

 //Presentation
 printf("\n\n  <NbRegMin:%d>",NbRegMin);
 
 //Init
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) 
   { if (SoftCont[i][j]>=254) MatTmp1[i][j]=254;
     else MatTmp1[i][j]=SoftCont[i][j]; }

 //DEBUG
 float incr=((float)NbRegMin/100.0);

 //Boucle
 MaxNbReg=0;
 EstThr=0;
 if (1)
   for(thr=0.50;thr>=0.20;thr-=0.01)
  {
   for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) MatTmp2[i][j]=MatTmp1[i][j];

   for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  
      { if (MatTmp2[i][j]<=(thr*255)) MatTmp2[i][j]=255.0; else MatTmp2[i][j]=0.0; }

   for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) MatTmp4[i][j]=MatTmp2[i][j];

   SEG_ConvertCertainClassRegionInf(MatTmp2,lgth,wdth,&NbRegCur,SizeMin);

   printf("[%d:(%.0f)]",NbRegCur,thr*100);
   fflush(stdout);
   if (NbRegCur>MaxNbReg) 
     { MaxNbReg=NbRegCur;
       EstThr=thr;
       for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) MatTmp3[i][j]=MatTmp2[i][j]; printf("*"); }
   //else break;
   //if (thr<0.15) break;
  }

 //DEBUG
 //thr=((1-incr)*0.25)+(incr*EstThr);
 thr=((1-0.25)*FACT_CPOT)+(0.25*EstThr); 

 //DEBUG
 if (1) 
   {
    //thr=(float)NbRegMin/100.0;
  
    for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) MatTmp2[i][j]=MatTmp1[i][j];

    for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  
      { if (MatTmp2[i][j]<=(thr*255)) MatTmp2[i][j]=255.0; else MatTmp2[i][j]=0.0; }

    SEG_ConvertCertainClassRegionInf(MatTmp2,lgth,wdth,&NbRegCur,SizeMin);

    for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) MatTmp3[i][j]=MatTmp2[i][j];
   }

 //Solution
 printf("\n   [Thresh::%.3f][Incr=::%.3f][Thr::%.3f]",EstThr,incr,thr);
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) MapReg[i][j]=MatTmp3[i][j]; 

 //DEBUG
 if (0)
   {
    Recal(MapReg,lgth,wdth);
    SaveImagePgm((char*)"MapRegions",MapReg,lgth,wdth);
    SaveImagePgm((char*)"SoftContour",SoftCont,lgth,wdth);
    SaveImagePgm((char*)"RegBelowThr",MatTmp4,lgth,wdth);
    exit(-1);
   }

  //FreeMemory
 if (MatTmp1)     free_fmatrix_2d(MatTmp1);
 if (MatTmp2)     free_fmatrix_2d(MatTmp2);
 if (MatTmp3)     free_fmatrix_2d(MatTmp3);
 if (MatTmp4)     free_fmatrix_2d(MatTmp4);
}

//----------------------------------------------------------
// SEG >> ContrainWithSeg
//----------------------------------------------------------
// ContrainWithSeg(float**,int,int,float**)
//
// Nota: float**overseg en NbRegions
//----------------------------------------------------------
void MD2S::SEG_ContrainWithSeg(float** overseg,int lgth,int wdth,float** seg)
{
  int i,j;
 int NbRegions;
 int NbClasses;
 float ClassMax;
 float ValClassMax;
 float** MatClass;
 float** segtmp;

 //Allocation Memoire
 segtmp=fmatrix_allocate_2d(lgth,wdth);

 //Calcul NbRegions
 NbRegions=0;
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)
   if (overseg[i][j]>NbRegions) NbRegions=(int)overseg[i][j];

 //Calcul NbClasses
 NbClasses=0;
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)
   if (seg[i][j]>NbClasses) NbClasses=(int)seg[i][j];  //!!!(int)overseg[i][j]

 //Info
 NbRegions++;
 NbClasses++;
 if (0)
   { printf("\n\n  ConstrainSeg >> NbRegionsMax[%d] Classes[%d]      ",NbRegions,NbClasses);
     fflush(stdout); }

 //Reservation Memoire
 MatClass=fmatrix_allocate_2d(NbRegions,(NbClasses+1));
 for(i=0;i<NbRegions;i++) for(j=0;j<(NbClasses+1);j++)  MatClass[i][j]=0.0;

 //Remplissage MatClass
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) MatClass[(int)(overseg[i][j])][(int)(seg[i][j])]++;

 //ChercheMax
 for(i=0;i<NbRegions;i++)
   { ClassMax=0;
     ValClassMax=0;
     for(j=0;j<NbClasses;j++) if (MatClass[i][j]>ValClassMax) { ValClassMax=MatClass[i][j]; ClassMax=j; }

     MatClass[i][NbClasses]=ClassMax; }

 //Contrain
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)
   { if (overseg[i][j]) segtmp[i][j]=MatClass[(int)overseg[i][j]][NbClasses];
     else  segtmp[i][j]=seg[i][j]; }

 //Copie
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) seg[i][j]=segtmp[i][j];

 //Liberation
 if (MatClass)  free_fmatrix_2d(MatClass);
 if (segtmp)    free_fmatrix_2d(segtmp);
}

//----------------------------------------------------------
// SEG >> SEG_SegmentationMCOLORKmeanFast_SpatConst
//----------------------------------------------------------     
// SegmentationCOLORKmean Avec Entier 
//----------------------------------------------------------     
void MD2S::SEG_SegmentationCOLORKmeanFast_SpatConst(float*** img,float** matout,int lgth,int wdth,int NbClass,\
 int SzW,int Seed,float** MatReg)
{
 int i,j,k,l;
 int nbpts; 
 int posr,posc;
 int nb;

 //Init
 int NbParam=3*SzW*SzW;   
 
 //Initialisatio memoire
 int** PtsTab=imatrix_allocate_2d(lgth*wdth,NbParam);
 float*** SquWin=fmatrix_allocate_3d(TROIS,SzW,SzW);
 int*  TabLabel=imatrix_allocate_1d(NbParam);
 int* VctLabel=imatrix_allocate_1d(lgth*wdth);  

 //Presentation
 printf("\n   >>> Segmentation-COLOR-Kmean <Spat_Constraint>");
 printf("[NbCl:%d]:::[Ws:%d][Par:%d]",NbClass,SzW,NbParam); 
 fflush(stdout);

 //---------------------------------------------------------------
 //Data   
 //----------------------------------------------------------------                       
 nbpts=0;
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  
    {
     for(k=0;k<SzW;k++) for(l=0;l<SzW;l++) 
        { 
	 posr=i-(SzW/2)+k;
         posc=j-(SzW/2)+l;

         if (posr<0)           posr+=lgth;
         if (posr>(lgth-1))    posr-=lgth;  
         if (posc<0)           posc+=wdth;
         if (posc>(wdth-1))    posc-=wdth;
        
         SquWin[0][k][l]=img[0][posr][posc];
         SquWin[1][k][l]=img[1][posr][posc];
         SquWin[2][k][l]=img[2][posr][posc]; 
        }

     //DataStructure
     for(k=0;k<NbParam;k++)  TabLabel[k]=0;
      
     nb=0;
     for(k=0;k<SzW;k++) for(l=0;l<SzW;l++)  
       { TabLabel[(3*nb)+0]=(int)SquWin[0][k][l];
         TabLabel[(3*nb)+1]=(int)SquWin[1][k][l];
         TabLabel[(3*nb)+2]=(int)SquWin[2][k][l]; 
         nb++; }
    
     for(k=0;k<NbParam;k++)    PtsTab[nbpts][k]=TabLabel[k];

     nbpts++;     
    }//----------------------------------------------------------------

 //K-Mean    
 SEG_KmeanFast_SpatConst(PtsTab,NbClass,(lgth*wdth),NbParam,VctLabel,Seed,1,MatReg,lgth,wdth);  
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  matout[i][j]=VctLabel[(i*wdth)+j];    

 //Liberation Memoire
 if (PtsTab)         free_imatrix_2d(PtsTab);
 if (SquWin)         free_fmatrix_3d(SquWin,TROIS);
 if (TabLabel)       free_imatrix_1d(TabLabel);
 if (VctLabel)       free_imatrix_1d(VctLabel);
}

//----------------------------------------------------------
// SEG >> SEG_KmeanFast_SpatConst
//----------------------------------------------------------
//  K-Mean  <Avec Entiers> <Contraint>
//----------------------------------------------------------
void MD2S::SEG_KmeanFast_SpatConst(int** MatrixPts,int NbClusters,int NbPts,int NbParam,int* VctLabel,int Seed,\
		int FlgMan,float** MatReg,int lgth,int wdth)
{
 int i,j,k,l;
 int classe;
 float aleat;
 int nb;
 float  dist,distmin;
 int flag;

 //Allocation memoire
 int** MatrixCtrs=imatrix_allocate_2d(NbClusters,NbParam);
 float** MatTmp=fmatrix_allocate_2d(lgth,wdth);
 float** MatPot=fmatrix_allocate_2d(lgth,wdth);
 float*  Prop=fmatrix_allocate_1d(NbClusters);
 
 //Presentation
 printf("\n  <Seed:%d> -> Funct.Kmean",Seed);
 //srand(Seed);
 srand(0);
 if (FlgMan) printf("\n  K-Moy > [Dist. Manhattan]"); 


 //--------------------------------------------------------------
 // -[0]- Initialize Centroid
 // --------------------------------------------------------------
  if (0)                    
  for(i=0;i<NbClusters;i++) 
     { aleat=(i+1)*(NbPts/(NbClusters+2));
       for(k=0;k<NbParam;k++) MatrixCtrs[i][k]=MatrixPts[(int)aleat][k]; } 

  if (1)
  for(i=0;i<NbClusters;i++) 
   { nb=0;
     do { aleat=(int)((randomize())*NbPts-1); nb++; }
     while((i)&&(nb<100)&&(SEG_Identical(MatrixPts[(int)aleat],MatrixCtrs,NbClusters,NbParam)));

     if (nb>1) printf(" <|%d Essais|>",nb);

     for(k=0;k<NbParam;k++) MatrixCtrs[i][k]=MatrixPts[(int)aleat][k]; } 
  //------------------------------------------------------------------
  //------------------------------------------------------------------
    
 
  printf("\n    ");
 //================================================================
 //BOUCLE =========================================================
 //================================================================
 for(l=0;l<38;l++)
   {  
       //-[1]- Select Cluster Membership
       //-------------------------------
       flag=0; classe=0;

       for(j=0;j<NbPts;j++)
          { distmin=BIGNUMBER;  

           for(i=0;i<NbClusters;i++)
              { dist=0.0; 
                
                //Euclidean distance
                if (!FlgMan) for(k=0;k<NbParam;k++) dist+=CARRE(MatrixCtrs[i][k]-MatrixPts[j][k]);

                //Manhattan distance
		if (FlgMan) for(k=0;k<NbParam;k++) dist+=abs(MatrixCtrs[i][k]-MatrixPts[j][k]);
		     
                if (dist<distmin) { distmin=dist; classe=i; }
	      }            
           if (VctLabel[j]!=classe) { VctLabel[j]=classe; flag++; }
	  }

       //-[1b]- Contrainte
       //-----------------
       if (l>10)  //(l>10)
          { for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  MatTmp[i][j]=VctLabel[(i*wdth)+j]; 
            SEG_ContrainWithSeg(MatReg,lgth,wdth,MatTmp);
            for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  VctLabel[(i*wdth)+j]=(int)MatTmp[i][j]; 
            printf("C"); }


       //-[2]- Stop Criterion & Info
       //---------------------------
       if ((!flag)&&(l>2))  break;
       printf("[%d>%d]",l,flag); fflush(stdout);
          
       
       //-[3]- Update Centroids For Each Clusters
       //----------------------------------------
        for(i=0;i<NbClusters;i++) 
	  {  
           nb=0;    
	   for(k=0;k<NbParam;k++) MatrixCtrs[i][k]=0;
                      
	   for(j=0;j<NbPts;j++) if (VctLabel[j]==i)
	      { nb++;
                for(k=0;k<NbParam;k++) MatrixCtrs[i][k]+=MatrixPts[j][k]; }

           for(k=0;k<NbParam;k++) if (nb) MatrixCtrs[i][k]/=nb;
          }

   }//==============================================================
 //================================================================= 

 //FreeMemory
 if (MatrixCtrs) free_imatrix_2d(MatrixCtrs);
 if (MatTmp)     free_fmatrix_2d(MatTmp);
 if (MatPot)     free_fmatrix_2d(MatPot);
 if (Prop)       free_fmatrix_1d(Prop); 
}

//----------------------------------------------------------
// SEG >> SEG_ConvertClassRegion
//----------------------------------------------------------
// Convert <ClassRegion>              
//----------------------------------------------------------
void MD2S::SEG_ConvertClassRegion(float** mat,int lgth,int wdth)
{
 int i,j,k,l,m;
 int label;
 float coul;
 int nb;
 int cpt;
 
  //Allocation 
  int** mattmp=imatrix_allocate_2d(lgth,wdth);
  int*  TabGrey=imatrix_allocate_1d(256);
  
  //Init 
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) mattmp[i][j]=0;
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) mat[i][j]=(int)mat[i][j];
  for(i=0;i<256;i++)  TabGrey[i]=0;

  //Check
   for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) 
     if ((mat[i][j]>=0)&&(mat[i][j]<256)) TabGrey[int(mat[i][j])]++;

   for(nb=0,i=0;i<256;i++)  if (TabGrey[i]) nb++;
   printf("\n >> NbClasses [%d]",nb);
   fflush(stdout);
   for(i=0;i<256;i++)  TabGrey[i]=0; 

  //Boucle
  //-------
  cpt=0;
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)
     if (mattmp[i][j]==0)
        {
	 label=(int)mat[i][j];

         SEG_Paint(mat,mattmp,i,j,lgth,wdth,label);

         do 
           { coul=(int)(randomize()*255); }
         while (TabGrey[(int)coul]);

         TabGrey[(int)coul]=1;
         ++cpt;
         
         if (cpt>=255)
	    { for(m=0;m<256;m++)  TabGrey[m]=0;
              cpt=0; printf("*Regions debordent*"); }        

         for(k=0;k<lgth;k++) for(l=0;l<wdth;l++)
	   if (mattmp[k][l]==-1) mat[k][l]=(int)coul;

         for(k=0;k<lgth;k++) for(l=0;l<wdth;l++)
	   if (mattmp[k][l]==-1) mattmp[k][l]=-2;
        }
 
   //Compte Regions
   for(i=0;i<256;i++)  TabGrey[i]=0;
      
   for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) TabGrey[int(mat[i][j])]++;

   for(nb=0,i=0;i<256;i++)  if (TabGrey[i]) nb++;
       printf("\n >> NbRegions [%d]",nb);
   fflush(stdout);

 //Liberation 
 free_imatrix_2d(mattmp); 
 free_imatrix_1d(TabGrey); 
}

//----------------------------------------------------------
// SEG >> SEG_ConvertCertainClassRegionInf
//----------------------------------------------------------
// ConvertCertainClassRegionIng <On ne VisitePasClass[0]>
//
// mat[][]=255 :: Region homogene
// mat[][]=0   :: Non
//
//----------------------------------------------------------
void MD2S::SEG_ConvertCertainClassRegionInf(float** mat,int lgth,int wdth,int* NbReg,int SizeMin)
{
 int i,j,k,l;
 int label;
 int CptReg;
 int Nb,NbRegions;

  //Allocation
  int** mattmp=imatrix_allocate_2d(lgth,wdth);
  int** matInt=imatrix_allocate_2d(lgth,wdth);
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) mattmp[i][j]=0;

 //Init>mattmp[][]=-2::Visite pas  mattmp[][]=0::Visite 
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) if (mat[i][j]==0) mattmp[i][j]=-2;

  //Boucle
  //-------
  CptReg=0;
  NbRegions=0;
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)
     if (mattmp[i][j]==0)
        {
         label=(int)mat[i][j];
         SEG_Paint(mat,mattmp,i,j,lgth,wdth,label);

         ++CptReg;

         Nb=0;
         for(k=0;k<lgth;k++) for(l=0;l<wdth;l++)
           if (mattmp[k][l]==-1) { mat[k][l]=CptReg; Nb++; }

         for(k=0;k<lgth;k++) for(l=0;l<wdth;l++)
           if (mattmp[k][l]==-1) mattmp[k][l]=-2;

         if (Nb>SizeMin) NbRegions++;
        }
  
  //Record
  (*NbReg)=NbRegions;

 //Liberation
 free_imatrix_2d(mattmp);
 free_imatrix_2d(matInt);
}

//=========================================================================
//=========================================================================
//>NETTOYAGE ==============================================================
//=========================================================================
//----------------------------------------------------------
// SEG >> SEG_FuseSmallRegions
//----------------------------------------------------------
// FuseRegions
//
// < Petite regions inferieur taille sz fusione avec
//    classe majoritaire l entourant >
//
//----------------------------------------------------------
int MD2S::SEG_FuseSmallRegionsMax(float** mat,int lgth,int wdth,int* nbreg,int sz,float elg,float** MatxCont,float Thresh)
{
 int i,j,k,l,m,n,t;
 float maxim;
 int  labfuse;
 int cpt;
 int initsize;
 int size_min;
 int nbregion_prec;
 int nbregion_init;
 int NbChgt;
 float Elong;
 float PotCont;
 float MaxPotCont;
 int nbper;
 int cl;

 //Allocation
 float**  mattmp=fmatrix_allocate_2d(lgth,wdth);
 float*   claslb=fmatrix_allocate_1d(256);

 //Init
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) mat[i][j]=(int)mat[i][j];
 NbChgt=0;

 //Init
 size_min=0;
 nbregion_prec=0;
 cpt=10;
 printf("\n  FuseSmallRegionsMax[S:%d][E:%.2f][T:%.0f]",sz,elg,Thresh);
 printf("\n  Regions>");

 //BOUCLE
 //========================================================================
 for(t=0;((t<12)&&(size_min<sz)&&(nbregion_prec!=cpt));t++)
   {

    //Init
    for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) mattmp[i][j]=0;

       //Boucle
       //-------------------------------------
        for(cpt=0,i=0;i<lgth;i++) for(j=0;j<wdth;j++) if (mattmp[i][j]==0)
              {
               initsize=0;
               SEG_Paint(mat,mattmp,i,j,lgth,wdth,(int)mat[i][j]);
               ++cpt;
               for(k=0;k<256;k++)  claslb[k]=0;

               //RegionsVoisines
               for(k=0;k<lgth;k++) for(l=0;l<wdth;l++) if (mattmp[k][l]==-1)
                    { initsize++;
                      for(m=-1;m<2;m++) for(n=-1;n<2;n++)
                        if (((k+m)>=0)&&((k+m)<lgth)&&((l+n)>=0)&&((l+n)<wdth))
                            if (mat[k+m][l+n]<256) claslb[(int)mat[k+m][l+n]]++;  }

               claslb[(int)mat[i][j]]=0;

               //ClasseVoisineMajoritaire
               labfuse=0;
               maxim=0;
               for(k=0;k<256;k++) if (claslb[k]>maxim) { maxim=claslb[k]; labfuse=k; };

               //Calcul Elongation / PotentielContour
               nbper=0;
               MaxPotCont=0;             
               cl=(int)mat[i][j];
               for(k=5;k<(lgth-5);k++) for(l=5;l<(wdth-5);l++) 
               if (mattmp[k][l]==-1) if (mat[k][l]==cl)
                 { if ((mat[k-1][l]!=cl)||(mat[k+1][l]!=cl)||(mat[k][l+1]!=cl)||(mat[k][l-1]!=cl))
		      { nbper++; 
                        PotCont=MatxCont[k][l]; 
                        if (PotCont>MaxPotCont) MaxPotCont=PotCont; } 
                 }

               Elong=((4*PI*initsize/CARRE(1+nbper))*100);
              
               //Si => 
               // [Size(region)<sz] || [Elong(region)<elg]] || [PotCont<Thresh]
               //   => Fusion
               if ((initsize<=sz)||(Elong<=elg)||(MaxPotCont<Thresh))
                  { for(k=0;k<lgth;k++) for(l=0;l<wdth;l++) if (mattmp[k][l]==-1)
                    mat[k][l]=labfuse;
                    NbChgt++; }

               for(k=0;k<lgth;k++) for(l=0;l<wdth;l++) if (mattmp[k][l]==-1) mattmp[k][l]=-2;
              }//---------------------------

     nbregion_init=cpt;
     size_min=lgth*wdth;
     for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) mattmp[i][j]=0;

     nbregion_prec=cpt;
     for(cpt=0,i=0;i<lgth;i++) for(j=0;j<wdth;j++) if (mattmp[i][j]==0)
        {
         initsize=0;
         SEG_Paint(mat,mattmp,i,j,lgth,wdth,(int)mat[i][j]);
         ++cpt;

         for(k=0;k<lgth;k++) for(l=0;l<wdth;l++) if (mattmp[k][l]==-1)  initsize++;

         if (initsize<size_min) size_min=initsize;

         for(k=0;k<lgth;k++) for(l=0;l<wdth;l++) if (mattmp[k][l]==-1) mattmp[k][l]=-2;
       }

      printf("(>%d)",cpt);
      (*nbreg)=cpt;
      fflush(stdout);
   }//=====================================================================

 //Liberation
 if (mattmp)  free_fmatrix_2d(mattmp);
 if (claslb)  free_fmatrix_1d(claslb);

 //Retour
 printf("   |NbChgt >[%d]",NbChgt);
 return NbChgt;
}

//----------------------------------------------------------
// SEG >> SEG_FuseSmallRegions(
//----------------------------------------------------------
// FuseRegions
//
// < Petite regions inferieur taille sz fusione avec
//    classe majoritaire l entourant >
//
//----------------------------------------------------------
int MD2S::SEG_FuseSmallRegions(float** mat,int lgth,int wdth,int* nbreg,int sz)
{
 int i,j,k,l,m,n,t;
 float maxim;
 int  labfuse;
 int cpt;
 int initsize;
 int size_min;
 int nbregion_prec;
 int nbregion_init;
 int NbChgt;

 //Allocation
 float**  mattmp=fmatrix_allocate_2d(lgth,wdth);
 float*   claslb=fmatrix_allocate_1d(256);

 //Init
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) mat[i][j]=(int)mat[i][j];
 NbChgt=0;

 //Init
 size_min=0;
 nbregion_prec=0;
 cpt=10;
 printf("\n  FuseSmallRegions[%d]",sz);
 printf("\n  Regions>");

 //BOUCLE
 //========================================================================
 for(t=0;((t<12)&&(size_min<sz)&&(nbregion_prec!=cpt));t++)
   {

    //Init
    for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) mattmp[i][j]=0;

       //Boucle
       //-------------------------------------
        for(cpt=0,i=0;i<lgth;i++) for(j=0;j<wdth;j++) if (mattmp[i][j]==0)
              {
               initsize=0;
               SEG_Paint(mat,mattmp,i,j,lgth,wdth,(int)mat[i][j]);
               ++cpt;
               for(k=0;k<256;k++)  claslb[k]=0;

               //RegionsVoisines
               for(k=0;k<lgth;k++) for(l=0;l<wdth;l++) if (mattmp[k][l]==-1)
                    { initsize++;
                      for(m=-1;m<2;m++) for(n=-1;n<2;n++)
                        if (((k+m)>=0)&&((k+m)<lgth)&&((l+n)>=0)&&((l+n)<wdth))
                            if (mat[k+m][l+n]<256) claslb[(int)mat[k+m][l+n]]++;  }

               claslb[(int)mat[i][j]]=0;

               //ClasseVoisineMajoritaire
               labfuse=0;
               maxim=0;
               for(k=0;k<256;k++) if (claslb[k]>maxim) { maxim=claslb[k]; labfuse=k; };

               //SiCond. { [Size(region)<sz]  ==> Fusion
               if ((initsize<=sz))
                  { for(k=0;k<lgth;k++) for(l=0;l<wdth;l++) if (mattmp[k][l]==-1)
                    mat[k][l]=labfuse;
                    NbChgt++; }

               for(k=0;k<lgth;k++) for(l=0;l<wdth;l++) if (mattmp[k][l]==-1) mattmp[k][l]=-2;
              }//---------------------------

     nbregion_init=cpt;
     size_min=lgth*wdth;
     for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) mattmp[i][j]=0;

     nbregion_prec=cpt;
     for(cpt=0,i=0;i<lgth;i++) for(j=0;j<wdth;j++) if (mattmp[i][j]==0)
        {
         initsize=0;
         SEG_Paint(mat,mattmp,i,j,lgth,wdth,(int)mat[i][j]);
         ++cpt;

         for(k=0;k<lgth;k++) for(l=0;l<wdth;l++) if (mattmp[k][l]==-1)  initsize++;

         if (initsize<size_min) size_min=initsize;

         for(k=0;k<lgth;k++) for(l=0;l<wdth;l++) if (mattmp[k][l]==-1) mattmp[k][l]=-2;
       }

      printf("(>%d)",cpt);
      (*nbreg)=cpt;
      fflush(stdout);
   }//=====================================================================

 //Liberation
 if (mattmp)  free_fmatrix_2d(mattmp);
 if (claslb)  free_fmatrix_1d(claslb);

 //Retour
 printf("   |NbChgt >[%d]",NbChgt);
 return NbChgt;
}

//----------------------------------------------------------
// SEG >> SEG_MedianFilter
//----------------------------------------------------------
// MedianFilter
//      voisinage SzN*SzN
//----------------------------------------------------------
void MD2S::SEG_MedianFilter(float** mt,int lgth,int wdth,int SzN) 
{
 int i,j,k,l,m;
 int nb;
 float tmp;

 float** mat_tmp;
 float* vect;

 //reservation memoire
 mat_tmp=fmatrix_allocate_2d(lgth,wdth);
 vect=fmatrix_allocate_1d(SzN*SzN);

 //recopie matrice
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)
    mat_tmp[i][j]=mt[i][j];

 //Filtre median
 for(i=(SzN/2);i<(lgth-(SzN/2));i++) for(j=(SzN/2);j<(wdth-(SzN/2));j++)
   {
    nb=0;
    for(k=-(SzN/2);k<=(SzN/2);k++) for(l=-(SzN/2);l<=(SzN/2);l++)  
        vect[nb++]=mat_tmp[i+k][j+l];

    for(l=0;l<((SzN*SzN)-1);l++) for(m=0;m<((SzN*SzN)-1);m++)
       if (vect[m]>vect[m+1]) 
          { tmp=vect[m];
            vect[m]=vect[m+1];
            vect[m+1]=tmp; } 

   mt[i][j]=vect[((SzN*SzN)/2)]; 
   }

 //liberation memoire
 free_fmatrix_2d(mat_tmp);
 free(vect); 
}

//--------------------
//--------------------
//----- CONTOUR ------
//--------------------
//----------------------------------------------------------
// SEG >> SEG_ComputeColGradMap
//----------------------------------------------------------     
// Carte Gradient Couleur
//----------------------------------------------------------     
void MD2S::SEG_ComputeColGradMap(float*** img,float** matout,int lgth,int wdth,int dist)
{
  int i,j,k;
  int posrA,poscA,posrB,poscB;
  float tmp;

  //Loop
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) matout[i][j]=0.0;

  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)
    { 
      posrB=i-dist;
      poscB=j-dist;
      posrA=i+dist;
      poscA=j+dist;

      if (posrB<0)        posrB+=lgth;
      if (poscB<0)        poscB+=wdth;
      if (posrA>(lgth-1)) posrA-=lgth;
      if (poscA>(wdth-1)) poscA-=wdth; 

      tmp=0.0;
      for(k=0;k<TROIS;k++) 
	{ tmp+=CARRE(img[k][posrB][j]-img[k][posrA][j])+CARRE(img[k][i][poscB]-img[k][i][poscA]);
          tmp+=CARRE(img[k][posrB][poscA]-img[k][posrA][poscB])+CARRE(img[k][posrB][poscB]-img[k][posrA][poscA]);
        }
      matout[i][j]=sqrt(tmp); }

  //Recal
  Recal(matout,lgth,wdth);
}

//----------------------------------------------------------
// GradMCS <<MultiColorSpace>> <<avec diagonale>>
//
// dist=3
// div=5
// SzW=7
//----------------------------------------------------------     
void MD2S::SEG_GradMCS(float*** img,float** matgrd,int lgth,int wdth,int dist,int div,int SzW)
 { 
  int i,j,k,l,m;
  int posr,posc;
  int rowU,colU,rowD,colD;
  int rowL,colL,rowR,colR;
  int NbParam;
  float lab,grad,divis,max;

  //Constante
  const int NBBC=10;

  //Initialization
  NbParam=div*div*div; 
  divis=(256.0/(float)div); 

  //Presentation
  printf("\n Calcul GradMultiColorSpace...");
  fflush(stdout);

  //Conversion Couleurs
  int**** imgV=imatrix_allocate_4d(NBBC,TROIS,lgth,wdth);
  for(k=0;k<NBBC;k++) ColorBase(imgV[k],img,lgth,wdth,k);
  
  //Initialisatio memoire
  float***  SquWin=fmatrix_allocate_3d(TROIS,SzW,SzW);
  float*** MatHis=fmatrix_allocate_3d(NbParam,lgth,wdth);

  //Initialization
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) matgrd[i][j]=0.0;    
  for(k=0;k<NbParam;k++) for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) MatHis[k][i][j]=0;

 //=============================================================
 for(m=0;m<NBBC;m++)
    {
      printf("[%d]",m);
      fflush(stdout);

  //Recup Histo ColorSpaces
  //-----------------------
      for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  
        {
         for(k=0;k<SzW;k++) for(l=0;l<SzW;l++) 
            { posr=i-(SzW/2)+k;
              posc=j-(SzW/2)+l;

              if (posr<0)           posr+=lgth;
              if (posr>(lgth-1))    posr-=lgth;  
              if (posc<0)           posc+=wdth;
              if (posc>(wdth-1))    posc-=wdth;

              SquWin[0][k][l]=imgV[m][0][posr][posc];
              SquWin[1][k][l]=imgV[m][1][posr][posc];
              SquWin[2][k][l]=imgV[m][2][posr][posc]; }
   
         for(k=0;k<SzW;k++) for(l=0;l<SzW;l++)  
	    { lab=((int)(SquWin[0][k][l]/divis))*(div*div);
              lab+=((int)(SquWin[1][k][l]/divis))*(div);  
              lab+=((int)(SquWin[2][k][l]/divis)); 
   
              MatHis[(int)(lab)][i][j]++; }

         for(k=0;k<NbParam;k++) MatHis[k][i][j]/=(SzW*SzW);
        }
    
  
  //Compute GradText
  //----------------
   for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  
    {
     rowU=(i-dist); colU=j;
     rowD=(i+dist); colD=j;

     if (rowU<0)        rowU+=lgth;
     if (rowD>(lgth-1)) rowD-=lgth;

     rowL=i; colL=(j-dist);
     rowR=i; colR=(j+dist);

     if (colL<0)        colL+=wdth;
     if (colR>(wdth-1)) colR-=wdth;

     grad=0;
     for(l=0;l<NbParam;l++) 
       { grad+=fabs(MatHis[l][rowU][colU]-MatHis[l][rowD][colD]);
         grad+=fabs(MatHis[l][rowR][colR]-MatHis[l][rowL][colL]); 
         grad+=fabs(MatHis[l][rowU][colL]-MatHis[l][rowD][colR]); 
         grad+=fabs(MatHis[l][rowU][colR]-MatHis[l][rowD][colL]); }

     matgrd[i][j]+=grad;
    }
 
    }//====================================================
 
  //==============================
  //Normalize
  //==============================
  max=0.0;
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) { if (matgrd[i][j]>max) max=matgrd[i][j]; }
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  matgrd[i][j]=((matgrd[i][j]*255.0)/max);
  //==============================

  //Liberation Memoire
  if (SquWin) free_fmatrix_3d(SquWin,TROIS);
  if (MatHis) free_fmatrix_3d(MatHis,NbParam);
  if (imgV)   free_imatrix_4d(imgV,NBBC,TROIS);
 }

//--------------------
//--------------------
//-- SPACE COLOR -----
//--------------------
//----------------------------------------------------------*/
// RGBToHSV 
// --------
// H=[0::360]   S=[0::1]  V=[0::255]
//                        
//----------------------------------------------------------*/
void MD2S::RGBToHSV(float r,float g,float b,float* h,float* s,float* v)
{
 float maxRGB=MAX(MAX(r,g),MAX(g,b));
 float minRGB=MIN(MIN(r,g),MIN(g,b));

 (*v)=maxRGB;                                                

 if (maxRGB==0)   (*s)=0.0;
 else             (*s)= (maxRGB-minRGB)/maxRGB;                    

        
 if ((*s)==0)     (*h)=0.0;                                              
 else
    { if (r==maxRGB)       (*h)=(g-b)/(maxRGB-minRGB);    
      else if (g==maxRGB)  (*h)=2+(b-r)/(maxRGB-minRGB);
      else                 (*h)=4+(r-g)/(maxRGB-minRGB);

      (*h)*=60;           
      if ((*h)<0.0)        (*h)+=360;  }
}

//----------------------------------------------------------*/
// RGBToYIQ  
//
//  |Y|  |0.299   0.587   0.114| |R|
//  |I|= |0.596  -0.275  -0.321| |G|
//  |Q|  |0.212  -0.523   0.311| |B|
//
//  Y=[0::1]
//  I=[-0.5957161349127745527::0.5957161349127745527]
//  Q=[-0.5225910452916111684::0.5225910452916111684]
//                   
//----------------------------------------------------------*/
void MD2S::RGBToYIQ(float r,float g,float b,float* y,float* i,float* q)
{
  //RGB en [0-1]
  r/=255.0;
  g/=255.0;
  b/=255.0;

  //Conversion
 (*y)=(0.299*r)+(0.587*g) +(0.114*b);
 (*i)=(0.596*r)+(-0.275*g)+(-0.321*b);
 (*q)=(0.212*r)+(-0.523*g)+(0.311*b);

 //Result
 if (0)
    {
      (*y)*=255.0;
      (*i)=(((*i)+0.6)/1.20)*255.0;
      (*q)=(((*q)+0.6)/1.20)*255.0; 
    }
}

//----------------------------------------------------------*/
// RGBToXYZ  
//
//   [ X ]   [  0.412453  0.357580  0.180423 ] [ R ] 
//   [ Y ] = [  0.212671  0.715160  0.072169 ] [ G ]
//   [ Z ]   [  0.019334  0.119193  0.950227 ] [ B ].
//
// [X,Y,Z] = [0::255]
//                   
//----------------------------------------------------------*/
void MD2S::RGBToXYZ(float r,float g,float b,float* x,float* y,float* z)
{
  //RGB en [0-1]
  r/=255.0;
  g/=255.0;
  b/=255.0;

  //Conversion
 (*x)=(0.412453*r)+(0.357580*g)+(0.180423*b);
 (*y)=(0.212671*r)+(0.715160*g)+(0.072169*b);
 (*z)=(0.019334*r)+(0.119193*g)+(0.950227*b);

  //RGB 
 (*x)*=255.0;
 (*y)*=255.0;
 (*z)*=255.0;

  //Tests
 if ((*x)>255)  (*x)=255;
 if ((*x)<0)    (*x)=0;

 if ((*y)>255)  (*y)=255;
 if ((*y)<0)    (*y)=0;

 if ((*z)>255)  (*z)=255;
 if ((*z)<0)    (*z)=0;
}

//----------------------------------------------------------*/
// RGBToLAB  
//
// LAB = [0::100][-85::110][0::255]
//
//
//        L* = 116 * (Y/Yn)1/3 - 16    for Y/Yn > 0.008856
//        L* = 903.3 * Y/Yn             otherwise
//
//        a* = 500 * ( f(X/Xn) - f(Y/Yn) )
//        b* = 200 * ( f(Y/Yn) - f(Z/Zn) )
//            where f(t) = t1/3      for t > 0.008856
//                      f(t) = 7.787 * t + 16/116    otherwise
//                   
//----------------------------------------------------------*/
void MD2S::RGBToLAB(float r,float g,float bl,float* l,float* a,float* b)
{
 float x,y,z;
 float xn,yn,zn;

 //RGB en [0-1]
  r/=255.0;
  g/=255.0;
  bl/=255.0;

 //Conversion RGB->XYZ
  RGBToXYZ(r,g,bl,&x,&y,&z);
  RGBToXYZ(1.0,1.0,1.0,&xn,&yn,&zn);

 //Conversion XYZ->LAB
  if ((y/yn)>0.008856)  (*l)=116.0*pow((y/yn),(1.0/3.0))-16.0;
  else                  (*l)=903.3*(y/yn);

  if ((x/xn)>0.008856)  (*a)=500.0*(pow((x/xn),(1.0/3.0)));
  else                  (*a)=7.787*(x/xn)+16.0/116.0;

  if ((y/yn)>0.008856)  (*a)-=500.0*(pow((y/yn),(1.0/3.0)));
  else                  (*a)-=7.787*(y/yn)+16.0/116.0;

  if ((y/yn)>0.008856)  (*b)=500.0*(pow((y/yn),(1.0/3.0)));
  else                  (*b)=7.787*(y/yn)+16.0/116.0;

  if ((z/zn)>0.008856)  (*b)-=500.0*(pow((z/zn),(1.0/3.0)));
  else                  (*b)-=7.787*(z/zn)+16.0/116.0;
}

//----------------------------------------------------------*/
// RGBToLUV  
//
// LUV = [0::100][-86::164][-131::116]
//  
//        L* =  116 * (Y/Yn)1/3 - 16
//        u* =  13L* * ( u' - un' )
//        v* =  13L* * ( v' - vn' )
//
//      un' = 0.2009 
//      vn' = 0.4610 
//
//        u' = 4X / (X + 15Y + 3Z) = 4x / ( -2x + 12y + 3 )
//        v' = 9Y / (X + 15Y + 3Z) = 9y / ( -2x + 12y + 3 ).
//
//       x = 27u' / ( 18u' - 48v' + 36 )
//       y = 12v' / ( 18u' - 48v' + 36 ).
//               
//----------------------------------------------------------*/
void MD2S::RGBToLUV(float r,float g,float bl,float* l,float* u,float* v)
{
 float x,y,z;
 float xn,yn,zn;
 float uprime,vprime;

 //Init
 uprime=0.0;
 vprime=0.0;

 //RGB en [0-1]
  r/=255.0;
  g/=255.0;
  bl/=255.0;

 //Conversion RGB->XYZ
  RGBToXYZ(r,g,bl,&x,&y,&z);
  RGBToXYZ(1.0,1.0,1.0,&xn,&yn,&zn);

  //Calcul uprime/vprime
  if (x+(15*y)+(3*z))
   { uprime=(4*x)/(x+(15*y)+(3*z));
     vprime=(9*y)/(x+(15*y)+(3*z)); }

 //Conversion XYZ->LUV
 (*l)=116.0*pow((y/yn),(1.0/3.0))-16.0;
 (*u)=13*(*l)*(uprime-0.2009);
 (*v)=13*(*l)*(vprime-0.4610);
}

//----------------------------------------------------------*/
// RGBToI1I2I3  
//
// I1=(1/3) (R+G+B)
// I2=(R-B)
// I3=(2G-R-B)/2
//                   
//----------------------------------------------------------*/
void MD2S::RGBToI1I2I3(float r,float g,float bl,float* i1,float* i2,float* i3)
{

  //Conversion RGB -> I1I2I3
  (*i1)=(1.0/3.0)*(r+g+bl);
  (*i2)=(r-bl);
  (*i3)=((1.0/2.0)*(2*g-r-bl));
}

//----------------------------------------------------------*/
// RGBToH1H2H3  
//
// H1= R+G
// H2= R-G
// H3= B- (1.0/2.0)(R+G)
//                   
//----------------------------------------------------------*/
void MD2S::RGBToH1H2H3(float r,float g,float bl,float* h1,float* h2,float* h3)
{

  //Conversion RGB -> H1H2H3
  (*h1)=(r+g);
  (*h2)=(r-g);
  (*h3)=(bl-(1.0/2.0)*(r+g));
}

//----------------------------------------------------------*/
// RGBToTSL
//
// rp=r-(1.0/3.0);
// gp=g-(1.0/3.0);
//
//  L=(1.0/2.0)* [MAX(r,g,b)+MIN(r,g,bl)] 
//
//  Si  (gp>0)  T=((1.0/(2*PI))*atan(rp/gp))+0.25
//  Si  (gp<0)  T=((1.0/(2*PI))*atan(rp/gp))+0.75
//  Si  (!gp)   T=0.0
//
//  S=sqrt[ (9.0/5.0) (rp^2 +gp^2) ]
//                   
//----------------------------------------------------------*/
void MD2S::RGBToTSL(float r,float g,float bl,float* t,float* s,float* l)
{

 float rp,gp;
 
 //RGB en [0-1]
 r/=255.0;
 g/=255.0;
 bl/=255.0;

 //r/gprime
 rp=r-(1.0/3.0);
 gp=g-(1.0/3.0);

 //min/maxRGB
 float maxRGB=MAX(MAX(r,g),MAX(g,bl));
 float minRGB=MIN(MIN(r,g),MIN(g,bl));

 //Conversion RGB -> HSV
 (*l)=(1.0/2.0)*(maxRGB+minRGB);

 if (gp>0)  (*t)=((1.0/(2*PI))*atan(rp/gp))+0.25;
 if (gp<0)  (*t)=((1.0/(2*PI))*atan(rp/gp))+0.75;
 if (!gp)   (*t)=0.0;
 if (*t<0)  (*t)=0.0;

 (*s)=sqrt((9.0/5.0)*(CARRE(rp)+CARRE(gp)));
}

//----------------------------------------------------------*/
// RGBToYCbCr
//
//  Y=(0.2989*r)+(0.5866*g)+(0.1145*bl);
//  Cb=(bl-Y)/(2-2*0.1145);
//  Cr=(r-Y)/(2-2*0.2989);
//
//                   
//----------------------------------------------------------*/
void MD2S::RGBToYCbCr(float r,float g,float bl,float* Y,float* Cb,float* Cr)
{  
 //Conversion RGB -> YCbCr
 (*Y)=(0.2989*r)+(0.5866*g)+(0.1145*bl);
 (*Cb)=(bl-(*Y))/(2-2*0.1145);
 (*Cr)=(r-(*Y))/(2-2*0.2989);
}

//----------------------------------------------------------*/
// RGBToHSL
//
//  H=ACOS[ 0.5(r-g)+(r-b)) / sqrt[ (r-g)^2=(r-b)*(g-b) ] ]
// 
//  L=(1.0/2.0)* [MAX(r,g,b)+MIN(r,g,bl)] 
//
// if (l<=0.5) S=MAX(r,g,b)-MIN(r,g,b)/(MAX(r,g,b)+MIN(r,g,b))
// if (l>0.5)  S=MAX(r,g,b)-MIN(r,g,b)/(2-(MAX(r,g,b)+MIN(r,g,b)))
//                   
//----------------------------------------------------------*/
void MD2S::RGBToHSL(float r,float g,float bl,float* h,float* s,float* l)
{
 float tmp1,tmp2;

 //RGB en [0-1]
 r/=255.0;
 g/=255.0;
 bl/=255.0;

 //min/maxRGB
 float maxRGB=MAX(MAX(r,g),MAX(g,bl));
 float minRGB=MIN(MIN(r,g),MIN(g,bl));

 //Conversion RGB -> HSV
 tmp1=(0.5)*((r-g)+(r-bl));
 tmp2=sqrt((CARRE(r-g))+(r-bl)*(g-bl));
 if (tmp2) (*h)=acos(tmp1/tmp2);
 else      (*h)=0.0;
 
 (*l)=(1.0/2.0)*(maxRGB+minRGB);

 (*s)=0.0;
 
  if ((*l)<=0.5) 
  if (maxRGB+minRGB)
  (*s)=(maxRGB-minRGB)/(maxRGB+minRGB);


  if ((*l)>0.5)  
  if (2.0-(maxRGB+minRGB))
  (*s)=(maxRGB-minRGB)/(2.0-(maxRGB+minRGB));
}

//----------------------------------------------------------*/
// RGBToP1P2
//
//  P1=(1.0/sqrt(2))*(g-r)/sumRGB;
//  P2=(1.0/sqrt(6))*(2*bl-r-g)/sumRGB;
//
//                   
//----------------------------------------------------------*/
void MD2S::RGBToP1P2(float r,float g,float bl,float* P1,float* P2,float* P3)
{  
 float sumRGB=(r+g+bl);

 //Init
 (*P1)=0.0;
 (*P2)=0.0; 

 //Conversion RGB -> P1P2
 if (sumRGB) (*P1)=(1.0/sqrt(2))*(g-r)/sumRGB;
 if (sumRGB) (*P2)=(1.0/sqrt(6))*(2*bl-r-g)/sumRGB;
 (*P3)=0.0;
}

//----------------------------------------------------------
//
//  image[lgth][wdth] 
// 
//  bascol::         
//  -0- RGB ->    [X,Y,Z] = [0,255]
//  -1- HSV ->    H=[0::360]  S=[0::1]   V=[0::255]
//  -2- YIQ -     Y=[0::1]    I=[-0.55::0.55]  Q=[-0.4::0.4]
//  -3- XYZ ->    [X,Y,Z] = [0,255]
//  -4- LAB -     L=[0::100]    A=[-128::143]  B=[-258::237]
//  -5- LUV -     L=[-16::100]  U=[-60::145]   V=[-130::117]  
//
//  -6- i123-     i1=[0::255]    i2=[-255::255]      i3=[-255::255]  
//  -7- h123-     h1=[0::510]    h2=[-255::255]      h3=[-255::255]
//  -8- YCbCr    Y=[0::255]     Cb=[-127.5::127.5]  Cr=[-127.5::127.5]
//  -9- TSL-      T=[0::1]       S=[0::1.265]        L=[0::1]
//  -10- HSL-      H=[0::3.142]   S=[0::1]            L=[0::1] 
//  -11- P1P2      P1=[-0.707::0.707]   P2=[-0.408::0.816]     
//
//           Recalage sur [0-255] 
//
//----------------------------------------------------------     
void MD2S::ColorBase(int***  imgout,float*** imgin,int lgth,int wdth,int bascol)
{
 int i,j;
 float red,green,blue;
 float col1,col2,col3;

 //Boucle                          
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  
    {
         red=imgin[0][i][j];
         green=imgin[1][i][j];
         blue=imgin[2][i][j];

         if (bascol==0)
	    { col1=red;
              col2=green;
              col3=blue;  }

         if (bascol==1)
	    { RGBToHSV(red,green,blue,&col1,&col2,&col3);
              col1=(col1/360.0)*255.0;
              col2=col2*255.0 ;
              col3=col3; }

         if (bascol==2)
	    { RGBToYIQ(red,green,blue,&col1,&col2,&col3);
              col1=col1*255.0;
              col2=((col2+0.55)/1.10)*255.0;
              col3=((col3+0.4)/0.8)*255.0; }

         if (bascol==3)
	    { RGBToXYZ(red,green,blue,&col1,&col2,&col3);
              col1=col1;
              col2=col2;
              col3=col3; } 
          
         if (bascol==4)
	    { RGBToLAB(red,green,blue,&col1,&col2,&col3);
              col1=col1*2.55;
              col2=((col2+128)/271.0)*255.0;              
              col3=((col3+258)/495.0)*255.0; }

          if (bascol==5)
	    { RGBToLUV(red,green,blue,&col1,&col2,&col3);
	       col1=((col1+16)/250.0)*255.0;
               col2=((col2+60)/205.0)*255.0;
               col3=((col3+130)/247.0)*255.0; 
           
               if (col1<0)      col1=0.0;
               if (isnan(col2)) col2=0.0;
               if (isnan(col3)) col3=0.0; }

	  if (bascol==6)
	    { RGBToI1I2I3(red,green,blue,&col1,&col2,&col3); 
              col1=col1; 
              col2=((col2+255)/2.0);
              col3=((col3+255)/2.0);  }

          if (bascol==7)
	    { RGBToH1H2H3(red,green,blue,&col1,&col2,&col3); 
              col1=col1/2.0; 
              col2=((col2+255)/2.0); 
              col3=((col3+255)/2.0); }

          if (bascol==8)
	     { RGBToYCbCr(red,green,blue,&col1,&col2,&col3); 
               col1=col1; 
              col2=col2+127.5; 
               col3=col3+127.5; }
 
          if (bascol==9)
	    { RGBToTSL(red,green,blue,&col1,&col2,&col3); 
              col1=col1*255; 
              col2=255*col2/1.265; 
              col3=col3*255;  }

          if (bascol==10)
	    { RGBToHSL(red,green,blue,&col1,&col2,&col3); 
              col1=255*col1/3.142;  
              col2=col2*254;
              col3=col3*255; }

          if (bascol==11)
	    { RGBToP1P2(red,green,blue,&col1,&col2,&col3); 
              col1=(col1+0.707)*255.0/1.414; 
              col2=(col2+0.408)*255.0/1.224; 
              col3=0.0;   }

          if (col1>255)  col1=255.0;
          if (col2>255)  col2=255.0;
          if (col3>255)  col3=255.0;
          if (col1<0)    col1=0.0;
          if (col2<0)    col2=0.0;
          if (col3<0)    col3=0.0;
        
	 imgout[0][i][j]=(int)col1;
         imgout[1][i][j]=(int)col2;
	 imgout[2][i][j]=(int)col3;  
    }
}

//---------------------------
//-- SEGMENTATION TEXTURE ---
//---------------------------
//----------------------------
//----<Histo><COLOR-BASE>-----
//----------------------------
//----------------------------------------------------------
//  Segmentation <Histo-COLOR-> Kmean
//       < Vraie Histo >
//
//  image[lgth][wdth] en NbClass 
//  Histo divise en <div> bin
//  SizeWindow: <Size>
// 
//  bascol::
//  -----------            
//  -0- RGB 
//  -1- HSV 
//  -2- YIQ 
//  -3- XYZ 
//  -4- LAB 
//  -5- LUV
//
//  -6- i123
//  -7- h123
//  -8- YCbCr  
//  -9- TSL
//  -10-HSL
//  -11- P1P2    
//
//  Recalage sur [0-255] avec
//
//             (C-min)*255/(max-min)
//
//----------------------------------------------------------     
//----------------------------------------------------------     
// SegmentationHistoTEXKmean Avec Entier 
//----------------------------------------------------------     
void MD2S::SEG_SegHistoTEXKmeanFast(float***  img,float** matout,int lgth,int wdth,int NbClass,int div,int SzW,int Seed,int bascol)
{
 int i,j,k,l;
 int nbpts; 
 int posr,posc;
 float divis;
 float lab;
 float col1,col2,col3;

 //Init
 int NbParam=div*div*div; 
 divis=(256.0/(float)div);  
 
 //Initialisatio memoire
 int** PtsTab=imatrix_allocate_2d(lgth*wdth,NbParam);
 float*** SquWin=fmatrix_allocate_3d(3,SzW,SzW);
 float*** ImgColorSpace=fmatrix_allocate_3d(3,lgth,wdth);
 int*  TabLabel=imatrix_allocate_1d(NbParam);
 int* VctLabel=imatrix_allocate_1d(lgth*wdth);  

 //Presentation
 printf("\n\n\n   >>> SegmentationHisto-COLOR-Kmean");
 printf("[NbCl:%d][Div:%d][Ws:%d][Col:%d][Par:%d]",NbClass,div,SzW,bascol,NbParam); 
 if (bascol==0)  printf(" >RGB");
 if (bascol==1)  printf(" >HSV");
 if (bascol==2)  printf(" >YIQ");
 if (bascol==3)  printf(" >XYZ");
 if (bascol==4)  printf(" >LAB");
 if (bascol==5)  printf(" >LUV");
 if (bascol==6)  printf(" >I1I2I3");
 if (bascol==7)  printf(" >H1H2H3");
 if (bascol==8)  printf(" >YCbCr");
 if (bascol==9)  printf(" >TSL");
 if (bascol==10) printf(" >HSL"); 
 if (bascol==11) printf(" >P1P2"); 
 fflush(stdout);
 

 //-------------------------------------------------------------------------
 //Conversion
 //-------------------------------------------------------------------------
 if (bascol==0)
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) 
    { ImgColorSpace[0][i][j]=img[0][i][j];
      ImgColorSpace[1][i][j]=img[1][i][j];
      ImgColorSpace[2][i][j]=img[2][i][j]; }

 if (bascol==1)
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) 
    { RGBToHSV(img[0][i][j],img[1][i][j],img[2][i][j],&col1,&col2,&col3); 
      ImgColorSpace[0][i][j]=(col1/360.0)*255.0;
      ImgColorSpace[1][i][j]=col2*255.0;
      ImgColorSpace[2][i][j]=col3; }

 if (bascol==2)
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) 
    { RGBToYIQ(img[0][i][j],img[1][i][j],img[2][i][j],&col1,&col2,&col3); 
      ImgColorSpace[0][i][j]=col1*255.0;
      ImgColorSpace[1][i][j]=((col2+0.55)/1.10)*255.0;
      ImgColorSpace[2][i][j]=((col3+0.4)/0.8)*255.0; }

 if (bascol==3)
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) 
    { RGBToXYZ(img[0][i][j],img[1][i][j],img[2][i][j],&col1,&col2,&col3);
      ImgColorSpace[0][i][j]=col1;
      ImgColorSpace[1][i][j]=col2;
      ImgColorSpace[2][i][j]=col3; }

 if (bascol==4)
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) 
    { RGBToLAB(img[0][i][j],img[1][i][j],img[2][i][j],&col1,&col2,&col3);
      ImgColorSpace[0][i][j]=col1*2.55;
      ImgColorSpace[1][i][j]=((col2+128)/271.0)*255.0;
      ImgColorSpace[2][i][j]=((col3+258)/495.0)*255.0; }

 if (bascol==5)
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) 
   {  RGBToLUV(img[0][i][j],img[1][i][j],img[2][i][j],&col1,&col2,&col3);
      ImgColorSpace[0][i][j]=((col1+16)/250.0)*255.0;
      ImgColorSpace[1][i][j]=((col2+60)/205.0)*255.0;
      ImgColorSpace[2][i][j]=((col3+130)/247.0)*255.0; }

 if (bascol==6)
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) 
    { RGBToI1I2I3(img[0][i][j],img[1][i][j],img[2][i][j],&col1,&col2,&col3);
      ImgColorSpace[0][i][j]=col1;
      ImgColorSpace[1][i][j]=((col2+255)/2.0);
      ImgColorSpace[2][i][j]=((col3+255)/2.0);  }

 if (bascol==7)
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) 
    { RGBToH1H2H3(img[0][i][j],img[1][i][j],img[2][i][j],&col1,&col2,&col3);
      ImgColorSpace[0][i][j]=col1/2.0; 
      ImgColorSpace[1][i][j]=((col2+255)/2.0); 
      ImgColorSpace[2][i][j]=((col3+255)/2.0); }

 if (bascol==8)
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) 
    { RGBToYCbCr(img[0][i][j],img[1][i][j],img[2][i][j],&col1,&col2,&col3);
      ImgColorSpace[0][i][j]=col1; 
      ImgColorSpace[1][i][j]=col2+127.5; 
      ImgColorSpace[2][i][j]=col3+127.5; }

 if (bascol==9)
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) 
    { RGBToTSL(img[0][i][j],img[1][i][j],img[2][i][j],&col1,&col2,&col3);
      ImgColorSpace[0][i][j]=col1*255;
      ImgColorSpace[1][i][j]=255*col2/1.265; 
      ImgColorSpace[2][i][j]=col3*255;  }

 if (bascol==10)
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) 
    { RGBToHSL(img[0][i][j],img[1][i][j],img[2][i][j],&col1,&col2,&col3);
      ImgColorSpace[0][i][j]=255*col1/3.142;  
      ImgColorSpace[1][i][j]=col2*254;
      ImgColorSpace[2][i][j]=col3*255; }

 if (bascol==11)
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) 
    { RGBToP1P2(img[0][i][j],img[1][i][j],img[2][i][j],&col1,&col2,&col3);
      ImgColorSpace[0][i][j]=(col1+0.707)*255.0/1.414; 
      ImgColorSpace[1][i][j]=(col2+0.408)*255.0/1.224; 
      ImgColorSpace[2][i][j]=0.0; }


 //Debug
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) 
   { if (ImgColorSpace[0][i][j]>255)  ImgColorSpace[0][i][j]=255.0;
     if (ImgColorSpace[1][i][j]>255)  ImgColorSpace[1][i][j]=255.0;
     if (ImgColorSpace[2][i][j]>255)  ImgColorSpace[2][i][j]=255.0;
     if (ImgColorSpace[0][i][j]<0)    ImgColorSpace[0][i][j]=0.0;
     if (ImgColorSpace[1][i][j]<0)    ImgColorSpace[1][i][j]=0.0;
     if (ImgColorSpace[2][i][j]<0)    ImgColorSpace[2][i][j]=0.0; }
 //-------------------------------------------------------------------------- 
  

 //---------------------------------------------------------------
 //Data   
 //----------------------------------------------------------------                       
 nbpts=0;
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  
    {
     for(k=0;k<SzW;k++) for(l=0;l<SzW;l++) 
        { 
	 posr=i-(SzW/2)+k;
         posc=j-(SzW/2)+l;

         if (posr<0)           posr+=lgth;
         if (posr>(lgth-1))    posr-=lgth;  
         if (posc<0)           posc+=wdth;
         if (posc>(wdth-1))    posc-=wdth;
        
         SquWin[0][k][l]=ImgColorSpace[0][posr][posc];
         SquWin[1][k][l]=ImgColorSpace[1][posr][posc];
         SquWin[2][k][l]=ImgColorSpace[2][posr][posc];  
        }

     //DataStructure
     for(k=0;k<NbParam;k++)  TabLabel[k]=0;
      
     for(k=0;k<SzW;k++) for(l=0;l<SzW;l++)  
	{ lab=((int)(SquWin[0][k][l]/divis))*(div*div);
          lab+=((int)(SquWin[1][k][l]/divis))*(div);  
          lab+=((int)(SquWin[2][k][l]/divis));  
          TabLabel[(int)(lab)]++; }
    
     for(k=0;k<NbParam;k++)    PtsTab[nbpts][k]=TabLabel[k];

     nbpts++;     
    }//----------------------------------------------------------------

 //K-Mean    
 SEG_KmeanFast(PtsTab,NbClass,(lgth*wdth),NbParam,VctLabel,Seed,1);
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  matout[i][j]=VctLabel[(i*wdth)+j];    

 //Liberation Memoire
 if (PtsTab)         free_imatrix_2d(PtsTab);
 if (SquWin)         free_fmatrix_3d(SquWin,3);
 if (TabLabel)       free_imatrix_1d(TabLabel);
 if (VctLabel)       free_imatrix_1d(VctLabel);
 if (ImgColorSpace)  free_fmatrix_3d(ImgColorSpace,3);
}


//---------------------
//---VISU CONTOUR------
//---------------------
//----------------------------------------------------------*/
// AddContour                               
//----------------------------------------------------------*/
void MD2S::AddContourImg(float** seg,float*** img,int lgth,int wdth,int gross) 
{
 int i,j,k,l;
 float tmp;
 int dep;

 //Init
 dep=(int)(gross/2);

 //AllocationMemoire
 float** mat_tmp=fmatrix_allocate_2d(lgth,wdth);
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) mat_tmp[i][j]=255.0;

 //Gradient
 for(i=1;i<(lgth-1);i++) for(j=1;j<(wdth-1);j++)
   { tmp=sqrt(CARRE(seg[i+1][j]-seg[i][j])+CARRE(seg[i][j+1]-seg[i][j]));
     if (tmp>0) mat_tmp[i][j]=0.0; }

 //Boucle
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) if (mat_tmp[i][j]==0.0)
      {
       for(k=-dep;k<=dep;k++) for(l=-dep;l<=dep;l++)
	 if (((i+k)>0)&&((i+k)<lgth)&&((j+l)>0)&&((j+l)<wdth))
	  { img[0][i+k][j+l]=0; 
            img[1][i+k][j+l]=0; 
            img[2][i+k][j+l]=0; }
      }    

 //liberation memoire
 free_fmatrix_2d(mat_tmp); 
}

//--------------------------
//---CONTOUR POTENTIEL------
//--------------------------
//----------------------------------------------------------*/
// EstimContPot
//----------------------------------------------------------*/
void MD2S::EstimContPot(float** ImgSeg,float** ImgPot,float** ImgRes,int lgth,int wdth)
{
 int i,j;
 float tmp;

 //AllocationMemoire
 float** ImgCont=fmatrix_allocate_2d(lgth,wdth);
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) ImgCont[i][j]=0.0;
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) ImgRes[i][j]=0.0;

  //Gradient
 for(i=1;i<(lgth-1);i++) for(j=1;j<(wdth-1);j++)
   { tmp=sqrt(CARRE(ImgSeg[i+1][j]-ImgSeg[i][j])+CARRE(ImgSeg[i][j+1]-ImgSeg[i][j]));
     if (tmp>0) ImgCont[i][j]=255.0; }

 //Boucle
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) if (ImgCont[i][j]==255.0)
 ImgRes[i][j]=ImgPot[i][j];

 //liberation memoire
 free_fmatrix_2d(ImgCont);
}

//----------------------------------------------------------*/
// EstimCont
//----------------------------------------------------------*/
void MD2S::EstimCont(float** ImgSeg,float** ImgRes,int lgth,int wdth)
{
 int i,j;
 float tmp;

 //Init
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) ImgRes[i][j]=0.0;

  //Gradient
 for(i=1;i<(lgth-1);i++) for(j=1;j<(wdth-1);j++)
   { tmp=sqrt(CARRE(ImgSeg[i+1][j]-ImgSeg[i][j])+CARRE(ImgSeg[i][j+1]-ImgSeg[i][j]));
     if (tmp>0) ImgRes[i][j]=255.0; }
}

//=======
//=======
// ESSAI
//=======
//=======

//----------------------------------------------------------*/
// EstimCont
//----------------------------------------------------------*/
void MD2S::MoySegForContPot(float*** VctImgSeg,float** ImgRes,int nb,int lgth,int wdth)
{
 int i,j,k;
 float tmp;

 //Init
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) ImgRes[i][j]=0.0;

 //Gradient
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)
   { tmp=0.0;
     for(k=0;k<nb;k++) tmp+=VctImgSeg[k][i][j];
     tmp/=nb; 
     ImgRes[i][j]=tmp; }  
}
