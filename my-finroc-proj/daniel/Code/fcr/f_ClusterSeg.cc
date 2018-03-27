//---------------------------------------------------
// module  : f_ClusterSeg.cc                       
// auteur  : Max Mignotte                            
// date    :                                              
// langage : C++                                      
// labo    :                                  
//---------------------------------------------------

//-----------------------------------------------
// FICHIERS INCLUS ------------------------------
//-----------------------------------------------
#include "ClusterSeg.h"

//------------------------------------------------
// METHODES DE LA CLASSE CLUSTERSEG  -------------
//------------------------------------------------
//----------------------------------------------------------
// Constructeur de la classe ClusterSeg
//----------------------------------------------------------
ClusterSeg::ClusterSeg(int*** imgt,int lgth,int wdth)           
 {
  int i,j,k;

  //Initialisation
  length=lgth;
  width=wdth;
  SizeS=1;

  //Allocation Memoire Matrice 
  mat_img=imatrix_allocate_3d(3,length,width);
  mat_imgc=imatrix_allocate_3d(3,length,width);
  mat_seg=fmatrix_allocate_2d(length,width);
  smat_seg=fmatrix_allocate_3d(NB_ESPACE_COUL,length,width);

  //Allocation Windows
  win_tabseg=imatrix_allocate_2d(length,width*NB_ESPACE_COUL);

  //Initialisation
  for(k=0;k<3;k++)
  for(i=0;i<length;i++) for(j=0;j<width;j++) 
     { mat_img[k][i][j]=imgt[k][i][j];
       mat_imgc[k][i][j]=imgt[k][i][j]; }
 }


//----------------------------------------------------------
// Destructeur de la classe ClusterSeg
//----------------------------------------------------------
ClusterSeg::~ClusterSeg()
{ 
  //Libere Memoire Matrice 
  if (mat_img)    free_imatrix_3d(mat_img,3);
  if (mat_imgc)   free_imatrix_3d(mat_imgc,3); 
  if (mat_seg)    free_fmatrix_2d(mat_seg);
  if (smat_seg)   free_fmatrix_3d(smat_seg,NB_ESPACE_COUL); 
   
  //Libere Windows
  if (win_tabseg)  free_imatrix_2d(win_tabseg);
}

//------------------//
//--WINDOW- N SEGS--//
//------------------//
//----------------------------------------------------------
// ComputeWindows
//----------------------------------------------------------     
void ClusterSeg::ComputeWindows(int lgth,int wdth,int nbW)
{
  int k,i,j;

  //Recal
  for(k=0;k<nbW;k++) Recal(smat_seg[k],lgth,wdth); 

 //Windows 
 for(k=0;k<nbW;k++) for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  
   win_tabseg[i][j+(k*wdth)]=(int)smat_seg[k][i][j];

 //Visu
 SizeS=nbW;
}

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
//  -0- RGB ->    [X,Y,Z] = [0,255]
//  -1- HSV ->    H=[0::360]  S=[0::1]       V=[0::255]
//  -2- YIQ -     Y=[0::1]    I=[-0.55::0.55]  Q=[-0.4::0.4]
//  -3- XYZ ->    [X,Y,Z] = [0,255]
//  -4- LAB -     L=[0::100]    A=[-128::143]  B=[-258::237]
//  -5- LUV -     L=[-16::100]  U=[-60::145]   V=[-130::117]    
//                  ([min::max])
//
//  Recalage sur [0-255] avec
//
//             (C-min)*255/(max-min)
//
//----------------------------------------------------------     
void ClusterSeg::SegmentationHistoCOLORKmean(int***  img,float** matout,int lgth,int wdth,int NbClass,
					     int div,int SzW,int Seed,int bascol)
{
 int i,j,k,l;
 int nbpts; 
 int posr,posc;
 float divis;
 float lab;
 float red,green,blue;
 float col1,col2,col3;
 float col1_min,col2_min,col3_min;
 float col1_max,col2_max,col3_max;

 //Init
 int NbParam=div*div*div; 
 divis=(256.0/(float)div); 

 red=green=blue=0.0;
 col1=col2=col3=0.0; 
 col1_min=col2_min=col3_min=10000;
 col1_max=col2_max=col3_max=0;
 
 //Initialisatio memoire
 float** TabPts=fmatrix_allocate_2d(NbParam,lgth*wdth);
 float** PtsTab=fmatrix_allocate_2d(lgth*wdth,NbParam);
 float*** SquWin=fmatrix_allocate_3d(3,SzW,SzW);
 float*  TabLabel=fmatrix_allocate_1d(NbParam);
 int* VctLabel=imatrix_allocate_1d(lgth*wdth);  

 for(k=0;k<NbParam;k++) TabLabel[k]=0.0;

 //Presentation
 printf("\n\n\n   >>> SegmentationHisto-COLOR-Kmean [NbCl:%d][Div:%d][Ws:%d][Col:%d][Par:%d]",NbClass,div,SzW,bascol,NbParam); 
 if (bascol==0) printf(" >RGB");
 if (bascol==1) printf(" >HSV");
 if (bascol==2) printf(" >YIQ");
 if (bascol==3) printf(" >XYZ");
 if (bascol==4) printf(" >LAB");
 if (bascol==5) printf(" >LUV");
 fflush(stdout);

 //Data                          
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

         red=img[0][posr][posc];
         green=img[1][posr][posc];
         blue=img[2][posr][posc];

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
               if (isnan(col3)) col3=0.0;
              }

          //Debug
	  if (col1>255)             printf("[1>%f]",col1);
          if (col2>255)             printf("[2>%f]",col2);
          if (col3>255)             printf("[3>%f]",col3);
          if (col1<0)               printf("[1<%f]",col1);
          if (col2<0)               printf("[2<%f]",col2);
          if (col3<0)               printf("[3<%f]",col3);
          if (isnan(col1))          printf("[*1*]");
          if (isnan(col2))          printf("[*2*]");
          if (isnan(col3))          printf("[*3*]");
          fflush(stdout);

          if (col1>255)  col1=255.0;
          if (col2>255)  col2=255.0;
          if (col3>255)  col3=255.0;
          if (col1<0)    col1=0.0;
          if (col2<0)    col2=0.0;
          if (col3<0)    col3=0.0;
        
         SquWin[0][k][l]=col1;
         SquWin[1][k][l]=col2;
         SquWin[2][k][l]=col3;  
        }

     //Test
     if (col1<col1_min) col1_min=col1;
     if (col2<col2_min) col2_min=col2;
     if (col3<col3_min) col3_min=col3;
     if (col1>col1_max) col1_max=col1;
     if (col2>col2_max) col2_max=col2;
     if (col3>col3_max) col3_max=col3;

     for(k=0;k<NbParam;k++)  TabLabel[k]=0.0;
      
     for(k=0;k<SzW;k++) for(l=0;l<SzW;l++)  
	{ lab=((int)(SquWin[0][k][l]/divis))*(div*div);
          lab+=((int)(SquWin[1][k][l]/divis))*(div);  
          lab+=((int)(SquWin[2][k][l]/divis)); 
   
          TabLabel[(int)(lab)]++; 
        }
    
     for(k=0;k<NbParam;k++)  TabLabel[k]/=(SzW*SzW);
     for(k=0;k<NbParam;k++)  TabPts[k][nbpts]=TabLabel[k];

     nbpts++;     
   }

 //Info
 printf("\n         [x>%.1f:%.1f][y>%.1f:%.1f][z>%.1f:%.1f]",col1_min,col1_max,col2_min,col2_max,col3_min,col3_max);

 //Input
 for(i=0;i<(lgth*wdth);i++) for(k=0;k<NbParam;k++)  PtsTab[i][k]=TabPts[k][i];

 //K-Mean    
 Kmean(PtsTab,NbClass,(lgth*wdth),NbParam,VctLabel,Seed,0);
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  matout[i][j]=VctLabel[(i*wdth)+j];    

 //Liberation Memoire
 if (TabPts)   free_fmatrix_2d(TabPts);
 if (PtsTab)   free_fmatrix_2d(PtsTab);
 if (SquWin)   free_fmatrix_3d(SquWin,3);
 if (TabLabel) free_fmatrix_1d(TabLabel);
 if (VctLabel) free_imatrix_1d(VctLabel);
}


//---------------------------------------
//--------------FUSION-------------------
//---<Vect><Label><(SzW*SzW)voisins>---
//---------------------------------------
//------------------------------------------------------------------
//  Segmentation <VectLabel> Kmean   
//                  (SzW*SzW) voisins 
//
//    Input:  vctlab[nblabfd][lgth][wdth]  de chps d'etiquettes
//   Output:  mat[][]
//
//          nblabfd : Nb, Segmentations
//          nbcps   : Nb classe par segmentation         
//
//          clustering en <NbClass> 
//          SizeWindow    <SzW>
//          avec Seed     <Seed>
//          et            <mv>
//
//          Distance de Bhattacharya : Oui
//
//--------------------------------------------------------------------  
void ClusterSeg::SegmentationVectLabel1Kmean(float**  mat,float*** vctlab,int lgth,int wdth,int nblabfd,int nbcps,int NbClass,int SzW,int Seed)
{
 int i,j,k,l,m;
 int posr,posc;
 int nbpts; 
 int NbParam;
 int decalag;  
 int lab;

  //Presentation
 printf("\n\n\n     >>> SegmentationVectLabel-1-Kmean"); 
 printf("   (> Nb Labels Fields [%d])",nblabfd);
 printf("   (> [NbCl:%d][Ws:%d]",NbClass,SzW); 
 fflush(stdout);

 //Information
 float*  NbLab=fmatrix_allocate_1d(nblabfd);

 for(k=0;k<nblabfd;k++) NbLab[k]=nbcps;
 
 NbParam=0;
 for(k=0;k<nblabfd;k++)  NbParam+=(int)NbLab[k];
 if (1) printf(" >>> [%d]\n",(int)NbParam);
 fflush(stdout);
 
 //Allocation Memoire
 float** PtsTab=fmatrix_allocate_2d(lgth*wdth,NbParam);
 float*** SquWin=fmatrix_allocate_3d(nblabfd,SzW,SzW);
 float*  TabLabel=fmatrix_allocate_1d(NbParam);
 int* VctLabel=imatrix_allocate_1d(lgth*wdth);  

 //Data                          
 nbpts=0;
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  
    {
     for(m=0;m<nblabfd;m++)
     for(k=0;k<SzW;k++) for(l=0;l<SzW;l++) 
        { 
	 posr=i-(SzW/2)+k;
         posc=j-(SzW/2)+l;

         if (posr<0)           posr+=lgth;
         if (posr>(lgth-1))    posr-=lgth;  
         if (posc<0)           posc+=wdth;
         if (posc>(wdth-1))    posc-=wdth;

         SquWin[m][k][l]=vctlab[m][posr][posc];
        }

     for(k=0;k<NbParam;k++)  TabLabel[k]=0.0;

     decalag=0;
     for(m=0;m<nblabfd;m++)
       { 
        if (m>0) decalag+=(int)NbLab[m-1];
        else     decalag=0; 
         
        for(k=0;k<SzW;k++) for(l=0;l<SzW;l++)   
	   {
	    lab=((int)(SquWin[m][k][l]))+decalag;

            //Debug
            if (lab>=NbParam)
	       { printf("[%d(%d)(%d)(%d)]",(int)lab,m,decalag,(int)SquWin[m][k][l]); 
                 fflush(stdout); }

	    TabLabel[lab]++; 
           } 
        }

     for(k=0;k<NbParam;k++)  TabLabel[k]/=(SzW*SzW*nblabfd);

     for(k=0;k<NbParam;k++)  PtsTab[nbpts][k]=TabLabel[k];
     nbpts++;
    }

 //>KmeanClustering
 Kmean(PtsTab,NbClass,(lgth*wdth),NbParam,VctLabel,Seed,1);   
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  mat[i][j]=VctLabel[(i*wdth)+j];
     

 //Liberation Memoire
 if (PtsTab)    free_fmatrix_2d(PtsTab); 
 if (NbLab)     free_fmatrix_1d(NbLab);
 if (SquWin)    free_fmatrix_3d(SquWin,nblabfd);
 if (TabLabel)  free_fmatrix_1d(TabLabel);
 if (VctLabel) free_imatrix_1d(VctLabel); 
}

//-------------------------//
//--METHODES SEGMENTATION--//
//-------------------------//
//----------------------------------------------------------
// Main Algorithm
//----------------------------------------------------------    
void ClusterSeg::Segmente()
{
 int k;
 int nb;

 //Pour Tests
 char MyBuff[100];
 strcpy(MyBuff,Extension);
 const int NB_SEGS=6;

 //Information
 //-----------
 printf("\n -----");
 printf("\n    > NbCl1 | NbCl2  [%d][%d]",NbCl1,NbCl2); 
 printf("\n    > DistBhatta  >>>>  [%.3f]",DistBhatta);
 printf("\n -----");
 printf("\n  NbBins [%d]",NbBins);  
 printf("\n    > Size1 | Size2  [%d][%d]",Size1,Size2); 
 printf("\n -----");
 printf("\n Extension >> [%s]",Extension);
 printf("\n -----");
 printf("\n    > SizeMin     >>>>  [%d]",SizeMin); 
 printf("\n -----");
 printf("\n");
 printf("\n Image: [%s]",Name_Img);
 printf("  Size:[%d - %d]",length,width);
 printf("\n");
 fflush(stdout);

 //=========================
 // ALGO ===================
 //=========================
  //>ColorSegmentation 
  for(nb=0,k=0;k<NB_SEGS;k++) 
  SegmentationHistoCOLORKmean(mat_img,smat_seg[k],length,width,NbCl1,NbBins,Size1,10*k,k%6);

  //Fusion 
  SegmentationVectLabel1Kmean(mat_seg,smat_seg,length,width,NB_SEGS,NbCl1,NbCl2,Size2,Seed);  
  Recal(mat_seg,length,width); 

  //Sauvegarde
  if (0) SaveImagePgm(Name_Img,"FUSION",mat_seg,length,width); 

  //Final Merging
  FuseSmallRegions(mat_seg,length,width,70);  
  ConvertClassRegion(mat_seg,length,width);   

  if (DistBhatta>0)
  for(;;) if (!FuseRegionBhattaharyaMIN(mat_seg,mat_img,length,width,DistBhatta,NbBins-1,1.2)) break;  
   

	// ------ My additions --------
	//
	//
	int ***ImgMyCont=imatrix_allocate_3d(3,length,width);
	char myImgName [NCHAR];		

	store_contour_image(ImgMyCont, mat_imgc, mat_seg, length, width);
	strcpy(myImgName, std::string(Name_Img).append("_MYCONT").c_str());	
	SaveImagePpm(myImgName, "", ImgMyCont,length,width);
	free_imatrix_3d(ImgMyCont,3);
	
	//
	//
	// ------- My additions ---------



  //Contour
  AddContourImg(mat_seg,mat_imgc,length,width,2);
        
  //Sauvegarde
  SaveImagePgm(Name_Img,Extension,mat_seg,length,width);

  if (flag_save)
     SaveImagePpm(Name_Img,Extension,mat_imgc,length,width);

  //===================

 //Affichage 
 if (flag_visu)  
   { ComputeWindows(length,width,NB_SEGS); }

 //Sauvegarde
  if (0)
     { 
      Recal(smat_seg[0],length,width); 
      SaveImagePgm(Name_Img,"RGB",smat_seg[0],length,width);
      Recal(smat_seg[1],length,width);
      SaveImagePgm(Name_Img,"HSV",smat_seg[1],length,width);
      Recal(smat_seg[2],length,width);
      SaveImagePgm(Name_Img,"YIQ",smat_seg[2],length,width);
      Recal(smat_seg[3],length,width);
      SaveImagePgm(Name_Img,"XYZ",smat_seg[3],length,width);   
      Recal(smat_seg[4],length,width);
      SaveImagePgm(Name_Img,"LAB",smat_seg[4],length,width);
      Recal(smat_seg[5],length,width);
      SaveImagePgm(Name_Img,"LUV",smat_seg[5],length,width);             
     }      
}

