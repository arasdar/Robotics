//---------------------------------------------------
// module   : f_SFSBM.cc                       
// author   : Max Mignotte                            
// date     : Mai 2009                                           
// language : C++                                      
// labo     :                                  
//---------------------------------------------------

//-----------------------------------------------
// FICHIERS INCLUS ------------------------------
//-----------------------------------------------
#include "SFSBM.h"

//------------------------------------------------
// METHODS [CLASS SFSBM] -------------------------
//------------------------------------------------
//----------------------------------------------------------
// Constructor [classe SFSBM]
//----------------------------------------------------------
SFSBM::SFSBM(int*** imgt,int lgth,int wdth)           
 {
  int i,j,k;

  //Initialisation
  length=lgth;
  width=wdth;

  //Allocation Memoire Matrice 
  mat_img=imatrix_allocate_3d(3,length,width);
  mat_seg=fmatrix_allocate_2d(length,width);
  mat_rest=fmatrix_allocate_2d(length,width); 

  mat1=fmatrix_allocate_2d(length,width);
  mat2=fmatrix_allocate_2d(length,width);
  mat3=fmatrix_allocate_2d(length,width);
  mat4=fmatrix_allocate_2d(length,width);
  mat_tmp0=fmatrix_allocate_2d(length,width);
  mat_tmp1=fmatrix_allocate_2d(length,width);

  //Initialisation
  for(k=0;k<3;k++)
  for(i=0;i<length;i++) for(j=0;j<width;j++) 
     mat_img[k][i][j]=imgt[k][i][j];        
 }


//----------------------------------------------------------
// Destructor [class SFSBM]
//----------------------------------------------------------
SFSBM::~SFSBM()
{ 
  //Free Memory Matrx 
  if (mat_img)   free_imatrix_3d(mat_img,3);
  if (mat_seg)   free_fmatrix_2d(mat_seg);
  if (mat_rest)  free_fmatrix_2d(mat_rest);

  if (mat1)  free_fmatrix_2d(mat1);
  if (mat2)  free_fmatrix_2d(mat2);
  if (mat3)  free_fmatrix_2d(mat3);
  if (mat4)  free_fmatrix_2d(mat4);
  if (mat_tmp0)  free_fmatrix_2d(mat_tmp0);
  if (mat_tmp1)  free_fmatrix_2d(mat_tmp1);
}

//----------------------------
//----<Histo><COLOR-BASE>-----
//----------------------------
//----------------------------------------------------------
//  Segmentation <Histo-COLOR> Kmean
//
//  image[lgth][wdth] into NbClass 
//  Histo divided into <div> bin
//  SizeWindow: <SzW>
// 
//  bascol::
//  -----------------------------------            
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
//  -10-HSL-      H=[0::3.142]   S=[0::1]            L=[0::1]    
//  -11- P1P2     P1=[-0.707::0.707]   P2=[-0.408::0.816]   
//  ------------------------------------------
//
//  Recalage sur [0-255] avec :: (C-min)*255/(max-min)
//
//----------------------------------------------------------     
//----------------------------------------------------------     
// SegmentationHistoCOLORKmean (Integer)
//----------------------------------------------------------     
void SFSBM::SegmentationHistoCOLORKmeanFast(int***  img,float** matout,int lgth,int wdth,int NbClass,int div,int SzW,int Seed,int bascol)
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
 
 //Initializatio memoire
 int** PtsTab=imatrix_allocate_2d(lgth*wdth,NbParam);
 float*** SquWin=fmatrix_allocate_3d(3,SzW,SzW);
 float*** ImgColorSpace=fmatrix_allocate_3d(3,lgth,wdth);
 int*  TabLabel=imatrix_allocate_1d(NbParam);
 int* VctLabel=imatrix_allocate_1d(lgth*wdth);  

 //Presentation
 printf("\n   >>> SegmentationHisto-COLOR-Kmean");
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
 KmeanFast(PtsTab,NbClass,(lgth*wdth),NbParam,VctLabel,Seed,1);
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  matout[i][j]=VctLabel[(i*wdth)+j];    

 //Free Memory
 if (PtsTab)         free_imatrix_2d(PtsTab);
 if (SquWin)         free_fmatrix_3d(SquWin,3);
 if (TabLabel)       free_imatrix_1d(TabLabel);
 if (VctLabel)       free_imatrix_1d(VctLabel);
 if (ImgColorSpace)  free_fmatrix_3d(ImgColorSpace,3);
}

//-------------------------------------------//
//---------- SEGMENTATION METHODS -----------//
//-------------------------------------------//
//----------------------------------------------------------
// SFSBM (given a Soft Boundary Map)
//----------------------------------------------------------   
 void SFSBM::SegmentFromSoftBoundaryMap()
{
 int i,j,k;
 char MyBuffImg[200];
 char MyBuffOut[400];	
	char MyBuff[NCHAR];

 //Stack Size Increased (128 MB)
 //----------------------------
 const rlim_t kStackSize=128*1024*1024;   
 struct rlimit rl;
 int result=getrlimit(RLIMIT_STACK,&rl);
 if (result==0)
 if (rl.rlim_cur<kStackSize)
    {  rl.rlim_cur=kStackSize;
       result=setrlimit(RLIMIT_STACK,&rl);
       if (result!=0) fprintf(stderr,"setrlimit returned result = %d\n",result); } 
 
 //Record
 //-------
 strcpy(MyBuffImg,Name_Img);
 strcpy(MyBuffOut,MyBuffImg);
 strcat(MyBuffOut,"_SEG");            

 //Information
 //-----------
 printf("\n ---------------------------------------------------- ");
 printf("\n SFSBM                                                ");
 printf("\n ---------------------------------------------------- ");
 printf("\n");
 printf("\n ---------------------");
 printf("\n               >> Beta  [%f]",Beta); 
 printf("\n               >> Xi    [%.2f]",Xi); 
 printf("\n ---------------------");
 printf("\n               >> NbCl  [%d]",NbCl0); 
 printf("\n ---------------------");
 printf("\n Size          >> [%d - %d]",length,width);
 printf("\n Extension     >> [%s]",Extension);
 printf("\n PathIn+Image  >> [%s]",Name_Img);
 printf("\n PathOut+Image >> [%s]",MyBuffOut);
 printf("\n ---------------------");
 printf("\n");
 fflush(stdout);

 //AllocationMemoire
 float**   segN=fmatrix_allocate_2d(length/SCALE,width/SCALE);
 
 //Init
 Zeros(mat1,length,width);
 Zeros(mat2,length,width);
 Zeros(mat3,length,width);

 //Matrice Potentiel N&B
 for(i=0;i<length;i++) for(j=0;j<width;j++) mat1[i][j]=mat_img[0][i][j];

 //=============================================================
 //> MULTIRESOLUTION 
 //>
 //> Potentiel Thikenned :
 //> Result: Matrix Gradient Potentiel > mat1
 //>       : Matrix Gradient Potentiel Multires > segN
 //>
 //============================================================= 
 multiresMax(mat1,SCALE,length,width,segN);
 Recal(segN,length/SCALE,width/SCALE);

 //Debug
 //SaveImagePgm("POTMultires","",segN,length/SCALE,width/SCALE); 
 //SaveImagePgm("POTinit","",mat1,length,width); 

 //=============================================================
 //> SFSBM                                        
 //============================================================= 
 MarkovSFSBM(mat1,segN,NbCl0,mat_seg,length,width,SIZE_BIGWINDOW,SCALE,Beta,Xi,mat1,segN, mat_img, MyBuff);

 //=============================================================
 //> Conversion 320x214 ou 214x320
 //=============================================================
 float** mat_out;
 if (length>width)    mat_out=fmatrix_allocate_2d(320,214);
 else                 mat_out=fmatrix_allocate_2d(214,320);
 Convert320x214(mat_seg,mat_out,&length,&width);


 //=============================================================
 //> Fuse small Regions  
 //============================================================= 
 for(k=0;k<10;k++) if (!FuseSmallRegions(mat_out,length,width,&NbRegions,SIZE_SMALLREGIONS))  break;  
 printf("\n >>>>>> NbRegions [%d]",NbRegions);
 fflush(stdout);

 //=============================================================
 //> Conversion 481x321 ou 321x481
 //=============================================================
 Convert481x321(mat_out,mat_seg,&length,&width);

 //Convert Class-->Regions
 ConvertClassRegion(mat_seg,length,width);
  

 //Recal
 Recal(mat_seg,length,width);

 //Sauvegarde 
 SaveImagePgm(MyBuffOut,(char*)"",mat_seg,length,width);
}


//----------------------------------------------------------
// Segment from a .ppm file 
//----------------------------------------------------------   
void SFSBM::Segment()
{
 int k;
 float TreshHigh,TreshLow;
 float complexity; 
 char MyBuff[NCHAR];

	int ***ImgMyCont=imatrix_allocate_3d(3,length,width);
	float** display_mat=fmatrix_allocate_2d(length,width);
	bool verbose = true;

 //Stack Size Increased (128 MB)
 const rlim_t kStackSize=128*1024*1024;   
 struct rlimit rl;
 int result=getrlimit(RLIMIT_STACK,&rl);
 if (result==0)
 if (rl.rlim_cur<kStackSize)
    {  rl.rlim_cur=kStackSize;
       result=setrlimit(RLIMIT_STACK,&rl);
       if (result!=0) fprintf(stderr,"setrlimit returned result = %d\n",result); } 

 //Record
 //------
 strcpy(MyBuff,Name_Img);
 strcat(MyBuff,Extension);

 TreshHigh=Size1/100.0;
 TreshLow=Size2/100.0;    

 //Information
 //------------
 printf("\n ----------------------------------------------------- ");
 printf("\n SFSBM                                                 ");
 printf("\n ----------------------------------------------------- ");
 printf("\n");
 printf("\n ---------------------");
 printf("\n               >> Beta    [%.2f]",Beta); 
 printf("\n               >> Xi      [%.2f]",Xi); 
 printf("\n ---------------------");
 printf("\n               >> Seuil Haut|Bas  [%.2f][%.2f]",TreshHigh,TreshLow); 
 printf("\n               >> NbCl    [%d]",NbCl0); 
 printf("\n ---------------------");
 printf("\n Size          >> [%d:%d]",length,width);
 printf("\n Extension     >> [%s]",Extension);
 printf("\n PathIn+Image  >> [%s]",Name_Img);
 printf("\n PathOut+Image >> [%s]",MyBuff);
 printf("\n ---------------------");
 printf("\n");
 fflush(stdout);

 //AllocationMemory
 float**   segN=fmatrix_allocate_2d(length/SCALE,width/SCALE);
 int***    mat_imgN=imatrix_allocate_3d(TROIS,length/SCALE,width/SCALE); 

 //Init
 Zeros(mat1,length,width);
 Zeros(mat2,length,width);
 Zeros(mat3,length,width);

 //=============================================================
 //> Tests ::: Complexity 
 //=============================================================
 complexity=Complexity(mat_img,length,width,NBDIV,SIZE_WINDOW);
 printf("    COMPLEXITY::: ! %f ! [%s]\n",complexity,Name_Img);
 fflush(stdout);
 
 //=============================================================
 //> Cany [mat_img] ::>>> [mat1]
 //  (mat_tmp0)
 //=============================================================
 for(k=0;k<10;k++) 
   { printf("\n >>>>[%d]",k);
     Zeros(mat_tmp0,length,width);   
     Cany(mat_img,mat_tmp0,length,width,TreshHigh,TreshLow,NBDIV+2,k,0); 
     AddContour(mat_tmp0,mat1,length,width,1); }
 Recal(mat1,length,width);

	if (verbose) {
		SaveImagePgm(MyBuff,".1_Canny",mat1,length,width);
	}

 //=============================================================
 //> Kmean ::> [NbClass][div][SzW][Seed][bascol] >>> [mat2]
 //  (mat_tmp0::mat_tmp1)
 //============================================================= 
 for(k=0;k<10;k++) 
    { printf("\n >>>>[%d]",k);
      Zeros(mat_tmp0,length,width);
      Zeros(mat_tmp1,length,width);
      SegmentationHistoCOLORKmeanFast(mat_img,mat_tmp0,length,width,NbCl0+1,NBDIV,SIZE_WINDOW,k,k);
      ComputeContour(mat_tmp1,mat_tmp0,length,width);
      AddContour(mat_tmp1,mat2,length,width,1); }
   Recal(mat2,length,width);

	if (verbose) {
		SaveImagePgm(MyBuff,".1_K_means",mat2,length,width);
	}

 //=============================================================
 //> Add  [mat1::mat2::mat3] >>> mat1   
 //============================================================= 
 AddTwoMatrix(mat1,mat1,mat2,length,width,0.50);
 
 //=============================================================
 //> Multiplication with Contour  >>> [mat1]
 //=============================================================  
 MultGrad2(mat_img,mat1,length,width,0.90);
 Recal(mat1,length,width);

	if (verbose) {
		SaveImagePgm(MyBuff,".1_SoftEdgeMap",mat1,length,width);
	}

 //=============================================================
 //> MULTIRESOLUTION on Potentiel & Thikenning
 //> -----------------------------------------
 //>
 //> Thikenning :
 //> Result: Matrix Gradient Potentiel > mat1
 //>       : Matrix Gradient Potentiel Multires > segN
 //
 //> Potentiel :
 //> Result: Matrix Gradient Potentiel > mat2
 //>       : Matrix Gradient Potentiel Multires > mat3
 //>
 //============================================================= 
 multiresMax(mat1,SCALE,length,width,segN);
 Recal(segN,length/SCALE,width/SCALE);

 copy_mat(mat1,mat2,length,width);
 copy_mat(segN,mat3,length/SCALE,width/SCALE);

 //Debug
 //SaveImagePgm("POTMultires","",segN,length/SCALE,width/SCALE); 
 //SaveImagePgm("POTini","",mat1,length,width);
 //exit(-1);

 Thicken(mat_img,mat1,mat1,length,width);
 Recal(mat1,length,width);

 multires(mat_img,SCALE,length,width,mat_imgN); 
 //SaveImagePpm("IMGMULTIRES","",mat_imgN,length/SCALE,width/SCALE);
 //SaveImagePgm("POTinithick","",mat1,length,width);
 //exit(-1);

 Thicken(mat_imgN,segN,segN,length/SCALE,width/SCALE);
 Recal(segN,length/SCALE,width/SCALE);
 //SaveImagePgm("POTMultires2","",segN,length/SCALE,width/SCALE);
 //exit(-1);

 //=============================================================
 //  SFSBM                                        
 //============================================================= 
 float regul;

 regul=Beta+complexity;  
 if (complexity<=0.50) regul-=0.70;
 MarkovSFSBM(mat1,segN,NbCl0,mat_seg,length,width,SIZE_BIGWINDOW,SCALE,regul,Xi,mat2,mat3, mat_img, MyBuff);

	if (verbose) {
		copy_mat(mat_seg, display_mat,length,width); 
		Recal(display_mat,length,width);	
		SaveImagePgm(MyBuff,".4_ICMLabelled",display_mat,length,width);
		store_contour_image(ImgMyCont, mat_img, mat_seg, length, width); 
		SaveImagePpm(MyBuff,".4_ICMLabelled",ImgMyCont,length,width);
	}

 //=============================================================
 //> Fuse small regions  
 //============================================================= 
 for(k=0;k<10;k++) if (!FuseSmallRegions(mat_seg,length,width,&NbRegions,SIZE_SMALLREGIONS))  break;  
 printf("\n >>>>>> NbRegions [%d]",NbRegions);


 //Convert Class-->Regions
 ConvertClassRegion(mat_seg,length,width);   

	if (verbose) {
		SaveImagePgm(MyBuff,".5_SmallRegionsRemoved",mat_seg,length,width);
		store_contour_image(ImgMyCont, mat_img, mat_seg, length, width); 
		SaveImagePpm(MyBuff,".5_SmallRegionsRemoved",ImgMyCont,length,width);
	}

 //=============================================================
 //> Fuse big regions  
 //============================================================= 
 int fuse;
 for(k=0;k<1;k++) 
   { fuse=FuseBigRegion(mat_seg,mat1,length,width,SIZE_BIGWINDOW,regul);
     printf("\n  >> FuseRegions [%d]",fuse); 
     if (!fuse) break; }
 
  //Copy
  Recal(mat_seg,length,width);

  //Sauvegarde
  	
	SaveImagePgm(MyBuff,".6_final",mat_seg,length,width);
	store_contour_image(ImgMyCont, mat_img, mat_seg, length, width);
	SaveImagePpm(MyBuff, ".6_final", ImgMyCont, length, width);


	free_imatrix_3d(ImgMyCont,3);
	free_fmatrix_2d(display_mat);
	
	

 //AddContourImg
 //strcpy(MyBuff,Name_Img);
 //strcat(MyBuff,"_CONT");
 //AddContourImg(mat_seg,mat_img,length,width,2);
 //SaveImagePpm(MyBuff,"",mat_img,length,width); 
}


