//------------------------------------------------------
// module  : FunctionSFSBM.cc
// auteur  : Mignotte Max
// date    :
// version : 1.0
// langage : C++
// labo    : DIRO
// note    :
//------------------------------------------------------
// quelques fonctions utiles 

//------------------------------------------------
// FICHIERS INCLUS -------------------------------
//------------------------------------------------
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "FunctionSFSBM.h"



//------------------------ My additions -------------
//
//

void store_contour_image(int ***ImgMyCont, int ***ImgC, float **ImgSeg, int height, int width) {

	int *orig, *regmap, *out;
	orig = (int *) malloc(width*height*3*sizeof(int));
	out = (int *) malloc(width*height*3*sizeof(int));
	regmap = (int *) malloc(width*height*sizeof(int));
	
	
	// copy rgb image and region map
	for (int y=0; y<height; y++) {
		for (int x=0; x<width; x++) {
			orig[(y*width+x)*3] = (int) ImgC[0][y][x];
			orig[(y*width+x)*3 + 1] = (int) ImgC[1][y][x];
			orig[(y*width+x)*3 + 2] = (int) ImgC[2][y][x];
			regmap[y*width+x] = (int) ImgSeg[y][x];
		}
	}
	
	draw_edges(orig,regmap, out, height, width, 3, 0.9);
	
	/*
	// copy back
	for (int y=0; y<height; y++) {
		for (int x=0; x<width; x++) {
			ImgMyCont[0][y][x] = orig[(y*width+x)*3];
			ImgMyCont[1][y][x] = orig[(y*width+x)*3 + 1];
			ImgMyCont[2][y][x] = orig[(y*width+x)*3 + 2];
		}
	}
	*/
	// copy back
	for (int y=0; y<height; y++) {
		for (int x=0; x<width; x++) {
			ImgMyCont[0][y][x] = out[(y*width+x)*3];
			ImgMyCont[1][y][x] = out[(y*width+x)*3 + 1];
			ImgMyCont[2][y][x] = out[(y*width+x)*3 + 2];
		}
	}
	
}


// function that draws contour line around regions
// arg RGB0: original color image
// arg rmap: region map, dont care where region numbers start
// arg RGB: image-sized buffer that stores the resulting overlayed image
void draw_edges(int *RGB0,int *rmap, int *RGB,
    int ny,int nx,int dim,float displayintensity)
{
  int iy,ix,i,j,datasize,l1,l2,mapsize;
  datasize = ny*nx*dim;
	for (i=0;i<datasize;i++) RGB[i] = RGB0[i]*displayintensity;
	
  l1 = 0;
  for (iy=0;iy<ny;iy++)
  {
    for (ix=0;ix<nx-1;ix++)
    {
      l2 = l1+1;
      if (rmap[l1]!=rmap[l2])
      {
        for (j=0;j<dim;j++) { RGB[dim*l1+j]=255; RGB[dim*l2+j]=255; }
      }
      l1++;
    }
    l1++;
  }
  l1 = 0;
  for (iy=0;iy<ny-1;iy++)
  {
    for (ix=0;ix<nx;ix++)
    {
      l2 = l1+nx;
      if (rmap[l1]!=rmap[l2])
      {
        for (j=0;j<dim;j++) { RGB[dim*l1+j]=255; RGB[dim*l2+j]=255; }
      }
      l1++;
    }
  }
/*
  for (i=0;i<mapsize;i++)
  {
    if (rmap[i]==0)
    {
      for (j=0;j<dim;j++) RGB[dim*i+j]=0;
    }
  }
*/	
}

//
//
// -------------------- My additions -------------------------







//--------------------------//
//-- Matrice de Flottant ---//
//--------------------------//
//---------------------------------------------------------
//  alloue de la memoire pour une matrice 1d de float
//----------------------------------------------------------
float* fmatrix_allocate_1d(int hsize)
 {
  float* matrix;
  matrix=new float[hsize]; return matrix; 
 }

//----------------------------------------------------------
//  alloue de la memoire pour une matrice 2d de float
//----------------------------------------------------------
float** fmatrix_allocate_2d(int vsize,int hsize)
 {
  float** matrix;
  float *imptr;

  matrix=new float*[vsize];
  imptr=new  float[(hsize)*(vsize)];
  for(int i=0;i<vsize;i++,imptr+=hsize) matrix[i]=imptr;
  return matrix;
 }

//----------------------------------------------------------
// alloue de la memoire pour une matrice 3d de float
//----------------------------------------------------------
float*** fmatrix_allocate_3d(int dsize,int vsize,int hsize)
 {
  float*** matrix;

  matrix=new float**[dsize];

  for(int i=0;i<dsize;i++)
    matrix[i]=fmatrix_allocate_2d(vsize,hsize);
  return matrix;
 }

//-----------------------//
//-- Matrice d'entiers --//
//-----------------------//
//---------------------------------------------------------
//  alloue de la memoire pour une matrice 1d de int
//----------------------------------------------------------
int* imatrix_allocate_1d(int hsize)
 {
  int* matrix;
  matrix=new int[hsize]; return matrix; }

//----------------------------------------------------------
//  alloue de la memoire pour une matrice 2d de int
//----------------------------------------------------------
int** imatrix_allocate_2d(int vsize,int hsize)
 {
  int** matrix;
  int*  imptr;

  matrix=new int*[vsize];
  imptr=new  int[(hsize)*(vsize)];
  for(int i=0;i<vsize;i++,imptr+=hsize) matrix[i]=imptr;
  return matrix;
 }

//----------------------------------------------------------
// alloue de la memoire pour une matrice 3d de int
//----------------------------------------------------------
int*** imatrix_allocate_3d(int dsize,int vsize,int hsize)
 {
  int*** matrix;

  matrix=new int**[dsize];

  for(int i=0;i<dsize;i++)
    matrix[i]=imatrix_allocate_2d(vsize,hsize);
  return matrix;
 }

//--------------------------//
//-- Matrice de Double -----//
//--------------------------//
//---------------------------------------------------------
//  alloue de la memoire pour une matrice 1d de double
//----------------------------------------------------------
double* dmatrix_allocate_1d(int hsize)
 {
  double* matrix;
  matrix=new double[hsize]; return matrix; }

//----------------------------------------------------------
//  alloue de la memoire pour une matrice 2d de double
//----------------------------------------------------------
double** dmatrix_allocate_2d(int vsize,int hsize)
 {
  double** matrix;
  double *imptr;

  matrix=new double*[vsize];
  imptr=new  double[(hsize)*(vsize)];
  for(int i=0;i<vsize;i++,imptr+=hsize) matrix[i]=imptr;
  return matrix;
 }

//---------------------------//
//--- Liberation memoire ----//
//---------------------------//
//----------------------------------------------------------
// libere la memoire de la matrice 1d de float
//----------------------------------------------------------
void free_fmatrix_1d(float* pmat)
{ delete[] pmat; }

//----------------------------------------------------------
// libere la memoire de la matrice 2d de float
//----------------------------------------------------------
void free_fmatrix_2d(float** pmat)
{ delete[] (pmat[0]);
  delete[] pmat;}

//----------------------------------------------------------
// libere la memoire de la matrice 3d de float
//----------------------------------------------------------
void free_fmatrix_3d(float*** pmat,int dsize)
{
 for(int i=0;i<dsize;i++)
  {
   delete[] (pmat[i][0]);
   delete[] (pmat[i]);
   }
 delete[] (pmat);
}

//----------------------------------------------------------
// libere la memoire de la matrice 1d de int
//----------------------------------------------------------
void free_imatrix_1d(int* pmat)
{ delete[] pmat; }

//----------------------------------------------------------
// libere la memoire de la matrice 2d de int
//----------------------------------------------------------
void free_imatrix_2d(int** pmat)
{ delete[] (pmat[0]);
  delete[] pmat;}

//----------------------------------------------------------
// libere la memoire de la matrice 3d de int
//----------------------------------------------------------
void free_imatrix_3d(int*** pmat,int dsize)
{ for(int i=0;i<dsize;i++)
   {
    delete[] (pmat[i][0]);
    delete[] (pmat[i]);
    }
  delete[] (pmat); }


//----------------------------------------------------------
// libere la memoire de la matrice 1d de double
//----------------------------------------------------------
void free_dmatrix_1d(double* pmat)
{ delete[] pmat; }

//----------------------------------------------------------
// libere la memoire de la matrice 2d de double
//----------------------------------------------------------
void free_dmatrix_2d(double** pmat)
{ delete[] (pmat[0]);
  delete[] pmat;}

//--------------------//
//-- Matrix Gestion --//
//--------------------//
//----------------------------------------------------------
// copie une matrice dans une autre
//----------------------------------------------------------
void copy_mat(float** pmatl1,float** pmatl2,int lgth,int wdth)
{

  for(int i=0;i<lgth;i++) for(int j=0;j<wdth;j++)
     { pmatl2[i][j]=pmatl1[i][j]; }
}

/*----------------------------------------------------------*/
/* Recal                                                    */
/*----------------------------------------------------------*/
void Recal(float** mat,int lgth,int wdth)
{
 int i,j;
 float max,min;

 /*Initialisation*/
 max=-1000000.0;
 min=100000000;

 /*Recherche du min*/
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)
    if (mat[i][j]<min) min=mat[i][j];

 /*plus min*/
   for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)
    mat[i][j]-=min;

 /*Recherche du max*/
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) 
    if (mat[i][j]>max) max=mat[i][j];

 /*Recalibre la matrice*/
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)
   if (max) mat[i][j]*=(GREY_LEVEL/max);      
}

/*----------------------------------------------------------*/
/* Zeros                                                    */
/*----------------------------------------------------------*/
void Zeros(float** mat,int lgth,int wdth)
{
 int i,j;

 //Fill Zeros
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  mat[i][j]=0.0;
}

//----------------------------------------------------------
// calcule le nombre de differences a chaque iteration
//----------------------------------------------------------
int DiffMat(float** pmatl1,float** pmatl2,int lgth,int wdth)
{
 int i,j;
 int chg=0;

    for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)
        { 
         if ((pmatl1[i][j])!=(pmatl2[i][j])) chg++; 
             pmatl2[i][j]=pmatl1[i][j];
        }
 return chg;
}

//----------------------------------------------------------
// AddTwoMatrix
//----------------------------------------------------------    
void AddTwoMatrix(float** out,float** mat1,float** mat2,int lgth,int wdth,float fact)
{
 int i,j;

 //Boucle
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)
   out[i][j]=(1-fact)*mat1[i][j]+fact*mat2[i][j];
}


//--------------//
// RANDOM ------//
//--------------//
//----------------------------------------------------------
// retourne un nombre aleatoire entre zero et un
//----------------------------------------------------------
float randomize(void)
{ return ((float)rand()/RAND_MAX); }

//----------------------------------------------------------
// retourne un nombre aleatoire entre 
//             0 et c (floatant)
//----------------------------------------------------------
float random_flo(float c)
{ 
  return (randomize()*c); 
}


//----------//
//-- FILE --//
//----------//
//----------------------------------------------------------
// Get Length and Width
//----------------------------------------------------------
void GetLengthWidth(char* path,int* length,int* width)
{
 unsigned char var;
 int   temp;
 char* tempc;
 char stringTmp1[100];
 char stringTmp2[100];
 int ta1,ta2;
 FILE *fic;

  //ouverture du fichier
  fic=fopen(path,"r");
  if (fic==NULL)
    { printf("\n- Grave erreur a l'ouverture de %s -\n",path);
      exit(-1); }

  //recuperation de l'entete
  tempc=fgets(stringTmp1,100,fic);
  for(;;) { temp=fread(&var,1,1,fic); if (var==35) tempc=fgets(stringTmp2,100,fic);
            else break; }
  fseek(fic,-1,SEEK_CUR);
  temp=fscanf(fic,"%d %d",&ta1,&ta2);
   
  //enregistrement
  (*length)=ta2; 
  (*width)=ta1;

  //fermeture du fichier
  fclose(fic);
}

//----------------------------------------------------------
// Chargement de l'image de nom <name> (en ppm)             
//----------------------------------------------------------
void load_image_ppm(char* path,int*** img,int lgth,int wdth)
 {
  int i,j;
  int temp;
  char* tempc;
  unsigned char varb,varv,varr;

  char stringTmp1[200],stringTmp2[200];
  int ta1,ta2,ta3;
  FILE *fic;

  //enregistrement du chemin;
   if (strstr(path,".ppm")==NULL) strcat(path,".ppm");

   //ouverture du fichier
  fic=fopen(path,"r");
  if (fic==NULL) 
   { printf("\n ->> Grave erreur a l'ouverture de %s -\n",path);
     exit(-1); }

  //recuperation de l'entete
  tempc=fgets(stringTmp1,100,fic); //P6
  tempc=fgets(stringTmp2,100,fic); //Comment

  for(;;)
    { if (((int)stringTmp2[0]==10)||((int)stringTmp2[0]==35))
      tempc=fgets(stringTmp2,100,fic); 

      else break; }

  temp=sscanf(tempc," %d %d",&ta1,&ta2);
  temp=fscanf(fic," %d",&ta3);

  //affichage de l'entete
  printf("\n\n -- Entete --");
  printf("\n ------------");
  //printf("\n %s %s %d %d \n %d",stringTmp1,stringTmp2,ta1,ta2,ta3);
  printf("\n %s  %d %d \n %d",stringTmp1,ta1,ta2,ta3);
  printf("\n ----------------------------------------------------------\n\n");
   
  //chargement dans la matrice
  for(i=0;i<lgth;i++) for(j=0;j<(wdth*3);j+=3)  
    { temp=fread(&varr,1,1,fic);
      temp=fread(&varv,1,1,fic);
      temp=fread(&varb,1,1,fic);
     
      img[2][i][j/3]=varr;        
      img[0][i][j/3]=varv;        
      img[1][i][j/3]=varb; }      

  //fermeture du fichier
  fclose(fic);
 }

//----------------------------------------------------------
// Sauvegarde de l'image de nom <name> au format pgm                        
//----------------------------------------------------------                
void SaveImagePgm(char* bruit,char* name,float** mat,int lgth,int wdth)
{
 int i,j;
 char buff[300];
 FILE* fic;

  //--extension--
  strcpy(buff,bruit);
  strcat(buff,name);
  strcat(buff,".pgm");

  //--ouverture fichier--
  fic=fopen(buff,"wb");
    if (fic==NULL) 
        { printf("Probleme dans la sauvegarde de %s",buff); 
          exit(-1); }
  printf("\n Sauvegarde de %s au format pgm\n",buff);

  //--sauvegarde de l'entete--
  fprintf(fic,"P5");
  fprintf(fic,"\n# IMG Module");
  fprintf(fic,"\n%d %d",wdth,lgth);
  fprintf(fic,"\n255\n");

  //--enregistrement--
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) 
	fprintf(fic,"%c",(char)mat[i][j]);
   
  //--fermeture fichier--
   fclose(fic); 
}

//----------------------------------------------------------
// Sauvegarde de l'image de nom <name> au format ppm        
//----------------------------------------------------------
void SaveImagePpm(char* Path_out,char* Name,int*** matrvb,int wdth,int lgth)
 {
  int i,j;
  char buff[200];
  FILE* fuser;

  //extension
  strcpy(buff,Path_out);
  strcat(buff,Name);
  strcat(buff,".ppm");

  //ouverture fichier
  fuser=fopen(buff,"w");
    if (fuser==NULL) 
        { printf(" probleme dans la sauvegarde de %s",buff); 
          exit(-1); }

  //affichage
  printf("\n sauvegarde dans -> %s au format %s [%d][%d]",buff,".ppm",wdth,lgth);
  fflush(stdout);
  //exit(-1);

  //sauvegarde de l'entete
  fprintf(fuser,"P6");
  fprintf(fuser,"\n# IMG Module");
  fprintf(fuser,"\n%d %d",lgth,wdth);
  fprintf(fuser,"\n255\n");

  //enregistrement
  for(i=0;i<wdth;i++) for(j=0;j<lgth;j++) 
    {
     fprintf(fuser,"%c",(char)matrvb[0][i][j]);
     fprintf(fuser,"%c",(char)matrvb[1][i][j]);
     fprintf(fuser,"%c",(char)matrvb[2][i][j]);
    }
       
  //fermeture fichier
   fclose(fuser); 
 }

//--------------------
//-- SPACE COLOR -----
//--------------------
//----------------------------------------------------------*/
// RGBToHSV 
// --------
// H=[0::360]   S=[0::1]  V=[0::255]
//                        
//----------------------------------------------------------*/
void RGBToHSV(float r,float g,float b,float* h,float* s,float* v)
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
void RGBToYIQ(float r,float g,float b,float* y,float* i,float* q)
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
void RGBToXYZ(float r,float g,float b,float* x,float* y,float* z)
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
//  XYZtoRGB 
//
//   [ R ]   [  3.240479 -1.537150 -0.498535 ] [ X ]
//   [ G ] = [ -0.969256  1.875992  0.041556 ] [ Y ]
//   [ B ]   [  0.055648 -0.204043  1.057311 ] [ Z ].
//
// [R,G,B] = [0::255]
//                   
//----------------------------------------------------------*/
void XYZToRGB(float x,float y,float z,float* r,float* g,float* b)
{
  //XYZ en [0-1]
  x/=255.0;
  y/=255.0;
  z/=255.0;

  //Conversion
 (*r)=(3.240479*x)+(-1.537150*y)+(-0.498535*z);
 (*g)=(-0.969256*x)+(1.875992*y)+(0.041556*z);
 (*b)=(0.055648*x)+(-0.204043*y)+(1.057311*z);

 //RGB en [0-255]
 (*r)*=255.0;
 (*g)*=255.0;
 (*b)*=255.0;

 //Tests
 if ((*r)>255)  (*r)=255;
 if ((*r)<0)    (*r)=0;

 if ((*g)>255)  (*g)=255;
 if ((*g)<0)    (*g)=0;

 if ((*b)>255)  (*b)=255;
 if ((*b)<0)    (*b)=0;
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
void RGBToLAB(float r,float g,float bl,float* l,float* a,float* b)
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
void RGBToLUV(float r,float g,float bl,float* l,float* u,float* v)
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
// LUVToRGB  
//
//  u' = u / ( 13L*) + un
//  v' = v / ( 13L* ) + vn
//  Y = (( L* + 16 ) / 116 )3
//  X = - 9Yu' / (( u' - 4 ) v' - u'v' )
//  Z = ( 9Y - 15v'Y - v'X ) / 3v' 
//
//      un' = 0.2009 
//      vn' = 0.4610 
//                   
//----------------------------------------------------------*/
void LUVToRGB(float l,float u,float v,float* r,float* g,float* bl)
{
 float x,y,z;
 float uprime,vprime;

 //Conversion LUV->XYZ
 uprime=u/(13*l)+0.2009;
 vprime=v/(13*l)+0.4610;
 
 y=pow(((l+16)/116),3.0);
 x=-9*y*uprime/(((uprime-4)*vprime)-uprime*vprime);
 z=((9*y-15*vprime*y-vprime*x)/(3*vprime));

 //Conversion XYZ->RGB
 XYZToRGB(x,y,z,r,g,bl);

 //RGB en [0-255]
 (*r)*=255.0;
 (*g)*=255.0;
 (*bl)*=255.0;

 //Tests
 if ((*r)>255)  (*r)=255;
 if ((*r)<0)    (*r)=0;

 if ((*g)>255)  (*g)=255;
 if ((*g)<0)    (*g)=0;

 if ((*bl)>255)  (*bl)=255;
 if ((*bl)<0)    (*bl)=0;
}

//----------------------------------------------------------*/
// RGBToI1I2I3  
//
// I1=(1/3) (R+G+B)
// I2=(R-B)
// I3=(2G-R-B)/2
//                   
//----------------------------------------------------------*/
void RGBToI1I2I3(float r,float g,float bl,float* i1,float* i2,float* i3)
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
void RGBToH1H2H3(float r,float g,float bl,float* h1,float* h2,float* h3)
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
void RGBToTSL(float r,float g,float bl,float* t,float* s,float* l)
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
void RGBToYCbCr(float r,float g,float bl,float* Y,float* Cb,float* Cr)
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
void RGBToHSL(float r,float g,float bl,float* h,float* s,float* l)
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
void RGBToP1P2(float r,float g,float bl,float* P1,float* P2,float* P3)
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
void ColorBase(int***  imgout,int*** imgin,int lgth,int wdth,int bascol)
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
        
	 imgout[0][i][j]=(int)col1;
         imgout[1][i][j]=(int)col2;
	 imgout[2][i][j]=(int)col3;  
    }
}

//---------------
//---K-MEAN------
//---------------
//----------------------------------------------------------
// Identical                      
//----------------------------------------------------------
int Identical(int* VctPts,int** MtxCtrs,int nbcl,int nbpar)
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
//  K-Mean  
//    Avec Entiers
//----------------------------------------------------------
void KmeanFast(int** MatrixPts,int NbClusters,int NbPts,int NbParam,int* VctLabel,int Seed,int FlgMan)
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
     while((i)&&(nb<100)&&(Identical(MatrixPts[(int)aleat],MatrixCtrs,NbClusters,NbParam)));

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

//----------------
//  FUSION -------
//----------------
//----------------------------------------------------------
// Paint        
//----------------------------------------------------------
void Paint(float** mat,float** mattmp,int row,int col,int lgth,int wdth,int lab)
{
  //if (((mattmp[row][col]==0)||(mattmp[row][col]==-2))&&(mat[row][col]==lab))

   if ((mattmp[row][col]==0)&&(mat[row][col]==lab))
    { 
     mattmp[row][col]=-1;
     
     if ((col+1)<=(wdth-1))   Paint(mat,mattmp,row,col+1,lgth,wdth,lab);
     if ((col-1)>=0)          Paint(mat,mattmp,row,col-1,lgth,wdth,lab);
     if ((row+1)<=(lgth-1))   Paint(mat,mattmp,row+1,col,lgth,wdth,lab);
     if ((row-1)>=0)          Paint(mat,mattmp,row-1,col,lgth,wdth,lab); 
     }

 else return;
}

//----------------------------------------------------------
// Paint        
//----------------------------------------------------------
void Paint2(float** mat,float** mattmp,int row,int col,int lgth,int wdth,int lab)
{
   if ((mattmp[row][col]==0)&&(mat[row][col]==lab))
    { 
     mattmp[row][col]=-1;
     
     if ((col+1)<=(wdth-1))   Paint(mat,mattmp,row,col+1,lgth,wdth,lab);
     if ((col-1)>=0)          Paint(mat,mattmp,row,col-1,lgth,wdth,lab);
     if ((row+1)<=(lgth-1))   Paint(mat,mattmp,row+1,col,lgth,wdth,lab);
     if ((row-1)>=0)          Paint(mat,mattmp,row-1,col,lgth,wdth,lab); 

     if (((row-1)>=0)&&((col+1)<=(wdth-1)))         Paint(mat,mattmp,row-1,col+1,lgth,wdth,lab); 
     if (((row+1)<=(lgth-1))&&((col+1)<=(wdth-1)))  Paint(mat,mattmp,row+1,col+1,lgth,wdth,lab); 
     if (((row+1)<=(lgth-1))&&((col-1)>=0))         Paint(mat,mattmp,row+1,col-1,lgth,wdth,lab);
     if (((row-1)>=0)&&((col-1)>=0))                Paint(mat,mattmp,row-1,col-1,lgth,wdth,lab);
     }

 else return;
}

//----------------------------------------------------------
// Paint        
//----------------------------------------------------------
void Paint(float** mat,int** mattmp,int row,int col,int lgth,int wdth,int lab)
{
 if ((mattmp[row][col]==0)&&(mat[row][col]==lab))
    { 
     mattmp[row][col]=-1;

     if ((col+1)<=(wdth-1))  Paint(mat,mattmp,row,col+1,lgth,wdth,lab);
     if ((col-1)>=0)         Paint(mat,mattmp,row,col-1,lgth,wdth,lab);
     if ((row+1)<=(lgth-1))  Paint(mat,mattmp,row+1,col,lgth,wdth,lab);
     if ((row-1)>=0)         Paint(mat,mattmp,row-1,col,lgth,wdth,lab); 
     }

 else return;
}

//----------------------------------------------------------
// Convert <ClassRegion>              
//----------------------------------------------------------
void ConvertClassRegion(float** mat,int lgth,int wdth)
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
         //printf("[^^^^%d-%d(%d)^^^^]",i,j,(int)label);

         Paint(mat,mattmp,i,j,lgth,wdth,label);

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
// Convert <ClassRegion>              
//----------------------------------------------------------
void ConvertClassRegionInf(float** mat,int lgth,int wdth)
{
 int i,j,k,l;
 int label;
 int cpt;
 
  //Allocation 
  int** mattmp=imatrix_allocate_2d(lgth,wdth);
  
  //Init 
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) mattmp[i][j]=0;
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) mat[i][j]=(int)mat[i][j];

  //Boucle
  //-------
  cpt=0;
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)
     if (mattmp[i][j]==0)
        {
	 label=(int)mat[i][j];
         Paint(mat,mattmp,i,j,lgth,wdth,label);

         ++cpt;      

         for(k=0;k<lgth;k++) for(l=0;l<wdth;l++)
	   if (mattmp[k][l]==-1) mat[k][l]=cpt;

         for(k=0;k<lgth;k++) for(l=0;l<wdth;l++)
	   if (mattmp[k][l]==-1) mattmp[k][l]=-2;
        }
 
   //Compte Regions
   printf("\n >> NbRegions [%d]",cpt);
   fflush(stdout);

 //Liberation 
 free_imatrix_2d(mattmp);  
}



//----------------------------------------------------------
// FuseRegions
//
// < Petite regions inferieur taille sz fusione avec
//    classe majoritaire l entourant >
//              
//----------------------------------------------------------
int FuseSmallRegions(float** mat,int lgth,int wdth,int* nbreg,int sz)
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
               Paint(mat,mattmp,i,j,lgth,wdth,(int)mat[i][j]);
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

               //SiCondition=>Fusion
               if (initsize<=sz)
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
         Paint(mat,mattmp,i,j,lgth,wdth,(int)mat[i][j]);
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

//---------------------
//---VISU CONTOUR------
//---------------------
//----------------------------------------------------------*/
// AddContour                               
//----------------------------------------------------------*/
void AddContourImg(float** seg,int*** img,int lgth,int wdth,int gross) 
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

//----------------------------------------------------------
//  ---------------------------------   
//  ContourSeg. and SoftContour
//  ---------------------------------                   
//----------------------------------------------------------
//----------------------------------------------------------
// ComputeContour                               
//----------------------------------------------------------
void ComputeContour(float** cont,float** seg,int lgth,int wdth) 
{
 int i,j;
 float tmp;

 //Init
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) cont[i][j]=0.0;

 //Gradient
 for(i=1;i<(lgth-1);i++) for(j=1;j<(wdth-1);j++)
   { tmp=sqrt(CARRE(seg[i+1][j]-seg[i][j])+CARRE(seg[i][j+1]-seg[i][j]));
     if (tmp>0) cont[i][j]=255.0; 
     else       cont[i][j]=0.0;  }    
}

//----------------------------------------------------------
// GradFoncText
//----------------------------------------------------------     
void GradFoncText(int*** img,float** matgrd,float** seg,int lgth,int wdth,int dist)
 { 
  int i,j,k,l;
  int posr,posc;
  int rowU,colU,rowD,colD;
  int rowL,colL,rowR,colR;
  int NbParam;
  int SzW,div;
  float lab,grad,divis,max;

  //Presentation
  printf("\n\n Calcul PotentielTexture ...  ");
  fflush(stdout);
  
  //Constantes
  div=5; 
  SzW=7;

  //Initialization
  NbParam=div*div*div; 
  divis=(256.0/(float)div); 

  //Initialisatio memoire
  float*** SquWin=fmatrix_allocate_3d(3,SzW,SzW);
  float*** MatHis=fmatrix_allocate_3d(NbParam,lgth,wdth);

  //Initialization
  for(k=0;k<NbParam;k++) for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) MatHis[k][i][j]=0;
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) matgrd[i][j]=0;    

  //Recup Histo
  //=============
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
   
     for(k=0;k<SzW;k++) for(l=0;l<SzW;l++)  
	{ lab=((int)(SquWin[0][k][l]/divis))*(div*div);
          lab+=((int)(SquWin[1][k][l]/divis))*(div);  
          lab+=((int)(SquWin[2][k][l]/divis)); 
   
          MatHis[(int)(lab)][i][j]++; }

     for(k=0;k<NbParam;k++) MatHis[k][i][j]/=(SzW*SzW);
    }//================================
  
  //Compute GradText
  //================
   for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  
     if (seg[i][j])
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
     for(k=0;k<NbParam;k++) 
       { grad+=fabs(MatHis[k][rowU][colU]-MatHis[k][rowD][colD]);
         grad+=fabs(MatHis[k][rowR][colR]-MatHis[k][rowL][colL]); }

     matgrd[i][j]=grad;
    }//=================
 
  //Normalize
  //=========
  max=0.0;
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) if (seg[i][j]) { if (matgrd[i][j]>max) max=matgrd[i][j]; }
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) if (seg[i][j]) matgrd[i][j]=((matgrd[i][j]*255.0)/max);
  //for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) if (seg[i][j]) matgrd[i][j]=255-matgrd[i][j];
  //========

  //Liberation Memoire
  if (SquWin) free_fmatrix_3d(SquWin,3);
  if (MatHis) free_fmatrix_3d(MatHis,NbParam);

  //Debug
  printf(" ...  fini \n\n");
  fflush(stdout);
 }

//----------------------------------------------------------
// GradFoncText
//----------------------------------------------------------     
void GradFoncText(int*** img,float** matgrd,float** seg,int lgth,int wdth,int dist1,int dist2)
 { 
  int i,j,k,l;
  int posr,posc;
  int rowU,colU,rowD,colD;
  int rowL,colL,rowR,colR;
  int NbParam;
  int SzW,div;
  float lab,grad,divis,max;

  //Presentation
  printf("\n\n Calcul PotentielTexture ...  ");
  fflush(stdout);
  
  //Constantes
  div=5; 
  SzW=7;

  //Initialization
  NbParam=div*div*div; 
  divis=(256.0/(float)div); 

  //Initialisatio memoire
  float*** SquWin=fmatrix_allocate_3d(3,SzW,SzW);
  float*** MatHis=fmatrix_allocate_3d(NbParam,lgth,wdth);

  //Initialization
  for(k=0;k<NbParam;k++) for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) MatHis[k][i][j]=0;
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) matgrd[i][j]=0;    

  //Recup Histo
  //=============
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
   
     for(k=0;k<SzW;k++) for(l=0;l<SzW;l++)  
	{ lab=((int)(SquWin[0][k][l]/divis))*(div*div);
          lab+=((int)(SquWin[1][k][l]/divis))*(div);  
          lab+=((int)(SquWin[2][k][l]/divis)); 
   
          MatHis[(int)(lab)][i][j]++; }

     for(k=0;k<NbParam;k++) MatHis[k][i][j]/=(SzW*SzW);
    }//================================
  
  //Compute GradText
  //================
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  matgrd[i][j]=0.0;

   for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  
   if (seg[i][j]) for(l=dist1;l<=dist2;l++)  
    {
     rowU=(i-l); colU=j;
     rowD=(i+l); colD=j;

     if (rowU<0)        rowU+=lgth;
     if (rowD>(lgth-1)) rowD-=lgth;

     rowL=i; colL=(j-l);
     rowR=i; colR=(j+l);

     if (colL<0)        colL+=wdth;
     if (colR>(wdth-1)) colR-=wdth;

     grad=0;
     for(k=0;k<NbParam;k++) 
       { grad+=fabs(MatHis[k][rowU][colU]-MatHis[k][rowD][colD]);
         grad+=fabs(MatHis[k][rowR][colR]-MatHis[k][rowL][colL]); }

     matgrd[i][j]+=grad;
    }//=================
 
  //Normalize
  //=========
  max=0.0;
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) if (seg[i][j]) { if (matgrd[i][j]>max) max=matgrd[i][j]; }
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) if (seg[i][j]) matgrd[i][j]=((matgrd[i][j]*255.0)/max);
  //for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) if (seg[i][j]) matgrd[i][j]=255- matgrd[i][j];
  //========

  //Liberation Memoire
  if (SquWin) free_fmatrix_3d(SquWin,3);
  if (MatHis) free_fmatrix_3d(MatHis,NbParam);

  //Debug
  printf(" ...  fini \n\n");
  fflush(stdout);
 }

//--------------------------
//-- CANY ------------------
//--------------------------
//----------------------------------------------------------
// CANY
// -----------------
//
//                 / 1     | 2        3
//                /        |          
//               []        []          []
// [][]--> 0    []         []           []
//----------------------------------------------------------    
void Cany(int*** img,float** segcany,int lgth,int wdth,float pH,float pB,int div,int bc,int ch)
{
 int i,j,c;
 float tau_L;
 float tau_H;
 float k;
 float sum,tmp,min,est;
 

 //Allocation memoire
 //------------------
 float** matgrd=fmatrix_allocate_2d(lgth,wdth);
 float** matorient=fmatrix_allocate_2d(lgth,wdth);
 float** matgrdsuppr=fmatrix_allocate_2d(lgth,wdth);
 float** mattmp=fmatrix_allocate_2d(lgth,wdth);
 float *hist=fmatrix_allocate_1d(256);

 //Gradient Magnitude
 //------------------
 if (ch==0) GradFoncTextCS(img,matgrd,matorient,lgth,wdth,bc,div);
 if (ch==2) Grad(img,matgrd,matorient,lgth,wdth,bc); 
 
 
 //Estimation Parametres
 //---------------------
 for(i=0;i<=GREY_LEVEL;i++) hist[i]=0.0;

 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) 
     if ((matgrd[i][j]>=0)&&(matgrd[i][j]<=GREY_LEVEL)) 
        hist[(int)(matgrd[i][j])]++;

 for(i=0;i<=GREY_LEVEL;i++)  hist[i]/=(wdth*lgth);

 for (sum=0.0,i=0;i<256 && sum<pH;i++) sum+=hist[i];

 tau_H=(float)(i+1);
 tau_L=pB*tau_H;

 //Approximation Orientation
 //-------------------------
 //>A pi pres
 for(i=0;i<lgth-1;i++) for(j=0;j<wdth-1;j++) if (matorient[i][j]<0) matorient[i][j]+=PI;
  
 //>Direction 45 deg pres
   for(i=0;i<lgth-1;i++) for(j=0;j<wdth-1;j++)
     { min=10000000; est=0;
       for(k=0.0,c=0;k<=PI;k=k+(PI/4),c++)
         { tmp=fabs(matorient[i][j]-k);
           if (tmp<min) { min=tmp; est=c;} }

       if(est==4) est=0;
       matorient[i][j]=est;  }

 //Record
 if (0)
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) segcany[i][j]=matgrd[i][j];

 //Suppression NonMaximumLocaux
 //----------------------------
  if (1)
  for(i=1;i<lgth-1;i++) for(j=1;j<wdth-1;j++)
    { matgrdsuppr[i][j]=matgrd[i][j];
      
     if (matorient[i][j]==0.0)
        { if ((matgrd[i][j]<matgrd[i][j+1])||(matgrd[i][j]<matgrd[i][j-1]))  
	  matgrdsuppr[i][j]=0.0;  } 

     if (matorient[i][j]==1.0)
        { if ((matgrd[i][j]<matgrd[i+1][j-1])||(matgrd[i][j]<matgrd[i-1][j+1]))  
	  matgrdsuppr[i][j]=0.0;  }  
    
     if (matorient[i][j]==2.0)
        { if ((matgrd[i][j]<matgrd[i+1][j])||(matgrd[i][j]<matgrd[i-1][j])) 
	  matgrdsuppr[i][j]=0.0;  } 

     if (matorient[i][j]==3.0)
        { if ((matgrd[i][j]<matgrd[i+1][j+1])||(matgrd[i][j]<matgrd[i-1][j-1])) 
	  matgrdsuppr[i][j]=0.0;  }  }

 //Hysteresis
 //----------
  if (1)
  for(i=1;i<lgth-1;i++) for(j=1;j<wdth-1;j++)
   { if (matgrd[i][j]>tau_H) 
	 follow(matgrdsuppr,i,j,matorient,segcany,lgth,wdth,tau_L);  }

 //Recalage Final
 Recal(segcany,lgth,wdth); 

 //Liberation memoire 
 free_fmatrix_2d(matgrd);
 free_fmatrix_2d(matorient);
 free_fmatrix_2d(matgrdsuppr);
 free_fmatrix_2d(mattmp);
 free_fmatrix_1d(hist);
}


//----------------------------------------------------------
// follow
//----------------------------------------------------------  
void follow(float** mat_s,int i,int j,float** mat_o,float** mat,int lgth,int wdth,float tau_L)
 {
   if ((i<0)||(i>=lgth))  return;
   if ((j<0)||(j>=wdth))  return;

   if (mat_s[i][j]>tau_L && mat[i][j]==0.0)
      {
       mat[i][j]=255.0;

       if ((mat_o[i][j]==0.0)&&((i-1)>1)) 
	 {                   
	   follow(mat_s,i-1,j,mat_o,mat,lgth,wdth,tau_L);
	   follow(mat_s,i+1,j,mat_o,mat,lgth,wdth,tau_L);
	 }
       
       if ((mat_o[i][j]==1.0)&&((i-1)>1)&&((j-1)>1))         
	 {
	   follow(mat_s,i-1,j-1,mat_o,mat,lgth,wdth,tau_L);  
	   follow(mat_s,i+1,j+1,mat_o,mat,lgth,wdth,tau_L);
	 }
       
       if ((mat_o[i][j]==2.0)&&((j-1)>1))                    
	 {
	   follow(mat_s,i,j-1,mat_o,mat,lgth,wdth,tau_L);  
	   follow(mat_s,i,j+1,mat_o,mat,lgth,wdth,tau_L);
	 }
       
       if ((mat_o[i][j]==3.0)&&((i-1)>1)&&((j+1)<(wdth-1)))  
	 {
	   follow(mat_s,i-1,j+1,mat_o,mat,lgth,wdth,tau_L); 
	   follow(mat_s,i+1,j-1,mat_o,mat,lgth,wdth,tau_L);
	 }       
      }
 }

//----------------------------------------------------------
// GradFoncText <<ColorSpace>>
//----------------------------------------------------------     
void GradFoncTextCS(int*** img,float** matgrd,float** matorient,int lgth,int wdth,int bc,int div)
 { 
  int i,j,k,l;
  int posr,posc;
  int rowU,colU,rowD,colD;
  int rowL,colL,rowR,colR;
  int NbParam;
  int SzW;
  float lab,grad,divis,max;
  //float gradh,gradl;
  int dist;

  //Presentation
  printf("\n Calcul PotentielTexture ...");
  fflush(stdout);

  //Allocation
  int*** imgt=imatrix_allocate_3d(3,lgth,wdth);

  //Conversion Couleurs
  printf(" > Conversion Couleurs[%d]",bc);
  ColorBase(imgt,img,lgth,wdth,bc);
  
  //Constantes
  SzW=7;
  dist=(int)((SzW/2)+0.5);

  //Initialization
  NbParam=div*div*div; 
  divis=(256.0/(float)div); 

  //Initialisatio memoire
  float*** SquWin=fmatrix_allocate_3d(3,SzW,SzW);
  float*** MatHis=fmatrix_allocate_3d(NbParam,lgth,wdth);

  //Initialization
  for(k=0;k<NbParam;k++) for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) MatHis[k][i][j]=0;
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) matgrd[i][j]=0;  

  //Recup Histo
  //==============================================
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  
    {
     for(k=0;k<SzW;k++) for(l=0;l<SzW;l++) 
        { posr=i-(SzW/2)+k;
          posc=j-(SzW/2)+l;

          if (posr<0)           posr+=lgth;
          if (posr>(lgth-1))    posr-=lgth;  
          if (posc<0)           posc+=wdth;
          if (posc>(wdth-1))    posc-=wdth;

          SquWin[0][k][l]=imgt[0][posr][posc];
          SquWin[1][k][l]=imgt[1][posr][posc];
          SquWin[2][k][l]=imgt[2][posr][posc]; }
   
     for(k=0;k<SzW;k++) for(l=0;l<SzW;l++)  
	{ lab=((int)(SquWin[0][k][l]/divis))*(div*div);
          lab+=((int)(SquWin[1][k][l]/divis))*(div);  
          lab+=((int)(SquWin[2][k][l]/divis)); 
   
          MatHis[(int)(lab)][i][j]++; }

     for(k=0;k<NbParam;k++) MatHis[k][i][j]/=(SzW*SzW);
    }//================================================
  
  //Compute Module Gradient Texture <GradText>
  //===========================================
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

     //Distance Manhattan
     grad=0.0;
     //gradl=0.0;
     for(k=0;k<NbParam;k++) 
       { grad+=fabs(MatHis[k][rowU][colU]-MatHis[k][rowD][colD]);
         grad+=fabs(MatHis[k][rowR][colR]-MatHis[k][rowL][colL]); }

     matgrd[i][j]=grad;
    }//========================================

  //Orientation du gradient Texture <GradText>
  //==========================================
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) matorient[i][j]=0.0;

  for(i=0;i<lgth-1;i++) for(j=0;j<wdth-1;j++)
  matorient[i][j]=atan((matgrd[i+1][j]-matgrd[i][j])/(matgrd[i][j]-matgrd[i][j+1]));
  //=========================================

  //Normalize
  max=0.0;
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) { if (matgrd[i][j]>max) max=matgrd[i][j]; }
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) matgrd[i][j]=((matgrd[i][j]*255.0)/max);

  //Liberation Memoire
  if (SquWin) free_fmatrix_3d(SquWin,3);
  if (MatHis) free_fmatrix_3d(MatHis,NbParam);
  if (imgt)   free_imatrix_3d(imgt,3);

  //Debug
  printf(" ...fini");
  fflush(stdout);
 }

//----------------------------------------------------------
// GradFoncText <<ColorSpace>>
//----------------------------------------------------------     
void GradFoncTextCS(int*** img,float** matgrd,float** matorient,int lgth,int wdth,int bc,int div,int sz)
 { 
  int i,j,k,l;
  int posr,posc;
  int rowU,colU,rowD,colD;
  int rowL,colL,rowR,colR;
  int NbParam;
  int SzW;
  float lab,grad,divis,max;
  //float gradh,gradl;
  int dist;

  //Presentation
  printf("\n Calcul PotentielTexture ...");
  fflush(stdout);

  //Allocation
  int*** imgt=imatrix_allocate_3d(3,lgth,wdth);

  //Conversion Couleurs
  printf(" > Conversion Couleurs[%d]",bc);
  ColorBase(imgt,img,lgth,wdth,bc);
  
  //Constantes
  SzW=sz;
  dist=(int)((SzW/2)+0.5);

  //Initialization
  NbParam=div*div*div; 
  divis=(256.0/(float)div); 

  //Initialisatio memoire
  float*** SquWin=fmatrix_allocate_3d(3,SzW,SzW);
  float*** MatHis=fmatrix_allocate_3d(NbParam,lgth,wdth);

  //Initialization
  for(k=0;k<NbParam;k++) for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) MatHis[k][i][j]=0;
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) matgrd[i][j]=0;  

  //Recup Histo
  //==============================================
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  
    {
     for(k=0;k<SzW;k++) for(l=0;l<SzW;l++) 
        { posr=i-(SzW/2)+k;
          posc=j-(SzW/2)+l;

          if (posr<0)           posr+=lgth;
          if (posr>(lgth-1))    posr-=lgth;  
          if (posc<0)           posc+=wdth;
          if (posc>(wdth-1))    posc-=wdth;

          SquWin[0][k][l]=imgt[0][posr][posc];
          SquWin[1][k][l]=imgt[1][posr][posc];
          SquWin[2][k][l]=imgt[2][posr][posc]; }
   
     for(k=0;k<SzW;k++) for(l=0;l<SzW;l++)  
	{ lab=((int)(SquWin[0][k][l]/divis))*(div*div);
          lab+=((int)(SquWin[1][k][l]/divis))*(div);  
          lab+=((int)(SquWin[2][k][l]/divis)); 
   
          MatHis[(int)(lab)][i][j]++; }

     for(k=0;k<NbParam;k++) MatHis[k][i][j]/=(SzW*SzW);
    }//================================================
  
  //Compute Module Gradient Texture <GradText>
  //===========================================
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

     //Distance Manhattan
     grad=0.0;
     //gradl=0.0;
     for(k=0;k<NbParam;k++) 
       { grad+=fabs(MatHis[k][rowU][colU]-MatHis[k][rowD][colD]);
         grad+=fabs(MatHis[k][rowR][colR]-MatHis[k][rowL][colL]); }

     matgrd[i][j]=grad;
    }//========================================

  //Orientation du gradient Texture <GradText>
  //==========================================
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) matorient[i][j]=0.0;

  for(i=0;i<lgth-1;i++) for(j=0;j<wdth-1;j++)
  matorient[i][j]=atan((matgrd[i+1][j]-matgrd[i][j])/(matgrd[i][j]-matgrd[i][j+1]));
  //=========================================

  //Normalize
  max=0.0;
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) { if (matgrd[i][j]>max) max=matgrd[i][j]; }
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) matgrd[i][j]=((matgrd[i][j]*255.0)/max);

  //Liberation Memoire
  if (SquWin) free_fmatrix_3d(SquWin,3);
  if (MatHis) free_fmatrix_3d(MatHis,NbParam);
  if (imgt)   free_imatrix_3d(imgt,3);

  //Debug
  printf(" ...fini");
  fflush(stdout);
 }

//----------------------------------------------------------
// GradCont
//----------------------------------------------------------     
void Grad(int*** img,float** matgrd,float** matorient,int lgth,int wdth,int bc)
 { 
  int i,j;
  float tmp;
  float max;

  //Presentation
  printf("\n Calcul Gradient Contour ...");
  fflush(stdout);

  //Allocation
  int*** imgt=imatrix_allocate_3d(3,lgth,wdth);

  //Conversion Couleurs
  printf(" > Conversion Couleurs[%d]",bc);
  ColorBase(imgt,img,lgth,wdth,bc);

  //Init
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) matgrd[i][j]=0.0;

  //Gradient
  for(i=1;i<(lgth-1);i++) for(j=1;j<(wdth-1);j++)
     { tmp=0.0;
       tmp+=CARRE(imgt[0][i+1][j]-imgt[0][i-1][j])+CARRE(imgt[0][i][j+1]-imgt[0][i][j-1]);
       tmp+=CARRE(imgt[1][i+1][j]-imgt[1][i-1][j])+CARRE(imgt[1][i][j+1]-imgt[1][i][j-1]);
       tmp+=CARRE(imgt[2][i+1][j]-imgt[2][i-1][j])+CARRE(imgt[2][i][j+1]-imgt[2][i][j-1]); 
       tmp=sqrt(tmp); 
       matgrd[i][j]=tmp; }

  //Orientation du gradient
  for(i=0;i<lgth-1;i++) for(j=0;j<wdth-1;j++)
     matorient[i][j]=atan((matgrd[i+1][j]-matgrd[i][j])/(matgrd[i][j]-matgrd[i][j+1])); 
     
  //Normalize
  max=0.0;
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) { if (matgrd[i][j]>max) max=matgrd[i][j]; }
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) matgrd[i][j]=((matgrd[i][j]*255.0)/max);

  //Liberation Memoire
  if (imgt)   free_imatrix_3d(imgt,3);

  //Debug
  printf(" ... fini");
  fflush(stdout);
 }

//----------------------------------------------------------*/
// AddContour                               
//----------------------------------------------------------*/
void AddContour(float** seg,float** img,int lgth,int wdth,int gross) 
{
 int i,j,k,l;
 float dist;
 float Gauss;

 //reservation memoire
 float** mat_tmp=fmatrix_allocate_2d(lgth,wdth);
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) mat_tmp[i][j]=0.0;

 //Epaisseur 0
 if (gross==0)
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) mat_tmp[i][j]=seg[i][j];

 //Boucle
 if (gross)
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) if (seg[i][j])
      { 
       for(k=-3*gross;k<=gross;k++) for(l=-3*gross;l<=gross;l++)
	  { 
	   dist=CARRE(k)+CARRE(l);
           Gauss=exp(-dist/(2.0*gross));
           mat_tmp[((i+k+lgth)%lgth)][((j+l+wdth)%wdth)]+=255*Gauss; 
          }
      }  

 //Addition
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) img[i][j]+=mat_tmp[i][j];

 //liberation memoire
 free_fmatrix_2d(mat_tmp); 
}

//----------------------------------------------------------
// Thickenning
//----------------------------------------------------------    
void Thicken(int*** img,float** matgrd,float** matout,int lgth,int wdth)
{
 int i,j,c;
 float k;
 float tmp;
 float min,est;
 
 //Allocation memoire
 //------------------
 float** matgrdsuppr=fmatrix_allocate_2d(lgth,wdth);
 float** matorient=fmatrix_allocate_2d(lgth,wdth);

 //Recalage 
 //--------
 Recal(matgrd,lgth,wdth);  
 
 //Orient
 //------
 Grad(img,matgrdsuppr,matorient,lgth,wdth,0); 

 //Approximation Orientation
 //-------------------------
 //>A pi pres
 for(i=0;i<lgth-1;i++) for(j=0;j<wdth-1;j++) if (matorient[i][j]<0) matorient[i][j]+=PI;
  
 //>Direction 45 deg pres
   for(i=0;i<lgth-1;i++) for(j=0;j<wdth-1;j++)
     { min=10000000; est=0;
       for(k=0.0,c=0;k<=PI;k=k+(PI/4),c++)
         { tmp=fabs(matorient[i][j]-k);
           if (tmp<min) { min=tmp; est=c;} }

       if(est==4) est=0;
       matorient[i][j]=est;  }

 //Suppression NonMaximumLocaux
 //----------------------------
  for(i=1;i<lgth-1;i++) for(j=1;j<wdth-1;j++)
    { matgrdsuppr[i][j]=matgrd[i][j];
      
     if (matorient[i][j]==0.0)
        { if ((matgrd[i][j]<matgrd[i][j+1])||(matgrd[i][j]<matgrd[i][j-1]))  
	  matgrdsuppr[i][j]=0.0;  } 

     if (matorient[i][j]==1.0)
        { if ((matgrd[i][j]<matgrd[i+1][j-1])||(matgrd[i][j]<matgrd[i-1][j+1]))  
	  matgrdsuppr[i][j]=0.0;  }  
    
     if (matorient[i][j]==2.0)
        { if ((matgrd[i][j]<matgrd[i+1][j])||(matgrd[i][j]<matgrd[i-1][j])) 
	  matgrdsuppr[i][j]=0.0;  } 

     if (matorient[i][j]==3.0)
        { if ((matgrd[i][j]<matgrd[i+1][j+1])||(matgrd[i][j]<matgrd[i-1][j-1])) 
	  matgrdsuppr[i][j]=0.0;  }  }

 //Transfert
 //---------
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) matout[i][j]=matgrdsuppr[i][j];

 //Recalage Final
 Recal(matout,lgth,wdth); 

 //Liberation memoire 
 free_fmatrix_2d(matgrdsuppr);
 free_fmatrix_2d(matorient);
}

//----------------------------------------------------------
//  ---------------------------------   
//  SFSBM
//  ---------------------------------                   
//----------------------------------------------------------
//---------------------------------------------------------- 
//----------------------------------------------------------
// EstimGrad                           
//----------------------------------------------------------
float EstimGrad(int rowbeg,int colbeg,int rowend,int colend,float** grad)
{
 int k;
 float cord;
 float diffrow,diffcol;
 float incrow,inccol;
 float pente;
 float max;

 //Init
 max=-100000.0;

 //Diff
 diffrow=(float)(rowend-rowbeg);
 diffcol=(float)(colend-colbeg);

 //Boucles
 if (fabs(diffrow)>=fabs(diffcol))
    { pente=(float)(diffcol)/(float)(diffrow);
      incrow=diffrow/fabs(diffrow); 
      cord=colbeg;

     if (rowend>=rowbeg)
     for(k=rowbeg;k<=rowend;k++)
        { if (grad[k][(int)cord]>max) max=grad[k][(int)cord];
          //printf("[1>%d::%d]",k,(int)cord); 
          cord+=pente; }
     
     if (rowbeg>rowend)
     for(k=rowbeg;k>=rowend;k--)
        { if (grad[k][(int)cord]>max) max=grad[k][(int)cord];
          //printf("[2>%d::%d]",k,(int)cord); 
          cord-=pente; }

     cord=colend;
     k=rowend;
     if (grad[k][(int)cord]>max) max=grad[k][(int)cord];
     //printf("[%d::%d]",k,(int)cord);      
    }

 if (fabs(diffcol)>fabs(diffrow))
    { pente=(float)(diffrow)/(float)(diffcol);
      inccol=diffcol/fabs(diffcol);    
      cord=rowbeg;

      if (colend>=colbeg)
          for(k=colbeg;k<=colend;k++)
             { if (grad[(int)cord][k]>max) max=grad[(int)cord][k];
               //printf("[3>%d::%d]",(int)cord,k); 
               cord+=pente;}

      if (colbeg>colend)
          for(k=colbeg;k>=colend;k--)
             { if (grad[(int)cord][k]>max) max=grad[(int)cord][k];
               //printf("[4>%d::%d]",(int)cord,k); 
               cord-=pente; } 

          cord=rowend;
          k=colend;
          if (grad[(int)cord][k]>max) max=grad[(int)cord][k];
          //printf("[%d::%d]",(int)cord,k);        
    }

 //Retour
 return max;
}

//----------------------------------------------------------
// Multiresolution
//----------------------------------------------------------
void multiresMax(float** mat,int szbl,int lgth,int wdth,float** sol)
{
 int i,j,k,l;
 int nb;
 float sum_pix;
 float max;

 //Constante
 printf("\n [SizeBlock:%d]",szbl);
 fflush(stdout);

 //Boucle
 for(i=0;i<(int)(lgth/szbl);i++) for(j=0;j<(int)(wdth/szbl);j++)
    {
     sum_pix=0.0;
     nb=0;
     max=0.0;
     for(k=i*szbl;k<(i*szbl+szbl);k++) for(l=j*szbl;l<(j*szbl+szbl);l++)
	  { if ((k<lgth)&&(l<wdth))
	      { if (mat[k][l]>max) max=mat[k][l]; } }

             sol[i][j]=max;
    }
}

//----------------------------------------------------------
// Multiresolution
//----------------------------------------------------------
void multires(int*** mat,int szbl,int lgth,int wdth,int*** sol)
{
 int i,j,k,l,m;
 int nb;
 float sum_pix;

 //Constante
 printf("\n [SizeBlock:%d]",szbl);
 fflush(stdout);

 //Boucle
 for(m=0;m<3;m++)
 for(i=0;i<(int)(lgth/szbl);i++) for(j=0;j<(int)(wdth/szbl);j++)
    {
     sum_pix=0.0;
     nb=0;
     for(k=i*szbl;k<(i*szbl+szbl);k++) for(l=j*szbl;l<(j*szbl+szbl);l++)
	  { if ((k<lgth)&&(l<wdth))
	       { sum_pix+=mat[m][k][l];   nb++; } }

             sum_pix/=(CARRE((float)(szbl)));
             sol[m][i][j]=(int)sum_pix;
    }
}

//----------------------------------------------------------
// MultGrad2
//----------------------------------------------------------     
void MultGrad2(int*** img,float** matgrd,int lgth,int wdth,float fct)
 { 
  int i,j;
  float tmp,max;
 
  //Presentation
  printf("\n [Calcul Gradient Contour]");
  fflush(stdout);

 //AllocMemoire
 float**  mat=fmatrix_allocate_2d(lgth,wdth);

  //Gradient
  for(i=1;i<(lgth-1);i++) for(j=1;j<(wdth-1);j++)
     { tmp=0.0;
       tmp+=CARRE(img[0][i+1][j]-img[0][i-1][j])+CARRE(img[0][i][j+1]-img[0][i][j-1]);
       tmp+=CARRE(img[1][i+1][j]-img[1][i-1][j])+CARRE(img[1][i][j+1]-img[1][i][j-1]);
       tmp+=CARRE(img[2][i+1][j]-img[2][i-1][j])+CARRE(img[2][i][j+1]-img[2][i][j-1]); 
       tmp=sqrt(tmp); 
       mat[i][j]=tmp; }
     
  //Normalize
  max=0.0;
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) { if (mat[i][j]>max) max=mat[i][j]; }
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) mat[i][j]=((mat[i][j]*1.0)/max);

  //Mutiplie
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) matgrd[i][j]=(fct*matgrd[i][j]*mat[i][j])+(matgrd[i][j]);

  //Liberation Memoire
  if (mat) free_fmatrix_2d(mat);

  //Debug
  printf(" ... fini");
  fflush(stdout);
 }

//----------------------------------------------------------
// ConvertCertainClassRegionInf   
//    On ne VisitePasClass[0]           
//----------------------------------------------------------
void ConvertCertainClassRegionInf(float** mat,int lgth,int wdth)
{
 int i,j,k,l;
 int label;
 int cpt;
 
  //Allocation 
  int** mattmp=imatrix_allocate_2d(lgth,wdth);
  
  //Init 
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) mattmp[i][j]=0;
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) mat[i][j]=(int)mat[i][j];

 //Class[0]>Visite
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) if (mat[i][j]==0) mattmp[i][j]=-2;

  //Boucle
  //-------
  cpt=0;
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)
     if (mattmp[i][j]==0)
        {
	 label=(int)mat[i][j];
         Paint(mat,mattmp,i,j,lgth,wdth,label);

         ++cpt;      

         for(k=0;k<lgth;k++) for(l=0;l<wdth;l++)
	   if (mattmp[k][l]==-1) mat[k][l]=cpt;

         for(k=0;k<lgth;k++) for(l=0;l<wdth;l++)
	   if (mattmp[k][l]==-1) mattmp[k][l]=-2;
        }
 
   //Compte Regions
   printf("\n >> NbRegions [%d]",cpt);
   fflush(stdout);

 //Liberation 
 free_imatrix_2d(mattmp);  
}

//----------------------------------------------------------     
// ContrainWithSeg(float**,int,int,float**)
//
// Nota: float**overseg en NbRegions
//----------------------------------------------------------     
void ContrainWithSeg(float** overseg,int lgth,int wdth,float** seg)
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
//  Convert320x214                     
//----------------------------------------------------------                
void Convert320x214(float** matin,float** matout,int* lgth,int* wdth)
{
 int i,j;
 float row,col;
 int length,width;
 
 //Record
 length=(*lgth);
 width=(*wdth);

 //Convertit
 if (length>width)
   for(i=0;i<320;i++) for(j=0;j<214;j++)
   { row=(int)((((float)length/320.0)*(float)i)+0.5);
     col=(int)((((float)width/214.0)*(float)j)+0.5);
     if ((row<length)&&(col<width)) matout[i][j]=matin[(int)row][(int)col]; }

 if (width>length)
   for(i=0;i<214;i++) for(j=0;j<320;j++)
     { row=(int)((((float)length/214.0)*(float)i)+0.5);
       col=(int)((((float)width/320.0)*(float)j)+0.5);
       if ((row<length)&&(col<width)) matout[i][j]=matin[(int)row][(int)col]; } 

 //Nouveau Lgth/Wdth
 if (length>width) { (*lgth)=320; (*wdth)=214; }
 if (width>length) { (*wdth)=320; (*lgth)=214;  }
}

//----------------------------------------------------------
//  Convert481x321                    
//----------------------------------------------------------                
void Convert481x321(float** matin,float** matout,int* lgth,int* wdth)
{
 int i,j;
 float row,col;
 int length,width;
 
 //Record
 length=(*lgth);
 width=(*wdth);

 //Convertit
 if (length>width)
   for(i=0;i<481;i++) for(j=0;j<321;j++)
   { row=(int)((((float)length/481.0)*(float)i)+0.5);
     col=(int)((((float)width/321.0)*(float)j)+0.5);
     if ((row<length)&&(col<width)) matout[i][j]=matin[(int)row][(int)col]; }

 if (width>length)
   for(i=0;i<321;i++) for(j=0;j<481;j++)
     { row=(int)((((float)length/321.0)*(float)i)+0.5);
       col=(int)((((float)width/481.0)*(float)j)+0.5);
       if ((row<length)&&(col<width)) matout[i][j]=matin[(int)row][(int)col]; } 

 //Nouveau Lgth/Wdth
 if (length>width) { (*lgth)=481; (*wdth)=321; }
 if (width>length) { (*wdth)=481; (*lgth)=321;  }
}


//----------------------------------------------------------
//  ---------------------------------   
//  Complexite d'une Image
//  ---------------------------------                   
//----------------------------------------------------------
//----------------------------------------------------------
// Complexity
//----------------------------------------------------------   
float Complexity(int***  img,int lgth,int wdth,int div,int SzW)
{
 int i,j,k,l;
 int posr,posc;
 int NbParam;
 float divis;
 int lab;
 float nrj;

 //>Record
 NbParam=div*div*div; 
 divis=(256.0/(float)div);   
 
 //AllocMemoire
 float*** SquWin=fmatrix_allocate_3d(3,SzW,SzW);
 double*   MatHisMoy=dmatrix_allocate_1d(NbParam);
 float*** MatHis=fmatrix_allocate_3d(NbParam,lgth,wdth);

  //Initialization
  for(k=0;k<NbParam;k++) for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) MatHis[k][i][j]=0; 

  //-------------------------------------------------------------
  //RecupLocalHistoMoy  [div][SzW] 
  //-------------------------------------------------------------     
  printf("\n\n   [Recup Histos Moy...]");
  fflush(stdout);   
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
   
     for(k=0;k<SzW;k++) for(l=0;l<SzW;l++)  
	{ lab=((int)(SquWin[0][k][l]/divis))*(div*div);
          lab+=((int)(SquWin[1][k][l]/divis))*(div);  
          lab+=((int)(SquWin[2][k][l]/divis)); 
   
          MatHis[(int)(lab)][i][j]++; 
          MatHisMoy[(int)(lab)]++; }

     for(k=0;k<NbParam;k++) MatHis[k][i][j]/=(SzW*SzW);
    }
  for(k=0;k<NbParam;k++) MatHisMoy[k]/=(lgth*wdth*SzW*SzW);
 //-----------------------------------------------------------

 //---------------------------------------------------
 //Boucle---------------------------------------------
 //---------------------------------------------------
 printf("\n   ComplexityImage ");
 fflush(stdout);
 nrj=0.0;
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) 
   for(k=0;k<NbParam;k++) nrj+=fabs(MatHis[k][i][j]-MatHisMoy[k]);

 nrj/=(lgth*wdth*2);
 //-------------------------------------------------

  //Liberation Memoire
  if (SquWin)    free_fmatrix_3d(SquWin,3);
  if (MatHis)    free_fmatrix_3d(MatHis,NbParam);
  if (MatHisMoy) free_dmatrix_1d(MatHisMoy);

 //Retour
  return nrj;
}


//----------------------------------------------------------*/
// MarkovSFSBM
// -----------
//
//  [seg][segN]        > Potentiel Contour
//  [segReg][segNReg]  > Sursegmentation Regions
//  [segP][segNP]      > Potentiel Contour non thikenned
//
// Tau=30
// hyper=2.3+complexity
//                           
//----------------------------------------------------------*/
void MarkovSFSBM(float** seg,float** segN,int nbcl,float** img,\
		       int lgth,int wdth,int sz,int scale,float hyper,float Tau,float** segP,float** segNP, int*** mat_img, char* MyBuff)
{
 int i,j,k,l,m,t;
 int label,labseg;
 int NbConnections,NbConnectionsN;
 int chgt,dist,posc,posr; 
 int nb,cpt,szN;
 float nrj,sumnrj,minnrj; 
 float ech,minBest,maxgrad;

	int ***ImgMyCont=imatrix_allocate_3d(3,lgth,wdth);
	float** display_mat=fmatrix_allocate_2d(lgth,wdth);
	float** display_mat_small=fmatrix_allocate_2d(lgth/scale,wdth/scale);
	bool verbose = true;
	bool verbose2 = true;
 

 //Presentation
 printf("\n\n\n");  
 printf("\n ----------------------------------------------- ");
 printf("\n ICM                                             ");
 printf("\n ----------------------------------------------- ");
 printf("\n  >>> [%d-%d]",lgth,wdth);
 printf("\n  >>> Size:[%d]",sz);
 printf("\n  >>> Scale:[%d]",scale);
 printf("\n  >>> NbClass:[%d]",nbcl);
 printf("\n");
 printf("\n  >>> HyperParam  :[%.2f]",hyper);
 printf("\n  >>> Tau  :[%.2f]",Tau);
 
 printf("\n");

 //Parametres
 //----------
 NbConnections=70; 
 szN=sz/scale;

 //Allocation
 //----------
  float***  matpds=fmatrix_allocate_3d(NbConnections+10,lgth,wdth);
  float**   matprec=fmatrix_allocate_2d(lgth,wdth);
  float**   matprec2=fmatrix_allocate_2d(lgth,wdth);
  float*    tabnrj=fmatrix_allocate_1d(nbcl);
  float*    tablab=fmatrix_allocate_1d(256);
  float*    distconx=fmatrix_allocate_1d(30*NbConnections);

  float***  matpdsN=fmatrix_allocate_3d(30*NbConnections,lgth/scale,wdth/scale);
  float**   matprecN=fmatrix_allocate_2d(lgth/scale,wdth/scale);
  float**   imgN=fmatrix_allocate_2d(lgth/scale,wdth/scale); 
  float**   imgNBest=fmatrix_allocate_2d(lgth/scale,wdth/scale);

  float**   segReg=fmatrix_allocate_2d(lgth,wdth);
  float**   segNReg=fmatrix_allocate_2d(lgth/scale,wdth/scale);
 
  for(k=0;k<NbConnections+10;k++) for(i=0;i<(lgth/scale);i++) for(j=0;j<(wdth/scale);j++)  matpdsN[k][i][j]=0.0;
  for(k=0;k<NbConnections+10;k++) for(i=0;i<lgth;i++)   for(j=0;j<wdth;j++)  matpds[k][i][j]=0.0;

  //Calcul RegionBlock [segReg][segNReg]
  //------------------------------------
  printf("\n  > RegionBlock");
  fflush(stdout);
  for(i=0;i<(lgth/scale);i++) for(j=0;j<(wdth/scale);j++) segNReg[i][j]=segNP[i][j];
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) segReg[i][j]=segP[i][j];

  for(i=0;i<(lgth/scale);i++) for(j=0;j<(wdth/scale);j++) 
    { if (segNReg[i][j]<Tau) segNReg[i][j]=255.0;      
      else segNReg[i][j]=0.0; }

  //for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) 
  //  { if (segReg[i][j]<Tau) segReg[i][j]=255.0;     
  //    else segReg[i][j]=0.0; }                      //MODIF 01.Mai.2009

  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)
    { k=(i/scale);
      l=(j/scale);
      if ((k<(lgth/scale))&&(l<(wdth/scale)))
      if (segNReg[k][l]==255) segReg[i][j]=255; }    

	if (verbose) { // stores regions map thats used for spatial constraining
		SaveImagePgm(MyBuff,".2_thresholding",segNReg,lgth/scale,wdth/scale);
		SaveImagePgm(MyBuff,".2_thresholding_interpol",segReg,lgth,wdth);
	}

  if (Tau>1)
     { ConvertCertainClassRegionInf(segReg,lgth,wdth);
       ConvertCertainClassRegionInf(segNReg,lgth/scale,wdth/scale); }

	

  
  //Connexion Basse-Resolution
  //--------------------------
  distconx[0]=1.0; 
  distconx[1]=wdth/scale; 
  distconx[2]=-1.0; 
  distconx[3]=-wdth/scale;

  cpt=4;
  for(k=-(szN/2);k<(szN/2);k++) for(l=-(szN/2);l<(szN/2);l++) 
    { dist=(k*(wdth/scale))+l;
      if (sqrt(k*k+l*l)>=2) { distconx[cpt]=dist; cpt++; } }
    
  NbConnectionsN=cpt;
  printf("\n  > Basse-Res:::Size[%d] NbConnections[%d]",szN,NbConnectionsN);
  fflush(stdout);

  //Calcul Pds [Homogeneite:1 | Contour:-1]
  //---------------------------------------
  for(i=0;i<(lgth/scale);i++) for(j=0;j<(wdth/scale);j++) 
      for(l=0;l<NbConnectionsN;l++)
	     {
               dist=(i*(wdth/scale))+j+(int)distconx[l]; 
               dist=dist%(((lgth/scale)*(wdth/scale)));              
               posr=(int)(dist/(wdth/scale));
               posc=(int)(dist%(wdth/scale));
               if (posr<0)                   posr=0;
               if (posr>((lgth/scale)-1))    posr=(lgth/scale)-1;  
               if (posc<0)                   posc=0;
               if (posc>((wdth/scale)-1))    posc=(wdth/scale)-1;

               if (sqrt(CARRE(i-posr)+CARRE(j-posc))>=2)
	       maxgrad=EstimGrad(i,j,posr,posc,segN);  

               else maxgrad=0;    
             
               matpdsN[l][i][j]=1-hyper*(maxgrad/255.0); } 

 
 //--------------------------------------------------
 //ICM [Basse-Resolution]
 //--------------------------------------------------
 printf("\n  > ICM Bass-Res...");
 fflush(stdout);
 minBest=1000000.0;
 labseg=0;
 for(t=0;t<15;t++)                 
 { 
  printf("\n[[%d]]\n",t);
  for(i=0;i<lgth/scale;i++) for(j=0;j<wdth/scale;j++) 
  imgN[i][j]=(int)(nbcl*(randomize()));
  //if (t==0) 
  //     { copy_mat(imgN,matprec,lgth/scale,wdth/scale);
  //       Recal(matprec,lgth/scale,wdth/scale);
  //       SaveImagePgm("Seg","",matprec,lgth/scale,wdth/scale); }

  //[ContrainteSegBlock]
  if (Tau>1) ContrainWithSeg(segNReg,lgth/scale,wdth/scale,imgN);

  for(k=0;;k++)
     {      
      sumnrj=0.0;
      copy_mat(imgN,matprecN,lgth/scale,wdth/scale);

        //Sauvegarde
        //if (t==0)
        //    { char Buffer[10];
        //    sprintf(Buffer,"%d",k);
        //    copy_mat(imgN,matprec,lgth/scale,wdth/scale); 
        //    Recal(matprec,lgth/scale,wdth/scale);
        //    SaveImagePgm("Seg",Buffer,matprec,lgth/scale,wdth/scale); } 

      for(i=0;i<lgth/scale;i++) for(j=0;j<wdth/scale;j++)
        {
  	 for(m=0;m<nbcl;m++)
	    {  
	     nrj=0.0;

	     for(l=0;l<NbConnectionsN;l++)
	          {
		   dist=(i*(wdth/scale))+j+(int)distconx[l];      
                   dist=dist%(((lgth/scale)*(wdth/scale)));        
                   posr=(int)(dist/(wdth/scale));
                   posc=(int)(dist%(wdth/scale));

                   if ((posr>0)&&(posr<((lgth/scale)-1))&&(posc>0)&&(posc<((wdth/scale)-1))) 
		      { if (imgN[posr][posc]!=m) nrj+=matpdsN[l][i][j];
                        if (imgN[posr][posc]==m) nrj-=matpdsN[l][i][j];
                        tabnrj[m]=nrj; }
                  }
            }     
	  minnrj=10000000;
          label=0;
          for(m=0;m<nbcl;m++) if (tabnrj[m]<minnrj) { minnrj=tabnrj[m]; label=m; }
          imgN[i][j]=label;
          sumnrj+=tabnrj[label]; 
	}

      //[ContrainteSegBlock]
      if (Tau>1) ContrainWithSeg(segNReg,lgth/scale,wdth/scale,imgN);

			if (t==0 && verbose2) { // stores labelling result of first low_res run for all iterations
				char ext[200];
				char iter[3];
				sprintf(iter, "%d", k);
				strcpy(ext, ".3_lowRes_");
				strcat(ext, iter);

				copy_mat(imgN, display_mat_small,lgth/scale,wdth/scale); 
			  Recal(display_mat_small,lgth/scale,wdth/scale);	
				SaveImagePgm(MyBuff,ext,display_mat_small,lgth/scale,wdth/scale);
			}

      chgt=DiffMat(imgN,matprecN,lgth/scale,wdth/scale);
      printf("\n  [%d]>%.2f [%d]",k,sumnrj,chgt);
      if (sumnrj<minBest) { printf("***"); minBest=sumnrj; labseg=t; copy_mat(imgN,imgNBest,lgth/scale,wdth/scale); }
      if ((chgt<1)||(k>2)) break; 
     }//--------------------------------------------   
  }
   
   //Info
   printf("\n  BestBest>>[%d]>%.2f      ",labseg,minBest);
   copy_mat(imgNBest,imgN,lgth/scale,wdth/scale);

  //Decimation Inverse
  //------------------
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) 
  if ((int(i/scale)<(lgth/scale))&&(int(j/scale)<(wdth/scale))) 
     img[i][j]=imgN[(int)(i/scale)][(int)(j/scale)]; 

  printf("\n\n  >> Decimation Inverse faite");

  //Debug
  copy_mat(img,matprec2,lgth,wdth); 
  Recal(matprec2,lgth,wdth);
  //SaveImagePgm("INIT","",matprec2,lgth,wdth); 

	if (verbose) { // stores result (= best of all runs) of low-res ICM
		SaveImagePgm(MyBuff,".3_lowResResult",matprec2,lgth,wdth);
		store_contour_image(ImgMyCont, mat_img, matprec2, lgth, wdth); 
		SaveImagePpm(MyBuff,".3_lowResResult",ImgMyCont,lgth,wdth);
	}
	

  //==========================================
  //======= FULL SCALE =======================
  //==========================================
  
  //Connexion Hte R\E9solution
  //------------------------
  distconx[0]=1.0; 
  distconx[1]=wdth; 
  distconx[2]=-1.0; 
  distconx[3]=-wdth;

  ech=(CARRE(sz)-4)/(float)NbConnections;
  ech=(int)(ech+0.4999);
  nb=0;
  cpt=4;
  for(k=-(sz/2);k<(sz/2);k++) for(l=-(sz/2);l<(sz/2);l++) 
    { dist=(k*wdth)+l;
      if (cpt>(NbConnections+4)) break;
      if (!(nb%(int)ech)) if (sqrt(k*k+l*l)>=2) 
	{ distconx[cpt]=dist; cpt++; } 
      nb++; }

  printf("\n  > Haute=Res:::NbCl[%d]  Size[%d]  NbCnx[%d]  Ech[%.0f]\n",nbcl,sz,cpt,ech);
  
  //Calcul Pds [Homogeneite:1 | Contour:-1]
  //---------------------------------------
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) 
      for(l=0;l<(NbConnections+4);l++)
	     {
               dist=(i*wdth)+j+(int)distconx[l]; 
               dist=dist%((lgth*wdth));              
               posr=(int)(dist/wdth);
               posc=(int)(dist%wdth);
               if (posr<0)           posr=0;
               if (posr>(lgth-1))    posr=(lgth-1);  
               if (posc<0)           posc=0;
               if (posc>(wdth-1))    posc=(wdth-1);

               if (sqrt(CARRE(i-posr)+CARRE(j-posc))>=2)
	       maxgrad=EstimGrad(i,j,posr,posc,seg);           

               else maxgrad=0;  
             
	       matpds[l][i][j]=1-hyper*(maxgrad/255.0); }

  printf("\n  > Calcul Pds Haute-Res. fait");
  fflush(stdout);

    
  //--------------------------------------------------
  //ICM [Haute resolution]
  //--------------------------------------------------
  for(k=0;;k++)
     {
      sumnrj=0.0;
      copy_mat(img,matprec,lgth,wdth);

       //Sauvegarde
       //char Buffer[10];
       //sprintf(Buffer,"_%d",k);
       //copy_mat(img,matprec2,lgth,wdth); 
       //Recal(matprec2,lgth,wdth);
       //SaveImagePgm("SEGMENT",Buffer,matprec2,lgth,wdth); 

      for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)
        {
  	 for(m=0;m<nbcl;m++)
	    {  
	     nrj=0.0;

	     for(l=0;l<(NbConnections+4);l++)
	          {
                   dist=(i*wdth)+j+(int)distconx[l];      
                   dist=dist%((lgth*wdth));        
                   posr=(int)(dist/wdth);
                   posc=(int)(dist%wdth);

                   if ((posr>0)&&(posr<(lgth-1))&&(posc>0)&&(posc<(wdth-1))) 
		     { if (img[posr][posc]!=m) nrj+=matpds[l][i][j];
                       if (img[posr][posc]==m) nrj-=matpds[l][i][j];
                       tabnrj[m]=nrj; }
                  }
            }     
	  minnrj=10000000;
          label=0;
          for(m=0;m<nbcl;m++) if (tabnrj[m]<minnrj) { minnrj=tabnrj[m]; label=m; }
          img[i][j]=label;
          sumnrj+=tabnrj[label]; 
	}

      //[ContrainteSegBlock]
      if (Tau>1) ContrainWithSeg(segReg,lgth,wdth,img);


			if (verbose2) { // stores labelling map for highres ICM for all iterations
				char ext[200];
				char iter[3];
				sprintf(iter, "%d", k);
				strcpy(ext, ".3_ResHigh_");
				strcat(ext, iter);

				copy_mat(img, display_mat,lgth,wdth); 
			  Recal(display_mat,lgth,wdth);	
				SaveImagePgm(MyBuff,ext,display_mat,lgth,wdth);
				store_contour_image(ImgMyCont, mat_img, img, lgth, wdth); 
				SaveImagePpm(MyBuff,ext,ImgMyCont,lgth,wdth);
			}

      chgt=DiffMat(img,matprec,lgth,wdth);
      if ((chgt<1)||(k>6)) break; //(k>4+Tau)
      printf("\n  [%d]>%.2f [%d]",k,sumnrj,chgt);
     }//--------------------------------------------
  //------------------------------------------------

 
 //Liberation 
	free_imatrix_3d(ImgMyCont,3);
	free_fmatrix_2d(display_mat);
	free_fmatrix_2d(display_mat_small);
	
  if (matpds)   free_fmatrix_3d(matpds,4);
  if (matprec)  free_fmatrix_2d(matprec);
  if (matprec2) free_fmatrix_2d(matprec2);
  if (distconx) free_fmatrix_1d(distconx);
  if (tabnrj)   free_fmatrix_1d(tabnrj);
  if (tablab)   free_fmatrix_1d(tablab);
  if (segReg)   free_fmatrix_2d(segReg);
  if (segNReg)  free_fmatrix_2d(segNReg);
}

//----------------------------------------------------------
//----------------------------------------------------------
// FuseBigRegion
// -------------
//  
//  pot: Potentiel Thick
//  seg: Img Regions
//                         
//----------------------------------------------------------
int FuseBigRegion(float** seg,float** pot,int lgth,int wdth,int sz,float hyper)
{
 int i,j,k,l,m,n,t;
 int coul,label;
 int NbConnections,dist,posc,posr; 
 int nb,cpt;
 int nbregions,NbRegionsInit,FuseRegion;

 float ech,maxgrad;
 float NRJ,NRJmin;
 float tmp;
 
 //Parametres
 //----------
 NbConnections=70; 
 FuseRegion=0;
 
 //Presentation
 //------------
 printf("\n\n\n");  
 printf("\n ------------------ ");
 printf("\n    FuseBigRegion   ");
 printf("\n ------------------ ");
 printf("\n  >>> [%d-%d]",lgth,wdth);
 printf("\n  >>> Size:[%d]",sz);
 printf("\n  >>> HyperParam  :[%.2f]",hyper);
 printf("\n");

 
 //Check NbRegions
 //---------------
 NbRegionsInit=0;
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) 
   if (seg[i][j]>NbRegionsInit)  NbRegionsInit=(int)seg[i][j];
 printf("\n  >>> NbRegionsInit/PixelMax  :[%d]",NbRegionsInit);


 //Allocation
 //----------
  float***  matpds=fmatrix_allocate_3d(NbConnections+10,lgth,wdth);
  float**  ImgTmp=fmatrix_allocate_2d(lgth,wdth);
  float**  segtmp=fmatrix_allocate_2d(lgth,wdth);
  float*   claslb=fmatrix_allocate_1d(NbRegionsInit);
  float*   TabNrj=fmatrix_allocate_1d(NbRegionsInit);
  float*   distconx=fmatrix_allocate_1d(NbConnections+10);

 //ChangeImgClass
 //--------------
 for(k=0;k<NbRegionsInit;k++) claslb[k]=0.0;

 k=0;
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) 
 if (!claslb[(int)seg[i][j]]) claslb[(int)seg[i][j]]=k++;

 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) seg[i][j]=claslb[(int)seg[i][j]];

 //Initialisation
 //--------------
  for(k=0;k<NbConnections+10;k++) for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  matpds[k][i][j]=0.0;
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) ImgTmp[i][j]=0.0;

  //Connexion Hte R\E9solution
  //------------------------
  distconx[0]=1.0; 
  distconx[1]=wdth; 
  distconx[2]=-1.0; 
  distconx[3]=-wdth;

  ech=(CARRE(sz)-4)/(float)NbConnections;
  ech=(int)(ech+0.4999);
  nb=0;
  cpt=4;
  for(k=-(sz/2);k<(sz/2);k++) for(l=-(sz/2);l<(sz/2);l++) 
    { dist=(k*wdth)+l;
      if (cpt>(NbConnections+4)) break;
      if (!(nb%(int)ech)) if (sqrt(k*k+l*l)>=2) 
	{ distconx[cpt]=dist; cpt++; } 
      nb++; }

  printf("\n  > Haute=Res:::Size[%d]  NbCnx[%d]  Ech[%.0f]\n",sz,cpt,ech);
  
  //Calcul Pds [Homogeneite:1 | Contour:-1]
  //---------------------------------------
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) 
      for(l=0;l<(NbConnections+4);l++)
	     {
               dist=(i*wdth)+j+(int)distconx[l]; 
               dist=dist%((lgth*wdth));              
               posr=(int)(dist/wdth);
               posc=(int)(dist%wdth);
               if (posr<0)           posr=0;
               if (posr>(lgth-1))    posr=(lgth-1);  
               if (posc<0)           posc=0;
               if (posc>(wdth-1))    posc=(wdth-1);

               if (sqrt(CARRE(i-posr)+CARRE(j-posc))>=2)
	       maxgrad=EstimGrad(i,j,posr,posc,pot);           

               else maxgrad=0;  
             
	       matpds[l][i][j]=1-hyper*(maxgrad/255.0); }

  printf("\n  > Calcul Pds Haute-Res. fait");
  fflush(stdout);

 //Calcul Nrj Init.
 //---------------
  NRJ=0.0;
  for(k=0;k<lgth;k++) for(l=0;l<wdth;l++) 
    { tmp=0.0;
      for(m=0;m<(NbConnections+4);m++)
         { dist=(k*wdth)+l+(int)distconx[m];      
           dist=dist%((lgth*wdth));        
           posr=(int)(dist/wdth);
           posc=(int)(dist%wdth);

           if ((posr>0)&&(posr<(lgth-1))&&(posc>0)&&(posc<(wdth-1))) 
	      { if (seg[posr][posc]!=seg[k][l]) tmp+=matpds[m][k][l];
                else                            tmp-=matpds[m][k][l]; } }
      NRJ+=tmp; }

  printf("\n  > NRJinit=[%.2f]",NRJ);
  fflush(stdout);

 //-------------------------------------------------------
 //Boucle
 //-------------------------------------------------------
  FuseRegion=0;
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) if (ImgTmp[i][j]==0)
        {
	 //PourChaqueRegion   
	 coul=(int)seg[i][j];  
         
         for(k=0;k<lgth;k++) for(l=0;l<wdth;l++) 
	   if ((int)seg[k][l]==coul) ImgTmp[k][l]=-1.0; 

         //Info
         printf("\n Region[%d]",coul); fflush(stdout);    

         //HistoRegionsVoisines 
         for(k=0;k<NbRegionsInit;k++)  claslb[k]=0;
         for(k=0;k<lgth;k++) for(l=0;l<wdth;l++) if (seg[k][l]==coul)   
	      {
                for(m=-1;m<2;m++) for(n=-1;n<2;n++) 
		  if (((k+m)>0)&&((k+m)<lgth)&&((l+n)>0)&&((l+n)<wdth))
		     claslb[(int)seg[k+m][l+n]]++;             
	      } 
          claslb[coul]=0; 

         //NumberRegionsVoisines >> claslb[nbregions]
         for(nbregions=0,n=0;n<NbRegionsInit;n++) if  (claslb[n]) nbregions++;
         printf(">%d::",nbregions);  
         for(m=0,n=0;n<NbRegionsInit;n++)  if (claslb[n]) { claslb[m]=n; m++; }
         for(k=0;k<nbregions;k++) printf("%.0f-",claslb[k]);
         fflush(stdout);

             //------------------------------------------------------------
             // Calcul NRJmin resultat fusion Etiq[ coul: claslb[label] ]
             //------------------------------------------------------------   
                 NRJ=0.0;
                 for(k=0;k<lgth;k++) for(l=0;l<wdth;l++) for(m=0;m<(NbConnections+4);m++)
  	            { dist=(k*wdth)+l+(int)distconx[m];      
                      dist=dist%((lgth*wdth));        
                      posr=(int)(dist/wdth);
                      posc=(int)(dist%wdth);

                      if ((posr>0)&&(posr<(lgth-1))&&(posc>0)&&(posc<(wdth-1))) 
		         { if (seg[posr][posc]!=seg[k][l]) NRJ+=matpds[m][k][l];
                           else                            NRJ-=matpds[m][k][l]; } }

		 for(t=0;t<nbregions;t++)
		    {
		     for(k=0;k<lgth;k++) for(l=0;l<wdth;l++) 
                       { if (seg[k][l]==claslb[t]) segtmp[k][l]=coul; 
                         else                      segtmp[k][l]=seg[k][l]; }

                     tmp=0.0;
                     for(k=0;k<lgth;k++) for(l=0;l<wdth;l++) for(m=0;m<(NbConnections+4);m++)
  	                { dist=(k*wdth)+l+(int)distconx[m];      
                          dist=dist%((lgth*wdth));        
                          posr=(int)(dist/wdth);
                          posc=(int)(dist%wdth);

                         if ((posr>0)&&(posr<(lgth-1))&&(posc>0)&&(posc<(wdth-1))) 
		            { if (segtmp[posr][posc]!=segtmp[k][l]) tmp+=matpds[m][k][l];
                              else                                  tmp-=matpds[m][k][l]; } }

                      
		     TabNrj[t]=tmp;
	            }
 
                 NRJmin=1000000000;
                 label=-1;
                 for(t=0;t<nbregions;t++) if (TabNrj[t]<NRJmin) { NRJmin=TabNrj[t]; label=t; }
             //------------------------------------------------------------       
	         
         
         //Fusion---------------------
         if (NRJmin<NRJ)
	    {
	     printf("\n    Fusion > [Rnb=%.0f]->[Rnb=%.0f] (d=%.3f)(T=%.3f)",seg[i][j],claslb[(int)label],NRJmin,NRJ);
             fflush(stdout);
             FuseRegion++;

             for(k=0;k<lgth;k++) for(l=0;l<wdth;l++) 
		if (seg[k][l]==coul)  seg[k][l]=claslb[(int)label];  
	    }//-----------------------	
	}  

 //Liberation 
  //if (ImgTmp)   free_fmatrix_2d(ImgTmp);
  //if (segtmp)   free_fmatrix_2d(segtmp);
  //if (claslb)   free_fmatrix_1d(claslb);
  //if (TabNrj)   free_fmatrix_1d(TabNrj);

  //Info & Retour
 printf("\n  >>>   FuseRegion [%d]",FuseRegion);
 return FuseRegion; 
}

