//------------------------------------------------------
// module  : FunctionMDSCCT.cc
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
#include "FunctionMDSCCT.h"


//------------------------ My additions -------------
//
//

void store_contour_image(float ***ImgMyCont, float ***ImgC, float **ImgSeg, int height, int width) {

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
  mapsize = ny*nx;
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



//-------------------------//
//-- Matrice de Flottant --//
//-------------------------//
//---------------------------------------------------------
//  alloue de la memoire pour une matrice 1d de float
//----------------------------------------------------------
float* fmatrix_allocate_1d(int hsize)
 {
  float* matrix;
  matrix=new float[hsize]; return matrix; }

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

//----------------------------------------------------------
// alloue de la memoire pour une matrice 4d de float
//----------------------------------------------------------
float**** fmatrix_allocate_4d(int fsize,int dsize,int vsize,int hsize)
 {
  float**** matrix;

  matrix=new float***[fsize];

  for(int i=0;i<fsize;i++)
    matrix[i]=fmatrix_allocate_3d(dsize,vsize,hsize);
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

//----------------------------------------------------------
// alloue de la memoire pour une matrice 4d de int
//----------------------------------------------------------
int**** imatrix_allocate_4d(int fsize,int dsize,int vsize,int hsize)
 {
  int**** matrix;

  matrix=new int***[fsize];

  for(int i=0;i<fsize;i++)
    matrix[i]=imatrix_allocate_3d(dsize,vsize,hsize);
  return matrix;
 }


//-----------------------------//
//-- Matrice d'entiers short --//
//-----------------------------//
//---------------------------------------------------------
//  alloue de la memoire pour une matrice 1d de short int
//----------------------------------------------------------
short int* simatrix_allocate_1d(int hsize)
 {
  short int* matrix;
  matrix=new short int[hsize]; return matrix; }

//----------------------------------------------------------
//  alloue de la memoire pour une matrice 2d de short int
//----------------------------------------------------------
short int** simatrix_allocate_2d(int vsize,int hsize)
 {
  short int** matrix;
  short int*  imptr;

  matrix=new short int*[vsize];
  imptr=new  short int[(hsize)*(vsize)];
  for(int i=0;i<vsize;i++,imptr+=hsize) matrix[i]=imptr;
  return matrix;
 }

//----------------------------------------------------------
// alloue de la memoire pour une matrice 3d de short int
//----------------------------------------------------------
short int*** simatrix_allocate_3d(int dsize,int vsize,int hsize)
 {
  short int*** matrix;

  matrix=new short int**[dsize];

  for(int i=0;i<dsize;i++)
    matrix[i]=simatrix_allocate_2d(vsize,hsize);
  return matrix;
 }

//-------------------------//
//--- Matrice de double ---//
//-------------------------//
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
  double* imptr;

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
// libere la memoire de la matrice 4d de float
//----------------------------------------------------------
void free_fmatrix_4d(float**** pmat,int fsize,int dsize)
{
 for(int i=0;i<fsize;i++)
  free_fmatrix_3d(pmat[i],dsize);
  
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
// libere la memoire de la matrice 4d de int
//----------------------------------------------------------
void free_imatrix_4d(int**** pmat,int fsize,int dsize)
{
 for(int i=0;i<fsize;i++)
  free_imatrix_3d(pmat[i],dsize);

 delete[] (pmat);
}

//----------------------------------------------------------
// libere la memoire de la matrice 1d de short int
//----------------------------------------------------------
void free_simatrix_1d(short int* pmat)
{ delete[] pmat; }

//----------------------------------------------------------
// libere la memoire de la matrice 2d de short int
//----------------------------------------------------------
void free_simatrix_2d(short int** pmat)
{ delete[] (pmat[0]);
  delete[] pmat;}

//----------------------------------------------------------
// libere la memoire de la matrice 3d de short int
//----------------------------------------------------------
void free_simatrix_3d(short int*** pmat,int dsize)
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
//--------------------//
//-- Matrix Gestion --//
//--------------------//
//----------------------------------------------------------
// copie une matrice dans une autre
//----------------------------------------------------------
void CopyMat(float*** pmatl1,float*** pmatl2,int lgth,int wdth,int nbcoul)
{
 int i,j,k;

  for(k=0;k<nbcoul;k++)
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)
    pmatl1[k][i][j]=pmatl2[k][i][j]; 
}

//----------------------------------------------------------
// copie une matrice dans une autre
//----------------------------------------------------------
void CopyMat(float** mat1,float** mat2,int lgth,int wdth)
{
 int i,j;

  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)
  mat1[i][j]=mat2[i][j]; 
}

//---------------------------------------------------
// Recal                                  
//---------------------------------------------------
void Recal(float** mat,int lgth,int wdth)
{
 int i,j;
 float max,min;

 //Initialisation
 max=-1000000.0;
 min=100000000;

 //Recherche du min
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)
    if (mat[i][j]<min) min=mat[i][j];

 //plus min
   for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)
    mat[i][j]-=min;

 //Recherche du max
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) 
    if (mat[i][j]>max) max=mat[i][j];

 //Recalibre la matrice
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)
   if (max) mat[i][j]*=(GREY_LEVEL/max);      
}


//---------------------------------------------------
// Recal                                  
//---------------------------------------------------
void Recal(float*** mat,int lgth,int wdth)
{
 int i,j,k;
 float max,min;
 float tmp;

 //Boucle
 for(k=0;k<3;k++)
   {
    max=-10000;
    min=10000;

    //>min/max
    for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) if (mat[k][i][j]<min) min=mat[k][i][j];
    for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) if (mat[k][i][j]>max) max=mat[k][i][j];

    //>Debug
    printf("[min=%.1f max=%.1f]",min,max);
    fflush(stdout);
  
    //>Recalibre
    for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) if (max) 
      { tmp=((float)mat[k][i][j]-(float)min)*(GREY_LEVEL/(float)(max-min));
        if (tmp<0)    tmp=0.0;
        if (tmp>255) tmp=255.0; 
        mat[k][i][j]=tmp; }  
   }     
}


//----------------------------------------------------------
// ComputeDiff
//----------------------------------------------------------
float ComputeDiff(float** mat1,float** mat2,int lgth,int wdth)
{
 int i,j;
 float Diff;

 Diff=0.0;
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) Diff+=fabs(mat1[i][j]-mat2[i][j]);
 Diff/=(lgth*wdth);

 return Diff;
}

//----------------------------------------------------------
// AddMatrix   <matres=mata+fact*matb>               
//----------------------------------------------------------     
void AddMatrix(float** matout,float** matin1,float fact,float** matin2,int lgth,int wdth)
{
 int i,j;

 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) 
     matout[i][j]=matin1[i][j]+(fact*matin2[i][j]);
}

//----------------------------------------------------------
// MultMatrix   <matout=matin1*matin2>               
//----------------------------------------------------------     
void MultMatrix(float** matout,float Beta,float** matin,int lgth,int wdth)
{
 int i,j;

 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) matout[i][j]=Beta*matin[i][j];
}

//----------------------------------------------------------
// InitMatrix
//----------------------------------------------------------     
void InitMatrix(float** mat,int lgth,int wdth)
{
 int i,j;

 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) mat[i][j]=0.0;
}


//----------------------//
//----------------------//
//-- FILE -PGM/PPM------//
//----------------------//
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
void load_image_ppm(char* path,float*** img,int lgth,int wdth)
 {
  int i,j;
  int temp;
  char* tempc;
  unsigned char varb,varv,varr;
  char Buff[600],stringTmp1[600],stringTmp2[600];
  int ta1,ta2,ta3;
  FILE *fic;

  //enregistrement du chemin;
   strcpy(Buff,path);
   if (strstr(Buff,".ppm")==NULL) strcat(Buff,".ppm");

   //ouverture du fichier
  fic=fopen(Buff,"r");
  if (fic==NULL) 
   { printf("\n ->> Grave erreur a l'ouverture de %s -\n",Buff);
     exit(-1); }
   printf("\n Ouverture de -> %s au format ppm",Buff);
   fflush(stdout);
  
  //recuperation de l'entete
  tempc=fgets(stringTmp1,100,fic); //P6
  tempc=fgets(stringTmp2,600,fic); //Comment

  for(;;)
    { if (((int)stringTmp2[0]==10)||((int)stringTmp2[0]==35))
      tempc=fgets(stringTmp2,600,fic); 

      else break; }

  temp=sscanf(tempc," %d %d",&ta1,&ta2);
  temp=fscanf(fic," %d",&ta3);

  //affichage de l'entete
  //printf("\n [%s::%d::%d::%d]",stringTmp1,ta1,ta2,ta3);
  printf("  [%d::%d::%d]",ta1,ta2,ta3);
  fflush(stdout);
   
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
// Chargement de l'image de nom <name> (en pgm)             
//----------------------------------------------------------
void load_image_pgm(char* path,float** img,int lgth,int wdth)
 {
  int i,j;
  int temp;
  char* tempc;
  unsigned char var;
  char Buff[600],stringTmp1[600],stringTmp2[600];
  int ta1,ta2,ta3;
  FILE *fic;

  //enregistrement du chemin;
   strcpy(Buff,path);
   if (strstr(Buff,".pgm")==NULL) strcat(Buff,".pgm");

   //ouverture du fichier
  fic=fopen(Buff,"r");
  if (fic==NULL) 
   { printf("\n ->> Grave erreur a l'ouverture de %s -\n",Buff);
     exit(-1); }
   printf("\n Ouverture de -> %s au format pgm",Buff);
   fflush(stdout);
  
  //recuperation de l'entete
  tempc=fgets(stringTmp1,100,fic); //P5
  tempc=fgets(stringTmp2,600,fic); //Comment

  for(;;)
    { if (((int)stringTmp2[0]==10)||((int)stringTmp2[0]==35))
      tempc=fgets(stringTmp2,600,fic); 

      else break; }

  temp=sscanf(tempc," %d %d",&ta1,&ta2);
  temp=fscanf(fic," %d",&ta3);

  //affichage de l'entete
  printf("  [%d::%d::%d]",ta1,ta2,ta3);
  fflush(stdout);
   
  //chargement dans la matrice
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)  
    { temp=fread(&var,1,1,fic);  
      img[i][j]=var; }      

  //fermeture du fichier
  fclose(fic);
 }

//----------------------------------------------------------
// Sauvegarde de l'image de nom <name> au format ppm        
//----------------------------------------------------------
void SaveImagePpm(char* Path_out,float*** matrvb,int lgth,int wdth)
 {
   int i,j,k;
  char buff[200];
  FILE* fuser;

  //extension
  strcpy(buff,Path_out);
  strcat(buff,".ppm");

  //ouverture fichier
  fuser=fopen(buff,"w");
    if (fuser==NULL) 
        { printf(" probleme dans la sauvegarde de %s",buff); 
          exit(-1); }

  //affichage
  printf("\n Sauvegarde dans -> %s au format %s",buff,"ppm");
  fflush(stdout);

  //sauvegarde de l'entete
  fprintf(fuser,"P6");
  //fprintf(fuser,"\n# %s \n",Com);
  fprintf(fuser,"%d %d",wdth,lgth);
  fprintf(fuser,"\n255\n");

  for(k=0;k<3;k++) for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)
    { matrvb[k][i][j]=(int)(matrvb[k][i][j]+0.49);
      if (matrvb[k][i][j]<0)   matrvb[k][i][j]=0;
      if (matrvb[k][i][j]>255) matrvb[k][i][j]=255; }      

  //enregistrement
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) 
    { fprintf(fuser,"%c",(char)matrvb[0][i][j]);
      fprintf(fuser,"%c",(char)matrvb[1][i][j]);
      fprintf(fuser,"%c",(char)matrvb[2][i][j]); }
       
  //fermeture fichier
   fclose(fuser); 
 }

//----------------------------------------------------------
// Sauvegarde de l'image de nom <name> au format pgm                        
//----------------------------------------------------------                
void SaveImagePgm(char* name,float** mat,int lgth,int wdth)
{
 int i,j;
 char buff[300];
 FILE* fic;

  //--extension--
  strcpy(buff,name);
  strcat(buff,".pgm");

  //--ouverture fichier--
  fic=fopen(buff,"wb");
    if (fic==NULL) 
        { printf("Probleme dans la sauvegarde de %s",buff); 
          exit(-1); }
  printf("\n Sauvegarde dans -> %s au format pgm",buff);

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


//--------------------------------
//--------------------------------
//---GESTION COULEUR--------------
//--------------------------------
//----------------------------------------------------------*/
// RGBToLAB                     
//----------------------------------------------------------*/
void RGBToLAB(float r,float g,float bl,float* l,float* a,float* b)
{
 float x,y,z;

 RGBToXYZ(r,g,bl,&x,&y,&z);
 XYZToLAB(x,y,z,l,a,b);
}

//----------------------------------------------------------*/
// RGBToXYZ                    
//----------------------------------------------------------*/
void RGBToXYZ(float r,float g,float b,float* x,float* y,float* z)
{
  r=(r/255.0);         //R from 0 to 255
  g=(g/255.0);         //G from 0 to 255
  b=(b/255.0);         //B from 0 to 255

  if (r>0.04045) r=powf(((r+0.055)/1.055),2.4);
  else           r=r/12.92;
  if (g>0.04045) g=powf(((g+0.055)/1.055),2.4);
  else           g=g/12.92;
  if (b>0.04045) b=powf(((b+0.055)/1.055),2.4);
  else           b=b/12.92;

  r=r*100;
  g=g*100;
  b=b*100;

  (*x)=r*0.4124 + g*0.3576 + b*0.1805;
  (*y)=r*0.2126 + g*0.7152 + b*0.0722;
  (*z)=r*0.0193 + g*0.1192 + b*0.9505;
}

//----------------------------------------------------------*/
// XYZToLAB                     
//----------------------------------------------------------*/
void XYZToLAB(float x,float y,float z,float* l,float* a,float* b)
{
  x=x/95.047;           
  y=y/100.000;          
  z=z/108.883;       

  if (x>0.008856) x=powf(x,1.0/3.0);
  else            x=(7.787*x)+(16.0/116.0);
  if (y>0.008856) y=powf(y,1.0/3.0);
  else            y=(7.787*y)+(16.0/116.0);
  if (z>0.008856) z=powf(z,1.0/3.0); 
  else            z=(7.787*z)+(16.0/116.0);

  (*l)=(116.0*y)-16.0;
  (*a)=500.0*(x-y);
  (*b)=200.0*(y-z);
}

//----------------------------------------------------------
// Convert RGBToLAB
//----------------------------------------------------------
void ConvertImgRGBToLAB(float*** RGB,float*** LAB,int lgth,int wdth)
{
 int i,j;

 //Boucle
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)
 RGBToLAB(RGB[0][i][j],RGB[1][i][j],RGB[2][i][j],&(LAB[0][i][j]),&(LAB[1][i][j]),&(LAB[2][i][j]));
}

//-------------
// RANDOM -----
//-------------
//----------------------------------------------------------
// retourne un nombre aleatoire entre zero et un
//----------------------------------------------------------
float randomize(void)
{ return ((float)rand()/RAND_MAX); }


//-----------------
// CONTOUR VALUE --
//-----------------
//----------------------------------------------------------
// EstimPotCont
//----------------------------------------------------------
float EstimPotCont(int rowbeg,int colbeg,int rowend,int colend,float** grad)
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
          cord+=pente; }
     
     if (rowbeg>rowend)
     for(k=rowbeg;k>=rowend;k--)
        { if (grad[k][(int)cord]>max) max=grad[k][(int)cord]; 
          cord-=pente; }

     cord=colend;
     k=rowend;
     if (grad[k][(int)cord]>max) max=grad[k][(int)cord];    
    }

 if (fabs(diffcol)>fabs(diffrow))
    { pente=(float)(diffrow)/(float)(diffcol);
      inccol=diffcol/fabs(diffcol);    
      cord=rowbeg;

      if (colend>=colbeg)
          for(k=colbeg;k<=colend;k++)
             { if (grad[(int)cord][k]>max) max=grad[(int)cord][k]; 
               cord+=pente;}

      if (colbeg>colend)
          for(k=colbeg;k>=colend;k--)
             { if (grad[(int)cord][k]>max) max=grad[(int)cord][k];
               cord-=pente; } 

          cord=rowend;
          k=colend;
          if (grad[(int)cord][k]>max) max=grad[(int)cord][k];       
    }

 //Retour
 return max/255.0;
}

//---------------------//
// COMPLEXITY ---------//
//---------------------//
//----------------------------------------------------------
// Complexity
//----------------------------------------------------------   
float Complexity(float***  img,int lgth,int wdth,int div,int SzW)
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


