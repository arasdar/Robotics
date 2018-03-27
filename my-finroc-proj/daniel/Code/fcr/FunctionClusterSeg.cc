//------------------------------------------------------
// module  : Function.cc
// auteur  : Mignotte Max
// date    :
// version : 1.0
// langage : C++
// labo    : DIRO
// note    :
//------------------------------------------------------
// quelques fonctions de reservations memoires utiles 

//------------------------------------------------
// FICHIERS INCLUS -------------------------------
//------------------------------------------------
#include "FunctionClusterSeg.h"


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
//-- Matrice de Flottant --//
//--------------------------//
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


//--------------------//
//-- Matrix Gestion --//
//--------------------//
//----------------------------------------------------------
// copie une matrice dans une autre
//----------------------------------------------------------
void copy_mat(int*** pmatl1,int*** pmatl2,int lgth,int wdth,int nbcoul)
{
 int i,j,k;

  for(k=0;k<nbcoul;k++)
  for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)
  pmatl2[k][i][j]=pmatl1[k][i][j]; 
}
//----------------------------------------------------------
// copy_matrix
//----------------------------------------------------------
void copy_matrix(float** mat1,float** mat2,int size)
{
 int i,j;

 for(i=0;i<size;i++) for(j=0;j<size;j++)
   mat1[i][j]=mat2[i][j]; 
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

//--------------//
// RANDOM ------//
//--------------//
//----------------------------------------------------------
// retourne un nombre aleatoire entre zero et un
//----------------------------------------------------------
float randomize(void)
{ return ((float)rand()/RAND_MAX); }


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
  time_t tm;

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
  fprintf(fuser,"\n# IMG Module, %s",ctime(&tm));
  fprintf(fuser,"%d %d",lgth,wdth);
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

 //RGB en [0-1]
  r/=255.0;
  g/=255.0;
  bl/=255.0;

 //Conversion RGB->XYZ
  RGBToXYZ(r,g,bl,&x,&y,&z);
  RGBToXYZ(1.0,1.0,1.0,&xn,&yn,&zn);

  //Calcul uprime/vprime
  uprime=(4*x)/(x+(15*y)+(3*z));
  vprime=(9*y)/(x+(15*y)+(3*z));

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

//----------------------------------------------------------
//
//  image[lgth][wdth] 
// 
//  bascol::         
//  -0- RGB ->    [X,Y,Z] = [0,255]
//  -1- HSV ->    H=[0::360]  S=[0::1]       V=[0::255]
//  -2- YIQ -     Y=[0::1]    I=[-0.55::0.55]  Q=[-0.4::0.4]
//  -3- XYZ ->    [X,Y,Z] = [0,255]
//  -4- LAB -     L=[0::100]    A=[-128::143]  B=[-258::237]
//  -5- LUV -     L=[-16::100]  U=[-60::145]   V=[-130::117]    
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
int Identical(float* VctPts,float** MtxCtrs,int nbcl,int nbpar)
{
 int k,l;
 int value=0;

 for(k=0;k<nbcl;k++) 
   { 
     value=1;
     for(l=0;l<nbpar;l++) if (VctPts[l]!=MtxCtrs[k][l]) value=0;
     if (value) return value;
   }
    
 return value;
}

//----------------------------------------------------------
//  K-Mean  
//----------------------------------------------------------
void Kmean(float** MatrixPts,int NbClusters,int NbPts,int NbParam,int* VctLabel,int Seed,int FlgBhat)
{
 int i,j,k,l;
 int classe;
 float aleat;
 float nb;
 float tmp;
 float  dist,distmin;

 float NrjIntra;
 float NrjInter;
 float Nrj;

 int flag;
 
 //Allocation memoire
 float** MatrixCtrs=fmatrix_allocate_2d(NbClusters,NbParam);
 float* PtsCtrs=fmatrix_allocate_1d(NbParam);
 float* Prop=fmatrix_allocate_1d(NbClusters);
 float* VDist=fmatrix_allocate_1d(NbPts);

 //Presentation
 printf("\n  <Seed:%d> -> Funct.Kmean",Seed);
 srand(Seed);
 if (FlgBhat) printf("\n  K-Moy > [Dist. Bhattacharyya]"); 


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

     if (nb>1) printf(" <|%.0f Essais|>",nb);

     for(k=0;k<NbParam;k++) MatrixCtrs[i][k]=MatrixPts[(int)aleat][k]; } 


  //------------------------------------------------------------------
  //------------------------------------------------------------------

  //Normalization/DistBhatta
  if (FlgBhat)
  for(i=0;i<NbClusters;i++)
     { tmp=0.0;
       for(k=0;k<NbParam;k++) tmp+=MatrixCtrs[i][k];
       for(k=0;k<NbParam;k++) if (tmp) MatrixCtrs[i][k]/=tmp; 

       if (!tmp) 
         for(k=0;k<NbParam;k++) MatrixCtrs[i][k]=(1.0/NbParam); } 
 
  //Debug
  if (0)
  for(i=0;i<NbClusters;i++) 
    {  printf("\n");
       for(k=0;k<NbParam;k++) printf("[%.2f]",MatrixCtrs[i][k]); }
    
 
  printf("\n    ");
 //================================================================
 //BOUCLE =========================================================
 //================================================================
 for(l=0;l<300;l++)
   {  
       //-[1]- Select Cluster Membership
       //-------------------------------
       flag=0; classe=0;

       for(j=0;j<NbPts;j++)
          {   
           distmin=BIGNUMBER;  

           for(i=0;i<NbClusters;i++)
              { dist=0.0; 
                
                //Square Euclidean or Cramer/Von Mises type distance
                if (!FlgBhat)
                for(k=0;k<NbParam;k++) dist+=CARRE(MatrixCtrs[i][k]-MatrixPts[j][k]);

                //Distance Bhattacharya
		if (FlgBhat)
                   { for(k=0;k<NbParam;k++) dist+=sqrt(MatrixCtrs[i][k]*MatrixPts[j][k]);
		     dist=exp(-dist); }  

                //Distance Euclidienne -[Ok]-
                //if (FlgBhat)
                //for(k=0;k<NbParam;k++) dist+=CARRE(MatrixCtrs[i][k]-MatrixPts[j][k]);

                //Distance Kullback -[Ok]-
                //if (FlgBhat)
                //for(k=0;k<NbParam;k++) 
                //   { if (!MatrixCtrs[i][k])  MatrixCtrs[i][k]=0.01;
                //     if (!MatrixPts[j][k])   MatrixPts[j][k]=0.01;
                //     dist+=(MatrixPts[j][k])*log(MatrixPts[j][k]/(MatrixCtrs[i][k])); }

                //Distance Jensen -[Ok]-
                //if (FlgBhat)
                //for(k=0;k<NbParam;k++) 
                //   { if (!MatrixCtrs[i][k])  MatrixCtrs[i][k]=0.01;
                //     if (!MatrixPts[j][k])   MatrixPts[j][k]=0.01;
                //
                //     float half=(MatrixPts[j][k]+MatrixCtrs[i][k])/2.0;
                // 
                //     dist+=(MatrixCtrs[i][k])*log(MatrixCtrs[i][k]/half);
                //     dist+=(MatrixPts[j][k])*log(MatrixPts[j][k]/half); 
                //     dist/=2.0; }

                //Distance de Kolmogorov-Smirnov -[Ok]-
                //if (FlgBhat)
                // {  for(k=0;k<NbParam;k++) 
                //      { if (fabs(MatrixCtrs[i][k]-MatrixPts[j][k])>dist)
                //       dist=fabs(MatrixCtrs[i][k]-MatrixPts[j][k]); } }

                //Distance Histogram intersection -[Ok]-
                //  if (FlgBhat)
		//     { for(k=0;k<NbParam;k++) 
                //        { if (MatrixCtrs[i][k]<MatrixPts[j][k]) dist+=MatrixCtrs[i][k];
                //          else  dist+=MatrixPts[j][k]; }
		//          dist=1-dist; }
                
                //Distance Chord -[Ok]-
                //if (FlgBhat) 
                //for(k=0;k<NbParam;k++) dist+=CARRE(sqrt(MatrixCtrs[i][k])-sqrt(MatrixPts[j][k]));

                //Distance Manhattan -[Ok]-
                //if (FlgBhat)
                //for(k=0;k<NbParam;k++) dist+=fabs(MatrixCtrs[i][k]-MatrixPts[j][k]);
             

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
	   for(k=0;k<NbParam;k++) MatrixCtrs[i][k]=0.0;
           
           
	   for(j=0;j<NbPts;j++) if (VctLabel[j]==i)
	      { nb++;
                for(k=0;k<NbParam;k++) MatrixCtrs[i][k]+=MatrixPts[j][k]; }

           for(k=0;k<NbParam;k++) if (nb) MatrixCtrs[i][k]/=nb;

           //Normalization Distance Bhattacharya
           if (FlgBhat)
	      { tmp=0.0;
                for(k=0;k<NbParam;k++) tmp+=MatrixCtrs[i][k];
                for(k=0;k<NbParam;k++) if (tmp) MatrixCtrs[i][k]/=tmp; 

                if (!tmp) 
                for(k=0;k<NbParam;k++) MatrixCtrs[i][k]=(1.0/NbParam); }
          }
            

       //Debug
       //-----
       if (0)
       for(i=0;i<NbClusters;i++) { printf("\n"); for(k=0;k<NbParam;k++) printf("[%.5f]",MatrixCtrs[i][k]); }

   }//==============================================================
 //================================================================= 

 //Proportion
 for(i=0;i<NbClusters;i++)  Prop[i]=0.0;
 
 for(i=0;i<NbClusters;i++) 
    { for(nb=0,j=0;j<NbPts;j++) if (VctLabel[j]==i) nb++;
      Prop[i]=(nb/((float)NbPts)); }

 
 //Calcul NRJ=Sum Dist_IntraClasse 
 NrjIntra=0.0;
 for(i=0;i<NbClusters;i++) 
    {
     for(j=0;j<NbPts;j++) if (VctLabel[j]==i)
        {
         dist=0.0;

         if (!FlgBhat)
         for(k=0;k<NbParam;k++) dist+=CARRE(MatrixCtrs[i][k]-MatrixPts[j][k]);

         else { for(k=0;k<NbParam;k++) dist+=CARRE(MatrixCtrs[i][k]*MatrixPts[j][k]);
                dist=exp(-dist); }

	 NrjIntra+=dist;
        }
    }  

 NrjIntra/=((float)NbPts);
 printf("\n NrjIntra > %f",NrjIntra);  

 //Calcul Centre 
 for(k=0;k<NbParam;k++) PtsCtrs[k]=0.0; 

 for(i=0;i<NbClusters;i++) for(k=0;k<NbParam;k++) PtsCtrs[k]+=MatrixCtrs[i][k];
 for(k=0;k<NbParam;k++)  PtsCtrs[k]/=((float)NbClusters);

 //Calcul NRJ=Sum Dist_InterClasse 
 NrjInter=0.0;
 for(i=0;i<NbClusters;i++) 
    { dist=0.0;

      if (!FlgBhat)
      for(k=0;k<NbParam;k++) dist+=CARRE(MatrixCtrs[i][k]-PtsCtrs[k]);

      else { for(k=0;k<NbParam;k++) dist+=CARRE(MatrixCtrs[i][k]*PtsCtrs[k]);
                dist=exp(-dist); }

      NrjInter+=Prop[i]*dist; }

 printf("\n NrjInter > %f",NrjInter);

 //Calcul NRJ
 Nrj=NrjIntra/NrjInter;
 printf("\n Nrj > %f",Nrj);
 
 //FreeMemory
 if (MatrixCtrs) free_fmatrix_2d(MatrixCtrs);
 if (PtsCtrs)    free_fmatrix_1d(PtsCtrs);
 if (Prop)       free_fmatrix_1d(Prop);
 if (VDist)      free_fmatrix_1d(VDist);
}



//----------------
//  FUSION -------
//----------------
//----------------------------------------------------------
// Paint        
//----------------------------------------------------------
void Paint(float** mat,float** mattmp,int row,int col,int lgth,int wdth,int lab)
{
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
// FuseRegions
//
// < Petite regions inferieur taille sz fusione avec
//    classe majoritaire l entourant >
//              
//----------------------------------------------------------
void FuseSmallRegions(float** mat,int lgth,int wdth,int sz)
{
 int i,j,k,l,m,n,t;
 float maxim;
 int  labfuse;
 int cpt;
 int initsize;
 int size_min;
 int nbregion_prec;
 int nbregion_init;

 //Allocation 
 float**  mattmp=fmatrix_allocate_2d(lgth,wdth);
 float*   claslb=fmatrix_allocate_1d(256);

 //Init 
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) mat[i][j]=(int)mat[i][j];

 //Init
 size_min=0;
 nbregion_prec=0;
 cpt=10;
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
                    mat[k][l]=labfuse; }

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
      fflush(stdout);
   }//=====================================================================

 
 //Liberation 
 if (mattmp)  free_fmatrix_2d(mattmp); 
 if (claslb)  free_fmatrix_1d(claslb); 
}

//----------------------------------------------------------*/
// FuseRegionBhattaharya different Space Color
//    Distance MIN
//                           
//----------------------------------------------------------*/
int FuseRegionBhattaharyaMIN(float** ImgSeg,int*** Img,int lgth,int wdth,float Thrb,int div,float Thrc) 
{
 int i,j,k,l,m,n;
 int  coul;
 int nbregions;
 float dist_min;
 float label;
 int nb;
 float VerifHisto[256];
 int FuseRegion;
 float Thrb_init;

 //Init
 printf("\n  SIZE>[%d-%d]",lgth,wdth);
 FuseRegion=0;
 Thrb_init=Thrb;

 //Allocation 
 int***   ImgcTmp=imatrix_allocate_3d(3,lgth,wdth);
 float**  ImgTmp=fmatrix_allocate_2d(lgth,wdth);
 float*   claslb=fmatrix_allocate_1d(256);
 float*   Histo=fmatrix_allocate_1d(div*div*div);
 float*   DistSC=fmatrix_allocate_1d(256);

 //Check
 nb=0;
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)
   if ((ImgSeg[i][j]<0)||(ImgSeg[i][j]>255)) nb++;

 if (nb) printf("    >>[%d] Pixels Problematiques",nb);

 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) ImgSeg[i][j]=(int)ImgSeg[i][j];

 //Init 
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) ImgTmp[i][j]=0;
 printf("\n\n   Fusion RegionBhattaharya MULTI SPACE-COLOR -DISTMIN-");
 printf("\n   ----------------------------------------------------");
 printf("\n   >> NbDiv=[%d]",div);
 printf("\n   >> Thrb=[%.3f]",Thrb);
 printf("\n");

 //Boucle
 //-------
   for(i=0;i<lgth;i++) for(j=0;j<wdth;j++)
     if (ImgTmp[i][j]==0)
        {
	 //PourChaqueRegion   
	 coul=(int)ImgSeg[i][j];  
         
         for(k=0;k<lgth;k++) for(l=0;l<wdth;l++) 
	   if ((int)ImgSeg[k][l]==coul) ImgTmp[k][l]=-1.0; 

         //Info
         printf("\n Region[%d]",coul); fflush(stdout);    

         //HistoRegionsVoisines ?
         for(k=0;k<256;k++)  claslb[k]=0;
         for(k=0;k<lgth;k++) for(l=0;l<wdth;l++) if (ImgSeg[k][l]==coul)   
	      {
                for(m=-1;m<2;m++) for(n=-1;n<2;n++) 
		  if (((k+m)>0)&&((k+m)<lgth)&&((l+n)>0)&&((l+n)<wdth))
		     if (ImgSeg[k+m][l+n]<256) claslb[(int)ImgSeg[k+m][l+n]]++;             
	      } 
          claslb[(int)ImgSeg[i][j]]=0; 

         //NumberRegionsVoisines
         for(nbregions=0,n=0;n<256;n++) if  (claslb[n]) nbregions++;
         printf(">%d::",nbregions);  
         for(m=0,n=0;n<256;n++)  if (claslb[n]) { claslb[m]=n; m++; }
         for(k=0;k<nbregions;k++) printf("%.0f-",claslb[k]);
         fflush(stdout);

         //------------------------
         // Appel DistMinHist
         //------------------------
         dist_min=DistMinHist_(Img,lgth,wdth,ImgSeg,coul,claslb,nbregions,div,&label);
         //------------------------       
	         
	 //UneSeuleRegionVoisine ?
         if (nbregions==1) { printf("*"); Thrb=Thrb_init*Thrc; }
         else Thrb=Thrb_init;
         
         //Fusion---------------------
         if (dist_min<Thrb)
	    {
	     printf("\n    Fusion > [Rnb=%.0f]->[Rnb=%.0f] (d=%.3f)(T=%.3f)",ImgSeg[i][j],claslb[(int)label],dist_min,Thrb);
             fflush(stdout);
             FuseRegion++;

             for(k=0;k<lgth;k++) for(l=0;l<wdth;l++) 
		if (ImgSeg[k][l]==coul)  ImgSeg[k][l]=claslb[(int)label];  
	    }//-----------------------

	 //Verif------------------
         for(k=0;k<256;k++) VerifHisto[k]=0.0;
         for(k=0;k<lgth;k++) for(l=0;l<wdth;l++)  
	    if ((ImgSeg[k][l]>=0)&&(ImgSeg[k][l]<256)) VerifHisto[(int)ImgSeg[k][l]]++;   
         //-----------------------
	}

 //Liberation 
 if (ImgTmp)   free_fmatrix_2d(ImgTmp); 
 if (ImgcTmp)  free_imatrix_3d(ImgcTmp,3); 
 if (claslb)   free_fmatrix_1d(claslb); 
 if (Histo)    free_fmatrix_1d(Histo);
 if (DistSC)   free_fmatrix_1d(DistSC);

 //Retour
 return FuseRegion; 
}

//----------------------------------------------------------*/
//  DistMinHist                          
//----------------------------------------------------------*/
float DistMinHist_(int*** Img,int lgth,int wdth,float** ImgSeg,int coul,float* claslb,int nbregions,int div,float* label) 
{
 int i,j,k,l,t,n;
 int posr,posc;
 int nb,flag,lab;
 float tmp;
 float dist,dist_min;

 //Init
 const int SzW=7;
 float divis=(256.0/(float)div);

 //Compte
 nb=0;
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) if ((int)ImgSeg[i][j]==coul) nb++; 
 printf("[Size=%d]",nb);
 fflush(stdout);

 //Allocation Memoire
 int****  ImgCoul=imatrix_allocate_4d(6,3,lgth,wdth);
 float**  HistR=fmatrix_allocate_2d(6,div*div*div);
 float**  HistN=fmatrix_allocate_2d(6,div*div*div);

 //ColorBase
 for(k=0;k<6;k++) ColorBase(ImgCoul[k],Img,lgth,wdth,k);

 //RemplissageRegion
 //-----------------
 for(t=0;t<6;t++) for(k=0;k<(div*div*div);k++) HistR[t][k]=0.0; 
 for(t=0;t<6;t++)
    {
     for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) if ((int)ImgSeg[i][j]==coul)
        {
         //>RecupHisto 
         lab=((int)(ImgCoul[t][0][i][j]/divis))*(div*div);
         lab+=((int)(ImgCoul[t][1][i][j]/divis))*(div);  
         lab+=((int)(ImgCoul[t][2][i][j]/divis)); 
         HistR[t][(int)(lab)]++; 
        }

     tmp=0.0; for(k=0;k<(div*div*div);k++) tmp+=HistR[t][k];
     if (tmp) for(k=0;k<(div*div*div);k++) HistR[t][k]/=tmp;            
    }


 //Calcul_Distance_Min
 //-------------------
 dist_min=1000000.0;
 for(n=0;n<nbregions;n++) 
 for(i=0;i<lgth;i++) for(j=0;j<wdth;j++) if ((int)ImgSeg[i][j]==(int)claslb[n])
         {
	  //>TotalementInclue
          flag=0;
          for(k=0;k<SzW;k++) for(l=0;l<SzW;l++) 
             { posr=i-(SzW/2)+k;
               posc=j-(SzW/2)+l;
               if (posr<0)           posr+=lgth;
               if (posr>(lgth-1))    posr-=lgth;  
               if (posc<0)           posc+=wdth;
               if (posc>(wdth-1))    posc-=wdth;
               if (ImgSeg[posr][posc]!=(int)claslb[n]) flag=1; }

	  if (flag) continue; 

          //>RecupHistos&Distance
          if (!flag)
	     { for(t=0;t<6;t++) for(k=0;k<(div*div*div);k++) HistN[t][k]=0.0;

               for(t=0;t<6;t++)
		  {
                   for(k=0;k<SzW;k++) for(l=0;l<SzW;l++) 
                      { posr=i-(SzW/2)+k;
                        posc=j-(SzW/2)+l;
                        if (posr<0)           posr+=lgth;
                        if (posr>(lgth-1))    posr-=lgth;  
                        if (posc<0)           posc+=wdth;
                        if (posc>(wdth-1))    posc-=wdth;        
                        lab=((int)(ImgCoul[t][0][posr][posc]/divis))*(div*div);
                        lab+=((int)(ImgCoul[t][1][posr][posc]/divis))*(div);  
                        lab+=((int)(ImgCoul[t][2][posr][posc]/divis));                  
                        HistN[t][(int)(lab)]++; }

                  tmp=0.0; for(k=0;k<(div*div*div);k++) tmp+=HistN[t][k];
                  if (tmp) for(k=0;k<(div*div*div);k++) HistN[t][k]/=tmp;
		  }
        
              dist=0.0;
              for(t=0;t<6;t++) 
	         { tmp=0.0;
                   for(k=0;k<(div*div*div);k++) tmp+=sqrt(HistR[t][k]*HistN[t][k]);
                   dist+=sqrt(1-tmp); }

              dist/=6;
              if (dist<dist_min) { dist_min=dist; (*label)=n; }
	     }
	 }

 //LiberationMemoire
 if (ImgCoul) free_imatrix_4d(ImgCoul,6,3);
 if (HistR)   free_fmatrix_2d(HistR);
 if (HistN)   free_fmatrix_2d(HistN);

 printf("<%f::%.0f>",dist_min,(*label)); fflush(stdout);

 //Retour
 if (dist_min>1.0) return 1.0;
 else return dist_min;
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
