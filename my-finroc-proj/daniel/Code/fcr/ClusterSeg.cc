//------------------------------------------------------
// module  : ClusterSeg.cc
// author  : Mignotte Max
// date    : 
// version : 1.0
// language: C++
// note    :
//------------------------------------------------------
//  

//------------------------------------------------
// INCLDED FILES ---------------------------------
//------------------------------------------------
#include "ClusterSeg.h"

//------------------------------------------------
// DEFINITIONS -----------------------------------
//------------------------------------------------
#define NAME_IMG      "Image"
#define EXTENSION     "NoN"

//------------------------------------------------
// CONSTANTES ------------------------------------
//------------------------------------------------
const int   METHOD=0;
const int   SEED=0;

const int   NBCL1=13;
const int   NBCL2=6;

const int   NBINS=5;
const int   SIZE1=7;
const int   SIZE2=7;
const int   FLGB=1;

const float DISTBHATTA=0.135;
const int   SIZE_MIN=0;

const int ZOOM=1;
const int QUIT=0;
const int SAVE=0;
const int VISU=0;


//------------------------------------------------
// VARIABLES GLOBALES ----------------------------
//------------------------------------------------
char Name_img[NCHAR];
char Extension[NCHAR];

int   method;
int   seed;

int   nbcl1;
int   nbcl2;
int   size_min;
float distbhatta;

int   nbbins;
int   size1;
int   size2;

int   zoom;
int   flag_save;
int   flag_quit;
int   flag_visu;

//-------------------------
//--- Windows -------------
//-------------------------
#include <X11/X.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xos.h>
#include <X11/Xatom.h>
#include <X11/cursorfont.h>

Display   *display;
int	  screen_num;
int 	  depth;
Window	  root;
Visual*	  visual;
GC	  gc;

/************************************************************************/
/* OPEN_DISPLAY()							*/
/************************************************************************/
int open_display()
{
  if ((display=XOpenDisplay(NULL))==NULL)
   { printf("Connection impossible\n");
     return(-1); }

  else
   { screen_num=DefaultScreen(display);
     visual=DefaultVisual(display,screen_num);
     depth=DefaultDepth(display,screen_num);
     root=RootWindow(display,screen_num);
     return 0; }
}

/************************************************************************/
/* FABRIQUE_WINDOW()							*/
/* Cette fonction crée une fenetre X et l'affiche à l'écran.	        */
/************************************************************************/
Window fabrique_window(char *nom_fen,int x,int y,int width,int height,int zoom)
{
  Window                 win;
  XSizeHints      size_hints;
  XWMHints          wm_hints;
  XClassHint     class_hints;
  XTextProperty  windowName, iconName;

  char *name=nom_fen;

  if(zoom<0) { width/=-zoom; height/=-zoom; }
  if(zoom>0) { width*=zoom;  height*=zoom;  }

  win=XCreateSimpleWindow(display,root,x,y,width,height,1,0,255);

  size_hints.flags=PPosition|PSize|PMinSize;
  size_hints.min_width=width;
  size_hints.min_height=height;

  XStringListToTextProperty(&name,1,&windowName);
  XStringListToTextProperty(&name,1,&iconName);
  wm_hints.initial_state=NormalState;
  wm_hints.input=True;
  wm_hints.flags=StateHint|InputHint;
  class_hints.res_name=nom_fen;
  class_hints.res_class=nom_fen;

  XSetWMProperties(display,win,&windowName,&iconName,
                   NULL,0,&size_hints,&wm_hints,&class_hints);

  gc=XCreateGC(display,win,0,NULL);

  XSelectInput(display,win,ExposureMask|KeyPressMask|ButtonPressMask| 
               ButtonReleaseMask|ButtonMotionMask|PointerMotionHintMask| 
               StructureNotifyMask);

  XMapWindow(display,win);
  return(win);
}

/****************************************************************************/
/* CREE_XIMAGE2()							    */
/* Crée une XImage à partir d'un tableau de float                           */
/* L'image peut subir un zoom.						    */
/****************************************************************************/
XImage* cree_Ximage2(float** mat,int z,int length,int width)
{
  int lgth,wdth,lig,col,zoom_col,zoom_lig;
  float somme;
  unsigned char	 pix;
  unsigned char* dat;
  XImage* imageX;

  /*Zoom positiv*/
  /*------------*/
  if (z>0)
  {
   lgth=length*z;
   wdth=width*z;

   dat=(unsigned char*)malloc(lgth*(wdth*4)*sizeof(unsigned char));
   if (dat==NULL)
      { printf("Impossible d'allouer de la memoire.");
        exit(-1); }

  for(lig=0;lig<lgth;lig=lig+z) for(col=0;col<wdth;col=col+z)
   { 
    pix=(unsigned char)mat[lig/z][col/z];
    for(zoom_lig=0;zoom_lig<z;zoom_lig++) for(zoom_col=0;zoom_col<z;zoom_col++)
      { 
       dat[((lig+zoom_lig)*wdth*4)+((4*(col+zoom_col))+0)]=pix;
       dat[((lig+zoom_lig)*wdth*4)+((4*(col+zoom_col))+1)]=pix;
       dat[((lig+zoom_lig)*wdth*4)+((4*(col+zoom_col))+2)]=pix;
       dat[((lig+zoom_lig)*wdth*4)+((4*(col+zoom_col))+3)]=pix; 
       }
    }
  } /*--------------------------------------------------------*/

  /*Zoom negatifv*/
  /*------------*/
  else
  {
   z=-z;
   lgth=(length/z);
   wdth=(width/z);

   dat=(unsigned char*)malloc(lgth*(wdth*4)*sizeof(unsigned char));
   if (dat==NULL)
      { printf("Impossible d'allouer de la memoire.");
        exit(-1); }

  for(lig=0;lig<(lgth*z);lig=lig+z) for(col=0;col<(wdth*z);col=col+z)
   {  
    somme=0.0;
    for(zoom_lig=0;zoom_lig<z;zoom_lig++) for(zoom_col=0;zoom_col<z;zoom_col++)
     somme+=mat[lig+zoom_lig][col+zoom_col];
           
     somme/=(z*z);    
     dat[((lig/z)*wdth*4)+((4*(col/z))+0)]=(unsigned char)somme;
     dat[((lig/z)*wdth*4)+((4*(col/z))+1)]=(unsigned char)somme;
     dat[((lig/z)*wdth*4)+((4*(col/z))+2)]=(unsigned char)somme;
     dat[((lig/z)*wdth*4)+((4*(col/z))+3)]=(unsigned char)somme; 
   }
  } /*--------------------------------------------------------*/

  imageX=XCreateImage(display,visual,depth,ZPixmap,0,(char*)dat,wdth,lgth,16,wdth*4);
  return (imageX);
}

/****************************************************************************/
/* CREE_XIMAGE4()							    */
/* Crée une XImage à partir d'un tableau d'entier                           */
/* L'image peut subir un zoom.						    */
/****************************************************************************/
XImage* cree_Ximage4(int** mat,int z,int length,int width)
{
  int lgth,wdth,lig,col,zoom_col,zoom_lig;
  float somme;
  unsigned char	 pix;
  unsigned char* dat;
  XImage* imageX;

  /*Zoom positif*/
  /*------------*/
  if (z>0)
  {
   lgth=length*z;
   wdth=width*z;

   dat=(unsigned char*)malloc(lgth*(wdth*4)*sizeof(unsigned char));
   if (dat==NULL)
      { printf("Impossible d'allouer de la memoire.");
        exit(-1); }

  for(lig=0;lig<lgth;lig=lig+z) for(col=0;col<wdth;col=col+z)
   { 
    pix=(unsigned char)mat[lig/z][col/z];
    for(zoom_lig=0;zoom_lig<z;zoom_lig++) for(zoom_col=0;zoom_col<z;zoom_col++)
      { 
       dat[((lig+zoom_lig)*wdth*4)+((4*(col+zoom_col))+0)]=pix;
       dat[((lig+zoom_lig)*wdth*4)+((4*(col+zoom_col))+1)]=pix;
       dat[((lig+zoom_lig)*wdth*4)+((4*(col+zoom_col))+2)]=pix;
       dat[((lig+zoom_lig)*wdth*4)+((4*(col+zoom_col))+3)]=pix; 
       }
    }
  } /*--------------------------------------------------------*/

  /*Zoom negatif*/
  /*------------*/
  else
  {
   z=-z;
   lgth=(length/z);
   wdth=(width/z);

   dat=(unsigned char*)malloc(lgth*(wdth*4)*sizeof(unsigned char));
   if (dat==NULL)
      { printf("Impossible d'allouer de la memoire.");
        exit(-1); }

  for(lig=0;lig<(lgth*z);lig=lig+z) for(col=0;col<(wdth*z);col=col+z)
   {  
    somme=0.0;
    for(zoom_lig=0;zoom_lig<z;zoom_lig++) for(zoom_col=0;zoom_col<z;zoom_col++)
     somme+=mat[lig+zoom_lig][col+zoom_col];
           
     somme/=(z*z);    
     dat[((lig/z)*wdth*4)+((4*(col/z))+0)]=(unsigned char)somme;
     dat[((lig/z)*wdth*4)+((4*(col/z))+1)]=(unsigned char)somme;
     dat[((lig/z)*wdth*4)+((4*(col/z))+2)]=(unsigned char)somme;
     dat[((lig/z)*wdth*4)+((4*(col/z))+3)]=(unsigned char)somme; 
   }
  } /*--------------------------------------------------------*/

  imageX=XCreateImage(display,visual,depth,ZPixmap,0,(char*)dat,wdth,lgth,16,wdth*4);
  return (imageX);
}

/****************************************************************************/
/* CREE_XIMAGECOUL()							    */
/* Crée une XImage à partir d'un tableau 3 d d'entier                       */
/* L'image peut subir un zoom.						    */
/****************************************************************************/
XImage* cree_XimageCoul(int*** matRVB,int z,int length,int width)
{
  int i;
  int lgth,wdth,lig,col,zoom_col,zoom_lig;
  float somme;
  float sum[3];
  unsigned char	 pixR,pixV,pixB,pixN;
  unsigned char* dat;
  XImage* imageX;

  /*Zoom positif*/
  /*------------*/
  if (z>0)
  {
   lgth=length*z;
   wdth=width*z;

   dat=(unsigned char*)malloc(lgth*(wdth*4)*sizeof(unsigned char));
   if (dat==NULL)
      { printf("Impossible d'allouer de la memoire.");
        exit(-1); }

  for(lig=0;lig<lgth;lig=lig+z) for(col=0;col<wdth;col=col+z)
   { 
    pixR=(unsigned char)matRVB[0][lig/z][col/z];
    pixV=(unsigned char)matRVB[1][lig/z][col/z];
    pixB=(unsigned char)matRVB[2][lig/z][col/z];
    somme=(1.0/3.0)*(pixR+pixV+pixB);
    pixN=(unsigned char)somme;

    for(zoom_lig=0;zoom_lig<z;zoom_lig++) for(zoom_col=0;zoom_col<z;zoom_col++)
      { 
       dat[((lig+zoom_lig)*wdth*4)+((4*(col+zoom_col))+0)]=pixB; 
       dat[((lig+zoom_lig)*wdth*4)+((4*(col+zoom_col))+1)]=pixV; 
       dat[((lig+zoom_lig)*wdth*4)+((4*(col+zoom_col))+2)]=pixR; 
       dat[((lig+zoom_lig)*wdth*4)+((4*(col+zoom_col))+3)]=0; 
       }
    }
  } /*--------------------------------------------------------*/

  /*Zoom negatif*/
  /*------------*/
  else
  {
   z=-z;
   lgth=(length/z);
   wdth=(width/z);

   dat=(unsigned char*)malloc(lgth*(wdth*4)*sizeof(unsigned char));
   if (dat==NULL)
      { printf("Impossible d'allouer de la memoire.");
        exit(-1); }

  for(lig=0;lig<(lgth*z);lig=lig+z) for(col=0;col<(wdth*z);col=col+z)
   {  
    sum[0]=sum[1]=sum[2]=0.0;
    
    for(i=0;i<3;i++)
    for(zoom_lig=0;zoom_lig<z;zoom_lig++) for(zoom_col=0;zoom_col<z;zoom_col++)
     sum[i]+=matRVB[i][lig+zoom_lig][col+zoom_col];
       
    for(i=0;i<3;i++)  sum[i]/=(z*z); 

     dat[((lig/z)*wdth*4)+((4*(col/z))+0)]=(unsigned char)sum[1];
     dat[((lig/z)*wdth*4)+((4*(col/z))+1)]=(unsigned char)sum[1];
     dat[((lig/z)*wdth*4)+((4*(col/z))+2)]=(unsigned char)sum[1];
     dat[((lig/z)*wdth*4)+((4*(col/z))+3)]=(unsigned char)sum[1]; 
   }
  } /*--------------------------------------------------------*/

  imageX=XCreateImage(display,visual,depth,ZPixmap,0,(char*)dat,wdth,lgth,16,wdth*4);
  return (imageX);
}



//---------------------------------------------------------
// Enregistre les arguments de la ligne de commande      
//---------------------------------------------------------
void read_arguments(int argc,char** argv)
{
 int i;

   //Options
  if (argc<2) 
    { printf("\n Usage ClusterSeg [Img. in ppm format]"); 
      printf("\n Options : value by default indicated in ()");
      printf("\n            -----------------------------------------");
      printf("\n            -------------");
      printf("\n            -a NbCl1 NbCl2  [%d][%d]",NBCL1,NBCL2); 
      printf("\n            -t DistanceBhattaFusion  > [%.4f]",DISTBHATTA);
      printf("\n            -------------"); 
      printf("\n            -b Size1 Size2..[%d][%d]",SIZE1,SIZE2);
      printf("\n            -d NbBins.......[%d]",NBINS);  
      printf("\n            -e Seed.........[%d]",SEED);
      printf("\n            -i SizeMin......[%d]",SIZE_MIN);      
      printf("\n            -------------");
      printf("\n            -g Nom Extension (%s)",EXTENSION);
      printf("\n            -------------");
      printf("\n            -z zoom (%d)",ZOOM);
      printf("\n            -v visu (%d)",VISU);
      printf("\n            -q quit (Sans)"); 
      printf("\n            -s save (Sans)"); 
      printf("\n            -------------");
      printf("\n");
      printf("\n  Example:  ClusterSeg 100075_320.ppm -a 13 6 -t 0.135 -v");
      printf("\n\n\n\n\n");
      exit(-1); }

      //Chargement Fichier
      if (argc>1) strcpy(Name_img,argv[1]);
     

   //Boucle 
   for(i=2;i<argc;i++)
     {   
          switch(argv[i][1])
          {
           case 'm': method=atoi(argv[++i]);               break;
           
           case 'a': nbcl1=atoi(argv[++i]);         
                     nbcl2=atoi(argv[++i]);                break;        
           case 't': distbhatta=atof(argv[++i]);           break;

           case 'b': size1=atoi(argv[++i]);         
                     size2=atoi(argv[++i]);                break; 
           case 'd': nbbins=atoi(argv[++i]);               break;
           case 'e': seed=atoi(argv[++i]);                 break;
           case 'i': size_min=atoi(argv[++i]);             break;
           
	   case 'g': strcpy(Extension,argv[++i]);          break;
           
           case 'z': zoom=atoi(argv[++i]);                 break;
	   case 'v': flag_visu=1;                          break;
	   case 'q': flag_quit=1;                          break;
           case 's': flag_save=1;                          break;                   
          }    
     } 
 }

//----------------------------------------------------------
//----------------------------------------------------------
// PROGRAMME PRINCIPAL -------------------------------------
//----------------------------------------------------------
//----------------------------------------------------------
int main(int argc,char** argv)
{
 int   flag;

 //Pour Xwindow
 //------------
 XEvent ev;
 
 Window win_ppicture_tabseg;
 Window win_ppicture_seg;
 Window win_ppicture_img;
 Window win_ppicture_imgc;
 
 XImage *x_ppicture_tabseg; 
 XImage *x_ppicture_seg; 
 XImage *x_ppicture_img;
 XImage *x_ppicture_imgc;

 char nomfen_ppicture_tabseg[NCHAR]; 
 char nomfen_ppicture_seg[NCHAR];
 char nomfen_ppicture_img[NCHAR];
 char nomfen_ppicture_imgc[NCHAR];
 

 //Initialisation
 //--------------
 method=METHOD;
 seed=SEED;

 nbcl1=NBCL1;
 nbcl2=NBCL2;
 size_min=SIZE_MIN;
 distbhatta=DISTBHATTA;

 nbbins=NBINS;
 size1=SIZE1;
 size2=SIZE2;
 
 zoom=ZOOM;
 flag_save=SAVE;
 flag_quit=QUIT;
 flag_visu=VISU;
 flag=0;

 //Lis Arguments
 //-------------
 read_arguments(argc,argv);


 //Affiche options
 //---------------
 printf("\n\n\n");
 printf("\n\n Info: ");
 printf("\n -----");
 printf("\n Ouverture de l'image : <%s>",Name_img); 
 printf("\n -----");  
 printf("\n NbCl1 | NbCl2   [%d][%d]",nbcl1,nbcl2); 
 printf("\n Distance Bhatta [%f]",distbhatta);
 printf("\n -----");
 printf("\n NbBins [%d]",nbbins);
 printf("\n Size1 | Size2   [%d][%d]",size1,size2);
 printf("\n SizeMin [%d]",size_min);
 printf("\n Seed   [%d]",seed);
 printf("\n -----");
 printf("\n Extension [%s]",Extension);
 printf("\n -----");
 printf("\n Zoom: %d",zoom);
 printf("\n Visu: %d",flag_visu); 
 if (flag_save)  printf("\n -> Avec Sauvegarde");
 if (flag_quit)  printf("\n -> Avec Quit");

//-------------------------------
//---- Chargement des Images -------------------------------
//-------------------------------

 int length;
 int width;
 GetLengthWidth(Name_img,&length,&width);
 printf("\n Taille de l'image: [%d - %d]",length,width);;

 int*** MatImg=imatrix_allocate_3d(3,length,width);
 
 load_image_ppm(Name_img,MatImg,length,width);


 //------------------------------------------
 //-------- Algorithme CLUSTER-SEG ----------
 //------------------------------------------
  ClusterSeg gseg(MatImg,length,width);

  //Options 
  gseg.NbCl1=nbcl1;
  gseg.NbCl2=nbcl2; 
  gseg.DistBhatta=distbhatta;

  gseg.NbBins=nbbins;
  gseg.Size1=size1;
  gseg.Size2=size2;

  gseg.SizeMin=size_min;
  gseg.Seed=seed;

  gseg.flag_save=flag_save;
  gseg.flag_visu=flag_visu;

  strcpy(gseg.Extension,Extension);
  strcpy(gseg.Name_Img,Name_img);

  //SEGMENTATION
  gseg.Segmente();

//---------------------------------------------------------------
//---------------- visu sous XWINDOW ----------------------------
//---------------------------------------------------------------
 if (!flag_quit)
 {
 //ouverture session graphique
 if (open_display()<0) printf(" Impossible d'ouvrir une session graphique");

 //commentaire dans les photos affichees
 sprintf(nomfen_ppicture_tabseg,"Images Segmentees : %s",Name_img);
 sprintf(nomfen_ppicture_img,"Image : %s",Name_img); 
 sprintf(nomfen_ppicture_seg,"Image Segmentee Finale %s",Name_img);
 sprintf(nomfen_ppicture_imgc,"Image + Contour : %s",Name_img); 
 
 
 //creation des fenetres
 win_ppicture_tabseg=fabrique_window(nomfen_ppicture_tabseg,10,10,width*gseg.SizeS,length,zoom); 
 win_ppicture_img=fabrique_window(nomfen_ppicture_img,10,10,width,length,zoom);
 win_ppicture_seg=fabrique_window(nomfen_ppicture_seg,10,10,width,length,zoom);
 win_ppicture_imgc=fabrique_window(nomfen_ppicture_imgc,10,10,width,length,zoom);


 //creation des Ximages
 x_ppicture_tabseg=cree_Ximage4(gseg.win_tabseg,zoom,length,width*gseg.SizeS);
 x_ppicture_img=cree_XimageCoul(gseg.mat_img,zoom,length,width);
 x_ppicture_seg=cree_Ximage2(gseg.mat_seg,zoom,length,width);
 x_ppicture_imgc=cree_XimageCoul(gseg.mat_imgc,zoom,length,width);

 printf("\n\n Pour quitter,appuyer sur la barre d'espace");
 fflush(stdout);

 //boucle d'evenements
  for(;;)
     {
      XNextEvent(display,&ev);
       switch(ev.type)
        {
	 case Expose:   

          XPutImage(display,win_ppicture_tabseg,gc,x_ppicture_tabseg,0,0,0,0,
                   x_ppicture_tabseg->width,x_ppicture_tabseg->height);  
      
          XPutImage(display,win_ppicture_img,gc,x_ppicture_img,0,0,0,0,
                   x_ppicture_img->width,x_ppicture_img->height);
          XPutImage(display,win_ppicture_seg,gc,x_ppicture_seg,0,0,0,0,
                   x_ppicture_seg->width,x_ppicture_seg->height);
          XPutImage(display,win_ppicture_imgc,gc,x_ppicture_imgc,0,0,0,0,
                   x_ppicture_imgc->width,x_ppicture_imgc->height); 
         
         break;

         case KeyPress: 

         XDestroyImage(x_ppicture_tabseg);

         XDestroyImage(x_ppicture_img);  
         XDestroyImage(x_ppicture_seg);
         XDestroyImage(x_ppicture_imgc);

         XFreeGC(display,gc);
         XCloseDisplay(display);
         flag=1;
         break;
         }
   if (flag==1) break;
   }
 } 
       
//--------------- fin de la session graphique --------------     
//----------------------------------------------------------

   //retour sans probleme
   //--------------------
   printf("\n C'est fini... \n");
   exit(-1);
   return 0;
 }
 


