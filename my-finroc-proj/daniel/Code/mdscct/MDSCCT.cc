//------------------------------------------------------
// module  : MDSCCT.cc
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
#include "MDSCCT.h"

//------------------------------------------------
// CONSTANTES ------------------------------------
//------------------------------------------------
const int ZOOM=1;
const int QUIT=0;
const int SAVE=0;
const int VISU=0;

//------------------------------------------------
// VARIABLES GLOBALES ----------------------------
//------------------------------------------------
char Name_img[NCHAR];
char Extension[NCHAR];

float RegulSeg;
int Km;

int  zoom;
int  flag_save;
int  flag_quit;
int  flag_visu;

 


//---------------------------------------------------------
//---------------------------------------------------------
// Enregistre les arguments de la ligne de commande
//---------------------------------------------------------
//---------------------------------------------------------
void read_arguments(int argc,char** argv)
{
 int i;

   //Options
  if (argc<2)
    { printf("\n Usage   : %s [Img. in ppm format]  Options",argv[0]);
      printf("\n   [Options : value by default indicated in ()]");
      printf("\n            -----------------------------------------");
      printf("\n            -g Nom Extension");
      printf("\n            -------------");
      printf("\n            -k Kmax (10)");
      printf("\n            -------------");
      printf("\n            -s save (Yes)");
      printf("\n            -v visu (Yes)");
      printf("\n            -q quit (No)");
      printf("\n            -z zoom (No)");
      printf("\n            -------------\n");
      printf("\n Example : %s 100075_320.ppm",argv[0]);
      printf("\n\n\n\n\n");
      exit(-1); }

      //Chargement Fichier
      if (argc>1) strcpy(Name_img,argv[1]);

   //Boucle
   for(i=2;i<argc;i++)
     {
          switch(argv[i][1])
          {
           case 'g': strcpy(Extension,argv[++i]);     break;

           case 'k': Km=atoi(argv[++i]);              break;
           case 't': RegulSeg=atof(argv[++i]);        break;

           case 's': flag_save=1;                     break;
           case 'v': flag_visu=atoi(argv[++i]);       break;
           case 'q': flag_quit=1;                     break;
           case 'z': zoom=atoi(argv[++i]);            break;
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

 int  flag;
 int lgth,wdth;

 
 //Initialisation
 //--------------
 //RegulSeg=REGULSEG;
 Km=KM_;

 zoom=ZOOM;
 flag_save=SAVE;
 flag_quit=QUIT;
 flag_visu=VISU;
 flag=0;

 //Lis Arguments
 //-------------
 read_arguments(argc,argv);

  //>Information
  //------------
  printf("\n\n");
  printf("\n SEGMENTATION MDSCCT [Km=%d]",Km);
  printf("\n");
  fflush(stdout);

 //------------------------------------------
 //-------- Algorithme            -----------
 //------------------------------------------
 GetLengthWidth(Name_img,&lgth,&wdth);

 MD2S* dsplay=new MD2S(lgth,wdth);

  //>Options
  strcpy((*dsplay).Name_Img,Name_img);
  strcpy((*dsplay).Extension,Extension);
  (*dsplay).flag_save=flag_save;
  (*dsplay).flag_visu=flag_visu;
  (*dsplay).PAR_KM=Km;

  //>For Segmentation into Regions For PRI measure
  (*dsplay).MD2S_Segment();

  //>For Segmentation into contour For F measure
  //(*dsplay).MD2S_SegmentCont();



   //retour sans probleme
   //--------------------
   printf("\n End... \n");
   return 0;
 }



