#include "utils.h"
#include <algorithm>

char name_seg_img[100];
float***  seg_img;
char name_orig_img[100];
float***  orig_img;


void read_color_image(char* fname, float*** img, int len, int wid)
{
  unsigned char *tmp; 
  inputimgpm(fname, &tmp,&len,&wid);
  for (int p=0; p<len*wid*3; p++)
    img[p%3][(p/3)/wid][(p/3)%wid] = float(tmp[p]);
  free(tmp);
}

void write_color_image(char* fname, float*** img, int len, int wid)
{
  unsigned char* tmp = (unsigned char *) malloc(len*wid*3*sizeof(unsigned char));
  for (int p=0; p<len*wid*3; p++)
    tmp[p] = char(img[p%3][(p/3)/wid][(p/3)%wid]);
  outputimgpm(fname,tmp,len,wid,3);
  free(tmp);
}

void read_gray_image(char* fname, float** img, int len, int wid)
{
  unsigned char *tmp; 
  inputimgpm(fname, &tmp,&len,&wid);
  for (int p=0; p<len*wid; p++)
    img[p/wid][p%wid] = float(tmp[p]);
  free(tmp);
}

void write_gray_image(char* fname, float** img, int len, int wid)
{
  unsigned char* tmp = (unsigned char *) malloc(len*wid*sizeof(unsigned char));
  for (int p=0; p<len*wid; p++)
    tmp[p] = char(img[p/wid][p%wid]);
  outputimgpm(fname,tmp,len,wid,1);
  free(tmp);
}


void read_arguments(int argc,char** argv)
{
    if (argc!=3) { 
      printf("\n Usage   : [Orig Img. in ppm format] [Segmentation Img. in ppm format]"); 
       
      printf("\n\n\n\n\n");
      exit(-1); 
  }

  strcpy(name_orig_img,argv[1]);
  strcpy(name_seg_img,argv[2]);
     
}

int main(int argc,char** argv)
{
  // set variables, allocate memory  
  int length, width;
  long long num_pix;
  
  read_arguments(argc,argv);
  GetLengthWidth(name_orig_img, &length, &width);
  num_pix = length*width;
  orig_img = fmatrix_allocate_3d(3,length,width);
  seg_img = fmatrix_allocate_3d(3,length,width);
  
  // load images
  read_color_image(name_orig_img, orig_img, length, width); 
  read_color_image(name_seg_img, seg_img, length, width);

	///////////////////////////////
	// compute image complexity
	////////////////////////////////


	float image_complexity =	Complexity(orig_img, length, width, 5, 7);
 
  ////////////////////////
  // mark invalid gt pixels (since gt creation has 1-pixel wide line of original pixel value between image segments)
  /////////////////////////

  // usually neighboring pixels on this line dont have the same color -> use that information to identify these boundary pixels
  bool valid_pixels[num_pix]; 
  for (int p=0; p<num_pix;p++) 
  {
    int px = p%width; int py = p/width; 
    valid_pixels[p] = false;
    if (px>0 && py>0) // upper left
      if ((seg_img[0][py][px] == seg_img[0][py-1][px-1]) && (seg_img[1][py][px] == seg_img[1][py-1][px-1]) && (seg_img[2][py][px] == seg_img[2][py-1][px-1]))
      { 
        valid_pixels[p] = true;
        continue;
      }
    if (py>0) // upper
      if ((seg_img[0][py][px] == seg_img[0][py-1][px]) && (seg_img[1][py][px] == seg_img[1][py-1][px]) && (seg_img[2][py][px] == seg_img[2][py-1][px]))
      { 
        valid_pixels[p] = true;
        continue;
      }
    if (py>0 && px<width-1) // upper right
      if ((seg_img[0][py][px] == seg_img[0][py-1][px+1]) && (seg_img[1][py][px] == seg_img[1][py-1][px+1]) && (seg_img[2][py][px] == seg_img[2][py-1][px+1]))
      { 
        valid_pixels[p] = true;
        continue;
      }
    if (px<width-1) // right
      if ((seg_img[0][py][px] == seg_img[0][py][px+1]) && (seg_img[1][py][px] == seg_img[1][py][px+1]) && (seg_img[2][py][px] == seg_img[2][py][px+1]))
      { 
        valid_pixels[p] = true;
        continue;
      }
    if (py<length-1 && px<width-1) // lower right
      if ((seg_img[0][py][px] == seg_img[0][py+1][px+1]) && (seg_img[1][py][px] == seg_img[1][py+1][px+1]) && (seg_img[2][py][px] == seg_img[2][py+1][px+1]))
      { 
        valid_pixels[p] = true;
        continue;
      }
    if (py<length-1) // lower
      if ((seg_img[0][py][px] == seg_img[0][py+1][px]) && (seg_img[1][py][px] == seg_img[1][py+1][px]) && (seg_img[2][py][px] == seg_img[2][py+1][px]))
      { 
        valid_pixels[p] = true;
        continue;
      }
    if (py<length-1 && px>0) // lower left
      if ((seg_img[0][py][px] == seg_img[0][py+1][px-1]) && (seg_img[1][py][px] == seg_img[1][py+1][px-1]) && (seg_img[2][py][px] == seg_img[2][py+1][px-1]))
      { 
        valid_pixels[p] = true;
        continue;
      }
    if (px>0) // left
      if ((seg_img[0][py][px] == seg_img[0][py][px-1]) && (seg_img[1][py][px] == seg_img[1][py][px-1]) && (seg_img[2][py][px] == seg_img[2][py][px-1]))
      { 
        valid_pixels[p] = true;
        continue;
      }
  } //for

  // save validity img for testing
  
  int num_seg_invalid_px = 0;
  float**  validity_img = fmatrix_allocate_2d(length,width); 
  for (int y =0; y<length; y++)
    for(int x=0; x<width; x++)
      if (valid_pixels[y*width+x])
        validity_img[y][x] = 255;
      else
      {
        validity_img[y][x] = 0;
        num_seg_invalid_px ++;
      }
  char validity_name [100];
  strcpy(validity_name, name_seg_img);
  strcat(validity_name, "_validity_img.pgm");
  //write_gray_image(validity_name, validity_img, length, width);
  

  /////////////////////////////////////////////////////
  // Compute Superpixel quality (as 'complexity' in MDSCCT)
  /////////////////////////////////////////////////////

  int sp_labels[500]; // change if more sp are expected in the segmap
  int num_sps = 0; // stores number of superpixels
  int win_size = 5; 
  int num_bpc = 5;
  int num_bins = num_bpc*num_bpc*num_bpc;
  int min_valid_neighbors = (win_size*win_size)/2;
  float *** colhist_img =  fmatrix_allocate_3d(num_bins,length,width);
  
  // collect sp labels
  for (int y =0; y<length; y++)
    for (int x=0; x<width; x++)
    {
      if (validity_img[y][x] == 0)
        continue;
      int label = int(seg_img[0][y][x])*256*256 + int(seg_img[1][y][x])*256 + int(seg_img[2][y][x]);
      int i=0;
      for (int j=0;j<num_sps; j++, i++)
        if (sp_labels[j] == label){
          break;}
      if (i == num_sps)
      {
        sp_labels[i] = label;
        num_sps++;
      } 
    }

    
  // allocate memory for avg_colhist, initialize
  float** avg_colhists =  fmatrix_allocate_2d(num_sps, num_bins);
  for (int i=0; i<num_sps; i++)
    for (int b=0; b<num_bins; b++)
      avg_colhists[i][b] = 0;
  int* num_sp_pixels = imatrix_allocate_1d(num_sps); // counts number of valid col hists for each sp
  float** valid_colhists = fmatrix_allocate_2d(length, width); 

  // create colhist img (each pixel is assigned a col hist)
  for (int y=0; y<length; y++)
    for (int x=0; x<width; x++)
    {
      for (int b=0; b<num_bins; b++)
        colhist_img[b][y][x] = 0;
      if (validity_img[y][x] == 0)
        continue;
      int label = int(seg_img[0][y][x])*256*256 + int(seg_img[1][y][x])*256 + int(seg_img[2][y][x]);
      int num_valid_neighbors = 0;
      for (int dy=-win_size/2; dy<win_size/2+1; dy++)
      {
        if (y+dy<0 || y+dy>length-1)
          continue;
        for (int dx=-win_size/2; dx<win_size/2+1; dx++)
        {
          if (x+dx<0 || x+dx>width-1)
            continue;
          if ((seg_img[0][y][x] != seg_img[0][y+dy][x+dx]) || (seg_img[1][y][x] != seg_img[1][y+dy][x+dx]) || (seg_img[2][y][x] != seg_img[2][y+dy][x+dx]))
            continue;
          num_valid_neighbors++;
          int rbin, gbin, bbin;
          rbin = std::min(int(orig_img[0][y+dy][x+dx]/(256/num_bpc)), num_bpc-1);
          gbin = std::min(int(orig_img[1][y+dy][x+dx]/(256/num_bpc)), num_bpc-1);
          bbin = std::min(int(orig_img[2][y+dy][x+dx]/(256/num_bpc)), num_bpc-1);
          colhist_img[rbin*num_bpc*num_bpc + gbin*num_bpc + bbin][y][x] ++;
          
        } //for dx
      } //for dy
      if (num_valid_neighbors < min_valid_neighbors) // discard col hist again
      {
        valid_colhists[y][x] = 0;
        for (int b=0; b<num_bins; b++)
          colhist_img[b][y][x] = 0;
        continue;
      }
      valid_colhists[y][x] = 255;
      for (int b=0; b<num_bins; b++)
        colhist_img[b][y][x] /= num_valid_neighbors; // normalize histogram
      // add colhist to avg_colhist
      int pos;
      for (pos=0; pos<num_sps; pos++)
        if (sp_labels[pos] == label)
          break;
      for (int b=0; b<num_bins; b++)
        avg_colhists[pos][b] += colhist_img[b][y][x];
      num_sp_pixels[pos]++;  
    } // for x

  
  /*
  // test print
  printf("\n\nnum_sp_pixels: \n");
  for (int i=0; i<num_sps; i++)
    printf("%d \n",num_sp_pixels[i]);

  printf("colhist(0,0): \n");
  float s=0;
  for (int i=0; i<num_bins; i++) {
    if (colhist_img[i][0][0] != 0) 
      printf("r: %d  g: %d  b: %d value: %.3f\n",i/(num_bpc*num_bpc), (i/num_bpc)%num_bpc, i%num_bpc, colhist_img[i][0][0]);
    s += colhist_img[i][0][0];
    if (i==num_bins-1)
      printf("sum of col hist(0,0): %.3f\n", s);
  }
  
  s=0;
  printf("colhist(10,10): \n");
  for (int i=0; i<num_bins; i++) {
    if (colhist_img[i][10][10] != 0) 
      printf("r: %d  g: %d  b: %d value: %.3f\n",i/(num_bpc*num_bpc), (i/num_bpc)%num_bpc, i%num_bpc, colhist_img[i][10][10]);
    s += colhist_img[i][10][10];
    if (i==num_bins-1)
      printf("sum of col hist(10,10): %.3f\n", s);
  }

  */

  for (int i=0; i<num_sps; i++)
    for (int b=0; b<num_bins; b++)
      avg_colhists[i][b] /= num_sp_pixels[i];


  /*
  // test print 
  for (int sp=0; sp<num_sps; sp++) {
    s=0;
    printf("avg. colhist #%d: \n", sp);
    for (int i=0; i<num_bins; i++) {
      if (avg_colhists[sp][i] != 0) 
        printf("r: %d  g: %d  b: %d value: %.3f\n",i/(num_bpc*num_bpc), (i/num_bpc)%num_bpc, i%num_bpc, avg_colhists[sp][i]);
      s += avg_colhists[sp][i];
      if (i==num_bins-1)
        printf("sum of avg. col hist #%d: %.3f\n", sp, s);
    }
  }

  */
  
  // allocate memory for deviations from avg_colhist for each sp and initialize
  float** std_colhists =  fmatrix_allocate_2d(num_sps, num_bins);
  for (int i=0; i<num_sps; i++)
    for (int b=0; b<num_bins; b++)
      std_colhists[i][b] = 0;

  
  // compute deviations for each col hist from avg
  float*** deviations_img =  fmatrix_allocate_3d(3, length, width);

  for (int y=0; y<length; y++)
    for (int x=0; x<width; x++) 
    {
      deviations_img[0][y][x] = 255; deviations_img[1][y][x] = 0; deviations_img[2][y][x] = 0;
      if (validity_img[y][x] == 0) 
        continue;
      if (valid_colhists[y][x] == 255) 
      {
        int i;
        int label = seg_img[0][y][x]*256*256 + seg_img[1][y][x]*256 + seg_img[2][y][x];
        for (i=0; i<num_sps; i++)
          if (label==sp_labels[i])
            break;
        float dev = 0;
        for (int b=0; b<num_bins; b++)
        {
          std_colhists[i][b] += fabs(colhist_img[b][y][x] - avg_colhists[i][b]); // between 0 and 2
          dev += fabs(colhist_img[b][y][x] - avg_colhists[i][b]); // between 0 and 2
        }
        deviations_img[0][y][x] = deviations_img[1][y][x] = deviations_img[2][y][x] = dev * (255.0/2.0);
      }
    } // for x

  // compute sp quality  

  int npix = 0; 
  float total_dev = 0;
    
  for (int i=0; i<num_sps; i++)
  {
    for (int b=0; b<num_bins; b++)
      total_dev += std_colhists[i][b];
    npix += num_sp_pixels[i];
  }

  total_dev /= 2*npix;
  char dev_name [100];
  strcpy(dev_name, name_seg_img);
  strcat(dev_name, "_deviations.ppm");
  write_color_image(dev_name, deviations_img, length, width);

  ////////////////////
  // printout results
  ////////////////////

	printf("\n\nimage name: %s \n\n",name_orig_img);
  printf("width: %d \n",width);
  printf("length: %d \n",length);
	printf("image complexity: %.3f\n", image_complexity);
  printf("# invalid px in seg_img: %d \n",num_seg_invalid_px);
  printf("sp labels : %d\n\n", num_sps);
  printf("sp quality win size : %d\n", win_size);
  printf("sp quality num_bpc  : %d\n", num_bpc);
  printf("avg. sp quality: %.3f \n", total_dev);

  /*
  for (int i=0; i<num_sps; i++)
    printf("%d %x %x %x \n", sp_labels[i], sp_labels[i]/(256*256), (sp_labels[i]/256)%256, sp_labels[i]%256);
  */

  free_fmatrix_3d(orig_img, 3);
  free_fmatrix_3d(seg_img, 3);
  free_fmatrix_3d(colhist_img, num_bins);
  free_fmatrix_2d(validity_img);
  free_fmatrix_2d(avg_colhists);
  free_fmatrix_2d(std_colhists);
  free_fmatrix_2d(valid_colhists);
  free_fmatrix_3d(deviations_img, 3);
  free_imatrix_1d(num_sp_pixels);
  std::cout << "\n";
}

//   CopyMat(Img,Imag1,length,width,TROIS);

