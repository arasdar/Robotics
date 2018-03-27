#include "utils.h"

char name_gt_img[100];
char name_seg_img[100];
float***  gt_img;
float***  seg_img;

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
      printf("\n Usage   : [Ground truth Img. in ppm format]  [Segmentation Img. in ppm format]"); 
       
      printf("\n\n\n\n\n");
      exit(-1); 
  }

  strcpy(name_gt_img,argv[1]);
  strcpy(name_seg_img,argv[2]);
     
}

int main(int argc,char** argv)
{
  // set variables, allocate memory  
  int length, width;
  long long num_pix, num_pairs;
  long long num_overseg_err, num_underseg_err, num_uneq_pairs, num_equal_pairs;

  read_arguments(argc,argv);
  GetLengthWidth(name_gt_img, &length, &width);
  num_pix = length*width;
  gt_img = fmatrix_allocate_3d(3,length,width);
  seg_img = fmatrix_allocate_3d(3,length,width);
  
  // load images
  read_color_image(name_gt_img, gt_img, length, width); 
  read_color_image(name_seg_img, seg_img, length, width);

  
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
      if ((gt_img[0][py][px] == gt_img[0][py-1][px-1]) && (gt_img[1][py][px] == gt_img[1][py-1][px-1]) && (gt_img[2][py][px] == gt_img[2][py-1][px-1]))
      { 
        valid_pixels[p] = true;
        continue;
      }
    if (py>0) // upper
      if ((gt_img[0][py][px] == gt_img[0][py-1][px]) && (gt_img[1][py][px] == gt_img[1][py-1][px]) && (gt_img[2][py][px] == gt_img[2][py-1][px]))
      { 
        valid_pixels[p] = true;
        continue;
      }
    if (py>0 && px<width-1) // upper right
      if ((gt_img[0][py][px] == gt_img[0][py-1][px+1]) && (gt_img[1][py][px] == gt_img[1][py-1][px+1]) && (gt_img[2][py][px] == gt_img[2][py-1][px+1]))
      { 
        valid_pixels[p] = true;
        continue;
      }
    if (px<width-1) // right
      if ((gt_img[0][py][px] == gt_img[0][py][px+1]) && (gt_img[1][py][px] == gt_img[1][py][px+1]) && (gt_img[2][py][px] == gt_img[2][py][px+1]))
      { 
        valid_pixels[p] = true;
        continue;
      }
    if (py<length-1 && px<width-1) // lower right
      if ((gt_img[0][py][px] == gt_img[0][py+1][px+1]) && (gt_img[1][py][px] == gt_img[1][py+1][px+1]) && (gt_img[2][py][px] == gt_img[2][py+1][px+1]))
      { 
        valid_pixels[p] = true;
        continue;
      }
    if (py<length-1) // lower
      if ((gt_img[0][py][px] == gt_img[0][py+1][px]) && (gt_img[1][py][px] == gt_img[1][py+1][px]) && (gt_img[2][py][px] == gt_img[2][py+1][px]))
      { 
        valid_pixels[p] = true;
        continue;
      }
    if (py<length-1 && px>0) // lower left
      if ((gt_img[0][py][px] == gt_img[0][py+1][px-1]) && (gt_img[1][py][px] == gt_img[1][py+1][px-1]) && (gt_img[2][py][px] == gt_img[2][py+1][px-1]))
      { 
        valid_pixels[p] = true;
        continue;
      }
    if (px>0) // left
      if ((gt_img[0][py][px] == gt_img[0][py][px-1]) && (gt_img[1][py][px] == gt_img[1][py][px-1]) && (gt_img[2][py][px] == gt_img[2][py][px-1]))
      { 
        valid_pixels[p] = true;
        continue;
      }
  } //for
    
  // save validity img for testing
  int num_gt_invalid_px = 0;
  float**  validity_img = fmatrix_allocate_2d(length,width); 
  for (int y =0; y<length; y++)
    for(int x=0; x<width; x++)
      if (valid_pixels[y*width+x])
        validity_img[y][x] = 255;
      else
      {
        validity_img[y][x] = 0;
        num_gt_invalid_px ++;
      }
  char validity_name [100];
  strcpy(validity_name, name_gt_img);
  strcat(validity_name, "_validity_img.pgm");
  //write_gray_image(validity_name, validity_img, length, width);
  



  ////////////
  // compute RI
  ////////////

  num_pairs = (num_pix)*(num_pix-1)/2;
  num_overseg_err = num_underseg_err = 0;
  int px,py,qx,qy;
  long long num_comparisons = num_uneq_pairs = num_equal_pairs = 0;

  for (int p=0; p<num_pix-1; p++)
  {
    if (valid_pixels[p] == false)
      continue;
    for (int q=p+1; q<num_pix; q++) 
    {
      if (valid_pixels[q] == false)
        continue;
      py = p/width; px = p%width;
      qy = q/width; qx = q%width;
      // only check one channel, as treated as gray scale  -> 256 segments at most
      if ((gt_img[0][py][px] == gt_img[0][qy][qx]) && (gt_img[1][py][px] == gt_img[1][qy][qx]) && (gt_img[2][py][px] == gt_img[2][qy][qx]))
      {  
        num_equal_pairs ++;
        if ((seg_img[0][py][px] != seg_img[0][qy][qx]) || (seg_img[1][py][px] != seg_img[1][qy][qx]) || (seg_img[2][py][px] != seg_img[2][qy][qx]))
        {
          // overseg. error
          num_overseg_err ++;
        } 
      } else if ((gt_img[0][py][px] != gt_img[0][qy][qx]) || (gt_img[1][py][px] != gt_img[1][qy][qx]) || (gt_img[2][py][px] != gt_img[2][qy][qx])) 
      {
        num_uneq_pairs ++;
        if ((seg_img[0][py][px] == seg_img[0][qy][qx]) && (seg_img[1][py][px] == seg_img[1][qy][qx]) && (seg_img[2][py][px] == seg_img[2][qy][qx]))
        {
          // underseg. error
          num_underseg_err ++;
        }
      }
      num_comparisons ++;
    } // for q
  } // for p



  ////////////////////
  // compute Recall (percentage of boundary pixels in gt that are also boundary pixels in seg_img)
  ///////////////////

  // first create boundary img of gt_img
  float** boundary_gt_img = fmatrix_allocate_2d(length,width);
  for (int y=0; y<length; y++)
    for (int x=0; x<width; x++)
    {
      float err = 0;
      for (int dy=-1; dy<2; dy++)
        {
          if (y+dy < 0 || y+dy > length-1)
            continue;
          for(int dx=-1; dx<2; dx++)
          {
            if (x+dx < 0 || x+dx > width-1)
              continue;
            err += fabs(gt_img[0][y][x]-gt_img[0][y+dy][x+dx]); 
            err += fabs(gt_img[1][y][x]-gt_img[1][y+dy][x+dx]);
            err += fabs(gt_img[2][y][x]-gt_img[2][y+dy][x+dx]);
          }
      } // for dy
      if (err == 0)
        boundary_gt_img[y][x] = 0;
      else
        boundary_gt_img[y][x] = 255;         
    }

  // now create boundary img of seg_img
  float** boundary_seg_img = fmatrix_allocate_2d(length,width);
  for (int y=0; y<length; y++)
    for (int x=0; x<width; x++)
    {
      float err = 0;
      for (int dy=-1; dy<2; dy++)
        {
          if (y+dy < 0 || y+dy > length-1)
            continue;
          for(int dx=-1; dx<2; dx++)
          {
            if (x+dx < 0 || x+dx > width-1)
              continue;
            err += fabs(seg_img[0][y][x]-seg_img[0][y+dy][x+dx]); 
            err += fabs(seg_img[1][y][x]-seg_img[1][y+dy][x+dx]);
            err += fabs(seg_img[2][y][x]-seg_img[2][y+dy][x+dx]);
          }
      } // for dy
      if (err == 0)
        boundary_seg_img[y][x] = 0;
      else
        boundary_seg_img[y][x] = 255;         
    } // for x

  // compute recall
  int count_bound_pix, found_matches, found_matches_1, found_matches_3;
  count_bound_pix = found_matches = found_matches_1 = found_matches_3 = 0;
  for (int y=0; y<length; y++)
    for (int x=0; x<width; x++)
      if (boundary_gt_img[y][x] == 255) 
      {
        count_bound_pix ++;
        bool found, found_1, found_3;
        found = found_1 = found_3 = false;
        for (int dy=-3; dy<4; dy++)
        {
          if (y+dy < 0 || y+dy > length-1)
            continue;
          for(int dx=-3; dx<4; dx++)
          {
            if (x+dx < 0 || x+dx > width-1)
              continue;
            if (boundary_seg_img[y+dy][x+dx] == 255)
              if (abs(dy)<=3 && abs(dx)<=3)
              {
                found_3 = true;
                if (abs(dy)<=1 && abs(dx)<=1)
                {
                  found_1 = true;
                  if (dx==0 && dy==0)
                  {
                    found = true;
                  }
                }
              }
                
          } // for dx
        } // for dy
        if (found)
          found_matches ++;
        if (found_1)
          found_matches_1 ++;
        if (found_3)
          found_matches_3 ++;
          
      } // if

  // save boundary images for debugging
  char boundary_gt_name [100];
  strcpy(boundary_gt_name, name_gt_img);
  strcat(boundary_gt_name, "_boundary_img.pgm");
  //write_gray_image(boundary_gt_name, boundary_gt_img, length, width);
  char boundary_seg_name [100];
  strcpy(boundary_seg_name, name_seg_img);
  strcat(boundary_seg_name, "_boundary_img.pgm");
  //write_gray_image(boundary_seg_name, boundary_seg_img, length, width);

  ////////////////////
  // printout results
  ////////////////////

	printf("\n\nimage name: %s \n\n",name_seg_img);
  printf("width: %d \n",width);
  printf("length: %d \n",length);
  printf("# invalid px in gt_img: %d \n",num_gt_invalid_px);
  printf("num_pairs: %lld; davon equal: %.3f % \n",num_pairs, (double(num_equal_pairs) / double(num_comparisons))*100);
  printf("num_comparisons: %.lld \n",num_comparisons);
  printf("overseg. error: %.3f % (%.3f %) \n",(double(num_overseg_err)/double(num_comparisons))*100, (double(num_overseg_err)/double(num_equal_pairs))*100);
  printf("underseg. error: %.3f % (%.3f %) \n",(double(num_underseg_err)/double(num_comparisons))*100, (double(num_underseg_err)/double(num_uneq_pairs))*100);
  printf("overall error (Rand Index RI): %.3f %\n",(double(num_underseg_err + num_overseg_err)/double(num_comparisons))*100); 
  printf("num_boundary_pix: %d \n", count_bound_pix);  
  printf("Recall 1-pix. (3-pix., exact): %.3f % (%.3f %, %.3f %) \n", (float(found_matches_1)/float(count_bound_pix))*100,(float(found_matches_3)/float(count_bound_pix))*100, (float(found_matches)/float(count_bound_pix))*100); 


  //char save_name [] = "test_out_img";
  //SaveImagePpm(save_name, out_img, length, width);

  free_fmatrix_3d(gt_img, 3);
  free_fmatrix_3d(seg_img, 3);
  free_fmatrix_2d(validity_img);
  free_fmatrix_2d(boundary_gt_img);
  free_fmatrix_2d(boundary_seg_img);
  std::cout << "\n";
}

//   CopyMat(Img,Imag1,length,width,TROIS);

