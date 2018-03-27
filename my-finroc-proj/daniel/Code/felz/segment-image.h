/*
Copyright (C) 2006 Pedro Felzenszwalb

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/

#ifndef SEGMENT_IMAGE
#define SEGMENT_IMAGE

#include <cstdlib>
#include <image.h>
#include <misc.h>
#include <filter.h>
#include "segment-graph.h"
#include <pnmfile.h>
#include <string>

// random color
rgb random_rgb(){ 
  rgb c;
  double r;
  
  c.r = (uchar)random();
  c.g = (uchar)random();
  c.b = (uchar)random();

  return c;
}

// dissimilarity measure between pixels
static inline float diff(image<float> *r, image<float> *g, image<float> *b,
			 int x1, int y1, int x2, int y2) {
  return sqrt(square(imRef(r, x1, y1)-imRef(r, x2, y2)) +
	      square(imRef(g, x1, y1)-imRef(g, x2, y2)) +
	      square(imRef(b, x1, y1)-imRef(b, x2, y2)));
}


// function that draws contour line around regions
// arg RGB0: original color image
// arg rmap: region map, starting numbering at 1
// arg RGB: image-sized buffer that stores the resulting overlayed image
void draw_edges(unsigned char *RGB0,unsigned char *rmap, unsigned char *RGB,
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

  for (i=0;i<mapsize;i++)
  {
    if (rmap[i]==0)
    {
      for (j=0;j<dim;j++) RGB[dim*i+j]=0;
    }
  }
	
}

/*
void DrawContoursAroundSegments(
	image<rgb> *			image,
	universe *				u,
	int						width,
	int						height,
	rgb 					color )
{
	const int dx8[8] = {-1, -1,  0,  1, 1, 1, 0, -1};
	const int dy8[8] = { 0, -1, -1, -1, 0, 1, 1,  1};

	int sz = width*height;

	std::vector<bool> istaken(sz, false);

	int mainindex(0);
	for( int j = 0; j < height; j++ )
	{
		for( int k = 0; k < width; k++ )
		{
			int np(0);
			for( int i = 0; i < 8; i++ )
			{
				int x = k + dx8[i];
				int y = j + dy8[i];

				if( (x >= 0 && x < width) && (y >= 0 && y < height) )
				{
					int index = y*width + x;

					if( false == istaken[index] )//comment this to obtain internal contours
					{
						if( u->find(mainindex) != u->find(index) ) np++;
					}
				}
			}
			if( np > 1 )//change to 2 or 3 for thinner lines
			{
				imRef(image, k, j) = color;
				istaken[mainindex] = true;
			}
			mainindex++;
		}
	}
}
*/

/*
 * Segment an image
 *
 * Returns a color image representing the segmentation.
 *
 * im: image to segment.
 * sigma: to smooth the image.
 * c: constant for treshold function.
 * min_size: minimum component size (enforced by post-processing stage).
 * num_ccs: number of connected components in the segmentation.
 */
image<rgb> *segment_image(image<rgb> *im, float sigma, float c, int min_size,
			  int *num_ccs, char *out_name) {
  int width = im->width();
  int height = im->height();

  image<float> *r = new image<float>(width, height);
  image<float> *g = new image<float>(width, height);
  image<float> *b = new image<float>(width, height);

  // smooth each color channel  
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      imRef(r, x, y) = imRef(im, x, y).r;
      imRef(g, x, y) = imRef(im, x, y).g;
      imRef(b, x, y) = imRef(im, x, y).b;
    }
  }
  image<float> *smooth_r = smooth(r, sigma);
  image<float> *smooth_g = smooth(g, sigma);
  image<float> *smooth_b = smooth(b, sigma);
  delete r;
  delete g;
  delete b;
 
  // build graph
  edge *edges = new edge[width*height*4];
  int num = 0;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      if (x < width-1) {
	edges[num].a = y * width + x;
	edges[num].b = y * width + (x+1);
	edges[num].w = diff(smooth_r, smooth_g, smooth_b, x, y, x+1, y);
	num++;
      }

      if (y < height-1) {
	edges[num].a = y * width + x;
	edges[num].b = (y+1) * width + x;
	edges[num].w = diff(smooth_r, smooth_g, smooth_b, x, y, x, y+1);
	num++;
      }

      if ((x < width-1) && (y < height-1)) {
	edges[num].a = y * width + x;
	edges[num].b = (y+1) * width + (x+1);
	edges[num].w = diff(smooth_r, smooth_g, smooth_b, x, y, x+1, y+1);
	num++;
      }

      if ((x < width-1) && (y > 0)) {
	edges[num].a = y * width + x;
	edges[num].b = (y-1) * width + (x+1);
	edges[num].w = diff(smooth_r, smooth_g, smooth_b, x, y, x+1, y-1);
	num++;
      }
    }
  }
  delete smooth_r;
  delete smooth_g;
  delete smooth_b;

  // segment
  universe *u = segment_graph(width*height, num, edges, c);
  
  // post process small components
  for (int i = 0; i < num; i++) {
    int a = u->find(edges[i].a);
    int b = u->find(edges[i].b);
    if ((a != b) && ((u->size(a) < min_size) || (u->size(b) < min_size)))
      u->join(a, b);
  }
  delete [] edges;
  *num_ccs = u->num_sets();

  image<rgb> *output = new image<rgb>(width, height);

	// -------- My additions -------------------------------
	//
	//

  /////// Visualize Segemnts ////////
  int output_mode = 0;
	image<rgb> * output_cont = NULL;
	image<rgb> * output_seg = NULL;
  
	
	//// Variant 1: contour
	unsigned char *orig, *regmap, *out;
	orig = (unsigned char *) malloc(width*height*3*sizeof(unsigned char));
	out = (unsigned char *) malloc(width*height*3*sizeof(unsigned char));
	regmap = (unsigned char *) malloc(width*height*sizeof(unsigned char));
	rgb col;

	
	// copy orig
	for (int y=0; y<height; y++) {
		for (int x=0; x<width; x++) {
			col = imRef(im, x, y);
 			orig[(y*width+x)*3] = col.r;
			orig[(y*width+x)*3 + 1] = col.g;
			orig[(y*width+x)*3 + 2] = col.b;
		}
	}

	// build region map
	for (int y=0; y<height; y++) {
		for (int x=0; x<width; x++) {
			int comp = u->find(y * width + x);
			regmap[y*width+x] = comp + 1; // region numbering shall start with 1
		}
	}

	
	draw_edges(orig,regmap, out, height, width, 3, 0.9);
	
	
	// copy back
	output_cont = im->copy();
	for (int y=0; y<height; y++) {
		for (int x=0; x<width; x++) {
			col.r = out[(y*width+x)*3];
			col.g = out[(y*width+x)*3 + 1];
			col.b = out[(y*width+x)*3 + 2];
			imRef(output_cont, x, y) = col;
		}
	}
	
	//
	//
	// ---------------- My additions -----------------


  // VARIANT 2: regions
  output_seg = new image<rgb>(width, height);
  // pick random colors for each component
  rgb *colors = new rgb[width*height];
  for (int i = 0; i < width*height; i++)
		colors[i] = random_rgb();

  for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
		  int comp = u->find(y * width + x);
		  imRef(output_seg, x, y) = colors[comp];
		}
  }
  delete [] colors;

  delete u;

	std::string str(out_name);
	std::string new_name = str + ".cont.ppm"; 
	savePPM(output_cont, new_name.c_str());
  return output_seg;
}

#endif
