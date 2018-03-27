#ifndef __IOUTIL_H
#define __IOUTIL_H


//void draw_edges(unsigned char *RGB0,unsigned char *rmap, unsigned char *RGB, int ny,int nx,int dim,float displayintensity);


 
 
void draw_edges(unsigned char *RGB0,unsigned char *rmap, unsigned char *RGB,
    int ny,int nx,int dim,float displayintensity)
{
  int iy,ix,i,j,datasize,l1,l2,mapsize;
  mapsize = ny*nx;
  datasize = ny*nx*dim;
	int d1 = (int) displayintensity;            // Get the integer part (678).
	float f2 = displayintensity - d1;     // Get fractional part (678.0123 - 678 = 0.0123).
	int d2 = f2 * 10;
	printf("displayintesity: %d.%d\n",d1,d2);
  for (i=0;i<datasize;i++) RGB[i] = RGB0[i]*0.9;//displayintensity;
	/*
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
	*/
}
#endif
