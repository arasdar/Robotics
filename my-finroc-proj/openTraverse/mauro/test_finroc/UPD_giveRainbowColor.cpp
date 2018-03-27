


//#include "UPD.h"
#include "projects/stereo_traversability_experiments/openTraverse/mauro/test_finroc/UPD.h"

namespace finroc
{
namespace mauro
{
namespace test
{



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////Rainbow color creator///////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t UPD::GiveRainbowColor(float position)

// this function gives 1D linear RGB color gradient
// color is proportional to position
// position  <0;1>
// position means position of color in color gradient

{
  if (position > 1)position = 1; //position-int(position);
  // if position > 1 then we have repetition of colors
  // it maybe useful
  uint8_t R, G, B;// byte
  int nmax = 6; // number of color bars
  float m = nmax * position;
  int n = int(m); // integer of m
  float f = m - n; // fraction of m
  uint8_t t = int(f * 255);


  switch (n)
  {
  case 0:
  {
    R = 0;
    G = 255;
    B = t;
    break;
  }

  case 1:
  {
    R = 0;
    G = 255 - t;
    B = 255;
    break;
  }
  case 2:
  {
    R = t;
    G = 0;
    B = 255;
    break;
  }
  case 3:
  {
    R = 255;
    G = 0;
    B = 255 - t;
    break;
  }
  case 4:
  {
    R = 255;
    G = t;
    B = 0;
    break;
  }
  case 5:
  {
    R = 255 - t;
    G = 255;
    B = 0;
    break;
  }
  case 6:
  {
    R = 0;
    G = 255;
    B = 0;
    break;
  }

  }; // case


  return (R << 16) | (G << 8) | B;
}




}
}
}
