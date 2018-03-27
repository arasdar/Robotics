

#include "projects/stereo_traversability_experiments/daniel/segmentation_with_texture/tStereoProcessing.h"
#include "projects/stereo_traversability_experiments/daniel/segmentation_with_texture/texture/tTextureExtractor.h"

using namespace finroc::stereo_traversability_experiments::daniel::segmentation_with_texture;

void
tStereoProcessing::processImage_texture()
{
  unsigned int radius = 2, xOffset = 1, yOffset = 1;
  texture::tTextureExtractor extractor = texture::tTextureExtractor();
  prev_texture_cloud = extractor.compute_texture(left_images[images_idx], radius, xOffset, yOffset);
}
