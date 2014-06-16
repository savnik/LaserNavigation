

#ifdef OPENCV2
#include <core/core_c.h>
#include <imgproc/imgproc_c.h>
#include <calib3d/calib3d.hpp>
#include <highgui/highgui_c.h>
#else
#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#endif
//#include <iostream>
#include <stdio.h>
#include <getopt.h>
#include <ugen4/uimage.h>


bool getCmdLineOptions(int argc, char *argv[],
                       const char ** src, const char ** dest, bool * half);

int main(int argc, char* argv[])
{
  const char * srcName = NULL, * destName = NULL;
  bool half = false;
  bool isOK = getCmdLineOptions(argc, argv, &srcName, &destName, &half);
  if (isOK and srcName != NULL and destName != NULL)
  { // board size;
    UImage img;
    char * p1 = strrchr((char*)srcName, '.');
    // make some buffer space for images
    img.setSize(800, 600, 1, 8, "BGGR");
    // is it a bmp source file
    if (strcasecmp(p1, ".bmp") == 0)
      isOK = img.loadBMP(srcName);
    else
      isOK = img.loadPNG(srcName);
    if (isOK)
    {
      if (half)
        isOK = img.toHalf();
      else
        isOK = img.toRGB();
      if (not isOK)
        printf("Failed to convert %s\n", srcName);
    }
    else
      printf("Did not fint %s\n", srcName);
    if (isOK)
    { // save in same format as source
      if (strcasecmp(p1, ".bmp") == 0)
        isOK = img.saveBMP(destName);
      else
        isOK = img.savePNG(destName);
      if (not isOK)
        printf("failed to save %s\n", destName);
    }
  }
  else
    isOK = false;
  return not isOK;
}

/////////////////////////////////////////////////////

bool getCmdLineOptions(int argc, char *argv[],
                       const char ** src, const char ** dest, bool * half)
{
  static struct option long_options[] = {
    {"help", 0, 0, 'a'},
    {"half", 0, 0, '2'},
    {0, 0, 0, 0}
  };
  bool ask4help = false;
  int opt;
  int option_index = 0;
  while(true)
  {
    opt = getopt_long(argc, argv, "a2w:h:",
                 long_options, &option_index);
    if (opt == -1)
      break;
    switch (opt)
    {
      case 'a': // help
        ask4help = true;
        break;
      case '2': // half size
        *half = true;
        break;
      default:
        break;
    }
  }
  if (ask4help or argc < 3)
  {
    printf("\nConverting a bayer coded image into a corresponding RGB image (bmp and png format only).\n");
    printf("    option -2 fill reduce source image to half size (half width and half height)\n");
    printf("    use a script to convert many files (see bayer2rgb.bash as an example)\n");
    printf("use:\n");
    printf("  $ %s [-2] source destination\n", argv[0]);
  }
  else
  {
    int n = 0;
    for (int i = 1; i < argc; i++)
    {
      const char * p1 = argv[i];
      if (*p1 != '-')
      {
        if (n++ == 0)
          *src = p1;
        else
          *dest = p1;
      }
    }
  }
  return not ask4help;
}

