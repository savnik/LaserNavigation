/***************************************************************************
                          main.cpp  -  description
                             -------------------
    begin                : Sun Oct 12 17:02:34 CEST 2003
    copyright            : (C) |YEAR| by Ch2003n Andersen
    email                : jca@oersted.dtu.dk
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

//#include <iostream.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ugen4/crc.h>

const int MAX_FILENAME_SIZE = 1000;
const int MAX_CODE_LENGTH = 64;
const int MAX_INFO_SIZE = 200;

int barcodeSize = 3;
int barcodeCellFactor = 2;
int code[MAX_CODE_LENGTH];
int codeCnt = 0; // used digits in code
char filename[MAX_FILENAME_SIZE];
float sideSize = 2.5; // cm
float left   = 1.5;   // cm
float bottom = 5.0;   // cm
unsigned long intCode = 0;
bool doHelp = false;
const int MPL=100;
char pretext[MPL] = "gmk"; // no pretext in front of big number
bool noCRC = false;

/**
Decode parameters from command line and set global variables */
bool findParameters(int argc, char *argv[]);
/**
Create barcode */
bool doMakeBarcodeFile();
/**
Prit help message */
void printHelp();
/**
Print options as enterpreted */
void printWhatToDo();
/**
Square integer variable */
inline int sqr(int value)
{
  return value * value;
}

/** Main program */
int main(int argc, char *argv[])
{
  bool result;
  //
  result = (argc > 1);
  if (result)
    result =  findParameters(argc, argv);
  if (result and not doHelp)
  { // try to do as told
    printWhatToDo();
    result = doMakeBarcodeFile();
    if (result and (intCode == 0))
      printf("** NB! code 0 is usually not legal\n");
  }
  if (doHelp)
    printHelp();
  else if (not result)
    printf("-- try barcode2d --help\n");
  return EXIT_SUCCESS;
}

//////////////////////////////////////////////

void printHelp()
{
  printf("\n");
  printf("Barcode2d creates a 2D barcode image file in postscript (and .pdf) format\n");
  printf("\n");
  printf("The command format is:\n");
  printf("  barcode2d [options] <code> [<filename>]\n");
  printf("    <code>\n");
  printf("       barcode in hex format - (max ((size-4)^2 - 2) hex digits)\n");
  printf("       NB! The hex digit 'F' is not legal in all places\n");
  printf("       <filename>\n");
  printf("    <filename> is filename for the postscript file.\n");
  printf("       if no filename is specified, then code is used as\n");
  printf("       filename. '.ps' (.pdf) is added to filename\n");
  printf("    -\n");
  printf("    options\n");
  printf("    --code=code    code defined as integer (up to ~ 500000000)\n");
  printf("    --size=<frame> frame size in blocks from 7 to 12\n");
  printf("                  --frame=7 makes 3x3 block code area for 7 hex digits code\n");
  printf("                  --frame=8 makes 4x4 block code area for 14 hex digits code\n");
  printf("                  --frame=9 makes 5x5 block code area for 23 hex digits code\n");
  printf("                  --frame=12 makes 8x8 block code area for 62 hex digits code\n");
  printf("                  default is 7\n");
  printf("    --width=<width> Width of frame in cm (should be less than page width.)\n");
  printf("                  default is 17.5 cm for size 7 (i.e. 2.5 cm each block)\n");
  printf("    --pretext=\"text\" text to be printed in front of code number at top of page\n");
  printf("    --noCRC       The code vil not be extended with an 8 bit CRC code\n");
  printf("    --help          This help list\n");
  printf("\n");
  printf("  Examples:\n");
  printf("   $ barcode2d 30a326\n");
  printf("  Creates a 7x7 frame with a 3x3 block code area with the code '30a326'\n");
  printf("  in file '30a326.ps' (and 30a326.pdf) in the default directory\n");
  printf("   $ barcode2d --code=1  gives same code (but in file 1.ps (1.pdf)) as:\n");
  printf("   $ barcode2d 0000001 ~/gmks/1.ps\n");
  printf("  Creates a 7x7 frame with a 3x3 block code area with the code '0000001'\n");
  printf("  decimal code '16' (first 8 hex digits) in file 'gmks/1.ps' (gmks/1.pdf) in home directory\n");
  printf("\n");
  printf("   $ barcode2d --frame=9 --width=4.5 0123456789abcdef small.ps\n");
  printf("  Creates a 9x9 frame with a 5x5 block code area with the a 16 digit code\n");
  printf("  out of 23 code spaces (rest is zeros filled)\n");
  printf("  Frame width is 4.5 cm in file 'small.ps' (and small.pdf)\n");
}

//////////////////////////////////////////////

bool findParameters(int argc, char * argv[])
{
  bool result = true;
  float w = -1.0;
  bool gotCode = false;
  //
  code[0] = 0xf;
  code[1] = 0xf;
  for (int i = 1; i < argc; i++)
  {
    char * op = argv[i];
    if (strstr(op, "--") != NULL)
    {
      int b;
      int n = sscanf(op, "--size=%d", &b);
      if (n == 1)
      { // set frame size
        if ((b >= 7) and (b <= 12))
          // code size is 4 less than frame size
          barcodeSize = b - 4;
        else
        {
          printf("*** %d blocks is not a legal frame size!\n", b);
          result = false;
        }
      }
      //
      n = sscanf(op, "--width=%f", &w);
      if (n == 1)
      { // set frame width
        if (w < 0.5)
          printf("*** '%f' cm is not a legal frame width!\n", w);
      }
      n = sscanf(op, "--code=%lu", &intCode);
      if (n == 1)
      { // set frame width
        gotCode = true;
      }
      n = strncmp(op, "--pretext", 9);
      if (n == 0)
      { // a pretext is specified
        char * tx = strchr(op, '=');
        if (tx != NULL)
        {
          strncpy(pretext, ++tx, MPL);
          pretext[MPL-1] = '\0';
        }
      }
      if (strcasestr(op, "--noCRC") != NULL)
      { // print help
        noCRC = true;
      }
      if (strstr(op, "--help") != NULL)
      { // print help
        doHelp = true;
      }
    }
    else
    { // not an option, so code or filename
      if (not gotCode)
      { // first non option is code
        int n = 0;
        // first two is ID
        int b = strlen(op);
        if (b > 0)
        {
          while (n < b)
          { // convert to hex values
            if ((op[n] >= '0') and (op[n] <= '9'))
              code[n + 2] = op[n] - '0';
            else if ((op[n] >= 'A') and (op[n] <= 'F'))
              code[n + 2] = op[n] - 'A' + 10;
            else if ((op[n] >= 'a') and (op[n] <= 'f'))
              code[n + 2] = op[n] - 'a' + 10;
            else
              break;
            if (n < 8)
            intCode = (intCode << 4) + code[n+2];
            n++;
          }
          result = (n > 0);
          codeCnt = n + 2;
          snprintf(filename, MAX_FILENAME_SIZE, "%s.ps", op);
//           while (n < 8)
//           { // shift such that first digit is
//             // MSB in 8 hex character integer
//             intCode = (intCode << 4);
//             n++;
//           }
          gotCode = true;
        }
      }
      else
      { // then it must be filename
        int n = strlen(op);
        if (strstr(&op[n-3], ".ps") == NULL)
          // does not end on .ps, so add
          snprintf(filename, MAX_FILENAME_SIZE, "%s.ps", op);
        else
          // filename ends at .ps, so just use as is
          strncpy(filename, op, MAX_FILENAME_SIZE);
      }
    }
  }
  if (result and gotCode)
  {
    int nBytes;
    unsigned long ui = intCode;
    // number of 4-bit fields
    codeCnt = barcodeSize * barcodeSize;
    if (noCRC)
      nBytes = codeCnt;
    else
      nBytes = codeCnt - 2;
    for (int i = nBytes - 1; i > 1; i--)
    {
      code[i] = (ui & 0xf);
      ui = ui >> 4;
    }
    snprintf(filename, MAX_FILENAME_SIZE, "%lu.ps", intCode);
    if (not noCRC)
    { // add CRC if needed
      const int M8L = 10;
      unsigned char code8[M8L];
      int code8Cnt = (nBytes + 1) / 2;
      int i;
      if (code8Cnt > M8L-1)
        code8Cnt = M8L-1;
      // zero 8-bit code for CRC add
      for (int i = 0; i < M8L; i++)
        code8[i] = 0;
      // convert 4-bit to 8-bit array
      // skip first byte, as it (pt) is 0xff
      i = code8Cnt - 1;
      for (int n = nBytes - 1; n > 1; n--)
      {
        if (n % 2 == 0)
          code8[i] = code[n];
        else
          code8[i--] += code[n] << 4;
      }
      code8[code8Cnt] = getCRC(code8, code8Cnt);
      printf("CRC: nBytes bytes:");
      for (int i = 0; i < nBytes; i++)
        printf(" %2x", code8[i]);
      printf("; check=%2x\n", getCRC(code8, code8Cnt + 1));
      code[nBytes] = code8[code8Cnt] >> 4;
      code[nBytes + 1] = code8[code8Cnt] & 0x0f;
    }
  }
  if (result)
  { // check code length
    result = codeCnt <= sqr(barcodeSize);
    if (not result)
      printf("*** Code is %d long, but only space for %d!\n",
         codeCnt - 2, sqr(barcodeSize) - 2);
  }
  if (result)
  { // set block size
    int n;
    if (w >= 0.5)
    {
      sideSize = w / float(barcodeSize + 4);
      if (w < 19.1)
        left = (21.1 - w) / 2.0;
    }
    else
      // automatic block size for 20 cm wide frame
      if (barcodeSize > 3)
        sideSize = 18.0 / float(barcodeSize + 4);
    // check result other corners must NOT be 0xff
    // is first two ff's is ID mark and should be unique
    // tol left corner
    n = barcodeSize;
    if (code[n] == 0xf)
    { // error
      printf("*** Code error! code digit %d is 'F'!\n", n);
      result = false;
    }
    if (result)
    { // top right corner
      n = barcodeSize - 1;
      int n2 = n - 1;
      int n3 = n + barcodeSize;
      if ((code[n] == 0xf) and ((code[n2] == 0xf) or (code[n3] == 0xf)))
        result = false;
      if (result)
      { // bottom left corner
        n = sqr(barcodeSize) - barcodeSize;
        n2 = n + 1;
        n3 = n - barcodeSize;
        if ((code[n] == 0xf) and ((code[n2] == 0xf) or (code[n3] == 0xf)))
          result = false;
      }
      if (result)
      { // bottom right corner
        n = sqr(barcodeSize) - 1;
        n2 = n - 1;
        n3 = n - barcodeSize;
        if ((code[n] == 0xf) and ((code[n2] == 0xf) or (code[n3] == 0xf)))
          result = false;
      }
      if (not result)
          printf("*** Code error! code digit %d and %d or %d is 'F'!\n",
                    n - 3, n2 - 3, n3 - 3);
    }
  }

  //
  return result;
}

//////////////////////////////////////////////

void printWhatToDo()
{
  int i;
  //
  printf("Pretext='%s'\n", pretext);
  printf("Do a %dx%d block frame of size %1.2fx%1.2f cm\n",
            barcodeSize + 4, barcodeSize + 4,
            sideSize * (barcodeSize + 4), sideSize * (barcodeSize + 4));
  printf("Block size %1.3f cm\n", sideSize);
  printf("With code: '");
  for (i = 2; i < codeCnt; i++)
    printf("%x", code[i]);
  printf("'\n");
  printf("To file: %s (and -.pdf)\n", filename);
}

//////////////////////////////////////////////

bool doMakeBarcodeFile()
{
  bool result = true;
  const float ppcm = 72.0/2.54; // points per cm
  FILE * fl;
  float x, y, ra, ss, ssc;
  int i, r, j, b;
  char topdf[MAX_FILENAME_SIZE];
  //
  fl = NULL;
  fl = fopen(filename, "w");
  //
  if (fl != NULL)
  { // print lead in data
    fprintf(fl, "%%! - this is not encapsulated postscript\n");
    fprintf(fl, "%% this is a 2D barcode page generated by barcode2d\n");
    fprintf(fl, "%% frame sixe %d and code size multiplier %d\n", barcodeSize, barcodeCellFactor);
    fprintf(fl, "%% code is ");
    for (i = 0; i < codeCnt; i++)
      fprintf(fl, "%x", code[i+2]);
    fprintf(fl, " - length %d (decimal integer value %lu)\n", codeCnt, intCode);
    // sideSize of squares in cm
    fprintf(fl, "%% definition of box side size of %f cm\n", sideSize);
    fprintf(fl, "/ss {72 2.54 div %f mul 0.5 sub} def\n", sideSize);
    fprintf(fl, "/ssc {72 2.54 div %f mul 0.5 sub} def\n", sideSize/float(barcodeCellFactor));
    // define box of side size
    fprintf(fl, "%% definition of square box\n");
    fprintf(fl, "/box {newpath moveto 0 ss rlineto ss 0 rlineto 0 0 ss sub rlineto closepath} def\n");
    fprintf(fl, "/boxc {newpath moveto 0 ssc rlineto ssc 0 rlineto 0 0 ssc sub rlineto closepath} def\n");
    // print text
    fprintf(fl, "%% text on bottom of page\n");
    // set font size for text
    fprintf(fl, "/Helvetica findfont 48 scalefont setfont\n");
    // print first text line with size info
    fprintf(fl, "newpath %5.1f %5.1f moveto\n", 1.5 * ppcm, 25.0 * ppcm);
    fprintf(fl, "(%s %lu) show\n", pretext, intCode);
    fprintf(fl, "/Helvetica findfont 14 scalefont setfont\n");
    // print first text line with size info
    fprintf(fl, "newpath %5.1f %5.1f moveto\n", 1.5 * ppcm, 3.0 * ppcm);
    fprintf(fl, "(Barcode %dx%d frame, block side size %1.3f cm) show\n",
                 barcodeSize + 4, barcodeSize + 4, sideSize);
    // print second text line with code
    fprintf(fl, "newpath %5.1f %5.1f moveto\n", 1.5 * ppcm, 2.0 * ppcm);
    fprintf(fl, "(code: ");
    for (i = 2; i < codeCnt; i++)
      fprintf(fl, "%x", code[i]);
    fprintf(fl, " hex, %lu decimal", intCode);
    fprintf(fl, ") show\n");
    //
    /*
    fprintf(fl, "%5.1f %5.1f moveto %5.1f %5.1 lineto stroke\n",
                     x - ppcm, y + 0.5, x + ppcm, y + 0.5);
    fprintf(fl, "%5.1f %5.1f moveto %5.1f %5.1 lineto stroke\n",
                     x + 0.5, y - ppcm, x + 0.5 , y + ppcm);
    fprintf(fl, "%5.1f %5.1f %5.1f %1.5f %1.5f arc stroke\n",
                  x,    y,  ppcm + 1,  0.0, 360.0);
    fprintf(fl, "%5.1f %5.1f moveto\n", x, y - ppcm);
    fprintf(fl, "%5.1f %5.1f %5.1f %1.5f %1.5f arc stroke\n",
                  x,    y,  ppcm,  -90.0, 270.0);
    */
    fprintf(fl, "0.00 setgray\n"); // black
    //
    // make boxes
    // bottom row
    fprintf(fl, "%% bottom frame\n");
    x = left * ppcm;
    y = bottom * ppcm;
    ss = sideSize * ppcm;
    ssc = sideSize * ppcm / float(barcodeCellFactor);
    for (r = 0; r < (barcodeSize + 4); r++)
    {
      y = bottom * ppcm + r * ss;
      for (i= 0; i < (barcodeSize + 4); i++)
      {
        if (((i+r) % 2) == 0)
          if ((r < 2) or (r >= (barcodeSize + 2)) or
              (i < 2) or (i >= (barcodeSize + 2)))
            fprintf(fl, "%6.2f %6.2f box fill\n", x + float(i) * ss, y);
      }
    }
    //
    // now the code blobs
    fprintf(fl, "%% code squares\n");
    for (i = 0; i < (codeCnt + 2); i++)
    { // find bottom left of code square
      x = left * ppcm + float(2 + (i % (barcodeSize))) * ss;
      y = bottom * ppcm +
            float(barcodeSize + 1 - i / (barcodeSize)) * ss;
      // get code
      b  = code[i];
      for (j = 0; j < 8; j++)
      { // a square for each '1' in the code
        // lsb first
        if (((b >> j) % 2) == 1)
        { // draw a square
          fprintf(fl, "%6.2f %6.2f boxc fill\n",
              x + (j % barcodeCellFactor) * ssc,
              y + ss - (1 + j / barcodeCellFactor) * ssc);
        }
      }
    }
    //
    // make center circle
    x = (left + (barcodeSize + 4) * sideSize / 2.0) * ppcm;
    y = (bottom + (barcodeSize + 4) * sideSize / 2.0) * ppcm;
    ra = ppcm * 0.5;
    fprintf(fl, "0.2 setlinewidth\n");
    fprintf(fl, "0.0 setgray\n");  // gray cross and circle
    fprintf(fl, "%5.2f %5.2f moveto %5.2f %5.2f lineto stroke\n",
                  x,   y - ra,       x , y + ra);
    fprintf(fl, "%5.2f %5.2f moveto %5.2f %5.2f lineto stroke\n",
                     x - ra, y, x + ra, y);
    fprintf(fl, "%5.2f %5.2f %5.2f %3.2f %3.2f arc stroke\n",
                  x,    y,    ra,   0.0, 360.0);
    fprintf(fl, "1.0 setgray\n");  // white circle
    fprintf(fl, "%5.2f %5.2f %5.2f %3.2f %3.2f arc stroke\n",
                  x,    y,    ra+0.2,   0.0, 360.0);
    //
    // final commands
    fprintf(fl, "%% show the result\n");
    fprintf(fl, "showpage\n");
    fprintf(fl, "%% end\n");
    // close file
    fclose(fl);
    snprintf(topdf, MAX_FILENAME_SIZE, "ps2pdf %s", filename);
    i = system(topdf);
  }
  else
  {
    printf("*** Could not create outputfile '%s'\n", filename);
    result = false;
  }
  return result;
}
