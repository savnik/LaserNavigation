#! /bin/bash
for f in $( ls BGGR*.png ); do 
  # convert image to RGB
  echo bayer2rgb $f rgb-$f
  bayer2rgb $f rgb-$f
  done
echo done
#ls -l *.mpeg