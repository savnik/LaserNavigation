#! /bin/bash
let COUNTER=100000
rm -r img10-1seq
mkdir img10-1seq
cd    img10-1seq
for f in $( ls ../img10-1/*.png ); do 
  echo ln -s $f campuspath-20110913a-10-$COUNTER.png -f
  TOF=campuspath-20110913a-10-$COUNTER.png
  bayer2rgb $f $TOF
  let COUNTER+=1
  let COUNT=0
  # bash while loop to create a number of same image to speed down mpeg film
  while [ $COUNT -lt 6 ]; do
  	# echo Value of count is: $COUNT
        ln -s $TOF campuspath-20110913a-10-$COUNTER.png
        let COUNTER+=1
  	let COUNT=COUNT+1
  done 
  let PREV=$COUNTER
  done
# convert to mpeg format
# -r 25 frames per second
# -b 5000000 bitrate på 5Mbit/sec
# -y betyder overskriv destination
ffmpeg -y -r 25 -b 5000000 -i campuspath-20110913a-10-1%06d.png ../campuspath1.mpeg
cd ..
ls -l *.mpeg