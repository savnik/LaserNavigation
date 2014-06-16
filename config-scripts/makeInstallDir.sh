#!/bin/bash
# makes - or ensures that switchtool environment is in place
#! /bin/bash
# ensure directories exist
RCPATH=/usr/local/etc
mkdir -p ${RCPATH}/mobotware/calib
mkdir -p /usr/local/smr.local
# copy new versions of configuration tools
# save old version, as it may be modified
cd config-scripts
if [ ! -x ${RCPATH}/mobotware/rc.start ]; then
  # do not replace startup script, as it is configuration dependent
  cp rc.start ${RCPATH}/mobotware/
else
  echo "${RCPATH}/mobotware/rc.start is already in place, and is not changed"
fi
#
#
cd ..
# rhdconfig.ini
if [ ! -e ${RCPATH}/mobotware/rhdconfig.xml ]; then
  #echo "no ${RCPATH}/mobotware/rhdconfig.xml found, so copying a default SMR version"
  cp build/config/smr/rhdconfig.xml ${RCPATH}/mobotware/
else
  echo "${RCPATH}/mobotware/rhdconfig.xml is already in place, and is not changed"
fi
# robot.conf
if [ ! -e ${RCPATH}/mobotware/calib/robot.conf ]; then
  #echo "no ${RCPATH}/mobotware/calib/robot.conf found, so copying a default SMR version"
  cp build/config/smr/calib/robot.conf ${RCPATH}/mobotware/calib
else
  echo "${RCPATH}/mobotware/calib/robot.conf is already in place, and is not changed"
fi
# aukeeper.ini
if   [ -e ${RCPATH}/mobotware/aukeeper.ini ]; then
  echo "${RCPATH}/mobotware/aukeeper.ini  is already in place, and is not changed"
else
  #echo "no ${RCPATH}/mobotware/aukeeper.ini found, so copying a default SMR version"
  cp build/config/smr/aukeeper.ini ${RCPATH}/mobotware/
fi
#
# MARG web interface
#
if [ -d "/srv/httpd/htdocs" ] ; then
  WWW_DIR=/srv/httpd/htdocs/
elif [ -d "/var/www" ] ; then
  WWW_DIR=/var/www/
else
  WWW_DIR=no-marg
fi
echo "MARG destination dir is $WWW_DIR"
SMRHN=`hostname`
if [ "$WWW_DIR" != "no-marg" ] ; then
  rm -f $WWW_DIR/index.html
  cp $PWD/marg/trunk/index.html $WWW_DIR/index.html
  cp marg/trunk/robot.xml $WWW_DIR/robot.xml
  mkdir -p $WWW_DIR/dist
  cp marg/trunk/dist/* $WWW_DIR/dist/ 
  sed -i "s/smrhost/$SMRHN/g" $WWW_DIR/robot.xml
  echo "Copied MARG to $WWW_DIR and changed smrhost to $SMRHN"
fi

