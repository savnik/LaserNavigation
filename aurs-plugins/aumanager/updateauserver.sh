#!/bin/bash

AUSERVERPATH=/home/andersbeck/rse/aurobotservers
MAPBASEPATH=/home/andersbeck/rse/mapbase

if [ "$1" = "updateserver" ]
then
echo "*** Update local version of AuRobotServers from RSE SVN ***"
rm -rf $AUSERVERPATH/trunk
svn update $AUSERVERPATH 
echo "Copy working makefile to trunk..."
cp makefile_for_auserver $AUSERVERPATH/trunk/Makefile
echo "Remaking aurobotservers"
make -s -C $AUSERVERPATH/trunk --jobs=3
make -s -C $AUSERVERPATH/trunk install
echo "*** Update local version of Mapbase plugin from RSE SVN ***"
echo "Remaking mapbase"
make -s -C $MAPBASEPATH/trunk --jobs=3 clean
make -s -C $MAPBASEPATH/trunk --jobs=3
fi

if [ "$1" = "updatemap" ]
then
echo "*** Update local version of Mapbase plugin from RSE SVN ***"
rm -rf $MAPBASEPATH/trunk
svn update $MAPBASEPATH
echo "Remaking mapbase"
make -s -C $MAPBASEPATH/trunk --jobs=3
fi

echo -n "   * Copying binaries from auserver and mapbase plugin.."
mkdir -p bin
cp $AUSERVERPATH/trunk/bin/ulmsserver    bin
cp $AUSERVERPATH/trunk/bin/aurule.so.0   bin
cp $AUSERVERPATH/trunk/bin/ausmr.so.0    bin
cp $AUSERVERPATH/trunk/bin/auefline.so.0 bin
cp $MAPBASEPATH/trunk/mapbase.so.0       bin
echo " Done"

