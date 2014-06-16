#!/bin/bash
mkdir -p build/bin
mkdir -p build/lib
# collect all executable files
cd build/bin
ln -sf ../../mrc/trunk/mrc/mrc .
ln -sf ../../rhd/trunk/build/bin/* .
ln -sf ../../aurobotservers/trunk/build/bin/* .
ln -sf ../../hakoclient/trunk/hakoclient .
ln -sf ../../aurobotservers/trunk/build/config/aursconf .
# add switchtool scripts
ln -sf ../../config-scripts/switchtool .
ln -sf ../../config-scripts/rc.start .
ln -sf ../../config-scripts/rc.shutdown .
ln -sf ../../config-scripts/update_smr .
# add common alias names
ln -sf userver aukeeper
ln -sf userver aunav
cd ../lib
ln -sf ../../rhd/trunk/build/lib/* .
ln -sf ../../aurobotservers/trunk/build/lib/* .
ln -sf ../../aurs-plugins/*/*.so.0 .
ln -sf ../../aurs-plugins/*/trunk/bin/*.so.0 .
ln -sf ../../rhd/trunk/build/lib/rhdplugin .
cd ../..
echo "--------------"
echo "build version will link to ${PWD}/build"
echo "--------------"
