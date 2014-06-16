# makefile to compile all parts of this package
# $Rev: 258 $
# $Id: Makefile 258 2013-10-06 17:51:30Z jcan $
# define all subdirectories to make
#
MAKE = make
plugins := auavoid       auball auballkl   audrivepos  augps    \
    aukeep     aulaserifscan       aupar    aurhdif  \
    auseq      austereo  auvarmrc  ucamif \
    auavoidk   aucamcog  aucron    auefline    augraphplan \
    aukinect   aulobst   aumapobst aupoly      auroaddrive \
    ausmr      locater   ulmspassable \
    aucamfocus audisp    aufile    auimu     \
    aulaserif  aulocalize          aulocalize2 \
    auobj3d    aupplfinder         aurule    \
    ausockbin  autof     mapbase   ulmsv360    aucamrectify aupipi \
    aulaserbox aufzclass auviewer aupcp aupcltest auv4lgst \
    
notcompiled := ausvs auv4lgst autree
# try to guess if PCL is installed
ifeq ($(USE_PCL),)
  # find version 1.4 include directory, and use if it is there
  PCL14 := $(shell ls -d /usr/include/pcl-1.4 2>/dev/null)
  ifeq ($(PCL14),)
  else
    export USE_PCL:=1.4
  endif
endif
ifeq ($(USE_PCL),)
else
  # add plugins that need PCL (and other PCL dependencies)
  plugins += aupcltest
  # auviewer
endif
# v4l2ok := $(shell ls -d /usr/include/libudev.h 2>/dev/null)
# ifeq ($(v4l2ok),)
# else
#   plugins += auv4l2
# endif


# missing : aucamlocalize
#
.PHONY : $(plugins) symlink

all : $(plugins) symlink

symlink:
	@ln -sf ../../plugin/mapbase/lib/mapbase.so.0 ../build/lib/ >/dev/null


$(plugins):
	@echo ----- compiling plugin $@
	-$(MAKE) --silent --jobs=3 -C $@
	@$(MAKE) --silent -C $@ install


clean :
	@for dir in $(plugins); do \
	  $(MAKE) --silent -C $$dir clean; \
	  echo --------- plugin $$dir is clean; \
	done

install : symlink
	@for dir in $(plugins); do \
	  $(MAKE) --silent -C $$dir install; \
	done

