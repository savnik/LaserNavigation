# makefile to compile all parts of  this AURS
# $Rev: 59 $
# $Id: Makefile 59 2012-10-21 06:25:02Z jcan $
#
MAKE = make
SUBDIRS = libs utils examples src plugin
# find version of openCV
#include ./include/opencv.mk
# try to guess if PCL is installed
ifeq ($(USE_PCL),)
  # find version 1.4 include directory, and use if it is there
  PCL14 := $(shell ls -d /usr/include/pcl-1.4 2>/dev/null)
  PCL15 := $(shell ls -d /usr/include/pcl-1.5 2>/dev/null)
  ifeq ($(PCL14),)
     ifeq ($(PCL15),)
     else
       export USE_PCL:=1.5
     endif
  else
    export USE_PCL:=1.4
  endif
endif

.PHONY : subdirs $(SUBDIRS) $(PLUGINSUBDIRS) doc debug optimized help

all : $(SUBDIRS)


$(SUBDIRS):
	$(MAKE) --silent -C $@

doc:
	doxygen Doxyfile

debug:
	sed -i 's#-g0 #-g3 #'  */*/Makefile
	sed -i 's#-O2 #-O0 #' */*/Makefile
	sed -i 's#-O3 #-O0 #' */*/Makefile
	sed -i 's#-O1 #-O0 #' */*/Makefile

optimized:
	sed -i 's#-g #-g0 #'  */*/Makefile
	sed -i 's#-g2 #-g0 #' */*/Makefile
	sed -i 's#-g3 #-g0 #' */*/Makefile
	sed -i 's#-O0 #-O2 #' */*/Makefile

help:
	@echo "¤¤¤¤¤¤¤¤¤¤ AURC compile options"
	@echo "¤  \t make           compiles all sources and fills build directory"
	@echo "¤  \t make clean     removes all binaries and backup files"
	@echo "¤  \t make optimized changes all makefiles to optimize -O2 and no debug -g0"
	@echo "¤  \t                needs to be followed by make clean && make"
	@echo "¤  \t make debug     changes all makefiles to no optimize -O0 and debug -g3"
	@echo "¤  \t                needs to be followed by make clean && make"
	@echo "¤  \t make doc       make doxygen html documentation into doc directory"
#	@echo "¤ info: USE_PCL=$(USE_PCL), OPENCV=$(OPENCV)"

clean :
	@for dir in $(SUBDIRS); do \
	  $(MAKE) --silent -C $$dir clean; \
	done
	@-rm -fr doc/html/*
	@bash removeolddir.bash

install : $(SUBDIRS) $(PLUGINSUBDIRS)
	@for dir in $(SUBDIRS); do \
	  $(MAKE) --silent -C $$dir install; \
	done
	@echo ---------- binaries are now in the ./build/bin and ./build/lib subdirectories
	@echo ---------- if you need them anywhere, e.g. in /usr/local, you need to do it yourself
