# makefile to compile all parts of this package
# $Rev: 59 $
# $Id: Makefile 59 2012-10-21 06:25:02Z jcan $
# define all subdirectories to make
#
MAKE = make
BASELIBS = ../include ../lib \
          ugen4 urob4 umap4 ucam4 \
          liblineext dglib freenect\
          aulibextractfeatures utils\
           ../src/ulmsserver
#
.PHONY : $(BASELIBS)

all : $(BASELIBS)

libs: $(BASELIBS)


$(BASELIBS):
	@#echo ----- compiling lib $@
	@echo ----- compiling lib $@  2>>../logerr.txt
	-$(MAKE) --silent --jobs=3 -C $@ 
	@$(MAKE) --silent -C $@ install
#	@echo ----- lib $@ is finished


clean :
	@for dir in $(BASELIBS); do \
	  $(MAKE) --silent -C $$dir clean; \
	  echo --------- baselib $$dir is clean; \
	done

