# version 2.670 (12 aug 2009)
# makefile to compile all utilities in this dir
#
MAKE = make
APPSUBDIRS = mapoffset smrcllogtrim cameracalibrate aulogrotate \
             barcode2d stereocalibrate utmpose2kml bayer2rgb \
             aulogtime
# cameracalibrate
#
.PHONY : subdirs  $(APPSUBDIRS)

all :  $(APPSUBDIRS)

$(APPSUBDIRS):
	$(MAKE) --silent --jobs=3 -C $@
	@$(MAKE) --silent -C $@ install
	@echo ----- util $@ is finished

clean :
	@for dir in $(APPSUBDIRS); do \
	  $(MAKE) --silent -C $$dir clean; \
	  echo --------- util $$dir is clean; \
	done
	@for dir in $(APPSUBDIRS); do \
	  if [ -f ../bin/$$dir ]; then \
	    rm ../bin/$$dir; \
	  fi; \
	done

install : $(APPSUBDIRS)
	@for dir in $(APPSUBDIRS); do \
	  $(MAKE) --silent -C $$dir install; \
	done

