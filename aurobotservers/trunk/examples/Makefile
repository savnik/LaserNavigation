# version 2.670 (12 aug 2009)
# makefile to compile all utilities in this dir
#
MAKE = make
APPSUBDIRS = minimum exvar aulmsnear
#
.PHONY : subdirs  $(APPSUBDIRS)

all :  $(APPSUBDIRS)

$(APPSUBDIRS):
	$(MAKE) --silent --jobs=3 -C $@
	@$(MAKE) --silent -C $@ install
	@echo ----- example $@ is finished

clean :
	@for dir in $(APPSUBDIRS); do \
	  $(MAKE) --silent -C $$dir clean; \
	  echo --------- example $$dir is clean; \
	done

install : $(APPSUBDIRS)
	@for dir in $(APPSUBDIRS); do \
	  $(MAKE) --silent -C $$dir install; \
	done

