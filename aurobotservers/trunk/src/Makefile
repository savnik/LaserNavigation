# makefile to compile all parts of this package
# $Rev: 59 $
# $Id: Makefile 59 2012-10-21 06:25:02Z jcan $
# define all subdirectories to make
#
MAKE = make
apps = auclient  auservertest  ucamserver  uclient  ulmsserver  userver  
#
.PHONY : $(apps)

all : $(apps) 

$(apps):
	@echo ----- compiling app $@
	$(MAKE) --silent --jobs=3 -C $@
	@$(MAKE) --silent -C $@ install


clean :
	@for dir in $(apps); do \
	  $(MAKE) --silent -C $$dir clean; \
	  echo --------- app $$dir is clean; \
	done

install:
	@for dir in $(apps); do \
	  $(MAKE) --silent -C $$dir install; \
	done
