# makefile
# $Rev: 59 $
# $Id: Makefile 59 2012-10-21 06:25:02Z jcan $

HDIRS = ugen4 urob4 umap4 ucam4
REDIRS = logerr ulmsserver aulibextractfeatures dglib mapbase freenect lel ausmr mhf eigen3 boost utils

.PHONY : $(HDIRS) $(REDIRS)
# compile all - all objects depend on all other
all: $(HDIRS) $(REDIRS)

# ensure that new headerfiles are in the include dirs

$(HDIRS):
	-@rm -f $(@)/*.h
	cp ../libs/$(@)/*.h $(@)/

# create logfile for selected errors
logerr:
	echo "Selected errors" >../logerr.txt

dglib:
	-@rm -f dglib/*.h
	cp ../libs/dglib/src/*.h dglib/

mapbase:
	cp ../plugin/mapbase/*.h .

freenect:
	cp ../libs/freenect/libfreenect.h .
	cp ../libs/freenect/libfreenect.hpp .
	cp ../libs/freenect/registration.h .

ulmsserver:
	-@rm -f ulms4/*.h
	cp ../src/ulmsserver/*.h ulms4

aulibextractfeatures:
	-@rm -f auef/*.h
	cp ../libs/aulibextractfeatures/*.h auef

lel:
	cp ../libs/liblineext/*.h .
	cp ../libs/liblineext/*.h lel/

ausmr:
	mkdir -p ausmr
	cp ../plugin/ausmr/uressmr.h ausmr

mhf:
	mkdir -p mhf
	cp ../libs/mhf/*.h mhf/
	cp ../libs/mhf/*.hpp mhf/
	cp ../libs/mhf/defaultTable mhf/
	cp ../libs/mhf/iau_ukf_eigen.hpp .
	cp ../libs/mhf/iau_ukf_eigen.h .

eigen3:
	ln -s -f ../libs/eigen3 .

boost:
	ln -s -f ../libs/boost .

utils:
	mkdir -p utils
	cp ../libs/utils/*.h utils/

clean :
	#-rm *~ 2>/dev/null

install:
	#@echo ----- include is OK
