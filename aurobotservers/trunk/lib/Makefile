# makefile

.PHONY : clean install all
# compile all - all objects depend on all other
all: svs/libsvs.a

# ensure that new headerfiles are in the include dirs
svs/libsvs.a :
	mv -f svs/libdcap.a.txt svs/libdcap.a
	mv -f svs/libsvs.a.txt svs/libsvs.a
	mv -f svs/libsvscalc.a.txt svs/libsvscalc.a
	mv -f svs/libsvscap.a.txt svs/libsvscap.a
	mv -f svs/libsvslagr.a.txt svs/libsvslagr.a


clean :
	#@echo lib/svs is OK

install:
	#@echo ----- lib/svs is OK
