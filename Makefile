# version 1290 (29 jan 2011)
# makefile to compile all parts of this package
# define all subdirectories to make
#
MAKE = make
APPSUBDIRS = aumat/trunk \
    aurobotservers/trunk/include \
    aurobotservers/trunk/libs \
    mapbase/trunk \
    rhd/trunk \
    mrc/trunk \
    hakoclient/trunk  \
    aurs-plugins/aumanager aurs-plugins/aumrcobst aurs-plugins/auplanner/trunk \
    aurobotservers/trunk
#
ifeq ("$(VER)","")
  VER=noname
endif

.PHONY : $(APPSUBDIRS) finished install help kalman

all : help $(APPSUBDIRS) finished

$(APPSUBDIRS):
	$(MAKE) --silent -C $@

clean :
	@for dir in $(APPSUBDIRS); do \
	  $(MAKE) --silent -C $$dir clean; \
	done

install:
	@if [ "$(USER)" != "root" ]; then \
	  echo "¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤";\
	  echo "NB! must be ROOT to install";\
	  echo "¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤";\
	else \
	  bash config-scripts/makeInstallDir.sh; \
          echo "Stopping current version of mobotware"; \
          bash config-scripts/rc.shutdown; \
          echo "Copying build/bin and build/bin to /usr/local/smr.local"; \
	  cp -rL $(CURDIR)/build/* /usr/local/smr.local/; \
	  echo "Do a:"; \
	  echo "  switchtool local"; \
	  echo "to use the installed version"; \
	  echo "Add this command - switchtool local - to rc.local to use this version at boot-time"; \
	fi

kalman:
	@echo "¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤"
	@echo "Install on Kalman for all smrs, like:"
	@echo "    make kalman VER=2011z"
	@echo "  this will copy the binary, lib-files and the tarball"
	@echo "  with the source to the /opt/smr.2011z directory"
	@echo "  directory must exist, and be filled with things"
	@echo "  like simulator and other stuff needed in /usr/local/smr"
	scp -r --force build/* kalman:/opt/smr.$(VER)/
	scp -r --force mobotware*.tar.gz kalman:/opt/smr.$(VER)/
	@echo "¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤"

help:
	@echo "¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤"
	@echo "Mobotware"
	@echo "    make"
	@echo "  will try to compile all mobotware modules."
	@echo "Mobotware install (as root):"
	@echo "  Install as a local version (into /usr/local/smr.local):"
	@echo "    make install"
	@echo "  Install a version for switchtool (e.g. for: switchtool 2012z):"
	@echo "    make install VER=2012z"
	@echo "  Install on Kalman for all smrs, like:"
	@echo "    make kalman VER=2012z"
	@echo "  this will copy the binary, lib-files and the tarball"
	@echo "  with the source to kalman:/opt/smr.2011z."
	@echo "  The directory is assumed to exist, and filled with things"
	@echo "  like simulator and other stuff that should exist in /usr/local/smr"
	@echo "¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤done¤¤¤¤¤¤¤"

finished:
	@echo ¤¤¤¤¤ finished the compile part
	bash config-scripts/makeBuildDir.sh 
	@echo binaries are now in $(PWD)/build/bin and should be added to your PATH
	@echo plug-ins are now in $(PWD)/build/lib and should be added to your LD_LIBRARY_PATH
	@echo "e.g. add to ~/.bashrc (or /etc/bash.bashrc)"
	@echo "      " export PATH="$$"PATH:$(PWD)/build/bin
	@echo "      " export LD_LIBRARY_PATH="$$"LD_LIBRARY_PATH:$(PWD)/build/lib
	@echo ¤¤¤¤¤
