######################################################################
#
# Top level Makefile for RobotManager Project
#
# $Id: Makefile 51 2009-03-20 12:48:05Z s021786 $
######################################################################

#Destination directories
BINDIR := bin
DOCDIR := doc

all: title robotmanager install

title: 
	@echo "====  AuPlanner plugin Project 1.x  ===="

robotmanager:
	@echo "=== Compiling AuManager ==="
	@$(MAKE) -s --jobs=3 -C src

install: 
	@echo "=== Copying files to build directory ==="
	@mkdir -p $(BINDIR)
	@$(MAKE) -s --jobs=3 -C src install
	rm -rf bin/*~
	@rm -rf bin/Makefile

clean:
	@echo "====   Cleaning RobotManager project   ===="
	@$(MAKE) -s -C src clean
	@echo "=== Removing binaries ==="
	@echo -n "   * Removing bin directory.. "
	@rm -rf ./$(BINDIR)
	@echo "Done"
	@echo -n "   * Removing kate *~ trash files.. "
	@rm -rf *~
	@echo "Done"


#Phony targets
.PHONY: title install robotmanager
