######################################################################
#
# Makefile for Robot Manager Resource
#
# $Id: Makefile 11 2008-12-12 13:12:46Z s021786 $

######################################################################

#Destination directories
BINDIR := ../../bin
DOCDIR := doc

install: 
	@echo -n "   * Copying configuration files.. "
	@mkdir -p $(BINDIR)
	@cp -r *.xml $(BINDIR)
	@cp -r *.ini $(BINDIR)
	@echo "Done"

	
#Phony targets
.PHONY: install