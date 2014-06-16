######################################################################
#
# Top level Makefile for all "robot" modules.
# $Id: Makefile,v 1.3 2006/06/14 11:32:26 ex30 Exp $

MODULES= robotinc commandreader polyline xfigreader odometry motioncontrol irsensor \
 linesensor ac3dreader poseio general smrsdl regul interpreter server  \
 statemachine wallest  blockingdetect xml hako gps filter smrsim planners mrc


######################################################################
#
# Main target. Compiles all the specified modules.

all:
	for module in $(MODULES); do\
		make --silent -C $$module;\
	done

clean:
	for module in $(MODULES); do\
		make -C $$module clean;\
	done;\
	rm -f robotinc/????*.h
	rm -f *~ *.o *.d core core.* *.a
