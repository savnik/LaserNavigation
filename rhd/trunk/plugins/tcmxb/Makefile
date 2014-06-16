######################################################################
#
# Makefile for RHD Plugins
#
# $Id: Makefile 59 2012-10-21 06:25:02Z jcan $
######################################################################

########## Configuration  ##########
CC = g++
CFLAGS := -O2 -Wall -fPIC
LIBS = -lpthread -lexpat -lm
#RHD Headers to include
INC  = -I../../include -I../../src
#Flags to link into shared lib
LDFLAGS := -shared -lc -Wl,-soname,

#Plugins path
PLUGINSPATH := ../../build/lib/rhdplugin


########## Source files ##########
#Plugin-source files
SRC := CommProtocol.cpp TCM.cpp
HEAD := TCMXB.h CommProtocol.h TCM.h TickGenerator.h Processes.h 

#Plugin filename
#PLUGIN := libtcmxb.so.1

#Test executeable
EXEC := tcm

#Create list of objects
OBJ=$(SRC:.c=.o)

#default rules
all : plugin

#Make the dynamic plugin library
plugin: #$(OBJ)
	$(CC) $(LDFLAGS) $(OBJ) -o $(EXEC) $(OBJ) $(LIBS)
	#$(CC) $(LDFLAGS) $(PLUGIN) -o $(PLUGIN) $(OBJ) $(LIBS) 
	#mkdir -p $(PLUGINSPATH)
	#cp -f $(PLUGIN) $(PLUGINSPATH)/$(PLUGIN)

# Clean plugin
clean:
	#rm -f *~ *.o *.d $(PLUGIN)


###### .c to .o #####
%.o: %.c
	$(CC) $(CFLAGS) $(INC) -c $<
