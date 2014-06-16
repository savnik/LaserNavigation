#CC=gcc-3.3
CC=gcc
#CFLAGS=-c -Wall -march=i586 -m3dnow -O2 -pipe -fomit-frame-pointer -mmmx
CFLAGS=-c -Wall -m3dnow -O2 -pipe -fomit-frame-pointer -mmmx
# LDFLAGS= -L/vhome/naa/robot2005/src/ 
LIBS= -lpthread -lm -lexpat 
# -lrobot 
SOURCES=client.c functions.c server.c turn_fishtail.c turn_omega.c \
        turn_wide.c turn_arc.c turn_gps_wide.c turn_gps_fishtail.c \
        turn_gps_link_lanes.c turn_support_lib_link_test.c \
        turn_support_lib_link_rr_test.c turn_support_lib_link_ll_test.c \
        turn_support_lib_link_rl_test.c turn_support_lib_link_lr_test.c \
        turn_support_lib_norm_angle.c vector_lib.c logtime.c \
        drivetowards.c command_store.c stop_tractor.c main.c \
        stream2line.c
# missionparts.c
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=hakoclient

all: $(SOURCES) $(EXECUTABLE)
clean: 
	rm -f *~ *.o *.d $(EXECUTABLE) core core.* *.bak log.txt
	
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $@

.c.o:
	$(CC) $(CFLAGS) $< -o $@

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
