# Makefile include file for opencv version control
# configures opencv flags
#
ifeq ($(OPENCV),2.1)
  export OPENCV=
endif
ifeq ($(OPENCV),)
  # for opencv version 2.1
  LDFLAGS += -L/usr/local/lib -lcxcore -lcv
  CXXFLAGS += -I/usr/include/opencv
else
  ifeq ($(OPENCV),2)
    # for opencv version >= 2.3.1 when installed on ubuntu version 12.4
    LDFLAGS  += `pkg-config opencv --libs`
    CXXFLAGS += `pkg-config opencv --cflags`
    CPPFLAGS += -I/usr/include/opencv2
  else
    # for opencv version 2.3.1 when installed on ubuntu version 11.10 or earlier
    LDFLAGS  += `pkg-config opencv-$(OPENCV) --libs`
    CXXFLAGS += `pkg-config opencv-$(OPENCV) --cflags`
  endif
endif
