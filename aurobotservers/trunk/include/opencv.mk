# addition to makefiles, to resolve differentversions of opencv
# the following is supported
# opencv version 2.1 - of neither 2.3.1 nor the general version 2 from ubuntu 12.04 LTS
# opencv version 2.3.1 - if includefiles are in /usr/include/opencv-2.3.1
# opencv version 2.3.1 in ubuntu 12.04 - if the includefiles are in /usr/include/opencv2
ifeq ($(OPENCV),)
  OCV2   := $(shell ls -d /usr/include/opencv2      2>/dev/null)
  OCV231 := $(shell ls -d /usr/include/opencv-2.3.1 2>/dev/null)
  ifeq ($(OCV2),)
    ifeq ($(OCV231),)
      export OPENCV:=2.1
    else
      export OPENCV:=2.3.1
    endif
  else
    export OPENCV:=2
    CXXFLAGS += -D OPENCV2
  endif
endif
