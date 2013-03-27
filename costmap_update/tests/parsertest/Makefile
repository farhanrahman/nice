programname := parsertest
programsrc := $(wildcard *.cpp)
programobjs := ${programsrc:.cpp=.o}


INCLUDE_DIRS = $(ROS_WORKSPACE)/nice/costmap_update/include
LIBRARY_DIRS =
LIBRARIES =

CPPFLAGS = -O3 -g -pg -Wall -Wno-deprecated
CPPFLAGS += $(foreach includedir,$(INCLUDE_DIRS),-I$(includedir)) 
LDFLAGS += $(foreach librarydir,$(LIBRARY_DIRS),-L$(librarydir))
LDFLAGS += $(foreach library,$(LIBRARIES),-l$(library))

TARGET = $(programname)

$(TARGET): $(programobjs)
	@$(CXX) -o $(TARGET) $(programobjs) $(LDFLAGS) $(NVCC_LDFLAGS)

all: @$(TARGET) 

clean:
	@rm -f *.o $(TARGET)
