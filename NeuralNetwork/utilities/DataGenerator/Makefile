### Data Generator Makefile ###

############### COMPILER ###############

TARGET=DataGenerator

############### COMPILER ###############

CC=g++

############### DIRECTORIES ###############

SDIR=./src
BIN=./bin
BUILD=./build
IDIR =./include

############### FLAGS ###############

CFLAGS  = -std=c++11 -g -Wall -O2 -I$(IDIR) $(STANDARD_INC) -DCPU_ONLY 
CFLAGS += -I/home/liam/caffe/include
CFLAGS += -I/home/liam/caffe/build/include
CFLAGS += -I/usr/include/hdf5/serial

LDFLAGS= -lgflags -lglog -lopencv_imgproc -lopencv_highgui -lopencv_core -lprotobuf -lcaffe

############### LIBRARIES ###############

LIBS  = -L/lib/x86_64-linux-gnu/ 
LIBS += -L/usr/local/lib
LIBS += -L/home/liam/caffe/build/lib

############### OBJECTS ###############

DEPS = $(patsubst %.hpp,$(IDIR)/%.d,$(wildcard *.hpp))
OBJ = $(patsubst $(SDIR)/%.cpp, $(BUILD)/%.o, $(wildcard $(SDIR)/*.cpp))


############### BUILD ###############

$(BUILD)/%.o: $(SDIR)/%.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

$(BIN)/$(TARGET): $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS) $(LDFLAGS)

############### CLEAN ###############

.PHONY: clean

clean:
	rm -f $(BUILD)/*.o 
	rm -f $(BIN)/$(TARGET)  