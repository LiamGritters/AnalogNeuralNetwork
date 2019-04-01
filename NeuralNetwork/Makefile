### Analog Robot Keypad Controller Makefile ###

############### COMPILER ###############

TARGET=NeuralNetworkController

############### COMPILER ###############

CC=g++

############### DIRECTORIES ###############

SDIR=./src
BIN=./bin
BUILD=./build
IDIR =./include

############### FLAGS ###############

CFLAGS  = -std=c++11 -g -Wall -O2 -I$(IDIR) -DCPU_ONLY 
CFLAGS += -I/home/liam/workspace/AnalogNeuralNetwork/LCMMessages/ANNLCM
CFLAGS += -I/usr/local
CFLAGS += -I/usr/include/hdf5/serial/
CFLAGS += -I/home/liam/caffe/include
CFLAGS += -I/home/liam/workspace/AnalogNeuralNetwork/RobotKeypadController/include

LDFLAGS= -llcm -lpthread -lcaffe

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
	rm -rf $(BUILD)/* 
	rm -rf $(BIN)/*  