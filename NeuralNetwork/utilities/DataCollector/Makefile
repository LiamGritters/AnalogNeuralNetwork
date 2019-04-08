### Data Generator Makefile ###

############### TARGET ###############

TARGET=DataCollector

############### COMPILER ###############

CC=g++

############### DIRECTORIES ###############

SDIR=./src
BIN=./bin
BUILD=./build
IDIR =./include

KEYPAD_DIR = /home/liam/workspace/AnalogNeuralNetwork/RobotKeypadController

############### FLAGS ###############

CFLAGS  = -std=c++11 -g -Wall -O2 -I$(IDIR) $(STANDARD_INC)
CFLAGS += -I$(KEYPAD_DIR)/include

LDFLAGS= -lopencv_imgproc -lopencv_highgui -lopencv_core

############### LIBRARIES ###############

LIBS  = -L/lib/x86_64-linux-gnu/ 
LIBS += -L/usr/local/lib

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
	rm -rf $(BUILD)/*.o 
	rm -rf $(BIN)/$(TARGET)  