# Set the path to the distingNT_API submodule located in the current project
NT_API_PATH := ./distingNT_API

# Set the include path for the API headers
INCLUDE_PATH := $(NT_API_PATH)/include

# Find all .cpp files in the current directory (i.e., MrFreezeNT.cpp)
inputs := $(wildcard *.cpp)

# Define the output object files to be placed in a 'plugins' subdirectory
outputs := $(patsubst %.cpp,plugins/%.o,$(inputs))

# Default target: build all found .cpp files
all: $(outputs)

# Target to remove compiled files
clean:
	rm -f $(outputs)

# The rule for compiling .cpp files into .o object files
plugins/%.o: %.cpp
	if not exist $(@D) mkdir $(@D)
	arm-none-eabi-c++ -std=c++11 -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -fno-rtti -fno-exceptions -Os -fPIC -Wall -I$(INCLUDE_PATH) -c -o $@ $^
