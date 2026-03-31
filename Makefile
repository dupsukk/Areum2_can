CXX = g++
CXXFLAGS = -Iinc -Wall -Wextra -std=c++20 -O2 
LDFLAGS = 

TARGET = Areum2_can

# Source and Object files
SRC_DIR = src
INC_DIR = inc
SRCS = $(wildcard $(SRC_DIR)/*.cpp)
OBJS = $(SRCS:.cpp=.o)

# Default target
all: $(TARGET)
	@chmod +x start.sh  

# Link the executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

# Compile source files to object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up
clean:
	rm -f $(SRC_DIR)/*.o $(TARGET)

.PHONY: all clean
