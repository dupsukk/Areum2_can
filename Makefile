CXX = g++
CXXFLAGS = -Iinc -Wall -Wextra -std=c++20 -O2 -MMD -MP
LDFLAGS = -lpthread

TARGET = Areum2_can

# Source and Object files
SRC_DIR = src
INC_DIR = inc
SRCS = $(wildcard $(SRC_DIR)/*.cpp)
OBJS = $(SRCS:.cpp=.o)
DEPS = $(OBJS:.o=.d)

# 코드 생성: robot.yaml / gen_robot.py 가 바뀌면 main.cpp 와 {robot_name}_config.h 를 재생성한다.
GEN_STAMP = .gen_stamp
$(GEN_STAMP): robot.yaml gen_robot.py
	python3 gen_robot.py robot.yaml --inc $(INC_DIR) --src $(SRC_DIR)
	@touch $@

# 생성물을 쓰는 main 은 생성 이후에 컴파일되어야 한다
$(SRC_DIR)/main.o: $(GEN_STAMP)

# Default target
all: $(TARGET)
	@chmod +x start.sh

# Link the executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

# Compile source files to object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Load auto-generated header dependencies
-include $(DEPS)

# Clean up
clean:
	rm -f $(SRC_DIR)/*.o $(SRC_DIR)/*.d $(TARGET) $(GEN_STAMP)

.PHONY: all clean
