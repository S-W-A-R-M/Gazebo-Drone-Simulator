# Compiler and flags
CXX = g++
CXXFLAGS = -Wall -std=c++17

# Target executable name
TARGET = my_program

# All .cpp files in the current directory
SRCS = $(wildcard *.cpp)

# Convert .cpp filenames to .o (object) filenames
OBJS = $(SRCS:.cpp=.o)

# Default rule to build and then run the program
all: $(TARGET)
	./$(TARGET)

# Link object files to create the final executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS)

# Compile each .cpp file into a .o file
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up build files
clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean