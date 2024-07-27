# Compiler and flags
CC = g++
CFLAGS = -std=c++11 -Wall

# Object files
OBJS = cpp_program.o optimise.o dubins.o

# Target executable
all: program

program: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o cpp_program

# Object files and dependencies
cpp_program.o: cpp_program.cpp optimise.h
	$(CC) $(CFLAGS) -c cpp_program.cpp

optimise.o: optimise.cpp optimise.h dubins.h
	$(CC) $(CFLAGS) -c optimise.cpp

dubins.o: dubins.c dubins.h
	$(CC) $(CFLAGS) -c dubins.c

# Clean target to remove generated files
clean:
	rm -f $(OBJS) cpp_program
