CC = g++
CFLAGS = -std=c++11 -Wall

OBJS = cpp_program.o optimise.o

all: program

program: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o cpp_program

main.o: cpp_program.cpp optimise.h
	$(CC) $(CFLAGS) -c cpp_program.cpp

optimise.o: optimise.cpp optimise.h
	$(CC) $(CFLAGS) -c optimise.cpp

clean:
	rm -f $(OBJS) cpp_program