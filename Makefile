CC = g++
EIGEN_PATH = "/c/eigen/eigen/Eigen"

all: feathers

feathers: spring.o io.o
	$(CC) -I $(EIGEN_PATH) spring.o io.o -o feathers feathers.cpp

spring.o: spring.cpp
	$(CC) -I $(EIGEN_PATH) -c spring.cpp

io.o: io.cpp
	$(CC) -I $(EIGEN_PATH) -c io.cpp

clean:
	rm -rf *o feathers