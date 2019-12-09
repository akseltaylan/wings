#ifndef SPRING_H
#define SPRING_H

#include <string>

class Spring {

	private:
		int a; // starting index in state of pt a
		int b; // starting index in state of pt b
		float ks;
		float kd;
		std::string type;

	public:
		Spring(int, int);
		Spring(int, int, float, float, std::string);
};

#endif