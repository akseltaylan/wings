#ifndef SPRING_H
#define SPRING_H

#include <string>
#include <math.h>
#include <Eigen>

enum SpringType { EDGE, BEND };

class Spring {

	private:
		int a; // starting index in state of pt a
		int b; // starting index in state of pt b
		float ks;
		float kd;
		float restLength;
		SpringType type;

	public:
		Spring(int, int);
		Spring(int, int, float, float, float, SpringType);
		int get_a_idx();
		int get_b_idx();
		void solve_spring(float *, float, float);
		
};

#endif