#ifndef SPRING_H
#define SPRING_H

class Spring {

	private:
		int a; // starting index in state of pt a
		int b; // starting index in state of pt b
		float ks;
		float kd;

	public:
		Spring(int, int);
};

#endif