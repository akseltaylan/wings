#include "spring.h"

Spring::Spring(int _a, int _b) {
    a = _a;
    b = _b;
    ks = 0.5f;
    kd = 0.5f;
}