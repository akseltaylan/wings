#include "spring.h"

Spring::Spring(int _a, int _b) {
    a = _a;
    b = _b;
    ks = 0.5f;
    kd = 0.5f;
}

Spring::Spring(int _a, int _b,
               float _ks, float _kd,
               std::string _type) {
    a = _a;
    b = _b;
    ks = _ks;
    kd = _kd;
    type = _type;
}