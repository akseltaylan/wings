#include "spring.h"

Spring::Spring(int _a, int _b) {
    a = _a;
    b = _b;
    ks = 0.5f;
    kd = 0.5f;
}

Spring::Spring(int _a, int _b,
               float _ks, float _kd, float _restLength,
               SpringType _type) {
    a = _a;
    b = _b;
    ks = _ks;
    kd = _kd;
    restLength = _restLength;
    type = _type;
}

void Spring::solve_spring(float * currentState, float m, float amp) {
    // create eigen vectors from state for easy vector math
    Eigen::Vector3f aPos = Eigen::Vector3f(currentState[a],
                                           currentState[a+1],
                                           currentState[a+2]);
    Eigen::Vector3f aVel = Eigen::Vector3f(currentState[a+3],
                                           currentState[a+4],
                                           currentState[a+5]);
    Eigen::Vector3f bPos = Eigen::Vector3f(currentState[b],
                                           currentState[b+1],
                                           currentState[b+2]);
    Eigen::Vector3f bVel = Eigen::Vector3f(currentState[b+3],
                                           currentState[b+4],
                                           currentState[b+5]);

    // calculate force on two points of the spring
    Eigen::Vector3f l = aPos - bPos;
    Eigen::Vector3f l_d = aVel - bVel;
    float left_component = ks * l.norm() - restLength;
    float right_component = kd * (l_d.dot(l) / l.norm());
    float component = -1.0f * (left_component + right_component);
    Eigen::Vector3f fa = component * l.normalized() * m * amp;
    fa = Eigen::Vector3f(isnan(fa.x()) ? 0.0f : fa.x(),
                         isnan(fa.y()) ? 0.0f : fa.y(),
                         isnan(fa.z()) ? 0.0f : fa.z());
    Eigen::Vector3f fb = -1.0f * fa;
    
    // apply force to points in the state array
    if (currentState[a+9] != 1.0f) {
        currentState[a+6] += fa.x(); currentState[a+7] += fa.y(); currentState[a+8] += fa.z();
    }
    if (currentState[b+9] != 1.0f) {
        currentState[b+6] += fb.x(); currentState[b+7] += fb.y(); currentState[b+8] += fb.z();
    }
}

int Spring::get_a_idx() { return a; }
int Spring::get_b_idx() { return b; }