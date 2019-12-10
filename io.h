#ifndef IO_H
#define IO_H

#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <map>
#include <string>
#include <Eigen>

struct HoudiniVertData {
    Eigen::Vector3f pos;
    char pt_type;
    HoudiniVertData(float vx, float vy, float vz, char c) {
        pos = Eigen::Vector3f(vx, vy, vz);
        pt_type = c;
    }
};

class IO {
    public:
        IO();
        std::vector<HoudiniVertData> read_data(const char *);
        void write_data(const char *, float *, const int &, const int &);
};

#endif