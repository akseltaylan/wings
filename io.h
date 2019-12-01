#ifndef IO_H
#define IO_H

#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <map>
#include <string>

class IO {
    public:
        IO();
        std::vector<float> read_data(const char *);
        std::map<std::string, float> read_params(const char *);
        void write_data(const char *, float *, const int &, const int &);
};

#endif