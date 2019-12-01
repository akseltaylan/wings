#include "io.h"

IO::IO() { }

std::vector<float> IO::read_data(const char * filename) {
    std::ifstream inFile;
    inFile.open(filename);
    if (!inFile) {
        std::cout << "Unable to open " << filename << std::endl;
        exit(1);
    }
    std::vector<float> data;
    float vx;
    float vy;
    float vz;
    while (inFile >> vx >> vy >> vz) {
        data.push_back(vx);
        data.push_back(vy);
        data.push_back(vz);
    }
    inFile.close();
    return data;
}

std::map<std::string, float> IO::read_params(const char * filename) {
    std::ifstream inFile;
    inFile.open(filename);
    std::map<std::string, float> params;
    std::string paramName;
    float val;
    while (inFile >> paramName >> val) {
        params.insert(std::pair<std::string, float>(paramName,val));
    }
    inFile.close();
    return params;
}

void IO::write_data(const char * filename, float * state, const int& size, const int& amtValues) {
    std::ofstream outFile;
    outFile.open(filename);
    for (int i = 0; i < size; i += amtValues) {
        outFile << state[i] << " " << state[i+1] << " " << state[i+2] << "\n";
    }
    outFile.close();
}
