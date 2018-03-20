// helper.h holds some simple structures which
// are commonly used within this project.
//
// Code based on the python implementation of Manuel Kudruss.
// C++ implementation by Martin Huber.

#ifndef UTILS_H_
#define UTILS_H_

#include <string>
#include <fstream>
#include <Eigen/Dense>

// Typedefs.
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RowMatrixXd;

// Structs.
struct BaseTypeSupportFoot {
    double x;
    double y;
    double q;
    std::string foot = "left";
    double ds = 0.;
    int step_number = 0;
    double time_limit = 0.;

    // Overload boolean operators.
    inline bool operator==(const BaseTypeSupportFoot& rhs){
        return std::tie(x, y, q, foot, ds, step_number, time_limit) ==
               std::tie(rhs.x, rhs.y, rhs.q, rhs.foot, rhs.ds, rhs.step_number, rhs.time_limit);
    }

    inline bool operator!=(const BaseTypeSupportFoot& rhs){
        return std::tie(x, y, q, foot, ds, step_number, time_limit) !=
               std::tie(rhs.x, rhs.y, rhs.q, rhs.foot, rhs.ds, rhs.step_number, rhs.time_limit);
    }
};

struct BaseTypeFoot {
    double x = 0.;
    double y = 0.;
    double z = 0.;
    double q = 0.;

    double dx = 0.;
    double dy = 0.;
    double dz = 0.;
    double dq = 0.;

    double ddx = 0.;
    double ddy = 0.;
    double ddz = 0.;
    double ddq = 0.;

    int support_foot = 0;
};

struct ComState {
    Eigen::Vector3d x;
    Eigen::Vector3d y;
    double z = 0.46;
    Eigen::Vector3d q;
};

struct ZmpState {
    double x = 0.;
    double y = 0.;
    double z = 0.;
};

struct PatternGeneratorState {
    Eigen::Vector3d com_x;
    Eigen::Vector3d com_y;
    double com_z;
    double foot_x;
    double foot_y;
    double foot_q;
    std::string foot = "left";
    Eigen::Vector3d com_q;
};

struct Circle {
    double x0     = 0.;
    double y0     = 0.;
    double r      = 1.;
    double margin = 0.;
};

// Load data from .csv to Eigen matrix.
template<typename M>
M ReadCsv(const std::string& path) {
    std::ifstream indata;
    indata.open(path);
    std::string line;
    std::vector<double> values;
    uint rows = 0;
    while (std::getline(indata, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, ',')) {
            values.push_back(std::stod(cell));
        }
        ++rows;
    }
    return Eigen::Map<const Eigen::Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, Eigen::RowMajor>>(values.data(), rows, values.size()/rows);
}

// Save data from Eigen matrix to .csv file.
const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

template<typename M> 
void WriteCsv(const std::string& path, M matrix)
{
    std::ofstream file(path.c_str());
    file << matrix.format(CSVFormat);
};

#endif