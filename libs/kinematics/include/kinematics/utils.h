#ifndef KINEMATICS_UTILS_H_
#define KINEMATICS_UTILS_H_

#include <fstream>
#include <Eigen/Core>

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
