#ifndef MPC_GENERATOR_H_
#define MPC_GENERATOR_H_

#include <qpOASES.hpp>
#include "yaml-cpp/yaml.h"

#include "base_generator.h"
#include "utils.h"
#include "interpolation.h"

// This is a reimplementation of the algorithms presented in:
//
// 'Walking without thinking about it', Herdt, A. et al., 2010.
//
// A full understanding on how the MPCGenerator class is ment to be used,
// can be gained through the Example() function which is implemented in
// this class.
//
// Code based on the python implementation of Manuel Kudruss.
// C++ implementation by Martin Huber.
//
// Solve quadratic problems for position and orientation of CoM
// and feet independent of each other in every time step. First
// solve for orientations, then solve for the positions.
// 
// The pattern generator has to solve the following kind of problem
// in each iteration:
//
// min_x 1/2 * x.T * H(w0) * x + x.T * g(w0)
// s.t.      lbA(w0) <= A(w0) * x <= ubA(w0)
//           lb(w0)  <=         x <= ub(w0)
//
// Because of varying H and A, we have to use the
// SQProblem class, which supports this kind of QPs.
class MPCGenerator : public BaseGenerator
{
public:
    MPCGenerator(const std::string config_file_loc = "../libs/pattern_generator/configs.yaml");

    void Solve();

    static void Example(const std::string config_file_loc, const std::string output_loc);

public:
    void PreprocessSolution();

    void UpdateOriQ();

    void UpdateOriP();

    void UpdatePosQ();

    void UpdatePosP(const std::string);

    void SolveQP();

    void PostprocessSolution();

    // qpOASES specific things.
    std::vector<double> cpu_time_;
    int nwsr_;
    qpOASES::Options options_;
    qpOASES::returnValue status_ori_;
    qpOASES::returnValue status_pos_;

    // Constraint dimensions.
    const int ori_nv_;
    const int ori_nc_;
    const int pos_nv_;
    const int pos_nc_;

    // Problem setup for orientation.
    Eigen::RowVectorXd ori_dofs_;
    qpOASES::SQProblem ori_qp_;
    
    RowMatrixXd ori_h_;
    RowMatrixXd ori_a_;
    Eigen::RowVectorXd ori_g_;
    Eigen::RowVectorXd ori_lb_;
    Eigen::RowVectorXd ori_ub_;
    Eigen::RowVectorXd ori_lba_;
    Eigen::RowVectorXd ori_uba_;

    bool ori_qp_is_initialized_;

    // Problem setup for position.
    Eigen::RowVectorXd pos_dofs_;
    qpOASES::SQProblem pos_qp_;
    
    RowMatrixXd pos_h_;
    RowMatrixXd pos_a_;
    Eigen::RowVectorXd pos_g_;
    Eigen::RowVectorXd pos_lb_;
    Eigen::RowVectorXd pos_ub_;
    Eigen::RowVectorXd pos_lba_;
    Eigen::RowVectorXd pos_uba_;

    bool pos_qp_is_initialized_;

    // Dummy matrices.
    Eigen::MatrixXd ori_q_;
    Eigen::VectorXd ori_p_;
    Eigen::MatrixXd pos_q_;
    Eigen::VectorXd pos_p_;
};

#endif
