#ifndef NMPC_GENERATOR_H_
#define NMPC_GENERATOR_H_

#include <qpOASES.hpp>
#include "yaml-cpp/yaml.h"

#include "base_generator.h"
#include "utils.h"
#include "interpolation.h"

// Nonlinear Model Predictive Control (NMPC) Pattern Generator,
// based on the work of Maximilien Naveau et al.:
//
// 'A Reactive Walking Pattern Generator Based on Nonlinear Model
// Predictive Control'. 
//
// A full understanding on how the NMPCGenerator class is ment to be used,
// can be gained through the Example() function which is implemented in
// this class.
//
// This is an implemenation of a combined quadratic
// problem to solve the pattern generator problem for positions and 
// orientations simultaneously. 
//
// Code based on the python implementation of Manuel Kudruss.
// C++ implementation by Martin Huber.
//
// The pattern generator has to solve the following
// kind of problem in each iteration:
//
// min_x 1/2 * x.T * H(w0) * x + x.T * g(w0)
// s.t.      lbA(w0) <= A(w0) * x <= ubA(w0)
//           lb(w0)  <=         x <= ub(w0)
//
// Because of varying H and A, we have to use the
// SQProblem class, which supports this kind of QPs.
class NMPCGenerator : public BaseGenerator
{
public:
    NMPCGenerator(const std::string config_file_loc = "../../libs/pattern_generator/configs.yaml");

    void Solve();
    
    PatternGeneratorState Update();

    static void Example(const std::string config_file_loc, const std::string output_loc);

    // Getters.
    inline const qpOASES::returnValue& GetStatus() const { return status_; };

public:
    void PreprocessSolution();

    void CalculateCommonExpressions();

    void CalculateDerivatives();

    void SolveQP();

    void PostprocessSolution();

    void UpdateFootSelectionMatrix();

    void Reset();

    // qpOASES specific things.
    std::vector<double> cpu_time_;
    int nwsr_;
    qpOASES::Options options_;
    qpOASES::returnValue status_;

    // Variable dimensions.
    const int nv_;
    
    // Constraints dimensions.
    const int nc_pos_;
    const int nc_ori_;
    const int nc_;

    // Problem setup.
    Eigen::VectorXd dofs_;
    Eigen::RowVectorXd delta_dofs_;
    qpOASES::SQProblem qp_;

    // Quadratic problem.
    RowMatrixXd qp_h_;
    RowMatrixXd qp_a_;
    Eigen::RowVectorXd qp_g_;
    Eigen::RowVectorXd qp_lb_;
    Eigen::RowVectorXd qp_ub_;
    Eigen::RowVectorXd qp_lba_;
    Eigen::RowVectorXd qp_uba_;

    bool qp_is_initialized_;

    // Analyzer for solution analysis.
    qpOASES::SolutionAnalysis analyser_;

    // Helper matrices for common expressions.
    Eigen::MatrixXd q_k_x_;
    Eigen::VectorXd p_k_x_;
    Eigen::VectorXd p_k_y_;

    Eigen::MatrixXd q_k_ql_;
    Eigen::MatrixXd q_k_qr_;
    Eigen::VectorXd p_k_ql_;
    Eigen::VectorXd p_k_qr_;

    Eigen::MatrixXd a_pos_x_;
    Eigen::MatrixXd a_pos_q_;
    Eigen::VectorXd uba_pos_;
    Eigen::VectorXd lba_pos_;

    Eigen::MatrixXd a_obs_;
    Eigen::VectorXd uba_obs_;
    Eigen::VectorXd lba_obs_;

    Eigen::MatrixXd a_ori_;
    Eigen::VectorXd uba_ori_;
    Eigen::VectorXd lba_ori_;

    Eigen::MatrixXd derv_a_cop_map_;
    Eigen::MatrixXd derv_a_foot_map_;    
};

#endif
