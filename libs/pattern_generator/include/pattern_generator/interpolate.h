#ifndef TESTINTERPOLATE_H_
#define TESTINTERPOLATE_H_

#include <Eigen/Dense>
#include "yaml-cpp/yaml.h"

#include "base_generator.h"

// Interpolation class of pattern generator for humanoids,
// cf. LAAS-UHEI walking report.
//
// Interpolation class provides all methods to interpolate from
// the solution of the pattern generator. It interpolates
// the CoM, the ZMP, and the feet state along the whole trajectory
// with a given interpolation period (input).
//
// Written by Martin Huber.
class TESTInterpolation
{
public:
    TESTInterpolation(BaseGenerator& base_generator);

    void TESTInterpolate();

    // Getters.
    inline const Eigen::MatrixXd& Trajectories() const { return trajectories_; };

public:
    void TESTInitializeLIPM();

    void TESTInitializeTrajectories();

    void TESTInterpolateFeet();

    void TESTInterpolateLIPM();

    // TODO
    // void TESTInitializePolynomial();

    // void TESTDerivative();

    // Base generator.
    const BaseGenerator& base_generator_;

    // Constants.
    const double g_;

    // Preview control period t_, and command period tc_.
    const double t_;
    const double tc_;

    // Center of mass initial values.
    const double h_com_;

    // Number of interpolation intervals.
    const int intervals_;

    // Number of intervals that the robot stays still in the beginning.
    const int n_still_;

    // Interpolated trajectories.
    Eigen::MatrixXd trajectories_;
    Eigen::MatrixXd trajectories_buffer_;

    // Center of mass.
    Eigen::Ref<Eigen::MatrixXd> com_x_buffer_;
    Eigen::Ref<Eigen::MatrixXd> com_y_buffer_;
    Eigen::Ref<Eigen::MatrixXd> com_z_buffer_;
    Eigen::Ref<Eigen::MatrixXd> com_q_buffer_;

    // Zero moment point.
    Eigen::Ref<Eigen::MatrixXd> zmp_x_buffer_;
    Eigen::Ref<Eigen::MatrixXd> zmp_y_buffer_;
    Eigen::Ref<Eigen::MatrixXd> zmp_z_buffer_;

    // Left foot.
    Eigen::Ref<Eigen::MatrixXd> lf_x_buffer_;
    Eigen::Ref<Eigen::MatrixXd> lf_y_buffer_;
    Eigen::Ref<Eigen::MatrixXd> lf_z_buffer_;
    Eigen::Ref<Eigen::MatrixXd> lf_q_buffer_;

    // Right foot.
    Eigen::Ref<Eigen::MatrixXd> rf_x_buffer_;
    Eigen::Ref<Eigen::MatrixXd> rf_y_buffer_;
    Eigen::Ref<Eigen::MatrixXd> rf_z_buffer_;
    Eigen::Ref<Eigen::MatrixXd> rf_q_buffer_;

    // Linear inverted pendulum.
    Eigen::Matrix3d a_;
    Eigen::Vector3d b_;
    Eigen::Vector3d c_;

    Eigen::Matrix3d ac_;
    Eigen::Vector3d bc_;
};

#endif
