#ifndef TESTINTERPOLATE_H_
#define TESTINTERPOLATE_H_

#include <Eigen/Dense>
#include <unsupported/Eigen/Polynomials>
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

    void Interpolate();

    // Getters.
    inline const Eigen::MatrixXd& Trajectories() const { return trajectories_; };

public:
    template <typename Derived>
    void Derivative(const Eigen::MatrixBase<Derived>& coef, Eigen::MatrixBase<Derived>& dcoef);

    void EvaluateInterpolationXYQ(double& f_x, double& f_dx, double& f_ddx, 
                                  double& f_y, double& f_dy, double& f_ddy,
                                  double& f_q, double& f_dq, double& f_ddq, 
                                  double time);

    void InitializeLIPM();

    void InitializeTrajectories();

    void InterpolateFeet();

    void InterpolateLIPM();

    template <typename Derived>
    void Set4thOrderCoefficients(Eigen::MatrixBase<Derived>& coef,
                                 double final_time, double middle_pos,
                                 double init_pos, double init_vel);

    template <typename Derived>
    void Set5thOrderCoefficients(Eigen::MatrixBase<Derived>& coef,
                                 double final_time, double final_pos,
                                 double init_pos, double init_vel, double init_acc);


    // Base generator.
    const BaseGenerator& base_generator_;

    // Constants.
    const double g_;

    // Preview control period t_, and command period tc_.
    const double t_;
    const double tc_;

    // Double and single support time.
    const double t_ds_;
    const double t_ss_;

    // Center of mass initial values.
    const double h_com_;

    // Number of interpolation intervals.
    const int intervals_;

    // Number of intervals that the robot stays still in the beginning.
    const int n_still_;

    // Step height while walking.
    const double step_height_;

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

    Eigen::MatrixXd lf_dx_buffer_;
    Eigen::MatrixXd lf_dy_buffer_;
    Eigen::MatrixXd lf_dz_buffer_;
    Eigen::MatrixXd lf_dq_buffer_;

    Eigen::MatrixXd lf_ddx_buffer_;
    Eigen::MatrixXd lf_ddy_buffer_;
    Eigen::MatrixXd lf_ddz_buffer_;
    Eigen::MatrixXd lf_ddq_buffer_;

    // Right foot.
    Eigen::Ref<Eigen::MatrixXd> rf_x_buffer_;
    Eigen::Ref<Eigen::MatrixXd> rf_y_buffer_;
    Eigen::Ref<Eigen::MatrixXd> rf_z_buffer_;
    Eigen::Ref<Eigen::MatrixXd> rf_q_buffer_;

    Eigen::MatrixXd rf_dx_buffer_;
    Eigen::MatrixXd rf_dy_buffer_;
    Eigen::MatrixXd rf_dz_buffer_;
    Eigen::MatrixXd rf_dq_buffer_;

    Eigen::MatrixXd rf_ddx_buffer_;
    Eigen::MatrixXd rf_ddy_buffer_;
    Eigen::MatrixXd rf_ddz_buffer_;
    Eigen::MatrixXd rf_ddq_buffer_;

    // Foot interpolation coefficients for position, velocity, and acceleration.
    Eigen::VectorXd f_coef_x_;
    Eigen::VectorXd f_coef_y_;
    Eigen::VectorXd f_coef_z_;
    Eigen::VectorXd f_coef_q_;

    Eigen::VectorXd f_coef_dx_;
    Eigen::VectorXd f_coef_dy_;
    Eigen::VectorXd f_coef_dz_;
    Eigen::VectorXd f_coef_dq_;
        
    Eigen::VectorXd f_coef_ddx_;
    Eigen::VectorXd f_coef_ddy_;
    Eigen::VectorXd f_coef_ddz_;
    Eigen::VectorXd f_coef_ddq_; 

    // Linear inverted pendulum.
    Eigen::Matrix3d a_;
    Eigen::Vector3d b_;
    Eigen::Vector3d c_;

    Eigen::Matrix3d ac_;
    Eigen::Vector3d bc_;
};

#endif
