#ifndef INTERPOLATION_H_
#define INTERPOLATION_H_

#include <fstream>

#include "base_generator.h"
#include "utils.h"

// Polynomial class of walking pattern generator for humanoids,
// cf. LAAS-UHEI walking report.
//
// Polynomial class provides basic mathematic tools for interpolation
//
// Code based on the python implemenation of Manuel Kudruss.
// C++ implementation by Martin Huber.
class Polynomial
{
public:
    Polynomial(const int degree);

    double Compute(double& time);

    double ComputeDerivative(double& time);

    double ComputeSecondDerivative(double& time);

protected:
    int degree_;
    Eigen::VectorXd coef_;

    // Final time.
    double ft_;
};

// Polynomial4 class of walking pattern generator for humanoids,
// cf. LAAS-UHEI walking report.
//
// Polynomial4 class provides all methods to interpolate from a point
// A to a point B with certain velocity an acceleration with a 
// 4th degree polynomial.
//
// Code based on the python implemenation of Manuel Kudruss.
class Polynomial4 : public Polynomial
{
public:
    Polynomial4();

    void SetParameters(double final_time, double middle_position, 
                       double initial_position, double initial_speed);
};

class Polynomial5 : public Polynomial
{
public:
    Polynomial5();

    void SetParameters(double final_time, double final_position,
                       double initial_position, double initial_speed, double initial_acceleration);
};

// LIPM class of walking pattern generator for humanoids,
// cf. LAAS-UHEI walking report.
//
// LIPM class provides all methods to interpolate the CoM
// and the ZMP from the pattern generator, according to the
// linearized inverted pendulum (Kajita 2003, Walking).
//
// Code based on the python implemenation of Manuel Kudruss.
class LIPM
{
public:
    LIPM(double control_period = 0.1, double command_period = 0.005, double h_com = 0.46);

    void Interpolate(const double dddc_k_x_0, const double dddc_k_y_0,
                     ComState& cur_com, std::vector<ComState>& com_buffer, std::vector<ZmpState>& zmp_buffer);

private:
    void InitializeSystem();

    // Define some constants.
    const double g_ = 9.81;

    const double tc_;
    const double t_;
    const double h_com_;

    // For t_ sampling interpolation.
    Eigen::Matrix3d a_;
    Eigen::Vector3d b_;
    Eigen::Vector3d c_;

    // For tc_ sampling interpolation.
    Eigen::Matrix3d ac_;
    Eigen::Vector3d bc_;

    const int interval_;    
};

// FootInterpolation class of walking pattern generator for
// humanoids, cf. LAAS-UHEI walking report.
//
// FootInterpolation class provides all methods to interpolate
// from the solution of the pattern generator. It interpolates
// the feet trajectory during the QP period.
//
// Code based on the python implemenation of Manuel Kudruss.
class FootInterpolation
{
public:
    FootInterpolation(const BaseGenerator& base_generator, double qp_sampling_period = 0.1, int nb_sampling_previewed = 16,
                      double command_period = 0.005,
                      double feet_distance = 0.14, double step_height = 0.03, double step_time = 0.8, double double_support_time = 0.1);

    void Interpolate(double time, const BaseTypeSupportFoot& current_support, 
                     BaseTypeFoot& cur_left, BaseTypeFoot& cur_right,
                     double f_k_x_0, double f_k_y_0, double f_k_q_0,
                     std::vector<BaseTypeFoot>& lf_buffer, std::vector<BaseTypeFoot>& rf_buffer);

    // Getters.
    inline const double& FeetDistance() const { return feet_distance_; };

private:
    // Compute foot states at time t.
    void ComputeXYQ(BaseTypeFoot& foot, double t);

    // QP sampling period.
    const double t_;

    // Low level control period.
    const double tc_;

    // Normal distance between both feet.
    const double feet_distance_; 

    // Standard maximal step height.
    const double step_height_;

    // Order 5 polynomial for continuity in position,
    // velocity, and acceleration.
    Polynomial5 polynomial_x_;
    Polynomial5 polynomial_y_;
    Polynomial5 polynomial_q_;

    // Order 4 polynomial for a defined middle point.
    Polynomial4 polynomial_z_;

    // Time of single support.
    const double tss_;

    // Time of double support.
    const double tds_;
    const double step_time_;

    // Number of interpolated samples.
    const int interval_;

    // Base generator.
    const BaseGenerator& base_generator_;
};

// Interpolation class of walking pattern generator for humoids,
// cf. LAAS-UHEI walking report.
//
// Interpolation class provides all methods to interpolate from
// from the solution of the pattern generator. It interpolates
// the CoM, the ZMP, and the feet state along the whole trajectory
// with a given interpolation period (input).
//
// Code based on the python implemenation of Manuel Kudruss.
// C++ implementation by Martin Huber.
class Interpolation
{
public:
    Interpolation(const double tc, const BaseGenerator& base_generator);

    void Interpolate(double time);

    void SaveToFile(const std::string loc);

private:
    const BaseGenerator& base_generator_;

    // QP sampling period t_,
    // sampling period of the robot low level controller tc_,
    // number of iterions in 100 ms interval_. 
    const double t_;
    const double tc_;
    const int interval_;

    // States used to interpolate.
    ComState cur_com_;
    ZmpState zmp_;

    BaseTypeFoot cur_left_;
    BaseTypeFoot cur_right_;

    // Buffers containing the trajectories over 100 ms.
    std::vector<ComState> com_buffer_;
    std::vector<ZmpState> zmp_buffer_;
    std::vector<BaseTypeFoot> lf_buffer_;
    std::vector<BaseTypeFoot> rf_buffer_;

    // Buffers containing the full trajectories.
    std::vector<std::vector<ComState>> com_traj_;
    std::vector<std::vector<ZmpState>> zmp_traj_;
    std::vector<std::vector<BaseTypeFoot>> lf_traj_;
    std::vector<std::vector<BaseTypeFoot>> rf_traj_;

    LIPM lipm_;
    FootInterpolation fi_;
};

#endif