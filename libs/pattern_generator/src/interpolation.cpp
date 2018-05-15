#include "interpolation.h"

#include <iostream>

Interpolation::Interpolation(BaseGenerator& base_generator)
    : // Base generator.
      base_generator_(base_generator),

      // Constants.
      g_(base_generator.configs_["gravity"].as<double>()),

      // Preview control period t_, and command period tc_.
      t_(base_generator.T()),
      tc_(base_generator.configs_["command_period"].as<double>()),

      // Double support time.
      t_ds_(base_generator_.configs_["t_ds"].as<double>()),
      t_ss_(base_generator_.TStep() - t_ds_),

      // Center of mass initial values.
      h_com_(base_generator_.Hcom()),
  
      // Number of interpolation intervals.
      intervals_(int(t_/tc_)),
      
      // Number of intervals that the robot stays still in the beginning.
      n_still_(base_generator.configs_["n_still"].as<int>()),

      // Step height while walking.
      step_height_(base_generator_.configs_["step_height"].as<double>()),

      // Interpolated trajectories.
      trajectories_(21, n_still_*intervals_),
      trajectories_buffer_(21, intervals_ + 1),

      // Don't store trajectories by default.
      store_trajectories_(false),

      // Center of mass.
      com_x_buffer_(trajectories_buffer_.block(0, 0, 3, intervals_)),
      com_y_buffer_(trajectories_buffer_.block(3, 0, 3, intervals_)),
      com_z_buffer_(trajectories_buffer_.block(6, 0, 1, intervals_)),
      com_q_buffer_(trajectories_buffer_.block(7, 0, 3, intervals_)),
  
      // Zero moment point.
      zmp_x_buffer_(trajectories_buffer_.block(10, 0, 1, intervals_)),
      zmp_y_buffer_(trajectories_buffer_.block(11, 0, 1, intervals_)),
      zmp_z_buffer_(trajectories_buffer_.block(12, 0, 1, intervals_)),
  
      // Left foot.
      lf_x_buffer_(trajectories_buffer_.block(13, 0, 1, intervals_ + 1)),
      lf_y_buffer_(trajectories_buffer_.block(14, 0, 1, intervals_ + 1)),
      lf_z_buffer_(trajectories_buffer_.block(15, 0, 1, intervals_ + 1)),
      lf_q_buffer_(trajectories_buffer_.block(16, 0, 1, intervals_ + 1)),
      
      lf_dx_buffer_(1, intervals_ + 1),
      lf_dy_buffer_(1, intervals_ + 1),
      lf_dz_buffer_(1, intervals_ + 1),
      lf_dq_buffer_(1, intervals_ + 1),
    
      lf_ddx_buffer_(1, intervals_ + 1),
      lf_ddy_buffer_(1, intervals_ + 1),
      lf_ddz_buffer_(1, intervals_ + 1),
      lf_ddq_buffer_(1, intervals_ + 1),
    
      // Right foot.
      rf_x_buffer_(trajectories_buffer_.block(17, 0, 1, intervals_ + 1)),
      rf_y_buffer_(trajectories_buffer_.block(18, 0, 1, intervals_ + 1)),
      rf_z_buffer_(trajectories_buffer_.block(19, 0, 1, intervals_ + 1)),
      rf_q_buffer_(trajectories_buffer_.block(20, 0, 1, intervals_ + 1)),
      
      rf_dx_buffer_(1, intervals_ + 1),
      rf_dy_buffer_(1, intervals_ + 1),
      rf_dz_buffer_(1, intervals_ + 1),
      rf_dq_buffer_(1, intervals_ + 1),
    
      rf_ddx_buffer_(1, intervals_ + 1),
      rf_ddy_buffer_(1, intervals_ + 1),
      rf_ddz_buffer_(1, intervals_ + 1),
      rf_ddq_buffer_(1, intervals_ + 1),

      // Foot interpolation coefficients.
      f_coef_x_(6),
      f_coef_y_(6),
      f_coef_z_(5),
      f_coef_q_(6),
      
      f_coef_dx_(f_coef_x_.size() - 1),
      f_coef_dy_(f_coef_y_.size() - 1),
      f_coef_dz_(f_coef_z_.size() - 1),
      f_coef_dq_(f_coef_q_.size() - 1),
      
      f_coef_ddx_(f_coef_dx_.size() - 1),
      f_coef_ddy_(f_coef_dy_.size() - 1),
      f_coef_ddz_(f_coef_dz_.size() - 1),
      f_coef_ddq_(f_coef_dq_.size() - 1) {

    // Interpolated trajectories.
    trajectories_.setZero();
    trajectories_buffer_.setZero();

    // Left foot.
    lf_dx_buffer_.setZero();
    lf_dy_buffer_.setZero();
    lf_dz_buffer_.setZero();
    lf_dq_buffer_.setZero();

    lf_ddx_buffer_.setZero();
    lf_ddy_buffer_.setZero();
    lf_ddz_buffer_.setZero();
    lf_ddq_buffer_.setZero();

    // Right foot.
    rf_dx_buffer_.setZero();
    rf_dy_buffer_.setZero();
    rf_dz_buffer_.setZero();
    rf_dq_buffer_.setZero();

    rf_ddx_buffer_.setZero();
    rf_ddy_buffer_.setZero();
    rf_ddz_buffer_.setZero();
    rf_ddq_buffer_.setZero();
      
    // Foot interpolation coefficients.
    f_coef_x_.setZero();
    f_coef_y_.setZero();
    f_coef_z_.setZero();
    f_coef_q_.setZero();

    f_coef_dx_.setZero();
    f_coef_dy_.setZero();
    f_coef_dz_.setZero();
    f_coef_dq_.setZero();
    
    f_coef_ddx_.setZero();
    f_coef_ddy_.setZero();
    f_coef_ddz_.setZero();
    f_coef_ddq_.setZero();

    // Initialize models.
    a_.setZero();
    b_.setZero();
    c_.setZero();

    InitializeTrajectories();
    InitializeLIPM();
}

void Interpolation::Interpolate() {
    
    // Interpolate.
    InterpolateLIPM();
    InterpolateFeet();

    // Append by buffered trajectories.
    if (store_trajectories_) {
        trajectories_.conservativeResize(trajectories_.rows(), trajectories_.cols() + intervals_);
        trajectories_.rightCols(intervals_) = trajectories_buffer_.leftCols(intervals_);
    }
}

template <typename Derived>
void Interpolation::Derivative(const Eigen::MatrixBase<Derived>& coef, Eigen::MatrixBase<Derived>& dcoef) {
    
    // Calculate the derivative of a coefficient vector.
    for (int i = 0; i < coef.rows() - 1; i++) {
        dcoef(i) = (i + 1)*coef(i + 1);
    }
}   

void Interpolation::InitializeLIPM() {

    // Taylor time approximations of the linear inverted pendulum.
    a_ << 1., tc_,  tc_*tc_*0.5,
          0.,  1.,          tc_,
          0.,  0.,           1.;

    b_ << tc_*tc_*tc_/6.,
              tc_*tc_/2.,
                     tc_;

    c_ <<         1.,
                  0.,
          -h_com_/g_;
}

void Interpolation::InitializeTrajectories() {
    
    // Initialize the standing still trajectories. Center of mass.
    com_x_buffer_.colwise() = base_generator_.Ckx0();
    com_y_buffer_.colwise() = base_generator_.Cky0();
    com_z_buffer_.setConstant(base_generator_.Hcom());
    com_q_buffer_.colwise() = base_generator_.Ckq0();

    // Zero moment point.
    zmp_x_buffer_.setConstant(base_generator_.Ckx0()(0) - base_generator_.Hcom()/g_*base_generator_.Ckx0()(2));
    zmp_y_buffer_.setConstant(base_generator_.Cky0()(0) - base_generator_.Hcom()/g_*base_generator_.Cky0()(2));

    // Feet, depending on the current support.
    if (base_generator_.CurrentSupport().foot == "left") {
        lf_x_buffer_.setConstant(base_generator_.Fkx0());
        lf_y_buffer_.setConstant(base_generator_.Fky0());
        lf_z_buffer_.setZero();
        lf_q_buffer_.setConstant(base_generator_.Fkq0());

        rf_x_buffer_.setConstant(base_generator_.Fkx0() + base_generator_.FootDistance()*sin(base_generator_.Fkq0()));
        rf_y_buffer_.setConstant(base_generator_.Fky0() - base_generator_.FootDistance()*cos(base_generator_.Fkq0()));
        rf_z_buffer_.setZero();
        rf_q_buffer_.setConstant(base_generator_.Fkq0());
    }
    else {
        lf_x_buffer_.setConstant(base_generator_.Fkx0() - base_generator_.FootDistance()*sin(base_generator_.Fkq0())); 
        lf_y_buffer_.setConstant(base_generator_.Fky0() + base_generator_.FootDistance()*cos(base_generator_.Fkq0()));
        lf_z_buffer_.setZero();
        lf_q_buffer_.setConstant(base_generator_.Fkq0());

        rf_x_buffer_.setConstant(base_generator_.Fkx0());
        rf_y_buffer_.setConstant(base_generator_.Fky0());
        rf_z_buffer_.setZero();
        rf_q_buffer_.setConstant(base_generator_.Fkq0());
    }

    // Unload the buffer.
    trajectories_ = trajectories_buffer_.leftCols(intervals_).replicate(1, n_still_);
}

void Interpolation::InterpolateFeet() {

    // Double or single support.
    if (base_generator_.TStep() - base_generator_.Vkp10().sum()*t_ < t_ds_) {

        // Double support. Stay still.
        lf_x_buffer_.setConstant(lf_x_buffer_(0, intervals_));
        lf_y_buffer_.setConstant(lf_y_buffer_(0, intervals_));
        lf_z_buffer_.setConstant(lf_z_buffer_(0, intervals_));
        lf_q_buffer_.setConstant(lf_q_buffer_(0, intervals_));

        rf_x_buffer_.setConstant(rf_x_buffer_(0, intervals_));
        rf_y_buffer_.setConstant(rf_y_buffer_(0, intervals_));
        rf_z_buffer_.setConstant(rf_z_buffer_(0, intervals_));
        rf_q_buffer_.setConstant(rf_q_buffer_(0, intervals_));

        // Define the polynomial for the regression of the
        // z movement of the feet during the double support phase
        // to allow the continous movement during the whole single
        // support phase.
        if (base_generator_.CurrentSupport().foot == "left") {

            // Right foot moving.
            Set4thOrderCoefficients(f_coef_z_, 
                                    t_ss_,
                                    step_height_, 
                                    rf_z_buffer_(0, intervals_), 
                                    rf_dz_buffer_(0, intervals_));
        }

        else {

            // Left foot moving.
            Set4thOrderCoefficients(f_coef_z_, 
                                    t_ss_,
                                    step_height_, 
                                    lf_z_buffer_(0, intervals_), 
                                    lf_dz_buffer_(0, intervals_));
        }

    }
    else {

        // Single support. Specify times that split the single support time
        // further into lift off, moving, and drop down. Lift off and drop
        // down time periods are called t_transition.
        const double t_transition = 0.05*t_ss_;

        // Time period t_moving in which the swing foot is moving.
        const double t_moving = t_ss_ - 2*t_transition;

        // Time left until the foot changes to the drop down transition.
        const double t_till_drop_down = base_generator_.Vkp10().sum()*t_ - t_transition;

        // Indicates the current time inside the single support phase.
        const double t_current = t_ss_ - base_generator_.Vkp10().sum()*t_;

        // Left or right foot.
        if (base_generator_.CurrentSupport().foot == "left") {

            // Set the coefficients for the interpolation.
            Set5thOrderCoefficients(f_coef_x_,
                                    t_till_drop_down, 
                                    base_generator_.Fkx()(0), 
                                    rf_x_buffer_(0, intervals_), 
                                    rf_dx_buffer_(0, intervals_), 
                                    rf_ddx_buffer_(0, intervals_));

            Set5thOrderCoefficients(f_coef_y_, 
                                    t_till_drop_down,
                                    base_generator_.Fky()(0), 
                                    rf_y_buffer_(0, intervals_), 
                                    rf_dy_buffer_(0, intervals_), 
                                    rf_ddy_buffer_(0, intervals_));

            Set5thOrderCoefficients(f_coef_q_,
                                    t_till_drop_down,
                                    base_generator_.Fkq()(0), 
                                    rf_q_buffer_(0, intervals_), 
                                    rf_dq_buffer_(0, intervals_), 
                                    rf_ddq_buffer_(0, intervals_));

            // Calculate first and second order derivatives for the velocities and accelerations.
            Derivative(f_coef_x_, f_coef_dx_);
            Derivative(f_coef_y_, f_coef_dy_);
            Derivative(f_coef_z_, f_coef_dz_);
            Derivative(f_coef_q_, f_coef_dq_);

            Derivative(f_coef_dx_, f_coef_ddx_);
            Derivative(f_coef_dy_, f_coef_ddy_);
            Derivative(f_coef_dz_, f_coef_ddz_);
            Derivative(f_coef_dq_, f_coef_ddq_);

            for (int i = 0; i <= intervals_; i++) {

                if (t_current + i*tc_ > t_transition && t_current + i*tc_ < t_ss_ - t_transition) {

                    // Evaluate interpolations for x, y, and q during the t_moving period.
                    rf_x_buffer_(0, i) = Eigen::poly_eval(f_coef_x_, i*tc_);
                    rf_y_buffer_(0, i) = Eigen::poly_eval(f_coef_y_, i*tc_);
                    rf_q_buffer_(0, i) = Eigen::poly_eval(f_coef_q_, i*tc_);

                    rf_dx_buffer_(0, i) = Eigen::poly_eval(f_coef_dx_, i*tc_);
                    rf_dy_buffer_(0, i) = Eigen::poly_eval(f_coef_dy_, i*tc_);
                    rf_dq_buffer_(0, i) = Eigen::poly_eval(f_coef_dq_, i*tc_);

                    rf_ddx_buffer_(0, i) = Eigen::poly_eval(f_coef_ddx_, i*tc_);
                    rf_ddy_buffer_(0, i) = Eigen::poly_eval(f_coef_ddy_, i*tc_);
                    rf_ddq_buffer_(0, i) = Eigen::poly_eval(f_coef_ddq_, i*tc_);
                }
                else if (t_current + i*tc_ <= t_transition) {

                    // Dont move in x, y, and q directions during lift off transitions.
                    rf_x_buffer_(0, i) = rf_x_buffer_(0, intervals_);
                    rf_y_buffer_(0, i) = rf_y_buffer_(0, intervals_);
                    rf_q_buffer_(0, i) = rf_q_buffer_(0, intervals_);

                    rf_dx_buffer_(0, i) = 0;
                    rf_dy_buffer_(0, i) = 0;
                    rf_dq_buffer_(0, i) = 0;

                    rf_ddx_buffer_(0, i) = 0;
                    rf_ddy_buffer_(0, i) = 0;
                    rf_ddq_buffer_(0, i) = 0;
                }
                else if (t_current + i*tc_ >= t_ss_ - t_transition) {

                    // Dont move in x, y, and q directions during drop down transitions.
                    rf_x_buffer_(0, i) = rf_x_buffer_(0, i - 1);
                    rf_y_buffer_(0, i) = rf_y_buffer_(0, i - 1);
                    rf_q_buffer_(0, i) = rf_q_buffer_(0, i - 1);

                    rf_dx_buffer_(0, i) = 0;
                    rf_dy_buffer_(0, i) = 0;
                    rf_dq_buffer_(0, i) = 0;

                    rf_ddx_buffer_(0, i) = 0;
                    rf_ddy_buffer_(0, i) = 0;
                    rf_ddq_buffer_(0, i) = 0;
                }

                // Evaluate interpolations for z during the whole single support period.
                rf_z_buffer_(0, i)   = Eigen::poly_eval(f_coef_z_, t_current + i*tc_);
                rf_dz_buffer_(0, i)  = Eigen::poly_eval(f_coef_dz_, t_current + i*tc_);
                rf_ddz_buffer_(0, i) = Eigen::poly_eval(f_coef_ddz_, t_current + i*tc_);
            }
        }
        else {

            // Set the coefficients for the interpolation.
            Set5thOrderCoefficients(f_coef_x_,
                                    t_till_drop_down, 
                                    base_generator_.Fkx()(0), 
                                    lf_x_buffer_(0, intervals_), 
                                    lf_dx_buffer_(0, intervals_), 
                                    lf_ddx_buffer_(0, intervals_));

            Set5thOrderCoefficients(f_coef_y_, 
                                    t_till_drop_down,
                                    base_generator_.Fky()(0), 
                                    lf_y_buffer_(0, intervals_), 
                                    lf_dy_buffer_(0, intervals_), 
                                    lf_ddy_buffer_(0, intervals_));

            Set5thOrderCoefficients(f_coef_q_,
                                    t_till_drop_down,
                                    base_generator_.Fkq()(0), 
                                    lf_q_buffer_(0, intervals_), 
                                    lf_dq_buffer_(0, intervals_), 
                                    lf_ddq_buffer_(0, intervals_));

            // Calculate first and second order derivatives for the velocities and accelerations.
            Derivative(f_coef_x_, f_coef_dx_);
            Derivative(f_coef_y_, f_coef_dy_);
            Derivative(f_coef_z_, f_coef_dz_);
            Derivative(f_coef_q_, f_coef_dq_);

            Derivative(f_coef_dx_, f_coef_ddx_);
            Derivative(f_coef_dy_, f_coef_ddy_);
            Derivative(f_coef_dz_, f_coef_ddz_);
            Derivative(f_coef_dq_, f_coef_ddq_);

            for (int i = 0; i <= intervals_; i++) {

                if (t_current + i*tc_ > t_transition && t_current + i*tc_ < t_ss_ - t_transition) {

                    // Evaluate interpolations for x, y, and q during the t_moving period.
                    lf_x_buffer_(0, i) = Eigen::poly_eval(f_coef_x_, i*tc_);
                    lf_y_buffer_(0, i) = Eigen::poly_eval(f_coef_y_, i*tc_);
                    lf_q_buffer_(0, i) = Eigen::poly_eval(f_coef_q_, i*tc_);

                    lf_dx_buffer_(0, i) = Eigen::poly_eval(f_coef_dx_, i*tc_);
                    lf_dy_buffer_(0, i) = Eigen::poly_eval(f_coef_dy_, i*tc_);
                    lf_dq_buffer_(0, i) = Eigen::poly_eval(f_coef_dq_, i*tc_);

                    lf_ddx_buffer_(0, i) = Eigen::poly_eval(f_coef_ddx_, i*tc_);
                    lf_ddy_buffer_(0, i) = Eigen::poly_eval(f_coef_ddy_, i*tc_);
                    lf_ddq_buffer_(0, i) = Eigen::poly_eval(f_coef_ddq_, i*tc_);
                }
                else if (t_current + i*tc_ <= t_transition) {

                    // Dont move in x, y, and q directions during transitions.
                    lf_x_buffer_(0, i) = lf_x_buffer_(0, intervals_);
                    lf_y_buffer_(0, i) = lf_y_buffer_(0, intervals_);
                    lf_q_buffer_(0, i) = lf_q_buffer_(0, intervals_);

                    lf_dx_buffer_(0, i) = 0;
                    lf_dy_buffer_(0, i) = 0;
                    lf_dq_buffer_(0, i) = 0;

                    lf_ddx_buffer_(0, i) = 0;
                    lf_ddy_buffer_(0, i) = 0;
                    lf_ddq_buffer_(0, i) = 0;
                }
                else if (t_current + i*tc_ >= t_ss_ - t_transition) {

                    // Dont move in x, y, and q directions during transitions.
                    lf_x_buffer_(0, i) = lf_x_buffer_(0, i - 1);
                    lf_y_buffer_(0, i) = lf_y_buffer_(0, i - 1);
                    lf_q_buffer_(0, i) = lf_q_buffer_(0, i - 1);

                    lf_dx_buffer_(0, i) = 0;
                    lf_dy_buffer_(0, i) = 0;
                    lf_dq_buffer_(0, i) = 0;

                    lf_ddx_buffer_(0, i) = 0;
                    lf_ddy_buffer_(0, i) = 0;
                    lf_ddq_buffer_(0, i) = 0;
                }

                // Evaluate interpolations for z during the whole single support period.
                lf_z_buffer_(0, i)   = Eigen::poly_eval(f_coef_z_, t_current + i*tc_);
                lf_dz_buffer_(0, i)  = Eigen::poly_eval(f_coef_dz_, t_current + i*tc_);
                lf_ddz_buffer_(0, i) = Eigen::poly_eval(f_coef_ddz_, t_current + i*tc_);
            }
        }
    }
}

void Interpolation::InterpolateLIPM() {

    // Initialize buffer with current values.
    com_x_buffer_.col(0) = base_generator_.Ckx0();
    com_y_buffer_.col(0) = base_generator_.Cky0();
    zmp_x_buffer_.col(0) = c_.transpose()*base_generator_.Ckx0();
    zmp_y_buffer_.col(0) = c_.transpose()*base_generator_.Cky0();

    for (int i = 1; i < intervals_; i++) {
        com_x_buffer_.col(i) = a_*com_x_buffer_.col(i - 1) + b_*base_generator_.Dddckx().row(0);
        com_y_buffer_.col(i) = a_*com_y_buffer_.col(i - 1) + b_*base_generator_.Dddcky().row(0);
        zmp_x_buffer_.col(i) << c_.transpose()*com_x_buffer_.col(i - 1);
        zmp_y_buffer_.col(i) << c_.transpose()*com_y_buffer_.col(i - 1);
    }
}

template <typename Derived>
void Interpolation::Set4thOrderCoefficients(Eigen::MatrixBase<Derived>& coef,
                                            double final_time, double middle_pos,
                                            double init_pos, double init_vel) {
    
    // Set the 4th order coefficients for the interpolation.
    coef(0) = init_pos;
    coef(1) = init_vel;

    if (final_time == 0.) {
        coef.tail(3).setZero();
    }
    else {
        coef(2) = (-  4.*init_vel*final_time
                   - 11.*init_pos
                   + 16.*middle_pos)/Eigen::numext::pow(final_time, 2.);

        coef(3) = (   5.*init_vel*final_time
                   + 18.*init_pos
                   - 32.*middle_pos)/Eigen::numext::pow(final_time, 3.);

        coef(4) = (-  2.*init_vel*final_time
                   -  8.*init_pos
                   + 16.*middle_pos)/Eigen::numext::pow(final_time, 4.);
    }
}

template <typename Derived>
void Interpolation::Set5thOrderCoefficients(Eigen::MatrixBase<Derived>& coef,
                                            double final_time, double final_pos,
                                            double init_pos, double init_vel, double init_acc) {

    // Set the 5th order coefficients for the interpolation.
    coef(0) =     init_pos;
    coef(1) =     init_vel;
    coef(2) = 0.5*init_acc;

    if (final_time == 0.) {
        coef.tail(3).setZero();
    }
    else {
        coef(3) = (-  1.5*init_acc*final_time*final_time
                   -  6. *init_vel*final_time
                   - 10. *(init_pos - final_pos))/Eigen::numext::pow(final_time, 3.);
        
        coef(4) = (   1.5*init_acc*final_time*final_time
                   +  8. *init_vel*final_time
                   + 15. *(init_pos - final_pos))/Eigen::numext::pow(final_time, 4.);
        
        coef(5) = (-  0.5*init_acc*final_time*final_time
                   -  3. *init_vel*final_time
                   -  6. *(init_pos - final_pos))/Eigen::numext::pow(final_time, 5.);
    }
}
