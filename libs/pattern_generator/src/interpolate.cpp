#include "interpolate.h"

#include <iostream>

TESTInterpolation::TESTInterpolation(BaseGenerator& base_generator)
    : // Base generator.
      base_generator_(base_generator),

      // Constants.
      g_(base_generator.configs_["gravity"].as<double>()),

      // Preview control period t_, and command period tc_.
      t_(base_generator.T()),
      tc_(base_generator.configs_["command_period"].as<double>()),

      // Center of mass initial values.
      h_com_(base_generator_.Hcom()),
  
      // Number of interpolation intervals.
      intervals_(int(t_/tc_)),
      
      // Number of intervals that the robot stays still in the beginning.
      n_still_(base_generator.configs_["n_still"].as<int>()),

      // Interpolated trajectories.
      trajectories_(21, n_still_*intervals_),
      trajectories_buffer_(21, intervals_),

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
      lf_x_buffer_(trajectories_buffer_.block(13, 0, 1, intervals_)),
      lf_y_buffer_(trajectories_buffer_.block(14, 0, 1, intervals_)),
      lf_z_buffer_(trajectories_buffer_.block(15, 0, 1, intervals_)),
      lf_q_buffer_(trajectories_buffer_.block(16, 0, 1, intervals_)),
  
      // Right foot.
      rf_x_buffer_(trajectories_buffer_.block(17, 0, 1, intervals_)),
      rf_y_buffer_(trajectories_buffer_.block(18, 0, 1, intervals_)),
      rf_z_buffer_(trajectories_buffer_.block(19, 0, 1, intervals_)),
      rf_q_buffer_(trajectories_buffer_.block(20, 0, 1, intervals_)),
      
      // Foot interpolation coefficients.
      f_coef_x_(5),
      f_coef_y_(5),
      f_coef_z_(4),
      f_coef_q_(5) {

    // Interpolated trajectories.
    trajectories_.setZero();
    trajectories_buffer_.setZero();
      
    // Foot interpolation coefficients.
    f_coef_x_.setZero();
    f_coef_y_.setZero();
    f_coef_z_.setZero();
    f_coef_q_.setZero();

    // Initialize models.
    a_.setZero();
    b_.setZero();
    c_.setZero();

    ac_.setZero();
    bc_.setZero();

    TESTInitializeTrajectories();
    TESTInitializeLIPM();
}

void TESTInterpolation::TESTInterpolate(double time) {
    
    // Interpolate.
    TESTInterpolateLIPM();
    TESTInterpolateFeet(time);

    // Append by buffered trajectories.
    trajectories_.conservativeResize(trajectories_.rows(), trajectories_.cols() + intervals_);
    trajectories_.rightCols(intervals_) = trajectories_buffer_;
}

void Set4thOrderCoefficients(Eigen::VectorXd& coef,
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

void Set5thOrderCoefficients(Eigen::VectorXd& coef,
                             double final_time, double final_pos,
                             double init_pos, double init_vel, double init_acc) {

    // Set the 5th order coefficients for the interpolation.
    coef(0) = init_pos;
    coef(1) = init_vel;
    coef(2) = init_acc;

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

Eigen::VectorXd Derivative(Eigen::VectorXd& coef) {
    
    // Calculate the derivative of a coefficient vector.
    Eigen::VectorXd d_coef = Eigen::VectorXd::Zero(coef.size() - 1);

    for (int i = 0; i < coef.size() - 1; i++) {
        d_coef(i) = (i + 1)*coef(i + 1);
    }

    return d_coef;
}

void TESTInterpolation::TESTInitializeLIPM() {

    // 
    a_ << 1., t_, t_*t_*0.5,
          0., 1.,        t_,
          0., 0.,        1.;

    b_ << t_*t_*t_/6.,
             t_*t_/2.,
                   t_;

    c_ <<         1.,
                  0.,
          -h_com_/g_;

    ac_ << 1., tc_, tc_*tc_*0.5,
           0.,  1.,         tc_,
           0.,  0.,          1.;

    bc_ << tc_*tc_*tc_/6.,
               tc_*tc_/2.,
                      tc_;
}

void TESTInterpolation::TESTInitializeTrajectories() {
    
    // Initialize the standing still trajectories. Center of mass.
    com_x_buffer_.colwise() = base_generator_.Ckx0();
    com_y_buffer_.colwise() = base_generator_.Cky0();
    com_z_buffer_.setConstant(base_generator_.Hcom());
    com_q_buffer_.colwise() = base_generator_.Ckq0();

    // Zero moment point.
    zmp_x_buffer_.setConstant(base_generator_.Ckx0()(0) - base_generator_.Hcom()/g_*base_generator_.Ckx0()(2));
    zmp_y_buffer_.setConstant(base_generator_.Cky0()(0) - base_generator_.Hcom()/g_*base_generator_.Cky0()(2));

    // Feet, depending on the current support.
    if (base_generator_.CurrentSupport().foot == "left") { // TODO changed
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
    trajectories_ = trajectories_buffer_.replicate(1, n_still_);
}

void TESTInterpolation::TESTInterpolateLIPM() {

    // Initialize buffer with current values.
    com_x_buffer_.col(0) = base_generator_.Ckx0();
    com_y_buffer_.col(0) = base_generator_.Cky0();
    zmp_x_buffer_.col(0) = c_.transpose()*base_generator_.Ckx0();
    zmp_y_buffer_.col(0) = c_.transpose()*base_generator_.Cky0();

    for (int i = 1; i < intervals_; i++) { // TODO used ac instead.
        com_x_buffer_.col(i) = ac_*com_x_buffer_.col(i - 1) + bc_*base_generator_.Dddckx().row(0);
        com_y_buffer_.col(i) = ac_*com_y_buffer_.col(i - 1) + bc_*base_generator_.Dddcky().row(0);
        zmp_x_buffer_.col(i) << c_.transpose()*com_x_buffer_.col(i - 1);
        zmp_y_buffer_.col(i) << c_.transpose()*com_y_buffer_.col(i - 1);
    }

    // TODO current com?
}

void TESTInterpolation::TESTInterpolateFeet(double time) {

    // Double or single support.
    double time_preview = base_generator_.Vkp10().sum()*t_;

    if (base_generator_.TStep() < time_preview) {

        // Stay still in case of double support.
        lf_x_buffer_.setConstant(lf_x_buffer_(1, intervals_));
        lf_y_buffer_.setConstant(lf_y_buffer_(1, intervals_));
        lf_z_buffer_.setConstant(lf_z_buffer_(1, intervals_));
        lf_q_buffer_.setConstant(lf_q_buffer_(1, intervals_));

        rf_x_buffer_.setConstant(rf_x_buffer_(1, intervals_));
        rf_y_buffer_.setConstant(rf_y_buffer_(1, intervals_));
        rf_z_buffer_.setConstant(rf_z_buffer_(1, intervals_));
        rf_q_buffer_.setConstant(rf_q_buffer_(1, intervals_));
    }
    else if (base_generator_.TStep() > time_preview) {

        // Left or right support.
        if (base_generator_.CurrentSupport().foot == "left") {

        }
        else {

        }
    }
    
    
    
    
    // Compute x, y, z, and q coordinates for each foot in buffer.

}
