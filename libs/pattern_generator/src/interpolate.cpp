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
      rf_q_buffer_(trajectories_buffer_.block(20, 0, 1, intervals_)) {

    // Interpolated trajectories.
    trajectories_.setZero();
    trajectories_buffer_.setZero();

    // Initialize models.
    a_.setZero();
    b_.setZero();
    c_.setZero();

    ac_.setZero();
    bc_.setZero();

    TESTInitializeTrajectories();
    TESTInitializeLIPM();
}

void TESTInterpolation::TESTInterpolate() {
    // Interpolate.
    TESTInterpolateLIPM();
    TESTInterpolateFeet();

    // Append by buffered trajectories.
    trajectories_.conservativeResize(trajectories_.rows(), trajectories_.cols() + intervals_);
    trajectories_.rightCols(intervals_) = trajectories_buffer_;
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
        lf_q_buffer_.setConstant(base_generator_.Fkq0());
        rf_x_buffer_.setConstant(base_generator_.Fkx0() + base_generator_.FootDistance()*sin(base_generator_.Fkq0()));
        rf_y_buffer_.setConstant(base_generator_.Fky0() - base_generator_.FootDistance()*cos(base_generator_.Fkq0()));
        rf_q_buffer_.setConstant(base_generator_.Fkq0());
    }
    else {
        lf_x_buffer_.setConstant(base_generator_.Fkx0() - base_generator_.FootDistance()*sin(base_generator_.Fkq0())); 
        lf_y_buffer_.setConstant(base_generator_.Fky0() + base_generator_.FootDistance()*cos(base_generator_.Fkq0()));
        lf_q_buffer_.setConstant(base_generator_.Fkq0());
        rf_x_buffer_.setConstant(base_generator_.Fkx0());
        rf_y_buffer_.setConstant(base_generator_.Fky0());
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

void TESTInterpolation::TESTInterpolateFeet() {

}
