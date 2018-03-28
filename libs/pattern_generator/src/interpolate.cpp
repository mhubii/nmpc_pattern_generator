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
      trajectories_(n_still_, 21),
      trajectories_buffer_(intervals_, 21),

      // Center of mass.
      com_x_buffer_(trajectories_buffer_.block(0, 0, intervals_, 3)),
      com_y_buffer_(trajectories_buffer_.block(0, 3, intervals_, 3)),
      com_z_buffer_(trajectories_buffer_.block(0, 6, intervals_, 1)),
      com_q_buffer_(trajectories_buffer_.block(0, 7, intervals_, 3)),
  
      // Zero moment point.
      zmp_x_buffer_(trajectories_buffer_.col(10)),
      zmp_y_buffer_(trajectories_buffer_.col(11)),
      zmp_z_buffer_(trajectories_buffer_.col(12)),
  
      // Left foot.
      lf_x_buffer_(trajectories_buffer_.col(13)),
      lf_y_buffer_(trajectories_buffer_.col(14)),
      lf_z_buffer_(trajectories_buffer_.col(15)),
      lf_q_buffer_(trajectories_buffer_.col(16)),
  
      // Right foot.
      rf_x_buffer_(trajectories_buffer_.col(17)),
      rf_y_buffer_(trajectories_buffer_.col(18)),
      rf_z_buffer_(trajectories_buffer_.col(19)),
      rf_q_buffer_(trajectories_buffer_.col(20)) {

  // Interpolated trajectories.
  // TODO

  // Initialize linear inverted pendulum.
  a_.setZero();
  b_.setZero();
  c_.setZero();

  ac_.setZero();
  bc_.setZero();

  TESTInitializeLIPM();
}

void TESTInterpolation::TESTInterpolate() {
    // Interpolate.
    TESTInterpolateLIPM();
    // TESTInterpolateFeet();

    // Append trajectories.
    std::cout << trajectories_ << std::endl;
    trajectories_.conservativeResize(trajectories_.rows(), trajectories_.rows() + intervals_);
    trajectories_.bottomRows(intervals_) = trajectories_buffer_;
    std::cout << trajectories_ << std::endl;
}

void TESTInterpolation::TESTInitializeLIPM() {
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

void TESTInterpolation::TESTInterpolateLIPM() {
  // Initialize buffer with current values.
  com_x_buffer_.row(0) = base_generator_.Ckx0();
  com_y_buffer_.row(0) = base_generator_.Cky0();
  zmp_x_buffer_.row(0) = c_.transpose()*base_generator_.Ckx0();
  zmp_y_buffer_.row(0) = c_.transpose()*base_generator_.Cky0();

  for (int i = 1; i < intervals_; i++) {
      com_x_buffer_.row(i) << a_*com_x_buffer_.row(i - 1).transpose() + b_*base_generator_.Dddckx()(0);
      com_y_buffer_.row(i) << a_*com_y_buffer_.row(i - 1).transpose() + b_*base_generator_.Dddcky()(0);
      zmp_x_buffer_.row(i) << com_x_buffer_.row(i - 1)*c_;
      zmp_y_buffer_.row(i) << com_y_buffer_.row(i - 1)*c_;
  }

  // TODO
}
