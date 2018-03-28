#include "interpolation.h"
#include <iostream>

Polynomial::Polynomial(const int degree) 
    : degree_(degree),
      coef_(degree + 1),
      ft_(0.) {
  coef_.setZero();
}

double Polynomial::Compute(double& time) {
  if (time > ft_) {
    time = ft_;
  }
  else if (time < 0.) {
    time = 0.;
  }

  double r = 0.;
  double t = 1.;

  for (int i = 0; i < coef_.size(); i++) {
    r += coef_(i)*t;
    t *= time;
  }

  return r;
}

double Polynomial::ComputeDerivative(double& time) {
  if (time > ft_) {
    time = ft_;
  }
  else if (time < 0.) {
    time = 0.;
  }

  double r = 0.;
  double t = 1.;

  for (int i = 1; i < coef_.size(); i++) {
    r += i*coef_(i)*t;
    t *= time;
  }

  return r;
}

double Polynomial::ComputeSecondDerivative(double& time) {
  if (time > ft_) {
    time = ft_;
  }
  else if (time < 0.) {
    time = 0.;
  }

  double r = 0.;
  double t = 1.;

  for (int i = 2; i < coef_.size(); i++) {
    r += i*(i - 1)*coef_(i)*t;
    t *= time;
  }

  return r;
}

Polynomial4::Polynomial4()
    : Polynomial(4) {
  SetParameters(ft_, 0., 0., 0.);
}

void Polynomial4::SetParameters(double final_time, double middle_position, 
                                double initial_position, double initial_speed) {
  ft_ = final_time;
  coef_(0) = initial_position;
  coef_(1) = initial_speed;

  double t = ft_*ft_;

  if (t == 0.) {
    coef_.tail(3).setZero();
  }
  else {
    coef_(2) = (-4.*initial_speed*ft_ - 11.*initial_position + 16.*middle_position)/t; 
    t *= ft_;
    coef_(3) = ( 5.*initial_speed*ft_ + 18.*initial_position - 32.*middle_position)/t;
    t *= ft_;
    coef_(4) = (-2.*initial_speed*ft_ -  8.*initial_position + 16.*middle_position)/t;
  }
}

Polynomial5::Polynomial5()
    : Polynomial(5) {
  SetParameters(ft_, 0., 0., 0., 0.);
}

void Polynomial5::SetParameters(double final_time, double final_position,
                                double initial_position, double initial_speed, double initial_acceleration) {
  ft_ = final_time;
  coef_(0) = initial_position;
  coef_(1) = initial_speed;
  coef_(2) = initial_acceleration*0.5;

  double t = ft_*ft_*ft_;

  if (t == 0.) {
    coef_.tail(3).setZero();
  }
  else {
    coef_(3) = (-1.5*initial_acceleration*ft_*ft_ - 6.*initial_speed*ft_ - 10.*initial_position + 10.*final_position)/t;
    t *= ft_;
    coef_(4) = ( 1.5*initial_acceleration*ft_*ft_ + 8.*initial_speed*ft_ + 15.*initial_position - 15.*final_position)/t;
    t *= ft_;
    coef_(5) = (-0.5*initial_acceleration*ft_*ft_ - 3.*initial_speed*ft_ -  6.*initial_position +  6.*final_position)/t;
  }
}

LIPM::LIPM(double control_period, double command_period, double h_com)
    : tc_(control_period),
      t_(command_period),
      h_com_(h_com),
      
      interval_(int(control_period/command_period)) {
  // For t_ sampling interpolation.
  a_.setZero();
  b_.setZero();
  c_.setZero();

  // For tc_ sampling interpolation.
  ac_.setZero();
  bc_.setZero();

  InitializeSystem();
}

void LIPM::Interpolate(const double dddc_k_x_0, const double dddc_k_y_0,
                       ComState& cur_com, std::vector<ComState>& com_buffer, std::vector<ZmpState>& zmp_buffer) {
  com_buffer[0].x = cur_com.x;
  com_buffer[0].y = cur_com.y;
  zmp_buffer[0].x = c_.transpose()*cur_com.x;
  zmp_buffer[0].y = c_.transpose()*cur_com.y;

  for (int i = 1; i < interval_; i++) {
    com_buffer[i].x = a_*com_buffer[i - 1].x + b_*dddc_k_x_0;
    com_buffer[i].y = a_*com_buffer[i - 1].y + b_*dddc_k_y_0;
    zmp_buffer[i].x = c_.transpose()*com_buffer[i].x;
    zmp_buffer[i].y = c_.transpose()*com_buffer[i].y;
  }

  cur_com.x = ac_*cur_com.x + bc_*dddc_k_x_0;
  cur_com.y = ac_*cur_com.y + bc_*dddc_k_y_0;
}

void LIPM::InitializeSystem() {
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

FootInterpolation::FootInterpolation(const BaseGenerator& base_generator,
                                     double command_period,
                                     double step_height, double double_support_time)
    : t_(base_generator.T()),
      tc_(command_period),
      step_height_(step_height),
      polynomial_x_(Polynomial5()),
      polynomial_y_(Polynomial5()),
      polynomial_q_(Polynomial5()),
      polynomial_z_(Polynomial4()),
      tss_(base_generator.TStep() - double_support_time),
      tds_(double_support_time),
      interval_(int(base_generator.T()/command_period)),
      base_generator_(base_generator) {
  polynomial_z_.SetParameters(tss_, step_height_, 0., 0.);
}

void FootInterpolation::Interpolate(double time, const BaseTypeSupportFoot& current_support, 
                                    BaseTypeFoot& cur_left, BaseTypeFoot& cur_right,
                                    double f_k_x_0, double f_k_y_0, double f_k_q_0,
                                    std::vector<BaseTypeFoot>& lf_buffer, std::vector<BaseTypeFoot>& rf_buffer) {
  // Update the current_state to be valid at the next iteration
  // and fill up the queue containing all the intermediate values.
  //
  // |---------------||-------|-----------------------------|-------|
  // | double        ||             single support phase            |
  // | support       ||take   |     flying phase            |landing|
  // | phase         ||off    |                             |       |
  double time_limit = time + base_generator_.Vkp10().sum()*t_;
  double epsilon = 0.02;

  // In case of double support the policy is to stay still.
  if (time + epsilon < time_limit - base_generator_.TStep() + t_) {
    for (int i = 0; i < interval_; i++) {
      lf_buffer[i] = cur_left;
      rf_buffer[i] = cur_right;
    }

    // We define the z trajectory in the double support phase
    // to allow the robot to take off and land during the whole
    // single support duration.
    polynomial_z_.SetParameters(tss_, step_height_, cur_right.z, cur_right.dz);
  }
  else if (time + epsilon > time_limit - base_generator_.TStep() + t_) {
    // Deal with the lift off time and the landing time. During those
    // period the foot do not move along the x and y axis.

    // This is counted from the last double support phase.
    double local_interpolation_start_time = time - (time_limit - tss_);

    // This coefficient indicates how long you allow the foot to take off
    // and land (in %)
    double module_support_coefficient = 0.9;
    
    // This time indicates how long the foot will move in x, y and q.
    double unlocked_swing_period = tss_*module_support_coefficient;

    // This time indicates the time limit when the foot should have reached
    // lift off enough to move in x, y and q.
    double end_of_lift_off = 0.5*(tss_ - unlocked_swing_period);

    // This time shows the time when the foot has flight in the air.
    double swing_time_passed = 0.;

    if (local_interpolation_start_time > end_of_lift_off) {
      swing_time_passed = local_interpolation_start_time - end_of_lift_off;
    }

    // This time is the time remaining until landing.
    double time_interval = unlocked_swing_period - swing_time_passed;

    // Set the polynomials.
    if (current_support.foot == "left") {
      polynomial_x_.SetParameters(time_interval, f_k_x_0, cur_right.x, cur_right.dx, cur_right.ddx);
      polynomial_y_.SetParameters(time_interval, f_k_y_0, cur_right.y, cur_right.dy, cur_right.ddy);
      polynomial_q_.SetParameters(time_interval, f_k_q_0, cur_right.q, cur_right.dq, cur_right.ddq);
    
      for (int i = 0; i < interval_; i++) {
        // The non swing foot stays still.
        lf_buffer[i] = cur_left;
        lf_buffer[i].support_foot = 1;

        // Interpolation time.
        double ti = tc_*i;
        double t_local = local_interpolation_start_time + ti;

        // If we are landing or lifting the foot, do not modify the x,y and q.
        if (local_interpolation_start_time < end_of_lift_off) {
          // Remaining time.
          double tr = ti - end_of_lift_off;
          ComputeXYQ(rf_buffer[i], tr);
        }
        else {
          ComputeXYQ(rf_buffer[i], ti);
        }

        rf_buffer[i].z   = polynomial_z_.Compute(t_local);
        rf_buffer[i].dz  = polynomial_z_.ComputeDerivative(t_local);
        rf_buffer[i].ddz = polynomial_z_.ComputeSecondDerivative(t_local);
      }

      if (local_interpolation_start_time < end_of_lift_off) {
        // Compute the next iteration state.
        ComputeXYQ(cur_right, tc_*interval_ - end_of_lift_off);
      }
      else {
        ComputeXYQ(cur_right, tc_*interval_);
      }
    }
    else {
      polynomial_x_.SetParameters(time_interval, f_k_x_0, cur_left.x, cur_left.dx, cur_left.ddx);
      polynomial_y_.SetParameters(time_interval, f_k_y_0, cur_left.y, cur_left.dy, cur_left.ddy);
      polynomial_q_.SetParameters(time_interval, f_k_q_0, cur_left.q, cur_left.dq, cur_left.ddq);
    
      for (int i = 0; i < interval_; i++) {
        // The non swing foot stays still.
        rf_buffer[i] = cur_right;
        rf_buffer[i].support_foot = 1;

        // Interpolation time.
        double ti = tc_*i;
        double t_local = local_interpolation_start_time + ti;

        // If we are landing or lifting the foot, do not modify the x,y and q.
        if (local_interpolation_start_time < end_of_lift_off) {
          // Remaining time.
          double tr = ti - end_of_lift_off;
          ComputeXYQ(lf_buffer[i], tr);
        }
        else {
          ComputeXYQ(lf_buffer[i], ti);
        }

        lf_buffer[i].z   = polynomial_z_.Compute(t_local);
        lf_buffer[i].dz  = polynomial_z_.ComputeDerivative(t_local);
        lf_buffer[i].ddz = polynomial_z_.ComputeSecondDerivative(t_local);
      }

      if (local_interpolation_start_time < end_of_lift_off) {
        // Compute the next iteration state.
        ComputeXYQ(cur_left, tc_*interval_ - end_of_lift_off);
      }
      else {
        ComputeXYQ(cur_left, tc_*interval_);
      }
    }
  }
}

void FootInterpolation::ComputeXYQ(BaseTypeFoot& foot, double t) {
  // Compute the foot states at time t.
  foot.  x = polynomial_x_.Compute(t);
  foot. dx = polynomial_x_.ComputeDerivative(t);
  foot.ddx = polynomial_x_.ComputeSecondDerivative(t);

  foot.  y = polynomial_y_.Compute(t);
  foot. dy = polynomial_y_.ComputeDerivative(t);
  foot.ddy = polynomial_y_.ComputeSecondDerivative(t);

  foot.  q = polynomial_q_.Compute(t);
  foot. dq = polynomial_q_.ComputeDerivative(t);
  foot.ddq = polynomial_q_.ComputeSecondDerivative(t);
}

Interpolation::Interpolation(const double tc, const BaseGenerator& base_generator)
    : base_generator_(base_generator),
      
      t_(base_generator.T()),
      tc_(tc),
      interval_(int(t_/tc_)),

      // States used to interpolate.
      cur_com_({base_generator.Ckx0(),
                base_generator.Cky0(),
                base_generator.Hcom(),
                base_generator.Ckq0()}),
    
      zmp_({cur_com_.x(0) - cur_com_.z/base_generator.G()*cur_com_.x(2),
            cur_com_.y(0) - cur_com_.z/base_generator.G()*cur_com_.y(2)}),
  
      // Buffers containing the trajectories over 100 ms.
      com_buffer_(interval_),
      zmp_buffer_(interval_),
      lf_buffer_(interval_),
      rf_buffer_(interval_),

      // Buffers containing the full trajectories.
      com_traj_(30),
      zmp_traj_(30),
      lf_traj_(30),
      rf_traj_(30),
      
      lipm_(t_, tc_, cur_com_.z),
      fi_(base_generator) {
  if (base_generator_.CurrentSupport().foot == "left") {
    cur_left_.x  = base_generator_.Fkx0();
    cur_left_.y  = base_generator_.Fky0();
    cur_left_.q  = base_generator_.Fkq0();
    cur_right_.x = base_generator_.Fkx0() + base_generator_.FootDistance()*sin(base_generator_.Fkq0()); 
    cur_right_.y = base_generator_.Fky0() - base_generator_.FootDistance()*cos(base_generator_.Fkq0());
    cur_right_.q = base_generator_.Fkq0();
  }
  else {
    cur_left_.x  = base_generator_.Fkx0(); 
    cur_left_.y  = base_generator_.Fky0();
    cur_left_.q  = base_generator_.Fkq0();
    cur_right_.x = base_generator_.Fkx0() - base_generator_.FootDistance()*sin(base_generator_.Fkq0());
    cur_right_.y = base_generator_.Fky0() + base_generator_.FootDistance()*cos(base_generator_.Fkq0());
    cur_right_.q = base_generator_.Fkq0();
  }

  for (int i = 0; i < interval_; i++) {
    // Buffers containing the trajectories over 100 ms.
    com_buffer_[i] = cur_com_;
    zmp_buffer_[i] = zmp_;
    lf_buffer_[i] = cur_left_;
    rf_buffer_[i] = cur_right_;
  }

  for (int i = 0; i < 30; i++) {
    // Buffers containing the full trajectories.
    com_traj_[i] = com_buffer_;
    zmp_traj_[i] = zmp_buffer_;
    lf_traj_[i] = lf_buffer_;
    rf_traj_[i] = rf_buffer_;
  }
}

void Interpolation::Interpolate(double time) {
  lipm_.Interpolate(base_generator_.Dddckx()(0), base_generator_.Dddcky()(0),
                    cur_com_, com_buffer_, zmp_buffer_);
  
  fi_.Interpolate(time, base_generator_.CurrentSupport(),
                  cur_left_, cur_right_,
                  base_generator_.Fkx()(0), base_generator_.Fky()(0), base_generator_.Fkq()(0),
                  lf_buffer_, rf_buffer_);

  for (int i = 0; i < lf_buffer_.size(); i++) {
    com_buffer_[i].q(0) = 0.5*(lf_buffer_[i].  q + rf_buffer_[i].  q);
    com_buffer_[i].q(1) = 0.5*(lf_buffer_[i]. dq + rf_buffer_[i]. dq);
    com_buffer_[i].q(2) = 0.5*(lf_buffer_[i].ddq + rf_buffer_[i].ddq);
  }

  com_traj_.push_back(com_buffer_);
  zmp_traj_.push_back(zmp_buffer_);
  lf_traj_.push_back(lf_buffer_);
  rf_traj_.push_back(rf_buffer_);
}

void Interpolation::SaveToFile(const std::string loc) {
  // Save results to .txt file.
  std::ofstream out(loc.c_str());

  if (out.is_open()) {
    for (int i = 0; i < com_traj_.size(); i++) {
      for (int j = 0; j < com_traj_[i].size(); j++) {
        out << com_traj_[i][j].x(0)  << "," << com_traj_[i][j].x(1) << "," << com_traj_[i][j].x(2) << ","
            << com_traj_[i][j].y(0)  << "," << com_traj_[i][j].y(1) << "," << com_traj_[i][j].y(2) << ","
            << com_traj_[i][j].z     << ","
            << com_traj_[i][j].q(0)  << "," << com_traj_[i][j].q(1) << "," << com_traj_[i][j].q(2) << ","
            << zmp_traj_[i][j].x     << "," << zmp_traj_[i][j].y    << "," << zmp_traj_[i][j].z    << ","
            << lf_traj_[i][j].x      << "," << lf_traj_[i][j].y     << "," << lf_traj_[i][j].z     << "," << lf_traj_[i][j].q << ","
            << rf_traj_[i][j].x      << "," << rf_traj_[i][j].y     << "," << rf_traj_[i][j].z     << "," << rf_traj_[i][j].q
            << "\r\n";
      }
    }
    out.close();
  }
  else {
    std::cout << "Unable to open file." << std::endl;
  }
}
