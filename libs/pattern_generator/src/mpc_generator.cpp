#include "mpc_generator.h"
#include <iostream>

MPCGenerator::MPCGenerator(const std::string config_file_loc)
    : BaseGenerator::BaseGenerator(config_file_loc),

      // qpOASES specific things.
      cpu_time_(1, configs_["cpu_time"].as<double>()),
      nwsr_(configs_["nwsr"].as<int>()),

      // Constraint dimensions.
      ori_nv_(2*n_),
      ori_nc_(nc_fvel_eq_ + nc_fpos_ineq_ + nc_fvel_ineq_),
      pos_nv_(2*(n_ + nf_)),
      pos_nc_(nc_cop_ + nc_foot_position_ + nc_fchange_eq_),

      // Problem setup for orientation.
      ori_dofs_(ori_nv_),
      ori_qp_(ori_nv_, ori_nc_),
      
      ori_h_(ori_nv_, ori_nv_),
      ori_a_(ori_nc_, ori_nv_),
      ori_g_(ori_nv_),
      ori_lb_(ori_nv_),
      ori_ub_(ori_nv_),
      ori_lba_(ori_nc_),
      ori_uba_(ori_nc_),

      ori_qp_is_initialized_(false),

      // Problem setup for position.
      pos_dofs_(pos_nv_),
      pos_qp_(pos_nv_, pos_nc_),
      
      pos_h_(pos_nv_, pos_nv_),
      pos_a_(pos_nc_, pos_nv_),
      pos_g_(pos_nv_),
      pos_lb_(pos_nv_),
      pos_ub_(pos_nv_),
      pos_lba_(pos_nc_),
      pos_uba_(pos_nc_),

      pos_qp_is_initialized_(false),

      // Dummy matrices.
      ori_q_(2*n_, 2*n_),
      ori_p_(2*n_),
      pos_q_(n_ + nf_, n_ + nf_),
      pos_p_(n_ + nf_) {
  // qpOASES specific things.
  options_.setToMPC();
  options_.printLevel = qpOASES::PL_LOW;

  // Problem setup for orientation.
  ori_dofs_.setZero();
  ori_qp_.setOptions(options_);

  ori_h_.setZero();
  ori_a_.setZero();
  ori_g_.setZero();
  ori_lb_.setConstant(-1e+08);
  ori_ub_.setConstant(1e+08);
  ori_lba_.setConstant(-1e+08);
  ori_uba_.setConstant(1e+08);

  // Problem setup for position.
  pos_dofs_.setZero();
  pos_qp_.setOptions(options_);

  pos_h_.setZero();
  pos_a_.setZero();
  pos_g_.setZero();
  pos_lb_.setConstant(-1e+08);
  pos_ub_.setConstant(1e+08);
  pos_lba_.setConstant(-1e+08);
  pos_uba_.setConstant(1e+08);

  // Dummy matrices.
  ori_q_.setZero();
  ori_p_.setZero();
  pos_q_.setZero();
  pos_p_.setZero();
}

void MPCGenerator::Solve() {
  // Process and solve problem, s.t. pattern generator data is consistent.
  PreprocessSolution();
  SolveQP();
  PostprocessSolution();
}

void MPCGenerator::Example(const std::string config_file_loc, const std::string output_loc) {
  // Example() exemplarily implements a case on how
  // the MPCGenerator class is ment to be used. 
  //
  // NOTE that you need to specify a path where the
  // generated pattern shall be stored in a .csv file.

  // Instantiate pattern generator.
  MPCGenerator mpc(config_file_loc);

  // Pattern generator preparation.
  mpc.SetSecurityMargin(mpc.SecurityMarginX(), 
                        mpc.SecurityMarginY());

  // Set initial values.
  PatternGeneratorState pg_state = {mpc.Ckx0(),
                                    mpc.Cky0(),
                                    mpc.Hcom(),
                                    mpc.Fkx0(),
                                    mpc.Fky0(),
                                    mpc.Fkq0(),
                                    mpc.CurrentSupport().foot,
                                    mpc.Ckq0()};

  mpc.SetInitialValues(pg_state);
  Interpolation interpol_mpc(mpc);
  Eigen::Vector3d velocity_reference(0.1, 0., 0.1);

  // Pattern generator event loop.
  for (int i = 0; i < 220; i++) {
    std::cout << "Iteration: " << i << std::endl;

    // Change reference velocities.
    if (25 <= i && i < 50) {
      velocity_reference << 0.1, 0., -0.1;
    }
    else if (50 <= i && i < 150) {
      velocity_reference << 0.1, 0.1, 0.;
    }
    else if (150 <= i && i < 200) {
      velocity_reference << 0.2, 0.2, 0.;
    }
    else if (200 <= i) {
      velocity_reference << 0., 0., 0.;
    }

    // Set reference velocities.
    mpc.SetVelocityReference(velocity_reference);

    // Solve QP.
    mpc.Solve();
    mpc.Simulate();
    interpol_mpc.Interpolate();

    // Initial values embedding by internal states and simulation.
    pg_state = mpc.Update();
    mpc.SetInitialValues(pg_state);
  }
  
  // Save interpolated results.
  Eigen::MatrixXd trajectories = interpol_mpc.Trajectories().transpose();
  WriteCsv(output_loc, trajectories);
}

void MPCGenerator::PreprocessSolution() {
  // Update matrices and get them into the QP data structures.

  // Orientations.
  // Initialize with actual values, else take last known solution.
  // NOTE for warmstart last solution is taken from qpOASES internal memory.
  if (!ori_qp_is_initialized_) {
      // ori_dofs_ = ( dddf_k_qr_ )
      //             ( dddf_k_ql_ )
      ori_dofs_.transpose().tail(n_) = dddf_k_ql_;
      ori_dofs_.transpose().head(n_) = dddf_k_qr_;
  }

  // Define QP matrices.
  // h = ( q_k_q_ )
  // Update values in q.
  UpdateOriQ();
  ori_h_ = ori_q_;

  // g = ( p_k_q_ )
  // Update values in p.
  UpdateOriP();
  ori_g_ = ori_p_.transpose();

  // Orientation linear constraints.
  // Velocity constraints on support foot to freeze movement.
  ori_a_.topRows(nc_fvel_eq_) = a_fvel_eq_;
  ori_lba_.head(nc_fvel_eq_)  = b_fvel_eq_.transpose();
  ori_uba_.head(nc_fvel_eq_)  = b_fvel_eq_.transpose();

  // Box constraints for maximum orientation change.
  ori_a_.block(nc_fvel_eq_, 0, nc_fpos_ineq_, ori_nv_) = a_fpos_ineq_;
  ori_lba_.segment(nc_fvel_eq_, nc_fpos_ineq_)         = lbb_fpos_ineq_.transpose();
  ori_uba_.segment(nc_fvel_eq_, nc_fpos_ineq_)         = ubb_fpos_ineq_.transpose();

  // Box constraints for maximum angular velocity.
  ori_a_.block(nc_fvel_eq_ + nc_fpos_ineq_, 0, nc_fvel_ineq_, ori_nv_) = a_fvel_ineq_;
  ori_lba_.segment(nc_fvel_eq_ + nc_fpos_ineq_, nc_fvel_ineq_)         = lbb_fvel_ineq_.transpose();
  ori_uba_.segment(nc_fvel_eq_ + nc_fpos_ineq_, nc_fvel_ineq_)         = ubb_fvel_ineq_.transpose();

  // Positions.
  // Initialize with actual values, else take last known solution.
  // NOTE for warmstart last solution is taken from qpOASES internal memory.
  if (!pos_qp_is_initialized_) {
      // pos_dofs = ( dddc_kp1_x_ )
      //            (      f_k_x_ )
      //            ( dddc_kp1_y_ )
      //            (      f_k_y_ )
      pos_dofs_.transpose().head(n_) = dddc_k_x_;
      pos_dofs_.transpose().segment(n_, nf_) = f_k_x_;
      pos_dofs_.transpose().segment(n_ + nf_, n_) = dddc_k_y_;
      pos_dofs_.transpose().segment(2*n_ + nf_, nf_) = f_k_y_;
  }

  // Define QP matrices.
  // h = ( q_k   0 )
  //     ( 0   q_k )
  UpdatePosQ();
  pos_h_.topLeftCorner(n_ + nf_, n_ + nf_)     = pos_q_;
  pos_h_.bottomRightCorner(n_ + nf_, n_ + nf_) = pos_q_;

  // g = ( p_k_x )
  //     ( p_k_y )
  UpdatePosP("x");
  pos_g_.head(n_ + nf_) = pos_p_.transpose();
  UpdatePosP("y");
  pos_g_.tail(n_ + nf_) = pos_p_.transpose();

  // lba <= a x <= uba
  // (            ) <= ( a_cop_     ) <= ( b_cop_    )
  // ( eq_b_foot_ ) <= ( eq_a_foot_ ) <= ( eq_b_foot )
  // (            ) <= ( a_foot_    ) <= ( b_foot_   )

  // CoP constraints.
  pos_a_.topRightCorner(nc_cop_, pos_nv_) = a_cop_;
  pos_lba_.head(nc_cop_) = lbb_cop_;
  pos_uba_.head(nc_cop_) = ubb_cop_;

  // Foot inequality constraints.
  pos_a_.block(nc_cop_, 0, nc_foot_position_, pos_nv_) = a_foot_;
  pos_lba_.segment(nc_cop_, nc_foot_position_) = lbb_foot_;
  pos_uba_.segment(nc_cop_, nc_foot_position_) = ubb_foot_;

  // Foot equality constraints. 
  pos_a_.block(nc_cop_ + nc_foot_position_, 0, nc_fchange_eq_, pos_nv_) = eq_a_foot_;
  pos_lba_.segment(nc_cop_ + nc_foot_position_, nc_fchange_eq_) = eq_b_foot_;
  pos_uba_.segment(nc_cop_ + nc_foot_position_, nc_fchange_eq_) = eq_b_foot_;
}

void MPCGenerator::UpdateOriQ() {
  // Update hessian block q according to walking report
  //
  // min a/2 || dc_kp1_q_ref_ - df_k_q_ ||_2^2
  // q = ( qr 0  )
  //     ( 0  ql )
  // qr = ( a*pvu_^T*e_fr_^T*e_fr_*pvu_ )
  ori_q_.topLeftCorner(n_, n_) = alpha_*pvu_.transpose()*e_fr_.transpose()*e_fr_*pvu_;

  // ql = ( a*pvu_^T*e_fl_^T*e_fl_*pvu_)
  ori_q_.bottomRightCorner(n_, n_) = alpha_*pvu_.transpose()*e_fl_.transpose()*e_fl_*pvu_;
}

void MPCGenerator::UpdateOriP() {
  // Update pass gradient block p according to walking report.
  //
  // min a/2 || dc_kp1_q_ref_ - d_f_k_q_ ||_2^2
  // p = ( pr )
  //     ( pl )
  // pr = ( a*pvu_^T*e_fr_*(e_fr_*pvs_*f_k_qr_0_ - dc_kp1_q_ref_) )
  ori_p_.head(n_) = alpha_*pvu_.transpose()*e_fr_.transpose()*(e_fr_*pvs_*f_k_qr_0_ - dc_kp1_q_ref_);

  // pl = ( a*pvu_^T*e_fl_*(e_fl_*pvs_*f_k_ql_0_ - dc_kp1_q_ref_) )
  ori_p_.tail(n_) = alpha_*pvu_.transpose()*e_fl_.transpose()*(e_fl_*pvs_*f_k_ql_0_ - dc_kp1_q_ref_);
}

void MPCGenerator::UpdatePosQ() {
  // Update hessian block q according to walking report.
  //
  // q = ( q*pvu_*pvu_ + b*ppu_*e^T*e*ppu_ + c*pzu_*pzu_ + d*I,   -c*pzu_*v_kp1_ )
  //     (                                      -c*pzu_*v_kp1_,  c*v_kp1_*v_kp1_ )

  // q = ( [*],  *  ) = a*pvu_*pvu_ + b*ppu_*e*e*ppu_ + c*pzu*pzu_ + d*I
  //     (  * ,  *  )
  pos_q_.topLeftCorner(n_, n_) = alpha_*pvu_.transpose()*pvu_ + gamma_*pzu_.transpose()*pzu_ + delta_*Eigen::MatrixXd::Identity(n_, n_);

  // q = (  * , [*] )
  //     (  * ,  *  )
  pos_q_.topRightCorner(n_, nf_) = -gamma_*pzu_.transpose()*v_kp1_.cast<double>();

  // q = (  * ,  *  ) = (  * , [*] )^T
  //     ( [*],  *  )   (  * ,  *  )
  pos_q_.bottomLeftCorner(nf_, n_) = pos_q_.topRightCorner(n_, nf_).transpose();
  
  // q = (  * ,  *  )
  //     (  * , [*] )
  pos_q_.bottomRightCorner(nf_, nf_) = gamma_*v_kp1_.transpose().cast<double>()*v_kp1_.cast<double>();
}

void MPCGenerator::UpdatePosP(const std::string type) {
  double f_k;
  Eigen::Vector3d c_k;
  Eigen::VectorXd dc_kp1_ref;

  if (type == "x") {
      f_k = f_k_x_0_;

      c_k        = c_k_x_0_;
      dc_kp1_ref = dc_kp1_x_ref_;
  }
  else if (type == "y") {
      f_k = f_k_y_0_;

      c_k        = c_k_y_0_;
      dc_kp1_ref = dc_kp1_y_ref_;
  }
  else {
      throw std::invalid_argument("Please use either type x or type y for this routine.");
  }

  // p = ([*])
  //     ( * )
  pos_p_.head(n_) = alpha_*pvu_.transpose()*(pvs_*c_k - dc_kp1_ref) + gamma_*pzu_.transpose()*(pzs_*c_k - v_kp1_0_.cast<double>()*f_k);

  // p = ( * )
  //     ([*])
  pos_p_.tail(nf_) = -gamma_*v_kp1_.transpose().cast<double>()*(pzs_*c_k - v_kp1_0_.cast<double>()*f_k);
}

void MPCGenerator::SolveQP() {
  // Solve the QP in the first run with init functionality and further on with warm warmstart.
  int nwsr_temp  = nwsr_;
  std::vector<double> cpu_time_temp = cpu_time_;

  // Call QP solver.
  if (ori_qp_is_initialized_) {
      status_ori_ = ori_qp_.hotstart(ori_h_.data(),
                                     ori_g_.data(),
                                     ori_a_.data(),
                                     ori_lb_.data(),
                                     ori_ub_.data(),
                                     ori_lba_.data(),
                                     ori_uba_.data(),
                                     nwsr_temp, cpu_time_temp.data());
  }
  else {
      status_ori_ = ori_qp_.init(ori_h_.data(),
                                 ori_g_.data(),
                                 ori_a_.data(),
                                 ori_lb_.data(),
                                 ori_ub_.data(),
                                 ori_lba_.data(),
                                 ori_uba_.data(),
                                 nwsr_temp, cpu_time_temp.data());

      ori_qp_is_initialized_ = true;
  }

  // Orientation primal solution.
  ori_qp_.getPrimalSolution(ori_dofs_.data());

  nwsr_temp  = nwsr_;
  cpu_time_temp = cpu_time_;

  if (pos_qp_is_initialized_) {
      status_pos_ = pos_qp_.hotstart(pos_h_.data(),
                                     pos_g_.data(),
                                     pos_a_.data(),
                                     pos_lb_.data(),
                                     pos_ub_.data(),
                                     pos_lba_.data(),
                                     pos_uba_.data(),
                                     nwsr_temp, cpu_time_temp.data());
  }
  else {
      status_pos_ = pos_qp_.init(pos_h_.data(),
                                 pos_g_.data(),
                                 pos_a_.data(),
                                 pos_lb_.data(),
                                 pos_ub_.data(),
                                 pos_lba_.data(),
                                 pos_uba_.data(),
                                 nwsr_temp, cpu_time_temp.data());

      pos_qp_is_initialized_ = true;
  }

  // Position primal solution.
  pos_qp_.getPrimalSolution(pos_dofs_.data());
}

void MPCGenerator::PostprocessSolution() {
  // Get solution and put it back into generator data structures.
  
  // Extract dofs.
  // ori_dofs_ = ( dddf_k_qr_ )
  //             ( dddf_k_ql_ )
  dddf_k_ql_ = ori_dofs_.transpose().tail(n_);
  dddf_k_qr_ = ori_dofs_.transpose().head(n_);

  // Extract dofs.
  // pos_dofs = ( dddc_kp1_x_ )
  //            (      f_k_x_ )
  //            ( dddc_kp1_y_ )
  //            (      f_k_y_ )
  dddc_k_x_ = pos_dofs_.transpose().head(n_);
  f_k_x_    = pos_dofs_.transpose().segment(n_, nf_);

  dddc_k_y_ = pos_dofs_.transpose().segment(n_ + nf_, n_);
  f_k_y_    = pos_dofs_.transpose().tail(nf_);
}
