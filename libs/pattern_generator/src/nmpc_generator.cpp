#include "nmpc_generator.h"
#include <iostream>

NMPCGenerator::NMPCGenerator(const std::string config_file_loc)
    : BaseGenerator::BaseGenerator(config_file_loc),

      // qpOASES specific things.
      cpu_time_(1, configs_["cpu_time"].as<double>()),
      nwsr_(configs_["nwsr"].as<int>()),

      // Variable dimensions.
      nv_(2*(2*n_ + nf_)),

      // Constraint dimensions.
      nc_pos_(nc_cop_ + nc_foot_position_ + nc_fchange_eq_),
      nc_ori_(nc_fvel_eq_ + nc_fpos_ineq_ + nc_fvel_ineq_),
      nc_(nc_pos_ + nc_obs_ + nc_ori_),

      // Problem setup.
      dofs_(nv_),
      delta_dofs_(nv_),
      qp_(nv_, nc_),

      // Quadratic problem.
      qp_h_(nv_, nv_),
      qp_a_(nc_, nv_),
      qp_g_(nv_),
      qp_lb_(nv_),
      qp_ub_(nv_),
      qp_lba_(nc_),
      qp_uba_(nc_),

      qp_is_initialized_(false),

      // Helper matrices for common expressions.
      q_k_x_(n_ + nf_, n_ + nf_),
      p_k_x_(n_ + nf_),
      p_k_y_(n_ + nf_),

      q_k_ql_(n_, n_),
      q_k_qr_(n_, n_),
      p_k_ql_(n_),
      p_k_qr_(n_),

      a_pos_x_(nc_pos_, 2*(n_ + nf_)),
      a_pos_q_(nc_pos_, 2*n_),
      uba_pos_(nc_pos_),
      lba_pos_(nc_pos_),

      a_obs_(nc_obs_, 2*(n_ + nf_)),
      uba_obs_(nc_obs_),
      lba_obs_(nc_obs_),

      a_ori_(nc_ori_, 2*n_),
      uba_ori_(nc_ori_),
      lba_ori_(nc_ori_),

      derv_a_cop_map_(nc_cop_, n_),
      derv_a_foot_map_(nc_foot_position_, n_) {

  // Reset the NMPCGenerator.
  Reset();
}

void NMPCGenerator::Solve() {
  // Process and solve problem, s.t. pattern generator
  // data is consistent.
  PreprocessSolution();
  SolveQP();
  PostprocessSolution();
}

PatternGeneratorState NMPCGenerator::Update() {
  // Define time dependent foot selections matrix.
  PatternGeneratorState ret = BaseGenerator::Update();

  // Update selection matrix when something has changed.
  UpdateFootSelectionMatrix();

  return ret;
}

PatternGeneratorState NMPCGenerator::Update(double dt) {
  // Define time dependent foot selections matrix.
  PatternGeneratorState ret = BaseGenerator::Update(dt);

  // Update selection matrix when something has changed.
  UpdateFootSelectionMatrix();

  return ret;
}

void NMPCGenerator::Example(const std::string config_file_loc, const std::string output_loc) {
  // Example() exemplarily implements a case on how
  // the NMPCGenerator class is ment to be used. 
  //
  // NOTE that you need to specify a path where the
  // generated pattern shall be stored in a .csv file.
  
  // Initialize pattern generator.
  NMPCGenerator nmpc(config_file_loc);

  // Pattern generator preparation.
  nmpc.SetSecurityMargin(nmpc.SecurityMarginX(), 
                         nmpc.SecurityMarginY());

  // Set initial values.
  PatternGeneratorState pg_state = {nmpc.Ckx0(),
                                    nmpc.Cky0(),
                                    nmpc.Hcom(),
                                    nmpc.Fkx0(),
                                    nmpc.Fky0(),
                                    nmpc.Fkq0(),
                                    nmpc.CurrentSupport().foot,
                                    nmpc.Ckq0()};

  nmpc.SetInitialValues(pg_state);
  Interpolation interpol_nmpc(nmpc);
  interpol_nmpc.StoreTrajectories(true);
  Eigen::Vector3d velocity_reference(0.1, 0., 0.);

  // Pattern generator event loop.
  for (int i = 0; i < 200; i++) {
    std::cout << "Iteration: " << i << std::endl;

    // Change reference velocities.
    if (25 <= i && i < 50) {
      velocity_reference << 0.1, 0., 0.1;
    }
    else if (50 <= i && i < 150) {
      velocity_reference << 0.1, 0.1, 0.1;
    }
    else if (150 <= i && i < 200) {
      velocity_reference << 0., 0., 0.;
    }

    // Set reference velocities.
    nmpc.SetVelocityReference(velocity_reference);

    // Solve QP.
    nmpc.Solve();
    nmpc.Simulate();
    interpol_nmpc.InterpolateStep();

    // Initial value embedding by internal states and simulation.
    pg_state = nmpc.Update();
    nmpc.SetInitialValues(pg_state);
  }

  // Save interpolated results.
  Eigen::MatrixXd trajectories = interpol_nmpc.GetTrajectories().transpose();
  WriteCsv(output_loc, trajectories);
}

void NMPCGenerator::PreprocessSolution() {
  // Update matrices and get them into the QP data
  // stuctures.

  // Degine position and orientation dofs.
  // u_k = ( u_k_xy, u_k_q ).T
  // u_k_xy = ( dddc_k_x_ ) n_
  //          (    f_k_x_ ) nf_
  //          ( dddy_k_y_ ) n_
  //          (    f_k_y_ ) nf_
  // u_k_q  = ( dddf_k_q_ ) n_
  //          ( dddf_k_q_ ) n_

  // Position dofs.
  Eigen::Ref<Eigen::VectorXd> u_k    = dofs_;
  Eigen::Ref<Eigen::VectorXd> u_k_xy = u_k.head(2*(n_ + nf_));
  Eigen::Ref<Eigen::VectorXd> u_k_x  = u_k_xy.head(n_ + nf_);
  Eigen::Ref<Eigen::VectorXd> u_k_y  = u_k_xy.tail(n_ + nf_);

  // Orientation dofs.
  Eigen::Ref<Eigen::VectorXd> u_k_q  = u_k.tail(2*n_);
  Eigen::Ref<Eigen::VectorXd> u_k_ql = u_k_q.tail(n_);
  Eigen::Ref<Eigen::VectorXd> u_k_qr = u_k_q.head(n_);

  // Position dimensions.
  const int n_u_k    = u_k.size();
  const int n_u_k_xy = u_k_xy.size();
  const int n_u_k_x  = u_k_x.size();
  const int n_u_k_y  = u_k_y.size();

  // Orientation dimensions.
  const int n_u_k_q  = u_k_q.size();
  const int n_u_k_ql = u_k_ql.size();
  const int n_u_k_qr = u_k_qr.size();

  // Initialize with actual values, else take last
  // known solution.
  // NOTE for warmstart last solution is taken from
  // qpOASES internal memory.
  if (!qp_is_initialized_) {
    // TODO gues initial active set.
  }

  // Calculate the common sub expressions.
  CalculateCommonExpressions();

  // Calculate Jacobian parts that are non-trivial, i.e.
  // wrt. orienation.
  CalculateDerivatives();

  // Define qp matrices.
  // Gauss-Newton Hessian approximation.
  // Define sub blocks.
  // H = ( hxy | hqx )
  //     ( hxq | hqq )
  Eigen::Ref<RowMatrixXd> hxx = qp_h_.topLeftCorner(n_u_k_xy, n_u_k_xy);
  Eigen::Ref<RowMatrixXd> hxq = qp_h_.topRightCorner(n_u_k_xy, n_u_k - n_u_k_xy);
  Eigen::Ref<RowMatrixXd> hqx = qp_h_.bottomLeftCorner(n_u_k - n_u_k_xy, n_u_k_xy);
  Eigen::Ref<RowMatrixXd> hqq = qp_h_.bottomRightCorner(n_u_k - n_u_k_xy, n_u_k - n_u_k_xy);

  // Fill matrices.
  hxx.topLeftCorner(n_u_k_x, n_u_k_x) = q_k_x_;
  hxx.bottomRightCorner(n_u_k_y, n_u_k_y) = q_k_x_; // NOTE q_k_x_ = q_k_y_

  hqq.bottomRightCorner(n_u_k_ql, n_u_k_ql) = q_k_ql_;
  hqq.topLeftCorner(n_u_k_qr, n_u_k_qr) = q_k_qr_;

  // Gradient of objective.
  // Define sub blocks.
  // g = ( gx )
  //     ( gq )
  Eigen::Ref<Eigen::VectorXd> gx = qp_g_.head(n_u_k_xy).transpose();
  Eigen::Ref<Eigen::VectorXd> gq = qp_g_.tail(n_u_k_q).transpose();

  // gx = ( u_k_x*q_k_x_ + p_k_x_ )
  gx.head(n_u_k_x) = q_k_x_.transpose()*u_k_x + p_k_x_;
  gx.tail(n_u_k_y) = q_k_x_.transpose()*u_k_y + p_k_y_; // NOTE q_k_x_ = q_k_y_

  // gq = ( u_k_q_*q_k_q_ + p_k_q_ )
  gq.tail(n_u_k_ql) = q_k_ql_.transpose()*u_k_ql + p_k_ql_;
  gq.head(n_u_k_qr) = q_k_qr_.transpose()*u_k_qr + p_k_qr_;

  // Constraints.
  // lba_xy  < a = ( a_xy , a_xyq ) < uba_xy 
  // lba_obs <     ( a_obs, 0     ) < uba_obs
  // lba_q   <     ( 0    , a_q   ) < uba_q
  Eigen::Ref<RowMatrixXd> a_xy   = qp_a_.topLeftCorner(nc_pos_, n_u_k_xy);
  Eigen::Ref<RowMatrixXd> a_xyq  = qp_a_.topRightCorner(nc_pos_, n_u_k_q);
  Eigen::Ref<Eigen::VectorXd> lba_xy = qp_lba_.head(nc_pos_).transpose();
  Eigen::Ref<Eigen::VectorXd> uba_xy = qp_uba_.head(nc_pos_).transpose();

  Eigen::Ref<RowMatrixXd> a_obs   = qp_a_.block(nc_pos_, 0, nc_obs_, n_u_k_xy);
  Eigen::Ref<Eigen::VectorXd> lba_obs = qp_lba_.segment(nc_pos_, nc_obs_).transpose();
  Eigen::Ref<Eigen::VectorXd> uba_obs = qp_uba_.segment(nc_pos_, nc_obs_).transpose();

  Eigen::Ref<RowMatrixXd> a_q   = qp_a_.bottomRightCorner(nc_ori_, n_u_k_q);
  Eigen::Ref<Eigen::VectorXd> lba_q = qp_lba_.tail(nc_ori_).transpose();
  Eigen::Ref<Eigen::VectorXd> uba_q = qp_uba_.tail(nc_ori_).transpose();

  // Linearized contraints.
  // lba - a*u_k <= nabla a*delta_u_k <= uba - a*u_k
  a_xy   = a_pos_x_;
  a_xyq  = a_pos_q_;
  lba_xy = lba_pos_ - a_pos_x_*u_k_xy;
  uba_xy = uba_pos_ - a_pos_x_*u_k_xy;

  a_obs = a_obs_;
  Eigen::VectorXd delta_foot(nc_obs_);
  delta_foot.setZero();

  for (int i = 0; i < nc_obs_; i++) {
    for (int j = 0; j < 2*(n_ + nf_); j++) {
      delta_foot(i) += BaseGenerator::a_obs_(i, j)*u_k_xy(j);

      for (int k = 0; k < 2*(n_ + nf_); k++) {
        delta_foot(i) += u_k_xy(j)*h_obs_(i, j, k)*u_k_xy(k);
      }
    }
    lba_obs(i) = lba_obs_(i) - delta_foot(i);
    uba_obs(i) = uba_obs_(i) - delta_foot(i);
  }

  a_q = a_ori_;
  lba_q = lba_ori_ - a_ori_*u_k_q;
  uba_q = uba_ori_ - a_ori_*u_k_q;
}

void NMPCGenerator::CalculateCommonExpressions() {
  // Encapsulation of complicated matrix assembly of 
  // former orientation and position QP sub matrices.

  // Position QP matrices.
  Eigen::Ref<Eigen::MatrixXd> q_k_xxx = q_k_x_.block(0, 0, n_, n_);
  Eigen::Ref<Eigen::MatrixXd> q_k_xxf = q_k_x_.block(0, n_, n_, nf_);
  Eigen::Ref<Eigen::MatrixXd> q_k_xfx = q_k_x_.block(n_, 0, nf_, n_);
  Eigen::Ref<Eigen::MatrixXd> q_k_xff = q_k_x_.block(n_, n_, nf_, nf_);

  // q_k_xxx = (  0.5 * a * pvu_^T   * pvu_ + c * pzu_^T * pzu_^T + d * I )
  // q_k_xxf = ( -0.5 * c * pzu_^T   * v_kp1_ )
  // q_k_xfx = ( -0.5 * c * pzu_^T   * v_kp1_ )^T
  // q_k_xff = (  0.5 * c * v_kp1_^T * v_kp1_ )
  q_k_xxx <<   alpha_*pvu_.transpose()*pvu_
             + beta_*pzu_.transpose()*pzu_
             + gamma_*Eigen::MatrixXd::Identity(n_, n_);
  
  q_k_xxf << -beta_*pzu_.transpose()*v_kp1_.cast<double>();
  q_k_xfx << q_k_xxf.transpose();
  q_k_xff <<  beta_*v_kp1_.transpose().cast<double>()*v_kp1_.cast<double>();

  // p_k_x = ( p_k_xx )
  //         ( p_k_xf )
  Eigen::Ref<Eigen::VectorXd> p_k_xx = p_k_x_.head(n_);
  Eigen::Ref<Eigen::VectorXd> p_k_xf = p_k_x_.tail(nf_);
  
  // p_k_xx = (  0.5 * a * pvu_^T * pvu_ + c * pzu_^T * pzu_ + d * I )
  // p_k_xf = ( -0.5 * c * pzu_^T * v_kp1_ )
  p_k_xx <<   alpha_*pvu_.transpose()*(pvs_*c_k_x_0_ - dc_kp1_x_ref_)
            + beta_*pzu_.transpose()*(pzs_*c_k_x_0_ - v_kp1_0_.cast<double>()*f_k_x_0_);
  p_k_xf <<  -beta_*v_kp1_.transpose().cast<double>()*(pzs_*c_k_x_0_ - v_kp1_0_.cast<double>()*f_k_x_0_);

  // p_k_y = ( p_k_yx )
  //         ( p_k_yf )
  Eigen::Ref<Eigen::VectorXd> p_k_yx = p_k_y_.head(n_);
  Eigen::Ref<Eigen::VectorXd> p_k_yf = p_k_y_.tail(nf_);

  // p_k_yx = (  0.5 * a * pvu_^T * pvu_ + c * Pzu^T * Pzu + d * I )
  // p_k_yf = ( -0.5 * c * pzu_^T * v_kp1_ )
  p_k_yx <<   alpha_*pvu_.transpose()*(pvs_*c_k_y_0_ - dc_kp1_y_ref_)
            + beta_*pzu_.transpose()*(pzs_*c_k_y_0_ - v_kp1_0_.cast<double>()*f_k_y_0_);  
  p_k_yf <<  -beta_*v_kp1_.transpose().cast<double>()*(pzs_*c_k_y_0_ - v_kp1_0_.cast<double>()*f_k_y_0_);

  // Orientation QP matrices.
  // q_k_ql_ = ( 0.5 * a * pvu_^T * e_fl_^T *  e_fl_ * pvu_ )
  q_k_ql_ << alpha_*pvu_.transpose()*e_fl_.transpose()*e_fl_*pvu_;

  // p_k_ql_ = ( a * pvu_^T * e_fl_^T * (e_fl_ * pvs_ * f_k_ql_0_ + dc_kp1_q_ref_) )
  p_k_ql_ << alpha_*pvu_.transpose()*e_fl_.transpose()*(e_fl_*pvs_*f_k_ql_0_ - dc_kp1_q_ref_);

  // q_k_qr_ = ( 0.5 * a * pvu_^T * e_fr_^T *  e_fr_ * pvu_ )
  q_k_qr_ << alpha_*pvu_.transpose()*e_fr_.transpose()*e_fr_*pvu_;

  // p_k_qr_ = ( a * pvu_^T * e_fr_^T * (e_fr_ * pvs_ * f_k_qr_0_ + dc_kp1_q_ref_) )
  p_k_qr_ << alpha_*pvu_.transpose()*e_fr_.transpose()*(e_fr_*pvs_*f_k_qr_0_ - dc_kp1_q_ref_);

  // Linear constraints.
  // CoP constraints.
  a_pos_x_.topRows(nc_cop_) << a_cop_;
  lba_pos_.topRows(nc_cop_) << lbb_cop_;
  uba_pos_.topRows(nc_cop_) << ubb_cop_;

  // Foot inequality constraints.
  a_pos_x_.block(nc_cop_, 0, nc_foot_position_, a_foot_.cols()) << a_foot_;
  lba_pos_.block(nc_cop_, 0, nc_foot_position_, lbb_foot_.cols()) << lbb_foot_;
  uba_pos_.block(nc_cop_, 0, nc_foot_position_, ubb_foot_.cols()) << ubb_foot_;

  // Foot equality constraints.
  a_pos_x_.block(nc_cop_ + nc_foot_position_, 0, nc_fchange_eq_, eq_a_foot_.cols()) << eq_a_foot_;
  lba_pos_.block(nc_cop_ + nc_foot_position_, 0, nc_fchange_eq_, eq_b_foot_.cols()) << eq_b_foot_;
  uba_pos_.block(nc_cop_ + nc_foot_position_, 0, nc_fchange_eq_, eq_b_foot_.cols()) << eq_b_foot_;

  // Obstacle inequality constraints.
  lba_obs_ << lb_obs_;
  uba_obs_ << ub_obs_; 

  // Velocity constraints on support foot to freeze movement.
  a_ori_.topRows(nc_fvel_eq_) << a_fvel_eq_;
  lba_ori_.topRows(nc_fvel_eq_) << b_fvel_eq_;
  uba_ori_.topRows(nc_fvel_eq_) << b_fvel_eq_;

  // Box constraints for maximum orientation change.
  a_ori_.block(nc_fvel_eq_, 0, nc_fpos_ineq_, a_fpos_ineq_.cols()) << a_fpos_ineq_;
  lba_ori_.block(nc_fvel_eq_, 0, nc_fpos_ineq_, lbb_fpos_ineq_.cols()) << lbb_fpos_ineq_;
  uba_ori_.block(nc_fvel_eq_, 0, nc_fpos_ineq_, ubb_fpos_ineq_.cols()) << ubb_fpos_ineq_;

  // Box constraints for maximum angular velocity.
  a_ori_.block(nc_fvel_eq_ + nc_fpos_ineq_, 0, nc_fpos_ineq_, a_fvel_ineq_.cols()) << a_fvel_ineq_;
  lba_ori_.block(nc_fvel_eq_ + nc_fpos_ineq_, 0, nc_fpos_ineq_, lbb_fvel_ineq_.cols()) << lbb_fvel_ineq_;
  uba_ori_.block(nc_fvel_eq_ + nc_fpos_ineq_, 0, nc_fpos_ineq_, ubb_fvel_ineq_.cols()) << ubb_fvel_ineq_;
}



void NMPCGenerator::CalculateDerivatives() {
  // Calculate the Jacobian of the constraint function.

  // CoP Constraints.
  // Build the constraint enforcing the center of pressure 
  // to stay inside the support polygon given through the 
  // convex hull of the foot.

  // Define dummy values.
  // d_kp1 = ( d_kp1_x, d_kp1_y )
  Eigen::MatrixXd d_kp1(n_foot_edge_*n_, 2*n_);
  Eigen::Ref<Eigen::MatrixXd> d_kp1_x = d_kp1.leftCols(n_);
  Eigen::Ref<Eigen::MatrixXd> d_kp1_y = d_kp1.rightCols(n_);
  Eigen::VectorXd b_kp1(n_foot_edge_*n_);

  d_kp1.setZero();
  b_kp1_.setZero();

  // Change entries according to support order changes in d_kp1
  Eigen::VectorXd theta_vec(nf_ + 1);

  theta_vec[0]=f_k_q_0_;
  for(int i=0 ; i<nf_ ; ++i)
  {
    theta_vec[i+1]=f_k_q_(i);
  }


  Eigen::MatrixXd  a0(n_foot_edge_, 2);
  Eigen::VectorXd  b0(n_foot_edge_);
  Eigen::MatrixXd a0d(n_foot_edge_, 2);
  Eigen::VectorXd b0d(n_foot_edge_);

  Eigen::Matrix2d rot_mat;

  for (int i = 0; i < n_; i++) {

    const double theta = theta_vec(support_deque_[i].step_number);

    // NOTE this changes due to applying the derivative.
    rot_mat << -sin(theta),  cos(theta),
               -cos(theta), -sin(theta);

    if (support_deque_[i].foot == "left") {
      a0  <<    a0lf_*rot_mat;
      b0  <<  ubb0lf_;
      a0d <<   a0dlf_*rot_mat;
      b0d << ubb0dlf_;
    }
    else {
      a0  <<    a0rf_*rot_mat;
      b0  <<  ubb0rf_;
      a0d <<   a0drf_*rot_mat;
      b0d << ubb0drf_;
    }

    // // Get support foot and check if it is double support.
    // for (int j = 0; j < nf_; j++) {
    //   if (v_kp1_(i, j) == 1) {
    //     if (fsm_states_[j] == "D") { // D is never assumed as for now.
    //       a0 << a0d;
    //       b0 << b0d;
    //     }
    //   }
    // }

    for (int k = 0; k < n_foot_edge_; k++) {
      // Get d_i+1^x(f^theta).
      d_kp1_x(i*n_foot_edge_ + k, i) = a0(k, 0);
      // Get d_i+1^y(f^theta).
      d_kp1_y(i*n_foot_edge_ + k, i) = a0(k, 1);
      // Get right hand side of equation.
      b_kp1(i*n_foot_edge_ + k) = b0(k);
    }
  }
  
  // Build constraint transformation matrices.
  // pzuv_ = ( pzuvx_ )
  //         ( pzuvy_ )

  // pzuvx_ = ( pzu_ | -v_kp1_ | 0 | 0 )
  pzuvx_.block(0, pzu_.cols(), v_kp1_.rows(), v_kp1_.cols()) = -v_kp1_.cast<double>();

  // pzuvy_ = ( 0 | 0 | pzu_ | -v_kp1_ )
  pzuvy_.rightCols(v_kp1_.cols()) = -v_kp1_.cast<double>();

  // pzuv_ = ( pzscx_ ) = ( pzs * c_k_x_0_ )
  //         ( pzscy_ )   ( pzs * c_k_y_0_ )
  pzscx_ = pzs_*c_k_x_0_;
  pzscy_ = pzs_*c_k_y_0_;

  // v_kp1fc_ = ( v_kp1fc_x_ ) = ( v_kp1_ * f_k_x_0_ )
  //            ( v_kp1fc_y_ )   ( v_kp1_ * f_k_y_0_ )
  v_kp1fc_x_ = v_kp1_0_.cast<double>() * f_k_x_0_;
  v_kp1fc_y_ = v_kp1_0_.cast<double>() * f_k_y_0_;

  // Build CoP linear constraints.
  Eigen::MatrixXd dummy1(nc_cop_, 2*(n_ + nf_));

  dummy1 = d_kp1*pzuv_;
  dummy1 = dummy1*dofs_.head(2*(n_ + nf_));

  // CoP constraints.
  for (int i = 0; i < nc_cop_; i++) {
    a_pos_q_.block(i, 0, 1, n_) = dummy1.transpose()*derv_a_cop_map_*e_fr_bar_*ppu_;
    a_pos_q_.block(i, n_, 1, n_) = dummy1.transpose()*derv_a_cop_map_*e_fl_bar_*ppu_;
  }











  Eigen::MatrixXi mat_selec = -Eigen::MatrixXi::Ones(nf_, nf_);
  mat_selec = mat_selec.triangularView<Eigen::UnitLower>();

  Eigen::MatrixXd foot_selec(nf_, 2);
  foot_selec.setZero();
  foot_selec(0, 0) = f_k_x_0_;
  foot_selec(0, 1) = f_k_y_0_;

  theta_vec[0]=f_k_q_0_;
  for(int i = 1; i < nf_; i++)
  {
    theta_vec[i]=f_k_q_(i-1);
  }

  // Helping matrices.
  Eigen::MatrixXd x_mat(nc_foot_position_, nf_); // rather nf than 2
  Eigen::MatrixXd  a0_x(nc_foot_position_, nf_); // nf*n_foot_pos_hull_edges = nc_foot_position
  Eigen::MatrixXd y_mat(nc_foot_position_, nf_);
  Eigen::MatrixXd  a0_y(nc_foot_position_, nf_);

  x_mat.setZero();
  a0_x.setZero();
  y_mat.setZero();
  a0_y.setZero();

  Eigen::MatrixXd a_f(n_foot_pos_hull_edges_, 2); // x and y

  // iterate l -> r -> l -> r .... for nf_
  for(int i = 0; i < nf_; i++)
  {
    double theta = theta_vec[i];

    Eigen::Matrix2d rot_mat;

    rot_mat << -cos(theta),  sin(theta),
               -sin(theta), -cos(theta);

    if (support_deque_[i].foot == "left") {

      a_f << a0r_*rot_mat;
    }
    else {

      a_f << a0l_*rot_mat;
    }

    // Set x and y mat.
    x_mat.block(i*n_foot_pos_hull_edges_, i, n_foot_pos_hull_edges_, 1) = a_f.col(0);
    y_mat.block(i*n_foot_pos_hull_edges_, i, n_foot_pos_hull_edges_, 1) = a_f.col(1);
  }

  a0_x  << x_mat*mat_selec.cast<double>();
  a0_y  << y_mat*mat_selec.cast<double>();

  Eigen::MatrixXd dummy2(nc_foot_position_, 2*(n_ + nf_));

  dummy2 << Eigen::MatrixXd::Zero(nc_foot_position_, n_), a0_x,
            Eigen::MatrixXd::Zero(nc_foot_position_, n_), a0_y;

  dummy2 = dummy2*dofs_.head(2*(n_ + nf_));

  // Foot inequality constraints.
  for (int i = nc_cop_; i < nc_cop_ + nc_foot_position_; i++) {
    a_pos_q_.block(i, 0, 1, n_) = dummy2.transpose()*derv_a_foot_map_*e_fr_bar_*ppu_;
    a_pos_q_.block(i, n_, 1, n_) = dummy2.transpose()*derv_a_foot_map_*e_fl_bar_*ppu_;
  }

  // Obstacle position constraints defined 
  // on the horizon.
  // Inequality constraint on both feet u^T H u + A u + B >= 0
  // Jac = 2*H*X + A
  a_obs_.setZero();

  for (int i = 0; i < nc_obs_; i++) {
    for (int j = 0; j < 2*(n_ + nf_); j++) {
        a_obs_(i, j) += BaseGenerator::a_obs_(i, j);

      for (int k = 0; k < 2*(n_ + nf_); k++) {
        a_obs_(i, j) += 2*dofs_(k)*h_obs_(i, k, j);
      }
    }
  }






  // // Foot position constraints defined on
  // // the horizon.
  // // Inequality constraint on both feet Au + B <= 0
  // // A0 R(theta) (Fx_k+1 - Fx_k) <= ubB0
  // //             (Fy_k+1 - Fy_k)
  // Eigen::Matrix2i mat_selec; 
  // mat_selec <<  1., 0.,
  //              -1., 1.;

  // Eigen::Matrix2d foot_selec;
  // foot_selec << f_k_x_0_, f_k_y_0_,
  //               0.      , 0.;

  // // Rotation matrix from F_k+1 to F_k
  // Eigen::Matrix2d rot_mat1;
  // Eigen::Matrix2d rot_mat2;

  // rot_mat1 << -sin(theta_vec(0)),  cos(theta_vec(0)),
  //             -cos(theta_vec(0)), -sin(theta_vec(0));

  // rot_mat2 << -sin(theta_vec(1)),  cos(theta_vec(1)),
  //             -cos(theta_vec(1)), -sin(theta_vec(1));

  // Eigen::MatrixXd a_f1(n_foot_pos_hull_edges_, 2);
  // Eigen::MatrixXd a_f2(n_foot_pos_hull_edges_, 2);

  // if (current_support_.foot == "left") {
  //   a_f1 << a0r_*rot_mat1;
  //   a_f2 << a0l_*rot_mat2;
  // }
  // else {
  //   a_f1 << a0l_*rot_mat1;
  //   a_f2 << a0r_*rot_mat2;
  // }

  // Eigen::MatrixXd tmp1(n_foot_pos_hull_edges_, 2);
  // Eigen::MatrixXd tmp2(n_foot_pos_hull_edges_, 2);
  // Eigen::MatrixXd tmp3(n_foot_pos_hull_edges_, 2);
  // Eigen::MatrixXd tmp4(n_foot_pos_hull_edges_, 2);

  // tmp1 << a_f1.col(0), Eigen::VectorXd::Zero(n_foot_pos_hull_edges_);
  // tmp2 << Eigen::VectorXd::Zero(n_foot_pos_hull_edges_), a_f2.col(0);
  // tmp3 << a_f1.col(1), Eigen::VectorXd::Zero(n_foot_pos_hull_edges_);
  // tmp4 << Eigen::VectorXd::Zero(n_foot_pos_hull_edges_), a_f2.col(1);

  // Eigen::MatrixXd x_mat(2*n_foot_pos_hull_edges_, 2);
  // Eigen::MatrixXd  a0_x(2*n_foot_pos_hull_edges_, 2);
  // Eigen::MatrixXd y_mat(2*n_foot_pos_hull_edges_, 2);
  // Eigen::MatrixXd  a0_y(2*n_foot_pos_hull_edges_, 2);

  // x_mat << tmp1, tmp2;
  // a0_x  << x_mat*mat_selec.cast<double>();
  // y_mat << tmp3, tmp4;
  // a0_y  << y_mat*mat_selec.cast<double>();

  // Eigen::MatrixXd dummy2(nc_foot_position_, 2*(n_ + nf_));

  // dummy2 << Eigen::MatrixXd::Zero(nc_foot_position_, n_), a0_x,
  //           Eigen::MatrixXd::Zero(nc_foot_position_, n_), a0_y;

  // dummy2 = dummy2*dofs_.head(2*(n_ + nf_));

  // // Foot inequality constraints.
  // for (int i = nc_cop_; i < nc_cop_ + nc_foot_position_; i++) {
  //   a_pos_q_.block(i, 0, 1, n_) = dummy2.transpose()*derv_a_foot_map_*e_fr_bar_*ppu_;
  //   a_pos_q_.block(i, n_, 1, n_) = dummy2.transpose()*derv_a_foot_map_*e_fl_bar_*ppu_;
  // }

  // // Obstacle position constraints defined 
  // // on the horizon.
  // // Inequality constraint on both feet u^T H u + A u + B >= 0
  // // Jac = 2*H*X + A
  // a_obs_.setZero();

  // for (int i = 0; i < nc_obs_; i++) {
  //   for (int j = 0; j < 2*(n_ + nf_); j++) {
  //       a_obs_(i, j) += BaseGenerator::a_obs_(i, j);

  //     for (int k = 0; k < 2*(n_ + nf_); k++) {
  //       a_obs_(i, j) += 2*dofs_(k)*h_obs_(i, k, j);
  //     }
  //   }
  // }
}

void NMPCGenerator::SolveQP() {
  // Solve QP first with initialization and after that with hotstart.
  int nwsr_temp  = nwsr_;
  std::vector<double> cpu_time_temp = cpu_time_;

  if (qp_is_initialized_) {
    status_ = qp_.hotstart(qp_h_.data(),
                           qp_g_.data(),
                           qp_a_.data(),
                           qp_lb_.data(),
                           qp_ub_.data(),
                           qp_lba_.data(),
                           qp_uba_.data(),
                           nwsr_temp, cpu_time_temp.data());
  }
  else {
    status_ = qp_.init(qp_h_.data(),
                       qp_g_.data(),
                       qp_a_.data(),
                       qp_lb_.data(),
                       qp_ub_.data(),
                       qp_lba_.data(),
                       qp_uba_.data(),
                       nwsr_temp, cpu_time_temp.data());

    qp_is_initialized_ = true;
  }

  // Orientation primal solution.
  qp_.getPrimalSolution(delta_dofs_.data());
}

void NMPCGenerator::PostprocessSolution() {
  // Get solution and put it back into generator data structures.

  // Extra dofs.
  // dofs = ( dddc_k_x_  ) n_
  //        (    f_k_x_  ) nf_
  //        ( dddc_k_y_  ) n_
  //        (    f_k_y_  ) nf_
  //        ( dddf_k_ql_ ) n_
  //        ( dddf_k_qr_ ) n_

  // Note this time we add an increment to the existing values
  // data(k + 1) = data(k) + alpha*dofs

  // TODO add lines search when problematic.
  double const alpha = 1.;

  dofs_ += alpha_*delta_dofs_.transpose();

  // X values.
  dddc_k_x_ = dofs_.head(n_);
  f_k_x_    = dofs_.segment(n_, nf_);

  // Y values.
  dddc_k_y_ = dofs_.segment(n_ + nf_, n_); 
  f_k_y_    = dofs_.segment(2*n_ + nf_, nf_);

  // Feet orientation.
  dddf_k_ql_ = dofs_.tail(n_);
  dddf_k_qr_ = dofs_.segment(2*(n_ + nf_), n_);
}

void NMPCGenerator::UpdateFootSelectionMatrix() {
  // Get right foot selection matrix.
  int i = 0;
  for (int j = 0; j < n_; j++) {
    derv_a_cop_map_.block(i, j, n_foot_edge_, 1).setConstant(1.);
    i += n_foot_edge_;
  }

  derv_a_foot_map_.setZero();

  i = n_foot_pos_hull_edges_;
  for (int j = 0; j < nf_ - 1; j++) {
    for (int k = 0; k < n_; k++) {
      if (v_kp1_(k, j) == 1) {
        derv_a_foot_map_.block(i, k, n_foot_pos_hull_edges_, 1).setConstant(1.);
        i += n_foot_pos_hull_edges_;
        break;
      }
      else {
        derv_a_foot_map_.block(i, j, n_foot_pos_hull_edges_, 1).setZero();
      }
    }
  }
}

void NMPCGenerator::Reset() {

  // qpOASES specific things.
  options_.setToMPC();
  options_.printLevel = qpOASES::PL_LOW;

  // Problem setup.
  dofs_.setZero();
  delta_dofs_.setZero();

  // Load NMPC options.
  qp_.setOptions(options_);

  qp_h_.setIdentity();
  qp_a_.setZero();
  qp_g_.setZero();
  qp_lb_.setConstant(-1.e+08);
  qp_ub_.setConstant(1.e+08);
  qp_lba_.setConstant(-1.e+08);
  qp_uba_.setConstant(1.e+08);

  qp_is_initialized_ = false;

  // Helper matrices for common expressions.
  q_k_x_.setZero();
  p_k_x_.setZero();
  p_k_y_.setZero();

  q_k_ql_.setZero();
  q_k_qr_.setZero();
  p_k_ql_.setZero();
  p_k_qr_.setZero();

  a_pos_x_.setZero();
  a_pos_q_.setZero();
  uba_pos_.setZero();
  lba_pos_.setZero();

  a_obs_.setZero();
  uba_obs_.setZero();
  lba_obs_.setZero();

  a_ori_.setZero();
  uba_ori_.setZero();
  lba_ori_.setZero();

  derv_a_cop_map_.setZero();
  derv_a_foot_map_.setZero();

  UpdateFootSelectionMatrix(); 

  // Reset the base generator.
  BaseGenerator::Reset();
}
