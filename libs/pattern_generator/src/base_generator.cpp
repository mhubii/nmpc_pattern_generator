#include "base_generator.h"
#include <iostream>

BaseGenerator::BaseGenerator(const std::string config_file_loc)
    : // Configurations
      configs_(YAML::LoadFile(config_file_loc)),
    
      // Constants.
      g_(configs_["gravity"].as<double>()),
      n_(configs_["n"].as<double>()),
      t_(configs_["t"].as<double>()), 
      t_step_(configs_["t_step"].as<double>()),
      t_fb_(configs_["t_feedback"].as<double>()),
      t_window_(n_*t_), nf_(int(t_window_/t_step_)),
      time_(0.),
      
      // Objective weights.
      alpha_(configs_["alpha"].as<double>()),
      beta_(configs_["beta"].as<double>()), 
      gamma_(configs_["gamma"].as<double>()),

      // Center of mass matrices.
        c_kp1_x_(n_),
       dc_kp1_x_(n_), 
      ddc_kp1_x_(n_),

        c_kp1_y_(n_), 
       dc_kp1_y_(n_),
      ddc_kp1_y_(n_),

        c_kp1_q_(n_),
       dc_kp1_q_(n_),
      ddc_kp1_q_(n_),

      // Jerk controls for center of mass.
      dddc_k_x_(n_), 
      dddc_k_y_(n_), 
      dddc_k_q_(n_),

      // Reference matrices.
      dc_kp1_x_ref_(n_), 
      dc_kp1_y_ref_(n_),
      dc_kp1_q_ref_(n_),

      // Feet matrices.
      f_k_x_(nf_),
      f_k_y_(nf_), 
      f_k_q_(nf_),

      // States for the foot orientation.
         f_k_ql_(n_), 
         f_k_qr_(n_),
      
        df_k_ql_(n_), 
        df_k_qr_(n_),
      
       ddf_k_ql_(n_), 
       ddf_k_qr_(n_),
      
      dddf_k_ql_(n_), 
      dddf_k_qr_(n_),

         f_kp1_ql_(n_),
        df_kp1_ql_(n_),
       ddf_kp1_ql_(n_),

         f_kp1_qr_(n_),
        df_kp1_qr_(n_),
       ddf_kp1_qr_(n_),

      f_kp1_q_(n_),

      // Foot angular velocity selection matrices objective.  
      e_f_(n_, 2*n_),
      e_fl_(e_f_.rightCols(n_)),
      e_fr_(e_f_.leftCols(n_)),

      e_f_bar_(n_, 2*n_),
      e_fl_bar_(e_f_bar_.rightCols(n_)),
      e_fr_bar_(e_f_bar_.leftCols(n_)),

      // Zero moment point matrices.
      z_kp1_x_(n_), 
      z_kp1_y_(n_), 

      // Transformation matrices.
      pps_(n_, 3),
      ppu_(n_, n_),
      
      pvs_(n_, 3),
      pvu_(n_, n_),
      
      pas_(n_, 3),
      pau_(n_, n_),
      
      pzs_(n_, 3),
      pzu_(n_, n_),

      // Convex hulls used to bound the free placement of the foot.     
      n_foot_pos_hull_edges_(5),
      lf_pos_hull_(5, 2),
      rf_pos_hull_(5, 2),
      
      // Set of cartesian equalities.
      a0l_(5, 2),
      ubb0l_(5),
      a0r_(5, 2),
      ubb0r_(5),

      // Linear constraints matrix.
      nc_fchange_eq_(2),
      eq_a_foot_(nc_fchange_eq_, 2*(n_ + nf_)),
      eq_b_foot_(nc_fchange_eq_),

      // Linear constraints vector.
      nc_foot_position_(nf_*n_foot_pos_hull_edges_),
      a_foot_(nc_foot_position_, 2*(n_ + nf_)),
      lbb_foot_(nc_foot_position_),
      ubb_foot_(nc_foot_position_),

      // Position of the foot in the local foot frame.
      n_foot_edge_(4),
      foot_width_(configs_["foot_length"].as<double>()),
      foot_height_(configs_["foot_width"].as<double>()),
      foot_distance_(configs_["foot_distance"].as<double>()),

      // Position of the vertices of the feet in the foot coordinates.
      lfoot_(n_foot_edge_, 2),
      rfoot_(n_foot_edge_, 2),

      lf_cop_hull_(n_foot_edge_, 2),
      rf_cop_hull_(n_foot_edge_, 2),
      
      ds_cop_hull_(n_foot_edge_, 2),

      // Corresponding linear system from polygonal set.
      a0rf_(n_foot_edge_, 2),
      ubb0rf_(n_foot_edge_),
      
      a0lf_(n_foot_edge_, 2),
      ubb0lf_(n_foot_edge_),

      a0dlf_(n_foot_edge_, 2),
      ubb0dlf_(n_foot_edge_),

      a0drf_(n_foot_edge_, 2),
      ubb0drf_(n_foot_edge_),

      // Transformation matrix for the constraints in BuildCopConstraints().
      // pzuv_= ( pzuvx_ )
      //        ( pzuvy_ )
      //      = ( pzu_ | -v_kp1_ |   0  |     0 )
      //        (   0  |      0  | pzu_ | -vkp1 )
      pzuv_(2*n_, 2*(n_ + nf_)),
      pzuvx_(pzuv_.topRows(n_)),
      pzuvy_(pzuv_.bottomRows(n_)),

      // pzsc_ = ( pzscx_ )
      //         ( pzscy_ )
      //       = ( pzs_*c_k_x_ + v_kp1_0_*f_k_x_0_ )
      //       = ( pzs_*c_k_y_ + v_kp1_0_*f_k_y_0_ )
      pzsc_(2*n_),
      pzscx_(pzsc_.head(n_)),
      pzscy_(pzsc_.tail(n_)),

      // v_kp1fc_ = ( v_kp1fc_x_ ) = ( v_kp1_ * f_k_x_0_ )
      //            ( v_kp1fc_y_ )   ( v_kp1_ * f_k_y_0_ )
      v_kp1fc_(2*n_),
      v_kp1fc_x_(v_kp1fc_.head(n_)),
      v_kp1fc_y_(v_kp1fc_.tail(n_)),

      // d_kp1_ = ( d_kp1_x_, dkp1_y_ )
      d_kp1_(n_foot_edge_*n_, 2*n_),
      d_kp1_x_(d_kp1_.leftCols(n_)),
      d_kp1_y_(d_kp1_.rightCols(n_)),
      b_kp1_(n_foot_edge_*n_),

      // Constraint matrices.
      nc_cop_(n_*n_foot_edge_),
      a_cop_(nc_cop_, 2*(n_ + nf_)),
      lbb_cop_(nc_cop_),
      ubb_cop_(nc_cop_),

      // Foot rotation constraints.
      nc_fvel_eq_(n_),
      a_fvel_eq_(nc_fvel_eq_, 2*n_),
      b_fvel_eq_(nc_fvel_eq_),

      nc_fpos_ineq_(n_),
      a_fpos_ineq_(nc_fpos_ineq_, 2*n_),
      ubb_fpos_ineq_(nc_fpos_ineq_),
      lbb_fpos_ineq_(nc_fpos_ineq_),

      nc_fvel_ineq_(n_),
      a_fvel_ineq_(nc_fvel_ineq_, 2*n_),
      ubb_fvel_ineq_(nc_fvel_ineq_),
      lbb_fvel_ineq_(nc_fvel_ineq_),

      // Current support state.
      support_deque_(n_),
      
      // Matrices containing constraints representing a 
      // strictly convex obstacle in space.
      obstacle_(configs_["obstacle"].as<bool>()),
      nc_obs_(1*nf_),
      x_obs_(configs_["x_pos"].as<double>()),
      y_obs_(configs_["y_pos"].as<double>()),
      r_obs_(configs_["radius"].as<double>()),
      r_margin_(0.2 + std::max(foot_width_, foot_height_)),
      co_({x_obs_, y_obs_, r_obs_ + r_margin_ , r_margin_}),

      h_obs_(nc_obs_, 2*(n_ + nf_), 2*(n_ + nf_)), // TODO: change for multiple objects!!
      a_obs_(nc_obs_, 2*(n_ + nf_)),
      b_obs_(nc_obs_),
      lb_obs_(nc_obs_),
      ub_obs_(nc_obs_),

      v_kp1_0_(n_),
      v_kp1_(n_, nf_) {

  // Reset the base generator.
  Reset(); 
}

void BaseGenerator::SetSecurityMargin(const double margin_x, const double margin_y) {
  // Define security margins for CoP constraints.
  // NOTE: Will recreate constraint matrices.
  //
  // Parameters
  // ----------
  // margin_x: 0 < double < foot_width_
  //           Security margin to narrow center of pressure
  //           constraints in x direction.
  //
  // margin_y: 0 < double < foot_height_
  //           Security margin to narrow center of pressure
  //           constraints in y direction.

  security_margin_x_ = margin_x;
  security_margin_y_ = margin_y;

  // Update CoP hulls.
  UpdateHulls();

  // Rebuild CoP constraints.
  InitializeConvexHullSystems();

  // Rebuild constraints.
  BuildConstraints();
}

void BaseGenerator::SetInitialValues(PatternGeneratorState& initial_state) {
  // Initial value embedding for pattern generator, i.e. each 
  // iteration of pattern generator differs in:
  //
  // - Initial com state, i.e. com_a = [c_a, dc_a, ddc_a], a in {x,y,q}.
  // - Initial support foot setup.
  //
  // NOTE: Will recreate constraint matrices, support order and 
  //       transformation matrices.
  //
  // Parameters
  // ----------
  //
  // com_x: [pos, vec, acc]
  //        Current x position, velocity, and acceleration of center of mass.
  //
  // com_y: [pos, vec, acc]
  //        Current y position, velocity, and acceleration of center of mass.
  //
  // com_z: double
  //        Current z position of center of mass.
  //
  // foot_x: double
  //         Current x position of support foot.
  //
  // foot_y: double
  //         Current y position of support foot.
  //
  // foot_q: double
  //         Current orientation of support foot.
  //
  // foot: string
  //       Tells actual support foot state, i.e. "right" or "left"
  //
  // com_q: [ang, vec, acc]
  //        Current orienation, angular velocity and acceleration of center of mass.

  // Update CoM states.
  c_k_x_0_ = initial_state.com_x;
  c_k_y_0_ = initial_state.com_y;

  if (h_com_0_ != initial_state.com_z) {
    h_com_0_ = initial_state.com_z;
    InitializeCopMatrices();
  }

  // Update support foot if necessary.
  BaseTypeSupportFoot new_support = {initial_state.foot_x, initial_state.foot_y,
                                     initial_state.foot_q, initial_state.foot};
  if (current_support_ != new_support  ||
      f_k_x_0_ != initial_state.foot_x ||
      f_k_y_0_ != initial_state.foot_y ||
      f_k_q_0_ != initial_state.foot_q) {
    // Take new_support as current_support.
    current_support_ = new_support;

    // Update support foot states.
    f_k_x_0_ = initial_state.foot_x;
    f_k_y_0_ = initial_state.foot_y;
    f_k_q_0_ = initial_state.foot_q;
  }

  // Always recalculate support order.
  CalculateSupportOrder();

  // Update current CoP values.
  z_k_x_0_ = c_k_x_0_(0) - h_com_0_/g_*c_k_x_0_(2);
  z_k_y_0_ = c_k_y_0_(0) - h_com_0_/g_*c_k_y_0_(2);

  // Rebuild all constraints.
  BuildConstraints();
}

void BaseGenerator::SetVelocityReference(Eigen::Vector3d& local_vel_ref) {
  // Velocity reference update and computed from
  // a local frame to a global frame using the
  // current support foot frame.

  // Get feet orientation states from jerks.
  local_vel_ref_ = local_vel_ref;
  f_kp1_ql_ = pps_*f_k_ql_0_ + ppu_*dddf_k_ql_;
  f_kp1_qr_ = pps_*f_k_qr_0_ + ppu_*dddf_k_qr_;

  Eigen::VectorXd flying_foot(n_);
  flying_foot = e_fr_*f_kp1_qr_ + e_fl_*f_kp1_ql_;
  Eigen::VectorXd support_foot(n_);
  support_foot = e_fr_bar_*f_kp1_qr_ + e_fl_bar_*f_kp1_ql_;
  const double q = (flying_foot(0) + support_foot(0))*0.5;
  dc_kp1_x_ref_.setConstant(local_vel_ref(0)*cos(q) - local_vel_ref(1)*sin(q));  
  dc_kp1_y_ref_.setConstant(local_vel_ref(0)*sin(q) + local_vel_ref(1)*cos(q));
  dc_kp1_q_ref_.setConstant(local_vel_ref(2));
}

void BaseGenerator::SetObstacle(Circle& circ) {
  // Set the obstacle.
  co_ = circ;

  // Rebuild the constraints.
  BuildObstacleConstraint();
}

PatternGeneratorState BaseGenerator::Update() {
  // Update all interior matrices and vectors.
  // Has to be used to prepare the QP after each iteration.
  
  // After solution simulate to get current states on horizon.
  Simulate();

  // Update internal time.
  time_ += t_fb_; 

  // Update matrices.
  BaseTypeSupportFoot old_support = current_support_;
  if (time_ >= t_) {
    time_ = 0.;
    UpdateSelectionMatrices();
  }

  if (f_k_x_0_ != current_support_.x ||
      f_k_y_0_ != current_support_.y ||
      f_k_q_0_ != current_support_.q) {
        throw std::invalid_argument("Support foot order not implemented (in base_generator.cpp).");
  }

  // Provide copy of updated state as return value.
  double f_k_q_0_temp = f_k_q_0_;

  // Get data for initialization of next iteration.
  Eigen::Vector3d c_k_x_0(c_kp1_x_(0), dc_kp1_x_(0), ddc_kp1_x_(0));
  Eigen::Vector3d c_k_y_0(c_kp1_y_(0), dc_kp1_y_(0), ddc_kp1_y_(0));
  Eigen::Vector3d c_k_q_0(c_kp1_q_(0), dc_kp1_q_(0), ddc_kp1_q_(0));

  // Left foot.
  f_k_ql_0_(0) =   f_kp1_ql_(0);
  f_k_ql_0_(1) =  df_kp1_ql_(0);
  f_k_ql_0_(2) = ddf_kp1_ql_(0);

  // Right foot.
  f_k_qr_0_(0) =   f_kp1_qr_(0);
  f_k_qr_0_(1) =  df_kp1_qr_(0);
  f_k_qr_0_(2) = ddf_kp1_qr_(0);

  if (current_support_.foot == "left") {
    f_k_q_0_ = f_k_ql_0_(0);
  }
  else {
    f_k_q_0_ = f_k_qr_0_(0);
  }
  current_support_.q = f_k_q_0_;

  SetVelocityReference(local_vel_ref_);

  return {c_k_x_0, c_k_y_0, h_com_0_, f_k_x_0_, f_k_y_0_, f_k_q_0_temp, current_support_.foot, c_k_q_0};
}

PatternGeneratorState BaseGenerator::Update(double dt) {
  // Update all interior matrices and vectors.
  // Has to be used to prepare the QP after each iteration.
  
  // After solution simulate to get current states on horizon.
  Simulate();

  // Update internal time.
  time_ += dt; 

  // Update matrices.
  BaseTypeSupportFoot old_support = current_support_;
  if (time_ >= t_) {
    time_ = 0.;
    UpdateSelectionMatrices();
  }

  if (f_k_x_0_ != current_support_.x ||
      f_k_y_0_ != current_support_.y ||
      f_k_q_0_ != current_support_.q) {
        throw std::invalid_argument("Support foot order not implemented (in base_generator.cpp).");
  }

  // Provide copy of updated state as return value.
  double f_k_q_0_temp = f_k_q_0_;

  // Get data for initialization of next iteration.
  Eigen::Vector3d c_k_x_0(c_kp1_x_(0), dc_kp1_x_(0), ddc_kp1_x_(0));
  Eigen::Vector3d c_k_y_0(c_kp1_y_(0), dc_kp1_y_(0), ddc_kp1_y_(0));
  Eigen::Vector3d c_k_q_0(c_kp1_q_(0), dc_kp1_q_(0), ddc_kp1_q_(0));

  // Left foot.
  f_k_ql_0_(0) =   f_kp1_ql_(0);
  f_k_ql_0_(1) =  df_kp1_ql_(0);
  f_k_ql_0_(2) = ddf_kp1_ql_(0);

  // Right foot.
  f_k_qr_0_(0) =   f_kp1_qr_(0);
  f_k_qr_0_(1) =  df_kp1_qr_(0);
  f_k_qr_0_(2) = ddf_kp1_qr_(0);

  if (current_support_.foot == "left") {
    f_k_q_0_ = f_k_ql_0_(0);
  }
  else {
    f_k_q_0_ = f_k_qr_0_(0);
  }
  current_support_.q = f_k_q_0_;

  SetVelocityReference(local_vel_ref_);

  return {c_k_x_0, c_k_y_0, h_com_0_, f_k_x_0_, f_k_y_0_, f_k_q_0_temp, current_support_.foot, c_k_q_0};
}

void BaseGenerator::InitializeConstantMatrices() {
  for (int i = 0; i < n_; i++) {
    const int n = i + 1;
    pps_.row(i) << 1., n*t_, pow(n*t_, 2)/2;
    pvs_.row(i) << 0.,   1.,            n*t_; 
    pas_.row(i) << 0.,   0.,               1;

    for (int j = 0; j < n_; j++) {
      if (j <= i) {
        ppu_(i, j) = (1. + 3*(i-j) + 3*pow(i-j, 2))*pow(t_, 3)/6.;
        pvu_(i, j) = (1. + 2*(i-j))*pow(t_, 2)/2.;
        pau_(i, j) = t_;
      }
    }
  }
}

void BaseGenerator::InitializeCopMatrices() {
  for (int i = 0; i < n_; i++) {
    const int n = i + 1;
    pzs_.row(i) << 1., n*t_, pow(n*t_, 2)/2. - h_com_0_/g_;

    for (int j = 0; j < n_; j++) {
      if (j <= i) {
      pzu_(i, j) = (1. + 3*(i-j) + 3*pow(i-j, 2))*pow(t_, 3)/6. - t_*h_com_0_/g_;
      }
    }
  }

  // Transformation matrix for the constraints in BuildCopConstraints().
  // pzuv_= ( pzuvx_ )
  //        ( pzuvy_ )
  //      = ( pzu_ | -v_kp1_ |   0  |     0 )
  //        (   0  |      0  | pzu_ | -vkp1 )
  pzuvx_.leftCols(n_) << pzu_;
  pzuvy_.block(0, n_ + nf_, n_, n_) << pzu_;
}

void BaseGenerator::InitializeSelectionMatrices() {
  // Number of time intervalls for prediction
  // that lie within a given support phase.
  const int n_int = int(t_step_/t_);
  v_kp1_0_.head(n_int).setConstant(1);

  for (int j = 0; j < nf_; j++) {
    int const a = std::min((j+1)*n_int, n_);
    int const b = std::min((j+2)*n_int, n_);
    v_kp1_.col(j).segment(a, b-a).setConstant(1);
  }

  CalculateSupportOrder();
}

void BaseGenerator::InitializeConvexHullSystems() {
  // Linear system corresponding to the convex hulls.
  ComputeLinearSystem(lf_pos_hull_, "left", a0l_, ubb0l_);
  ComputeLinearSystem(rf_pos_hull_, "right", a0r_, ubb0r_);

  // Linear system corresponding to the convex hulls.
  // Left foot.
  ComputeLinearSystem(lf_cop_hull_, "left", a0lf_, ubb0lf_);

  // Right foot.
  ComputeLinearSystem(rf_cop_hull_, "right", a0rf_, ubb0rf_);

  // double support.
  ComputeLinearSystem(ds_cop_hull_, "left", a0dlf_, ubb0dlf_);
  ComputeLinearSystem(ds_cop_hull_, "right", a0drf_, ubb0drf_);
}

void BaseGenerator::Reset() {

  // For internal usage.
  time_ = 0.;

  // Resets the whole base generator to its initial values.
  // Center of mass initial values.
  std::vector<double> tmp_com_x = configs_["com_x"].as<std::vector<double>>();
  std::vector<double> tmp_com_y = configs_["com_y"].as<std::vector<double>>();
  std::vector<double> tmp_com_q = configs_["com_q"].as<std::vector<double>>();

  c_k_x_0_ = Eigen::Vector3d::Map(tmp_com_x.data());
  c_k_y_0_ = Eigen::Vector3d::Map(tmp_com_y.data());
  h_com_0_ = configs_["com_z"].as<double>();
  c_k_q_0_ = Eigen::Vector3d::Map(tmp_com_q.data());

  // Center of mass matrices.
    c_kp1_x_.setZero();
   dc_kp1_x_.setZero();
  ddc_kp1_x_.setZero();

    c_kp1_y_.setZero();
   dc_kp1_y_.setZero();
  ddc_kp1_y_.setZero();

    c_kp1_q_.setZero();
   dc_kp1_q_.setZero();
  ddc_kp1_q_.setZero();

  // Jerk controls for center of mass.
  dddc_k_x_.setZero(); 
  dddc_k_y_.setZero(); 
  dddc_k_q_.setZero();

  // Reference matrices.
  dc_kp1_x_ref_.setZero();
  dc_kp1_y_ref_.setZero();
  dc_kp1_q_ref_.setZero();
  local_vel_ref_.setZero();

  // Feet matrices.
  f_k_x_0_ = configs_["foot_x"].as<double>();
  f_k_y_0_ = configs_["foot_y"].as<double>();
  f_k_q_0_ = configs_["foot_q"].as<double>();

  f_k_x_.setZero();
  f_k_y_.setZero();
  f_k_q_.setZero();

  // States for the foot orientation.
     f_k_ql_0_.setZero();
     f_k_qr_0_.setZero();

     f_k_ql_.setZero();
     f_k_qr_.setZero();
  
    df_k_ql_.setZero(); 
    df_k_qr_.setZero();
  
   ddf_k_ql_.setZero();
   ddf_k_qr_.setZero();
  
  dddf_k_ql_.setZero();
  dddf_k_qr_.setZero();

     f_kp1_ql_.setZero();
    df_kp1_ql_.setZero();
   ddf_kp1_ql_.setZero();

     f_kp1_qr_.setZero();
    df_kp1_qr_.setZero();
   ddf_kp1_qr_.setZero();

     f_kp1_q_.setZero();

  // Foot angular velocity selection matrices objective.  
  e_f_.setZero();
  e_fl_.setZero();
  e_fr_.setZero();

  e_f_bar_.setZero();
  e_fl_bar_.setZero();
  e_fr_bar_.setZero();

  // Zero moment point matrices.
  z_k_x_0_ = c_k_x_0_[0] - h_com_0_/g_*c_k_x_0_[2]; 
  z_k_y_0_ = c_k_y_0_[0] - h_com_0_/g_*c_k_y_0_[2];

  z_kp1_x_.setZero(); 
  z_kp1_y_.setZero();

  // Transformation matrices.
  pps_.setZero();
  ppu_.setZero();
  
  pvs_.setZero();
  pvu_.setZero();
  
  pas_.setZero();
  pau_.setZero();
  
  pzs_.setZero();
  pzu_.setZero();

  // Convex hulls used to bound the free placement of the foot.
  std::vector<double> tmp_lf_pos_hull = configs_["left_foot_convex_hull"].as<std::vector<double>>();
  std::vector<double> tmp_rf_pos_hull = configs_["right_foot_convex_hull"].as<std::vector<double>>();
  
  lf_pos_hull_ = Eigen::Map<RowMatrixXd>(tmp_lf_pos_hull.data(), 5, 2);
  rf_pos_hull_ = Eigen::Map<RowMatrixXd>(tmp_rf_pos_hull.data(), 5, 2);

  // Set of cartesian equalities.
  a0l_.setZero();
  ubb0l_.setZero();
  a0r_.setZero();
  ubb0r_.setZero();

  // Linear constraints matrix.
  eq_a_foot_.setZero();
  eq_b_foot_.setZero();

  // Linear constraints vector.
  a_foot_.setZero();
  lbb_foot_.setConstant(-1.e+08);
  ubb_foot_.setZero();

  // Security margins for cop constraints.
  security_margin_x_ = configs_["security_margin"].as<std::vector<double>>()[0];
  security_margin_y_ = configs_["security_margin"].as<std::vector<double>>()[1];

  // Position of the vertices of the feet in the foot coordinates.
  lfoot_.setZero();
  rfoot_.setZero();

  lf_cop_hull_.setZero();
  rf_cop_hull_.setZero();
  
  ds_cop_hull_.setZero();

  UpdateHulls();

  // Corresponding linear system from polygonal set.  
  a0lf_.setZero();
  ubb0lf_.setZero();

  a0rf_.setZero();
  ubb0rf_.setZero();

  a0dlf_.setZero();
  ubb0dlf_.setZero();
  a0drf_.setZero();
  ubb0drf_.setZero();

  // Transformation matrix for the constraints in BuildCopConstraints().
  pzuv_.setZero();
  pzuvx_.setZero();
  pzuvy_.setZero();

  pzsc_.setZero();
  pzscx_.setZero();
  pzscy_.setZero();

  v_kp1fc_.setZero();
  v_kp1fc_x_.setZero();
  v_kp1fc_y_.setZero();

  d_kp1_.setZero();
  d_kp1_x_.setZero();
  d_kp1_y_.setZero();
  b_kp1_.setZero();

  // Constraint matrices.
  a_cop_.setZero();
  lbb_cop_.setConstant(-1.e+08);
  ubb_cop_.setZero();

  // Foot rotation constraints.
  a_fvel_eq_.setZero();
  b_fvel_eq_.setZero();

  a_fpos_ineq_.setZero();
  ubb_fpos_ineq_.setZero();
  lbb_fpos_ineq_.setZero();

  a_fvel_ineq_.setZero();
  ubb_fvel_ineq_.setZero();
  lbb_fvel_ineq_.setZero();

  // Current support state.
  current_support_ = {f_k_x_0_, f_k_y_0_, f_k_q_0_, configs_["support_foot"].as<std::string>()};
  current_support_.time_limit = 0.;
  current_support_.ds = 0.;
  support_deque_[0].ds = 1;
  support_deque_[8].ds = 1;

  // Matrices containing constraints representing a 
  // strictly convex obstacle in space.
  h_obs_.setZero();
  a_obs_.setZero();
  b_obs_.setZero();
  lb_obs_.setZero();
  ub_obs_.setConstant(1.e+08);

  v_kp1_0_.setZero();
  v_kp1_.setZero();

  // Initialize all elementary problem matrices.
  InitializeConstantMatrices();
  InitializeCopMatrices();
  InitializeSelectionMatrices();
  InitializeConvexHullSystems(); 
}

void BaseGenerator::UpdateHulls() {
  // Ranaming for convenience.
  const double fw = foot_width_;
  const double fh = foot_height_;
  const double fd = foot_distance_;

  const double smx = security_margin_x_;
  const double smy = security_margin_y_;

  //        #<-------fw------># --- ---
  //        #<>=smx     smx=<>#  |   |=smy
  //        #  *-----------*  #  |  ---
  //        #  |           |  #  |=fh
  //        #  *-----------*  #  |  ---
  //        #                 #  |   |=smy
  //        #-----------------# --- ---

  // Left foot.
  lfoot_ <<  0.5*fw,  0.5*fh,
             0.5*fw, -0.5*fh,
            -0.5*fw, -0.5*fh,
            -0.5*fw,  0.5*fh;

  // Right foot.
  rfoot_ <<  0.5*fw, -0.5*fh,
             0.5*fw,  0.5*fh,
            -0.5*fw,  0.5*fh,
            -0.5*fw, -0.5*fh;

  // Left foot. 
  lf_cop_hull_ <<  (0.5*fw - smx),  (0.5*fh - smy),
                   (0.5*fw - smx), -(0.5*fh - smy),
                  -(0.5*fw - smx), -(0.5*fh - smy),
                  -(0.5*fw - smx),  (0.5*fh - smy);

  // Right foot.
  rf_cop_hull_ <<  (0.5*fw - smx), -(0.5*fh - smy),
                   (0.5*fw - smx),  (0.5*fh - smy),
                  -(0.5*fw - smx),  (0.5*fh - smy),
                  -(0.5*fw - smx), -(0.5*fh - smy);

  // double support.
  //     |<----d---->| d = 2*fh + fd
  //     |-----------|
  //     | *   *   * |
  //     |-^-------^-|
  //     left     right
  //     foot     foot
  ds_cop_hull_ <<  (0.5*fw - smx),  (0.5*(fd + fh) - smy),
                   (0.5*fw - smx), -(0.5*(fd + fh) - smy),
                  -(0.5*fw - smx), -(0.5*(fd + fh) - smy),
                  -(0.5*fw - smx),  (0.5*(fd + fh) - smy);
}

void BaseGenerator::UpdateSelectionMatrices() {
  // Update selection vector v_kp1_0_ and selection matrix v_kp1_.
  // Therfore, shift foot decision vector and matrix one row up,
  // i.e. the first entry in the selection vector and the first
  // row in the selection matrix drops out and selection vector's
  // dropped first value becomes the last entry in the decision
  // matrix.

  // Save first value for concatenation.
  const double first_entry_v_kp1_0 = v_kp1_0_(0);

  v_kp1_0_.head(n_ - 1) = v_kp1_0_.tail(n_ - 1);
  v_kp1_.topRows(n_ - 1) = v_kp1_.bottomRows(n_ - 1);

  // Clear last row.
  v_kp1_.row(n_ - 1).setZero();

  // Concatenate last entry.
  v_kp1_(n_ - 1, nf_ - 1) = first_entry_v_kp1_0;

  // When first column of selection matrix becomes zero,
  // then shift columns by one to the front.
  if (v_kp1_0_.isZero()) {
    v_kp1_0_ = v_kp1_.col(0);
    v_kp1_.leftCols(nf_ - 1) = v_kp1_.rightCols(nf_ - 1);
    v_kp1_.col(nf_ - 1).setZero();

    // Update support foot.
    f_k_x_0_ = f_k_x_(0);
    f_k_y_0_ = f_k_y_(0);
    f_k_q_0_ = f_k_q_(0);

    current_support_.x = f_k_x_0_;
    current_support_.y = f_k_y_0_;
    current_support_.q = f_k_q_0_;

    if (current_support_.foot == "right") {
      current_support_.foot = "left";
    }
    else {
      current_support_.foot = "right";
    }

    // Update support order with new foot state.
    CalculateSupportOrder();
  }
}

void BaseGenerator::CalculateSupportOrder() {
  std::string pair;
  std::string impair;

  // Find the correct initial support foot.
  if (current_support_.foot == "left") {
    pair = "left";
    impair = "right";
  }
  else {
    pair = "right";
    impair = "left";
  }

  // Define support feet for whole horizon.
  for (int i = 0; i < n_; i++) {
    if (v_kp1_0_(i) == 1) {
      support_deque_[i].foot = current_support_.foot;
      support_deque_[i].step_number = 0;
    }
    else {
      for (int j = 0; j < nf_; j++) {
        const double test = v_kp1_(i, j);

        if (v_kp1_(i, j) == 1) {
          support_deque_[i].step_number = j+1;
          if (j % 2 == 1) {
            support_deque_[i].foot = pair;
          }
          else {
            support_deque_[i].foot = impair;
          }
        }
      }
    }
  }

  if (v_kp1_0_.sum() == 8) {
    support_deque_[0].ds = 1;
  }
  else {
    support_deque_[0].ds = 0;
  }
  for (int i = 1; i < n_; i++) {
    support_deque_[i].ds = support_deque_[i].step_number - support_deque_[i-1].step_number;
  }

  double time_limit = support_deque_[0].time_limit;
  for (int i = 0; i < n_; i++) {
    if (support_deque_[i].ds == 1) {
      time_limit = time_limit + t_step_;
    }
    support_deque_[i].time_limit = time_limit;
  }
}

void BaseGenerator::ComputeLinearSystem(Eigen::MatrixXd& hull, const std::string foot, 
                                        Eigen::MatrixXd& a0, Eigen::VectorXd& b0) {
  // Automatically calculate linear constraints from
  // polygon description.

  // Get number of edges from the hull specification.
  const int n_edges = hull.rows();

  // Get sign for hull from given foot.
  int sign;  
  if (foot == "left") {
    sign = 1;
  }
  else {
    sign = -1;
  }

  // Calculate linear constraints from hull. Walk
  // around polygon and calculate coordinate representation
  // of constraints.
  int k;

  for (int i = 0; i < n_edges; i++) {
    // Special case for first and last entry in hull.
    if (i == n_edges - 1) {
      k = 0;
    }
    else {
      k = i + 1;
    }

    // Calculate support vectors.
    double const dx = hull(i, 1) - hull(k, 1);
    double const dy = hull(k, 0) - hull(i, 0);
    double const dc = dx*hull(i, 0) + dy*hull(i, 1);

    // Symmetrical constraints.
    a0(i, 0) = sign*dx;
    a0(i, 1) = sign*dy;
    b0(i)    = sign*dc;
  }
}

void BaseGenerator::Simulate() {
  // Intergrates model for given initial CoM states, jerks,
  // and feet positions and orientations by applying the 
  // linear time stepping scheme.

  // Get CoM states from jerks.
    c_kp1_x_ = pps_*c_k_x_0_ + ppu_*dddc_k_x_;
   dc_kp1_x_ = pvs_*c_k_x_0_ + pvu_*dddc_k_x_;
  ddc_kp1_x_ = pas_*c_k_x_0_ + pau_*dddc_k_x_;

    c_kp1_y_ = pps_*c_k_y_0_ + ppu_*dddc_k_y_;
   dc_kp1_y_ = pvs_*c_k_y_0_ + pvu_*dddc_k_y_;
  ddc_kp1_y_ = pas_*c_k_y_0_ + pau_*dddc_k_y_;

  // Get feet orientation states from feet jerks.
    f_kp1_ql_ = pps_*f_k_ql_0_ + ppu_*dddf_k_ql_; 
   df_kp1_ql_ = pvs_*f_k_ql_0_ + pvu_*dddf_k_ql_;
  ddf_kp1_ql_ = pas_*f_k_ql_0_ + pau_*dddf_k_ql_;

    f_kp1_qr_ = pps_*f_k_qr_0_ + ppu_*dddf_k_qr_;
   df_kp1_qr_ = pvs_*f_k_qr_0_ + pvu_*dddf_k_qr_;
  ddf_kp1_qr_ = pas_*f_k_qr_0_ + pau_*dddf_k_qr_;

    c_kp1_q_ = 0.5*(  f_kp1_ql_ +   f_kp1_qr_);
   dc_kp1_q_ = 0.5*( df_kp1_ql_ +  df_kp1_qr_);
  ddc_kp1_q_ = 0.5*(ddf_kp1_ql_ + ddf_kp1_qr_);

  // Get support orientation.
  f_kp1_q_ = e_fr_bar_*f_kp1_qr_ + e_fl_bar_*f_kp1_ql_;

  for (int j = 0; j < nf_; j++) {
    for (int i = 0; i < n_; i++) {
      if (v_kp1_(i, j) != 0) {
        f_k_q_(j) = f_kp1_q_(i);
        break;
      }
      else {
        f_k_q_(j) = 0.;
      }
    }
  }

  // Get ZMP states from jerks.
  z_kp1_x_ = pzs_*c_k_x_0_ + pzu_*dddc_k_x_;
  z_kp1_y_ = pzs_*c_k_y_0_ + pzu_*dddc_k_y_;
}

void BaseGenerator::BuildConstraints() {
  //  Builds contraint matrices for solver.
  //
  // NOTE problems are assembled in the solver implementations.
  BuildCopContraint();           
  BuildFootEqConstraint();       
  BuildFootIneqConstraint();     
  BuildFootRotationConstraints();
  BuildRotIneqConstraint();      
  BuildObstacleConstraint();
}

void BaseGenerator::BuildCopContraint() {
  //  Build the constraint enforcing the center of pressure
  //  to stay inside the support polygon given by the convex
  //  hull of the foot.

  // Change entries accoding to support order changes in d_kp1_
  UpdateCopConstraintTransformation();

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
  a_cop_ = d_kp1_*pzuv_;
  ubb_cop_ = b_kp1_ - d_kp1_*pzsc_ + d_kp1_*v_kp1fc_;
}

void BaseGenerator::BuildFootEqConstraint() {
  // Create constraints that freeze the foot position optimization
  // when the swing foot comes cloase to the foot step in preview
  // window. Needer for proper interpolation of trajectory.

  // B <= Ax <= B
  // support_foot(k+1) = support_foot(k)
  const int it_before_landing = v_kp1_0_.sum();
  const int it_before_landing_threshold = 3;
  if (it_before_landing < it_before_landing_threshold) {
    eq_a_foot_(0, n_) = 1.;
    eq_a_foot_(1, 2*n_ + nf_) = 1.;

    eq_b_foot_(0) = f_k_x_(0);
    eq_b_foot_(1) = f_k_y_(0);
  }
  else {
    eq_a_foot_(0, n_) = 0.;
    eq_a_foot_(1, 2*n_ + nf_) = 0.;

    eq_b_foot_(0) = 0.;
    eq_b_foot_(1) = 0.;
  }
}

void BaseGenerator::BuildFootIneqConstraint() {
  // Build linear inequality constraints for the placement
  // of the free. 
  // NOTE: needs actual support_foot_ to work properly.

  // Inequality constraint on both feet Au + B <= 0
  // A0 R(theta) (Fx_k+1 - Fx_k) <= ubB0
  //             (Fy_k+1 - Fy_k)

  Eigen::MatrixXi mat_selec = Eigen::MatrixXi::Identity(nf_, nf_);
  mat_selec.bottomLeftCorner(nf_-1, nf_-1) = -Eigen::MatrixXi::Identity(nf_-1, nf_-1);

  // Eigen::Matrix2d foot_selec;
  // foot_selec << f_k_x_0_, f_k_y_0_,
  //               0.      , 0.;

  Eigen::MatrixXd foot_selec(nf_, 2);
  foot_selec.setZero();
  foot_selec(0, 0) = f_k_x_0_;
  foot_selec(0, 1) = f_k_y_0_;

  Eigen::VectorXd theta_vec(nf_);

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

  Eigen::VectorXd b0_full(nf_*n_foot_pos_hull_edges_);
  Eigen::VectorXd b0(nf_*n_foot_pos_hull_edges_);

  Eigen::MatrixXd a_f(n_foot_pos_hull_edges_, 2); // x and y
  Eigen::VectorXd b_f(n_foot_pos_hull_edges_);

  // iterate l -> r -> l -> r .... for nf_
  for(int i = 0; i < nf_; i++)
  {
    double theta = theta_vec[i];

    Eigen::Matrix2d rot_mat;

    rot_mat << cos(theta), sin(theta),
              -sin(theta), cos(theta);

    if (support_deque_[i].foot == "left") {

      a_f << a0r_*rot_mat;
      b_f << ubb0r_;
    }
    else {

      a_f << a0l_*rot_mat;
      b_f << ubb0l_;
    }

    // Set x and y mat.
    x_mat.block(i*n_foot_pos_hull_edges_, i, n_foot_pos_hull_edges_, 1) = a_f.col(0);
    y_mat.block(i*n_foot_pos_hull_edges_, i, n_foot_pos_hull_edges_, 1) = a_f.col(1);

    // Set constraint.
    b0_full.segment(i*n_foot_pos_hull_edges_, n_foot_pos_hull_edges_) = b_f;
  }

  a0_x  << x_mat*mat_selec.cast<double>();
  a0_y  << y_mat*mat_selec.cast<double>();

  b0 << b0_full + x_mat*foot_selec.col(0) + y_mat*foot_selec.col(1);

  a_foot_ << Eigen::MatrixXd::Zero(nc_foot_position_, n_), a0_x,
             Eigen::MatrixXd::Zero(nc_foot_position_, n_), a0_y;
  ubb_foot_ << b0;



  // // Rotation matrix from F_k+1 to F_k
  // Eigen::Matrix2d rot_mat1;
  // Eigen::Matrix2d rot_mat2;

  // rot_mat1 << cos(theta_vec(0)), sin(theta_vec(0)),
  //            -sin(theta_vec(0)), cos(theta_vec(0));

  // rot_mat2 << cos(theta_vec(1)), sin(theta_vec(1)),
  //             -sin(theta_vec(1)), cos(theta_vec(1));

  // Eigen::MatrixXd a_f1(n_foot_pos_hull_edges_, nf_);
  // Eigen::MatrixXd a_f2(n_foot_pos_hull_edges_, nf_);
  // Eigen::VectorXd b_f1(n_foot_pos_hull_edges_);
  // Eigen::VectorXd b_f2(n_foot_pos_hull_edges_);

  // if (current_support_.foot == "left") {
  //   a_f1 << a0r_*rot_mat1;
  //   a_f2 << a0l_*rot_mat2;
  //   b_f1 << ubb0r_;
  //   b_f2 << ubb0l_;
  // }
  // else {
  //   a_f1 << a0l_*rot_mat1;
  //   a_f2 << a0r_*rot_mat2;
  //   b_f1 << ubb0l_;
  //   b_f2 << ubb0r_;
  // }

  // Eigen::MatrixXd tmp1(n_foot_pos_hull_edges_, nf_);
  // Eigen::MatrixXd tmp2(n_foot_pos_hull_edges_, nf_);
  // Eigen::MatrixXd tmp3(n_foot_pos_hull_edges_, nf_);
  // Eigen::MatrixXd tmp4(n_foot_pos_hull_edges_, nf_);

  // tmp1 << a_f1.col(0), Eigen::VectorXd::Zero(n_foot_pos_hull_edges_);
  // tmp2 << Eigen::VectorXd::Zero(n_foot_pos_hull_edges_), a_f2.col(0);
  // tmp3 << a_f1.col(1), Eigen::VectorXd::Zero(n_foot_pos_hull_edges_);
  // tmp4 << Eigen::VectorXd::Zero(n_foot_pos_hull_edges_), a_f2.col(1);


  // // here needs to be some kind of for loop over all steps nf
  // Eigen::MatrixXd x_mat(nc_foot_position_, nf_); // rather nf than 2
  // Eigen::MatrixXd  a0_x(nc_foot_position_, nf_); // nf*n_foot_pos_hull_edges = nc_foot_position
  // Eigen::MatrixXd y_mat(nc_foot_position_, nf_);
  // Eigen::MatrixXd  a0_y(nc_foot_position_, nf_);

  // x_mat << tmp1, tmp2;
  // a0_x  << x_mat*mat_selec.cast<double>();
  // y_mat << tmp3, tmp4;
  // a0_y  << y_mat*mat_selec.cast<double>();

  // Eigen::VectorXd b0_full(2*n_foot_pos_hull_edges_);
  // Eigen::VectorXd b0(2*n_foot_pos_hull_edges_);

  // b0_full << b_f1, b_f2;
  // b0 << b0_full + x_mat*foot_selec.col(0) + y_mat*foot_selec.col(1);

  // a_foot_ << Eigen::MatrixXd::Zero(nc_foot_position_, n_), a0_x, // how do i introduce for loops for this problem?
  //             Eigen::MatrixXd::Zero(nc_foot_position_, n_), a0_y;
  // ubb_foot_ << b0;
}

void BaseGenerator::BuildFootRotationConstraints() {
  // Constraints that freeze foot orientation for support leg.
  // 0 = e_f_bar_ * df_k_q_
  // <=>
  // ( 0 ) = ( e_fr_bar_        0 ) * ( pvs_ * f_k_qr_ + pvu_ * dddf_k_qr_ )
  // ( 0 )   (        0 e_fl_bar_ ) * ( pvs_ * f_k_ql_ + pvu_ * dddf_k_ql_ )
  // <=>
  // 0 = e_fr_bar_ * pvs_ * f_k_qr_ + e_fr_bar_ * pvu_ * dddf_k_qr_
  // 0 = e_fl_bar_ * pvs_ * f_k_ql_ + e_fl_bar_ * pvu_ * dddf_k_ql_
  // <=>
  // e_fr_bar_ * pvu_ * dddf_k_qr_ = - e_fr_bar_ * pvs_ * f_k_qr_
  // e_fl_bar_ * pvu_ * dddf_k_ql_ = - e_fl_bar_ * pvs_ * f_k_ql_
  // <=>
  // a_fvel_eq_ =
  // (e_fr_bar_ * pvu_                0 ) * dddf_k_qr_
  // (             0   e_fL_bar_ * pvu_ ) * dddf_k_ql_
  // b_rot_eq_ =
  // ( - e_fr_bar_ * pvs_ * f_k_qr_0_)
  // ( - e_fl_bar_ * pvs_ * f_k_ql_0_) 

  // Calculate proper selection matrices.
  UpdateFootSelectionMatrices();

  // Build foot angular velocity constraints.
  // a_fvel_eq_ =
  // (e_fr_bar_ * pvu_              0   ) * dddf_k_qr_
  // (               0 e_fl_bar_ * pvu_ ) * dddf_k_ql_
  // b_fvel_eq_ =
  // ( - e_fr_bar_ * pvs_ * f_k_qr)
  // ( - e_fl_bar_ * pvs_ * f_k_ql)
  a_fvel_eq_.leftCols(n_) << e_fr_bar_*pvu_;
  a_fvel_eq_.rightCols(n_) << e_fl_bar_*pvu_;

  b_fvel_eq_ << - e_fr_bar_*pvs_*f_k_qr_0_ - e_fl_bar_*pvs_*f_k_ql_0_;
}

void BaseGenerator::BuildRotIneqConstraint() {
  // Constraints on relative angular velocity.

  // Calculate proper selection matrices.
  UpdateFootSelectionMatrices();

  // Build foot position constraints
  // || f_kp1_qr_ - f_kp1_ql ||_2^2 <= 0.09 ~ 5 degrees
  // <=>
  // -0.09 <= f_kp1_qr - f_kp1_ql <= 0.09
  // -0.09 - pps_(f_k_qr_ - f_k_ql_) <= Ppu * ( 1 | -1 ) u_k <= 0.09 - pps_(f_k_qr_- f_k_ql_)
  a_fpos_ineq_.leftCols(n_)  <<   Eigen::MatrixXd::Identity(n_, n_);
  a_fpos_ineq_.rightCols(n_) << - Eigen::MatrixXd::Identity(n_, n_);
  a_fpos_ineq_ << ppu_*a_fpos_ineq_;

  ubb_fpos_ineq_ << Eigen::VectorXd::Constant(n_,  0.09) - pps_*(f_k_qr_0_ - f_k_ql_0_);
  lbb_fpos_ineq_ << Eigen::VectorXd::Constant(n_, -0.09) - pps_*(f_k_qr_0_ - f_k_ql_0_);

  // Build foot veloctiy constraints.
  a_fvel_ineq_.leftCols(n_) <<   Eigen::MatrixXd::Identity(n_, n_);
  a_fvel_ineq_.rightCols(n_)<< - Eigen::MatrixXd::Identity(n_, n_);
  a_fvel_ineq_ << pvu_*a_fvel_ineq_;

  ubb_fvel_ineq_ << Eigen::VectorXd::Constant(n_,   0.22) - pvs_*(f_k_qr_0_ - f_k_ql_0_);
  lbb_fvel_ineq_ << Eigen::VectorXd::Constant(n_,  -0.22) - pvs_*(f_k_qr_0_ - f_k_ql_0_);
}

void BaseGenerator::BuildObstacleConstraint() {
  // Constraints coming from obstacles.
  if (obstacle_) {

    // inf > X Hobs X + Aobs X > Bobs
    for (int i = 0; i < nc_obs_; i++) {
      for (int j = 0; j < nf_; j++) {
        h_obs_(i, n_ + j, n_ + j)                 = 1.;
        h_obs_(i, 2*n_ + nf_ + j, 2*n_ + nf_ + j) = 1.;
        a_obs_(i, n_ + j)                         = -2*co_.x0;
        a_obs_(i, 2*n_ + nf_ + j)                 = -2*co_.y0;
        b_obs_(i)                                 = co_.x0*co_.x0 + co_.y0*co_.y0 - co_.r*co_.r;
        lb_obs_(i)                                = -b_obs_(i);
      }
    }
  }
}

void BaseGenerator::UpdateCopConstraintTransformation() {
  // Update foot constraint tranformation matrices.

  // Every time instant in the pattern generator constraints
  // depends on the support order.
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
    rot_mat <<  cos(theta), sin(theta),
               -sin(theta), cos(theta);

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

    for (int k = 0; k < n_foot_edge_; k++) {
      // Get d_i+1^x(f^theta).
      d_kp1_x_(i*n_foot_edge_ + k, i) = a0(k, 0);
      // Get d_i+1^y(f^theta).
      d_kp1_y_(i*n_foot_edge_ + k, i) = a0(k, 1);
      // Get right hand side of equation.
      b_kp1_(i*n_foot_edge_ + k) = b0(k);
    }
  }
}

void BaseGenerator::UpdateFootSelectionMatrices() {
  // Update the foot selection matrices e_f_ and e_f_bar_
  e_fr_.setZero();
  e_fr_bar_.setZero();
  e_fl_.setZero();
  e_fl_bar_.setZero();

  for (int i = 0; i < support_deque_.size(); i++) {
    if (support_deque_[i].foot == "left") {
      e_fl_(i, i)     = 0.;
      e_fr_(i, i)     = 1.;
      
      e_fl_bar_(i, i) = 1.;
      e_fr_bar_(i, i) = 0.;
    }
    else {
      e_fl_(i, i)     = 1.;
      e_fr_(i, i)     = 0.;
      
      e_fl_bar_(i, i) = 0.;
      e_fr_bar_(i, i) = 1.;
    }
  }
}
