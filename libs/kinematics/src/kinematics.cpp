#include "kinematics.h"

Kinematics::Kinematics(const std::string config_file_loc) 
  
  : // Configurations.
    configs_(YAML::LoadFile(config_file_loc)),
    
    // Inverse kinematics initialization. 
    initialized_(false),
    n_init_(configs_["n_init"].as<uint>()) {

        // Load kinematic model from urdf file.
        model_ = new RigidBodyDynamics::Model();
        RigidBodyDynamics::Addons::URDFReadFromFile(configs_["urdf_loc"].as<std::string>().c_str(), model_, true, false);

        // Set optimization parameters.
        cs_.step_tol = configs_["step_tol"].as<double>();
        cs_.lambda = configs_["lambda"].as<double>();
        cs_.num_steps = configs_["num_steps"].as<uint>();

        // Initialize the joint angles.
        q_init_ = Eigen::VectorXd::Zero(model_->dof_count);
        q_res_ = Eigen::VectorXd::Zero(model_->dof_count);
        q_traj_ = Eigen::MatrixXd::Zero(model_->dof_count, 1);

        dq_init_ = Eigen::VectorXd::Zero(model_->dof_count);
}


Kinematics::~Kinematics() {
    
    // Delete model.
    delete model_;
}


void Kinematics::Inverse(Eigen::VectorXd& q_init,
                         Eigen::MatrixXd& com_traj,
                         Eigen::MatrixXd& lf_traj,
                         Eigen::MatrixXd& rf_traj) {

    // Resize q_traj if needed.
    if (q_traj_.rows() !=  model_->dof_count || q_traj_.cols() != com_traj.cols()) {

        q_traj_.resize(model_->dof_count, com_traj.cols());
    }

    // Check if inverse kinematics has gotten initialized.
    if (initialized_) {
        
        for (int i = 0; i < com_traj.cols(); i++) {

            // Use the real com as body point.
            RigidBodyDynamics::Utils::CalcCenterOfMass(*model_, q_init_, dq_init_, NULL, mass_, com_);

            cs_.body_points[com_id_] = RigidBodyDynamics::CalcBaseToBodyCoordinates(*model_, q_init_, model_->GetBodyId("chest"), com_);
    
            // Set position constraints.
            cs_.target_positions[com_id_] = 0.5*(lf_ori_init_ + rf_ori_init_)*com_traj.block(i, 0, 3, 1);
            cs_.target_positions[lf_id_]  = lf_ori_init_*lf_traj.block(i, 0, 3, 1);
            cs_.target_positions[lf_id_]  = rf_ori_init_*rf_traj.block(i, 0, 3, 1);

            // Set orientation constraints.
            com_ori_ = Eigen::AngleAxisd(com_traj(3, i), Eigen::Vector3d::UnitZ());
            lf_ori_ = Eigen::AngleAxisd(lf_traj(3, i), Eigen::Vector3d::UnitZ());
            rf_ori_ = Eigen::AngleAxisd(rf_traj(3, i), Eigen::Vector3d::UnitZ());

            cs_.target_orientations[com_id_] = com_ori_init_*com_ori_;
            cs_.target_orientations[lf_id_] = lf_ori_init_*lf_ori_;
            cs_.target_orientations[lf_id_] = rf_ori_init_*rf_ori_;

            // Inverse kinematics.
            if (!RigidBodyDynamics::InverseKinematics(*model_, q_init_, cs_, q_res_)) {
                std::cout << "Inverse kinematics did not converge with desired precision." << std::endl;
            }

            q_init_ = q_res_;
            q_traj_.col(i) = q_res_;
        }
    }
    else {

        // Take the initial orientation of the model as offset to the orientation of the generated pattern.
        com_ori_init_ = RigidBodyDynamics::CalcBodyWorldOrientation(*model_, q_init_, model_->GetBodyId("chest"));
        lf_ori_init_ = RigidBodyDynamics::CalcBodyWorldOrientation(*model_, q_init_, model_->GetBodyId("l_sole"));
        rf_ori_init_ = RigidBodyDynamics::CalcBodyWorldOrientation(*model_, q_init_, model_->GetBodyId("r_sole"));

        // Body points.
        com_bp_ = Eigen::Vector3d::Map(configs_["com_body_point"].as<std::vector<double>>().data());
        lf_bp_ = Eigen::Vector3d::Map(configs_["lf_body_point"].as<std::vector<double>>().data());
        rf_bp_ = Eigen::Vector3d::Map(configs_["rf_body_point"].as<std::vector<double>>().data());

        // Body orientations.
        com_ori_ = Eigen::AngleAxisd(com_traj(3, 0), Eigen::Vector3d::UnitZ());
        lf_ori_ = Eigen::AngleAxisd(lf_traj(3, 0), Eigen::Vector3d::UnitZ());
        rf_ori_ = Eigen::AngleAxisd(rf_traj(3, 0), Eigen::Vector3d::UnitZ());

        // Add constraints.
        com_id_ = cs_.AddFullConstraint(model_->GetBodyId("chest"), com_bp_, 0.5*(lf_ori_init_ + rf_ori_init_)*com_traj.block(0, 0, 3, 1), com_ori_init_*com_ori_);
        lf_id_ = cs_.AddFullConstraint(model_->GetBodyId("l_sole"), lf_bp_, lf_ori_init_*lf_traj.block(0, 0, 3, 1), lf_ori_init_*lf_ori_);
        lf_id_ = cs_.AddFullConstraint(model_->GetBodyId("r_sole"), rf_bp_, rf_ori_init_*rf_traj.block(0, 0, 3, 1), rf_ori_init_*rf_ori_);

        // Initial guess.
        q_init_ = q_init;

        // Pre-initialize inverse kinematics.
        for (int i = 0; i < n_init_; i++) {

            // Update the angles of the joints q iteratively, such that the body point 
            // represents the real center of mass of the robot.
            if (!RigidBodyDynamics::InverseKinematics(*model_, q_init_, cs_, q_res_)) {
                std::cout << "Inverse kinematics did not converge with desired precision." << std::endl;
            }
    
            q_init_ = q_res_;
            q_traj_.col(0) = q_res_;
    
            RigidBodyDynamics::Utils::CalcCenterOfMass(*model_, q_init_, dq_init_, NULL, mass_, com_);

            cs_.body_points[com_id_] = RigidBodyDynamics::CalcBaseToBodyCoordinates(*model_, q_init_, model_->GetBodyId("chest"), com_);
        }

        initialized_ = true;
    }
}
