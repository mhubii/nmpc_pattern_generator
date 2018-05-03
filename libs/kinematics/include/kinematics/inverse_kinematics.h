#ifndef KINEMATICS_INVERSE_KINEMATICS_H_
#define KINEMATICS_INVERSE_KINEMATICS_H_

#include <iostream>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <vector>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

class InverseKinematics
{
    public:

        InverseKinematics(const std::string config_file_loc = "../libs/kinematics/configs.yaml");

        ~InverseKinematics();

        // Perform inverse kinematics on pattern.
        void Invert(Eigen::VectorXd& q_init,
                    Eigen::MatrixXd& com_traj,
                    Eigen::MatrixXd& lf_traj,
                    Eigen::MatrixXd& rf_traj);

        // Get joint angles.
        inline const Eigen::MatrixXd& GetQTraj() const { return q_traj_; };

    public:

        // Configurations.
        YAML::Node configs_;

        // Inverse kinematics initialization.
        bool initialized_;
        const uint n_init_;

        // Model and constraint set.
        RigidBodyDynamics::Model* model_;
        RigidBodyDynamics::InverseKinematicsConstraintSet cs_;

        // Body points.
        Eigen::Vector3d com_bp_;
        Eigen::Vector3d lf_bp_;
        Eigen::Vector3d rf_bp_;

        // Center of mass.
        RigidBodyDynamics::Math::Vector3d com_real_;
        double mass_;

        // Body id's.
        uint com_;
        uint lf_;
        uint rf_;

        // Generalized coordinates of model.
        Eigen::VectorXd q_init_;
        Eigen::VectorXd q_res_;
        Eigen::MatrixXd q_traj_;

        Eigen::VectorXd dq_init_;

        // Initial orientations of the model.
        Eigen::Matrix3d com_ori_init_;
        Eigen::Matrix3d lf_ori_init_;
        Eigen::Matrix3d rf_ori_init_;

        // Orientations to be fulfilled by the model.
        Eigen::Matrix3d com_ori_;
        Eigen::Matrix3d lf_ori_;
        Eigen::Matrix3d rf_ori_;
};

#endif