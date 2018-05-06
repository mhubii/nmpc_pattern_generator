#ifndef KINEMATICS_KINEMATICS_H_
#define KINEMATICS_KINEMATICS_H_

#include <iostream>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <vector>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

class Kinematics
{
    public:

        Kinematics(const std::string config_file_loc = "../libs/kinematics/configs.yaml");

        ~Kinematics();

        // Perform forward kinematics.
        void Forward(Eigen::VectorXd& q);

        // Perform inverse kinematics on pattern.
        void Inverse(Eigen::VectorXd& q_init,
                     Eigen::MatrixXd& com_traj,
                     Eigen::MatrixXd& lf_traj,
                     Eigen::MatrixXd& rf_traj);

        // Get joint angles, center of mass and other things.
        inline const RigidBodyDynamics::Math::Vector3d GetCom()   const { return com_; };
        inline const Eigen::MatrixXd&                  GetQTraj() const { return q_traj_; };

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
        RigidBodyDynamics::Math::Vector3d com_;
        double mass_;

        // Body id's.
        uint com_id_;
        uint lf_id_;
        uint rf_id_;

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