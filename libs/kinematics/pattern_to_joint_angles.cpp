#include "main.h"
#include "../libs/pattern_generator/include/pattern_generator/utils.h"

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>


void getEulerAngles(Eigen::Matrix3d& R, Eigen::Vector3d& angles)
{
    angles(0) = atan2(R(1,2),R(2,2));
    angles(1) = -asin(R(0,2));
    angles(2) = atan2(R(0,1),R(0,0));
}


// RPatternToJointAngles() is a first try of converting a generated pattern
// via InverseKinematics() to a joint angle trajectory.
void PatternToJointAngles() {
    // Initialize RBDL model with urdf file.
    RigidBodyDynamics::Model* model = new RigidBodyDynamics::Model();

    const std::string model_path = "/home/mhuber/IK_walking/model_files/iCubHeidelberg01_no_weights.urdf";
    RigidBodyDynamics::Addons::URDFReadFromFile(model_path.c_str(), model, true, false);

    // Load generated pattern. The pattern is stored in the order:
    //           com                    zmp       lf         rf
    // x dx ddx y dy ddy z q dq ddq    x y z    x y z q    x y z q
    const std::string path_in = "/home/mhuber/robotics/results/nmpc_pattern/example_nmpc_generator_interpolated_results.csv";
    Eigen::MatrixXd pattern = ReadCsv<Eigen::MatrixXd>(path_in);

    Eigen::MatrixXd com_traj(pattern.rows(), 4);
    Eigen::MatrixXd lf_traj(pattern.rows(), 4);
    Eigen::MatrixXd rf_traj(pattern.rows(), 4);

    com_traj << pattern.col(0), pattern.col(3), pattern.col(6), pattern.col(7);
    lf_traj  << pattern.col(13), pattern.col(14), pattern.col(15), pattern.col(16);
    rf_traj  << pattern.col(17), pattern.col(18), pattern.col(19), pattern.col(20);

    // Constraints according to the generated pattern.
    RigidBodyDynamics::InverseKinematicsConstraintSet cs;

    // Options.
    cs.step_tol  = 1e-6;
    cs.num_steps = 100;
    cs.lambda    = 1e-03;

    Eigen::Vector3d com_bp(0., -0.2, 0.);
    Eigen::Vector3d lf_bp(0.01, 0., 0.);
    Eigen::Vector3d rf_bp(0.01, 0., 0.);

    Eigen::VectorXd zero(model->dof_count);
    zero.setZero();

    // Take the initial orientation of the model as offset to the orientation of the generated pattern.
    Eigen::Matrix3d com_ori_off = RigidBodyDynamics::CalcBodyWorldOrientation(*model, zero, model->GetBodyId("chest"));
    Eigen::Matrix3d lf_ori_off  = RigidBodyDynamics::CalcBodyWorldOrientation(*model, zero, model->GetBodyId("l_sole"));
    Eigen::Matrix3d rf_ori_off  = RigidBodyDynamics::CalcBodyWorldOrientation(*model, zero, model->GetBodyId("r_sole"));

    Eigen::Matrix3d com_ori;
    Eigen::Matrix3d lf_ori;
    Eigen::Matrix3d rf_ori;

    com_ori = Eigen::AngleAxisd(com_traj(0, 3), Eigen::Vector3d::UnitZ());
    lf_ori = Eigen::AngleAxisd(lf_traj(0, 3), Eigen::Vector3d::UnitZ());
    rf_ori = Eigen::AngleAxisd(rf_traj(0, 3), Eigen::Vector3d::UnitZ());

    // We only want to rotate the com_traj in the xy-plane.
    uint com = cs.AddFullConstraint(model->GetBodyId("chest"), com_bp, 0.5*(lf_ori_off + rf_ori_off)*com_traj.block(0, 0, 1, 3).transpose(), com_ori_off*com_ori);
    uint lf  = cs.AddFullConstraint(model->GetBodyId("l_sole"), lf_bp, lf_ori_off*lf_traj.block(0, 0, 1, 3).transpose(), lf_ori_off*lf_ori);
    uint rf  = cs.AddFullConstraint(model->GetBodyId("r_sole"), rf_bp, rf_ori_off*rf_traj.block(0, 0, 1, 3).transpose(), rf_ori_off*rf_ori);

    // Joint angles.
    Eigen::VectorXd q_init(model->dof_count);
    Eigen::VectorXd q_res(model->dof_count);
    Eigen::MatrixXd q_traj(pattern.rows(), model->dof_count);

    // Set to a fair initial guess.
    q_init(2)  = 0.6;
    q_init(6)  = 0.54;
    q_init(9)  = -0.57;
    q_init(10) = -0.23;
    q_init(12) = 0.54;
    q_init(15) = -0.57;
    q_init(16) = -0.23;

    Eigen::VectorXd dq_init(model->dof_count);
    RigidBodyDynamics::Math::Vector3d com_real;
    double mass = 0.;

    // Perform some initial inverse kinematics to get a closer q_init and real com.
    for(int i = 0; i < 20; i++) {
        RigidBodyDynamics::InverseKinematics(*model, q_init, cs, q_res);
  
        q_init = q_res;
  
        RigidBodyDynamics::Utils::CalcCenterOfMass(*model, q_init, dq_init, mass, com_real);

        cs.body_points[com] = RigidBodyDynamics::CalcBaseToBodyCoordinates(*model, q_init, model->GetBodyId("chest"), com_real);
    }

    // Log the time.
    Eigen::VectorXd time(pattern.rows());
    double t = 0.;

    // Event loop.
    for (int i = 0; i < pattern.rows(); i++) {
        // Use the real com as body point.
        RigidBodyDynamics::Utils::CalcCenterOfMass(*model, q_init, dq_init, mass, com_real);

        cs.body_points[com] = RigidBodyDynamics::CalcBaseToBodyCoordinates(*model, q_init, model->GetBodyId("chest"), com_real);
  
        // Set position constraints.
        cs.target_positions[com] = 0.5*(lf_ori_off + rf_ori_off)*com_traj.block(i, 0, 1, 3).transpose();
        cs.target_positions[lf]  = lf_ori_off*lf_traj.block(i, 0, 1, 3).transpose();
        cs.target_positions[rf]  = rf_ori_off*rf_traj.block(i, 0, 1, 3).transpose();

        // Set orientation constraints.
        com_ori = Eigen::AngleAxisd(-com_traj(i, 3), Eigen::Vector3d::UnitZ());
        lf_ori = Eigen::AngleAxisd(-lf_traj(i, 3), Eigen::Vector3d::UnitZ());
        rf_ori = Eigen::AngleAxisd(-rf_traj(i, 3), Eigen::Vector3d::UnitZ());

        cs.target_orientations[com] = com_ori_off*com_ori;
        cs.target_orientations[lf] = lf_ori_off*lf_ori;
        cs.target_orientations[rf] = rf_ori_off*rf_ori;

        // Inverse kinematics.
        RigidBodyDynamics::InverseKinematics(*model, q_init, cs, q_res);

        q_traj.row(i) = q_res.transpose();
        q_init = q_res;
        time(i) = t;
        t += 0.01;
    }

    // Save results to .csv file and format it for meshup to read it properly.
    // Eigen::MatrixXd result(pattern.rows(), model->dof_count + 1);
    // result << time, q_traj;

    // const std::string path_out = "/home/mhuber/robotics/results/nmpc_pattern/vx_0.02_tstep_2.0_tds_1.0.csv";

    // const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", ",\n");
    // std::ofstream file(path_out.c_str());
    // file << "COLUMNS:\n"
    //      << "time,\n"
    //      << "root_link:T:X,\n"
    //      << "root_link:T:Y,\n"
    //      << "root_link:T:Z,\n"
    //      << "root_link:R:X:rad,\n"
    //      << "root_link:R:Y:rad,\n"
    //      << "root_link:R:Z:rad,\n"
    //      << "l_hip_1:R:X:rad,\n"
    //      << "l_hip_2:R:-Z:rad,\n"
    //      << "l_upper_leg:R:Y:rad,\n"
    //      << "l_lower_leg:R:X:rad,\n"
    //      << "l_ankle_1:R:-X:rad,\n"
    //      << "l_ankle_2:R:-Z:rad,\n"
    //      << "r_hip_1:R:-X:rad,\n"
    //      << "r_hip_2:R:-Z:rad,\n"
    //      << "r_upper_leg:R:-Y:rad,\n"
    //      << "r_lower_leg:R:-X:rad,\n"
    //      << "r_ankle_1:R:X:rad,\n"
    //      << "r_ankle_2:R:-Z:rad\n"
    //      << "torso_1:R:X:rad,\n"
    //      << "torso_2:R:-Z:rad,\n"
    //      << "chest:R:-Y:rad\n"
    //      << "DATA:\n"
    //      << result.format(CSVFormat);

    WriteCsv("/home/mhuber/robotics/results/nmpc_pattern/vx_0.01_tstep_2.0_tds_1.0.csv", q_traj.rightCols(15));


    // Delete allocated memory.
    delete model;
}