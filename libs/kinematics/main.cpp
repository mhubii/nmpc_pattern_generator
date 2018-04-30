/**
 * Author: Yue Hu
 * This code performs a parsing of the parameters file that is updated from the pattern generator in matlab.
 * The feet and com trajectories are feed to the inverse kinematics to obtain the joints trajectories.
 */

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string.h>

#include <rbdl/rbdl.h>
// #include <IK.h>

#include "SplineInterpolator.h"
#include "UtilityFunctions.h"
#include "Model_builder.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

struct params
{
    double ts;
    double z_c;
    int N_shoot;
    int N_steps;
    double step_duration;
    double step_width;
    double step_length;
    double step_height;
    double com_height;
    double t_switch;
    double theta;
    int type;
} paramsList;

void getEulerAngles(Matrix3d R, Vector3d& angles, std::string order)
{
  if(order.compare("123") == 0)
  {
    angles[0] = atan2(R(1,2),R(2,2));
    angles[1] = -asin(R(0,2));
    angles[2] = atan2(R(0,1),R(0,0));
  } else if(order.compare("321") == 0)
  {
    angles[0] = atan2(-R(1,0),R(0,0));
    angles[1] = asin(R(2,0));
    angles[2] = atan2(-R(2,1),R(2,2));
  }
}

//by Kevin
MatrixNd CalcOrientationEulerXYZ (
    const VectorNd &input,
    std::string order
    ) {
    Matrix3d R1 = Matrix3dZero;
    Matrix3d R2 = Matrix3dZero;
    Matrix3d R3 = Matrix3dZero;
    
    Matrix3d result = Matrix3dZero;
    
    if(order.compare("123")==0)
    {
      R1 = rotx(input[0]);
      R2 = roty(input[1]);
      R3 = rotz(input[2]);
      result = R1*R2*R3;
    }
    else if(order.compare("321")==0)
    {
      R1 = rotx(input[2]);
      R2 = roty(input[1]);
      R3 = rotz(input[0]);
      result = R3*R2*R1;
    } else
    {
      std::cout << "Order " << order << " not implemented yet!" << std::endl;
      abort();
    }

    return result;
}

bool parseFromFile(char* argv[], params &paramsList);

double getRobotMass(RigidBodyDynamics::Model &model);
void IK_PG(Model& model, double ts, double step_duration, double switch_time, int N_steps);

// global variable where to store the path where you are reading the input file from
// note: the output files are being saved in the same place
string file_path = "/home/mhuber/IK_walking/WalkTrajectories";

int main (int argc, char* argv[])
{
    string filename;
#ifdef __APPLE__
    // Actually if compiled on OS X using xcode
    filename = "../../model_files/iCubHeidelberg01_no_weights.urdf";
#else
    filename = "../model_files/iCubHeidelberg01_no_weights.urdf";
//     filename = "../model_files/iCubGenova01.urdf";
#endif
  Model_builder builder(filename,1);
  
  // for Genova01 and Genova02
  std::vector<std::string> active_joints(16);
  active_joints[0] = "l_hip_pitch";
  active_joints[1] = "l_hip_roll";
  active_joints[2] = "l_hip_yaw";
  active_joints[3] = "l_knee";
  active_joints[4] = "l_ankle_pitch";
  active_joints[5] = "l_ankle_roll";
  active_joints[6] = "r_hip_pitch";
  active_joints[7] = "r_hip_roll";
  active_joints[8] = "r_hip_yaw";
  active_joints[9] = "r_knee";
  active_joints[10] = "r_ankle_pitch";
  active_joints[11] = "r_ankle_roll";
  active_joints[12] = "torso_pitch";
  active_joints[13] = "torso_roll";
  active_joints[14] = "torso_yaw";
  active_joints[15] = "base_joint";
  
  builder.setActiveJoints(active_joints);

  RigidBodyDynamics::Model icub_model;
  builder.build_model(&icub_model,false);
  
  cout << "Files path: " << string(argv[1]) << endl;
  file_path = string(argv[1]);
    
  // Parse parameters from walkingParams.txt
  if (!parseFromFile(argv, paramsList))
  {
      cout<<"ERROR parsing walkingParams.txt"<<endl;
      return 1;
  }
  
  // model debug output
  cout << RigidBodyDynamics::Utils::GetModelHierarchy(icub_model) << endl;
  cout << "Mass: " << getRobotMass(icub_model) << endl;
  
  // perform inverse kinematics
  IK_PG(icub_model,paramsList.ts,paramsList.step_duration,paramsList.t_switch,paramsList.N_steps);

  return 0;
}

double getRobotMass(Model &model)
{
  //Compute total mass of the robot
  double robotMass = 0;
  for (int i = 0; i < model.mBodies.size(); i++)
    robotMass += model.mBodies[i].mMass;
  
  return robotMass;
}

// Inverse kinematics for pattern generator test
void IK_PG(RigidBodyDynamics::Model& model, double ts, double step_duration, double switch_time, int N_steps)
{
  int step_N = step_duration/ts;
  int switch_N = switch_time/ts;
  int N = N_steps*step_N + 3*switch_N + 1;
  
  //NOTE hack for Debora's 3d sea walker
  // Original time as from IOC 363
//   int N = 1797;
  
  // quantities to store trajectories read from file
  RigidBodyDynamics::Math::VectorNd temp = RigidBodyDynamics::Math::VectorNd::Zero(6);
  vector<RigidBodyDynamics::Math::VectorNd> l_foot(N,temp);
  vector<RigidBodyDynamics::Math::VectorNd> r_foot(N,temp);
  vector<RigidBodyDynamics::Math::VectorNd> com(N,temp);
  RigidBodyDynamics::Math::VectorNd time_vec(N);
  
  // read the feet trajectories
  readFromCSV(file_path + "/l_foot_traj.csv",l_foot);
  readFromCSV(file_path + "/r_foot_traj.csv",r_foot);
  readFromCSV(file_path + "/com_traj.csv",com);
  // initial guess of the joint configuration
  RigidBodyDynamics::Math::VectorNd qinit = RigidBodyDynamics::Math::VectorNd::Zero(model.dof_count);
  // result of the inverse kinematics
  RigidBodyDynamics::Math::VectorNd qres = RigidBodyDynamics::Math::VectorNd::Zero(model.dof_count);
  // store all the resulting configurations
  vector<RigidBodyDynamics::Math::VectorNd> res(N,qres);
  
  
  ///////// Info for the constraint sets  
  string ort_order = "123";
  
  // these outputs are useful to know how the bodies are oriented wrt world in the zero position
  Vector3d l_foot_ang = Vector3dZero;
  getEulerAngles(CalcBodyWorldOrientation(model,VectorNd::Zero(model.dof_count),model.GetBodyId("l_sole")), l_foot_ang, ort_order); 
  Vector3d r_foot_ang = Vector3dZero;
  getEulerAngles(CalcBodyWorldOrientation(model,VectorNd::Zero(model.dof_count),model.GetBodyId("r_sole")), r_foot_ang, ort_order); 
  Vector3d chest_ang = Vector3dZero;
  getEulerAngles(CalcBodyWorldOrientation(model,VectorNd::Zero(model.dof_count),model.GetBodyId("chest")), chest_ang, ort_order); 
   
  cout << "l_foot_ang " << l_foot_ang.transpose() << endl;
  cout << "r_foot_ang " << r_foot_ang.transpose() << endl;
  cout << "chest " << chest_ang.transpose() << endl;
  
  // body ids of the bodies we want to use with IK
  std::vector<unsigned int> body_ids(3);
  // the points attached to the respective bodies
  std::vector<Vector3d> body_points(3);
  // taget positions in world ref frame of the specified points
  std::vector<Vector3d> target_pos(3);
  // target orientation
  std::vector<Matrix3d> target_orientation(4);
  
  // we want to match left and right feet and a point attached to the root as "CoM"
  // since IK of RBDL does not have the concept of orientation because it uses points, we need at least 3 points on each foot to ensure the foot comes flat on the ground
  body_ids[0] = model.GetBodyId("l_sole");
  body_ids[1] = model.GetBodyId("r_sole");
  body_ids[2] = model.GetBodyId("chest");
  body_points[0] = Vector3d(0.01,0.00,0.0);
  body_points[1] = Vector3d(0.01,0.00,0.0);
//   body_points[2] = RigidBodyDynamics::Math::Vector3d(-0.015,0,-0.12); // "CoM", fixed wrt root_link
  body_points[2] = RigidBodyDynamics::Math::Vector3d(0,-0.15,0); // "CoM", fixed wrt chest
  
  // set initial targets
// Heidelberg
  Vector3d l_foot_ort = Vector3d(l_foot_ang[0],l_foot_ang[1]+DEG2RAD(-l_foot[0][4]),l_foot_ang[2]);
  Vector3d r_foot_ort = Vector3d(r_foot_ang[0],r_foot_ang[0]+DEG2RAD(-r_foot[0][4]),r_foot_ang[2]);
  Vector3d com_ort = Vector3d(chest_ang[0],chest_ang[1],chest_ang[2]); // chest, 90,0,-90 Heidelberg, 90,0,-180 Genova01
  target_orientation[0] = CalcOrientationEulerXYZ(l_foot_ort,ort_order); //l_foot
  target_orientation[1] = CalcOrientationEulerXYZ(r_foot_ort,ort_order); //r_foot
  target_orientation[2] = CalcOrientationEulerXYZ(com_ort,ort_order); //chest
  target_pos[0] = l_foot[0].block(0,0,3,1);
  target_pos[1] = r_foot[0].block(0,0,3,1);
  target_pos[2] = com[0].block(0,0,3,1);
  
  
  /// Inverse kinematics with constraint sets
  InverseKinematicsConstraintSet CS;
  CS.step_tol = 1e-12;
  CS.num_steps = 100;
  CS.lambda = 1e-03;
  unsigned int left_foot = CS.AddFullConstraint(model.GetBodyId("l_sole"), body_points[0], target_pos[0], target_orientation[0]);
  unsigned int right_foot = CS.AddFullConstraint(model.GetBodyId("r_sole"), body_points[1], target_pos[1], target_orientation[1]);
  unsigned int Center_of_Mass = CS.AddFullConstraint(model.GetBodyId("chest"), body_points[2], target_pos[2], target_orientation[2]);
  
  // set initial position close to the target to avoid weird solutions
  qinit[2] = 0.6;
  qinit[6] = 0.54;
  qinit[9] = -0.57;
  qinit[10] = -0.23;
  qinit[12] = 0.54;
  qinit[15] = -0.57;
  qinit[16] = -0.23;
  
  vector<VectorNd> com_real(N,Vector3dZero);
  // for the real com
  VectorNd qdot = VectorNd::Zero(model.dof_count);
  double mass = 0;
  Vector3d com_temp = Vector3dZero;
  
  // perform some initial IK to get a closer qinit and real com
  int trials = 10;
  for(int k = 0; k < trials; k++)
  {    
    if (!InverseKinematics(model, qinit, CS, qres))
    {
        std::cout << "![WARNING] RigidBodyDynamics::InverseKinematics - Could not converge to a solution with the desired tolerance of " << CS.step_tol << std::endl;
    }
    qinit = qres;
    
    RigidBodyDynamics::Utils::CalcCenterOfMass(model,qinit,qdot,mass,com_temp);
    com_real[0] = com_temp;
    CS.body_points[Center_of_Mass] = CalcBaseToBodyCoordinates(model,qinit,body_ids[2],com_real[0]);
  }
  
  // fix the root_link ort to the same as the com only in case it's walking in circle
  Vector3d root_ort = Vector3dZero;
  root_ort = com[0].block(3,0,3,1);
  target_orientation[3] = CalcOrientationEulerXYZ(root_ort,ort_order); // root_link
  unsigned int root_link;
  if(paramsList.type == 4)
    root_link = CS.AddOrientationConstraint(model.GetBodyId("root_link"), target_orientation[3]);
  
  
  // perform inverse kinematics using all the defined points and the target positions as from the planned feet and com trajectories
  double t = 0;
  for(int i = 0; i < N; i++)
  {
    // use the real com as body point
    RigidBodyDynamics::Utils::CalcCenterOfMass(model,qinit,qdot,mass,com_temp);
    com_real[i] = com_temp;
    CS.body_points[Center_of_Mass] = CalcBaseToBodyCoordinates(model,qinit,body_ids[2],com_real[i]);
  
    // set targets
    CS.target_positions[left_foot] = l_foot[i].block(0,0,3,1);
    CS.target_positions[right_foot] = r_foot[i].block(0,0,3,1);
    CS.target_positions[Center_of_Mass] = com[i].block(0,0,3,1);
    
    l_foot_ort = Vector3d(l_foot_ang[0],DEG2RAD(-l_foot[i][4]) + l_foot_ang[1],DEG2RAD(l_foot[i][5]) + l_foot_ang[2]);
    r_foot_ort = Vector3d(r_foot_ang[0],DEG2RAD(-r_foot[i][4]) + r_foot_ang[1],DEG2RAD(r_foot[i][5]) + r_foot_ang[2]);
    com_ort = Vector3d(DEG2RAD(-com[i][3]) + chest_ang[0],DEG2RAD(-com[i][4]) + chest_ang[1],DEG2RAD(-com[i][5]) + chest_ang[2]);
    
    CS.target_orientations[left_foot] = CalcOrientationEulerXYZ(l_foot_ort,ort_order);
    CS.target_orientations[right_foot] = CalcOrientationEulerXYZ(r_foot_ort,ort_order);
    CS.target_orientations[Center_of_Mass] = CalcOrientationEulerXYZ(com_ort,ort_order);
    if(paramsList.type == 4)
    {
      root_ort = Vector3d(root_ort[0],root_ort[1],DEG2RAD(-com[i][5]));
      CS.target_orientations[root_link] = CalcOrientationEulerXYZ(root_ort,ort_order);
    }
    
    time_vec[i] = t;
    if (!InverseKinematics(model, qinit, CS, qres))
    {
        std::cout << "!![WARNING] RigidBodyDynamics::InverseKinematics - Could not converge to a solution with the desired tolerance of " << CS.step_tol << std::endl;
    }
    res[i] = qres;
    qinit = qres;
    t += ts;
  }

  // convert into degrees
  // store all the resulting configurations
  vector<RigidBodyDynamics::Math::VectorNd> res_deg(N,qres);
  RigidBodyDynamics::Math::VectorNd qres_no_fb(qres.size()-6);
  vector<RigidBodyDynamics::Math::VectorNd> res_deg_cut(N,qres_no_fb);
  RigidBodyDynamics::Math::VectorNd time_vec_new(N);
  t = 0;
  for(int i = 0; i < N; i++)
  {
    time_vec_new[i] = t;
    for(int j = 0; j < qres.size(); j++)
//       if(j < 3)
        res_deg[i][j] = res[i][j];
//       else
//         res_deg[i][j] = RAD2DEG(res[i][j]); // not transforming to degrees due to torqueBalancing which takes radiants as input
    t += ts;
  }
  
  // store the q only without floating base
  for(int i = 0; i < N; i++)
  {
    for(int j = 6; j < qres.size(); j++)
        res_deg_cut[i][j-6] = res_deg[i][j];
  }
  
  
  // Change the reference of the com from root to l_sole as per real iCub convention for torque control
  vector<VectorNd> com_l_sole(N,Vector3dZero);
  VectorNd l_sole_pos = CalcBodyToBaseCoordinates(model,res[0],model.GetBodyId("l_sole"),RigidBodyDynamics::Math::Vector3dZero);
  MatrixNd l_sole_ort = CalcBodyWorldOrientation(model,res[0],model.GetBodyId("l_sole"));
  cout << "l_sole pos wrt root: " << l_sole_pos.transpose() << endl;
  l_foot_ang = Vector3dZero;
  getEulerAngles(l_sole_ort, l_foot_ang, ort_order); 
  cout << "l_sole rot wrt root: " << l_foot_ang.transpose() << endl;
  for(int i = 0; i < N; i++)
  {
    com_l_sole[i] = l_sole_ort*com[i] + l_sole_pos;
  }
  
  
  // header for the meshup output file, needed for iCubHeidelberg01 only
  const char* meshup_header = "COLUMNS: \n\r"
                              "time,\n\r"
                              "root_link:T:X,\n\r"
                              "root_link:T:Y,\n\r"
                              "root_link:T:Z,\n\r"
                              "root_link:R:X:rad,\n\r"
                              "root_link:R:Y:rad,\n\r"
                              "root_link:R:Z:rad,\n\r"
                              "l_hip_1:R:X:rad,\n\r"
                              "l_hip_2:R:-Z:rad,\n\r"
                              "l_upper_leg:R:Y:rad,\n\r"
                              "l_lower_leg:R:X:rad,\n\r"
                              "l_ankle_1:R:-X:rad,\n\r"
                              "l_ankle_2:R:-Z:rad,\n\r"
                              "r_hip_1:R:-X:rad,\n\r"
                              "r_hip_2:R:-Z:rad,\n\r"
                              "r_upper_leg:R:-Y:rad,\n\r"
                              "r_lower_leg:R:-X:rad,\n\r"
                              "r_ankle_1:R:X:rad,\n\r"
                              "r_ankle_2:R:-Z:rad\n\r"
                              "torso_1:R:X:rad,\n\r"
                              "torso_2:R:-Z:rad,\n\r"
                              "chest:R:-Y:rad\n\r"
                              "DATA:\n\r";
  
  // write out the resulting joint trajectories for meshup visualization and for the robot
  cout<<"Wrote MESHUP file: " << string(file_path + "/test_ik_pg_meshup.csv") << endl;
  writeOnCSV(time_vec_new,res,file_path + "/test_ik_pg_meshup.csv",meshup_header);
  writeOnCSV(res_deg_cut,file_path + "/test_ik_pg.csv");
  writeOnCSV(res_deg,file_path + "/test_ik_pg_float.csv");
  writeOnCSV(com_real,file_path + "/real_com_traj.csv");
  writeOnCSV(com_l_sole,file_path + "/com_l_sole.csv");
}

bool parseFromFile(char *arg[], params &paramsList)
{
    string filename = string(arg[2]);
    cout << "Parameters file: " << filename << endl;
    
    // Parsing
    ifstream file( filename.c_str() );
    string line;
    while( std::getline( file, line ) )
    {
        std::istringstream iss( line );
        std::string result;
        if( std::getline( iss, result , ' ') )
        {
            if( result == "ts" )
            {
                std::string token;
                while( std::getline( iss, token, ' ' ) )
                {
                    //std::cout << token << std::endl;
                    paramsList.ts = stod(token);
                }
            }
            if( result == "z_c" )
            {
                std::string token;
                while( std::getline( iss, token, ' ' ) )
                {
                    //std::cout << token << std::endl;
                    paramsList.com_height = stod(token);
                }
            }
            if ( result == "n_samples" )
            {
                std::string token;
                while( std::getline( iss, token, ' ' ) )
                {
                    //std::cout << token << std::endl;
                    paramsList.N_shoot = (int)stod(token);
                }
            }
            if ( result == "n_strides" )
            {
                std::string token;
                while( std::getline( iss, token, ' ' ) )
                {
                    //std::cout << token << std::endl;
                    paramsList.N_steps = (int)stod(token);
                }
            }
            if ( result == "T_stride" )
            {
                std::string token;
                while( std::getline( iss, token, ' ' ) )
                {
                    //std::cout << token << std::endl;
                    paramsList.step_duration = stod(token);
                }
            }
            if ( result == "step_width" )
            {
                std::string token;
                while( std::getline( iss, token, ' ' ) )
                {
                    //std::cout << token << std::endl;
                    paramsList.step_width = stod(token);
                }
            }
            if ( result == "step_length" )
            {
                std::string token;
                while( std::getline( iss, token, ' ' ) )
                {
                    //std::cout << token << std::endl;
                    paramsList.step_length = stod(token);
                }
            }
            if ( result == "step_height" )
            {
                std::string token;
                while( std::getline( iss, token, ' ' ) )
                {
                    //std::cout << token << std::endl;
                    paramsList.step_height = stod(token);
                }
            }
            if ( result == "T_switch" )
            {
                std::string token;
                while( std::getline( iss, token, ' ' ) )
                {
                    //std::cout << token << std::endl;
                    paramsList.t_switch = stod(token);
                }
            }
            if ( result == "theta" )
            {
                std::string token;
                while( std::getline( iss, token, ' ' ) )
                {
                    //std::cout << token << std::endl;
                    paramsList.theta = stod(token);
                }
            }
            if ( result == "type" )
            {
                std::string token;
                while( std::getline( iss, token, ' ' ) )
                {
                    //std::cout << token << std::endl;
                    paramsList.type = (int)stod(token);
                }
            }
        }
    }

    return true;
}
