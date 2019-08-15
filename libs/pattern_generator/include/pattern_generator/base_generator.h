#ifndef BASE_GENERATOR_H_
#define BASE_GENERATOR_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/CXX11/Tensor>
#include <vector>
#include <string>
#include "yaml-cpp/yaml.h"
#include "utils.h"

// Base class of walking pattern generator for humanoids, 
// cf. LAAS-UHEI walking report. BaseGenerator provides all
// matrices and timestepping methods that are defined for
// hte pattern generator family. In derived classes different
// problems and their solvers can be realized.
//
// Code based on the python implementation of Manuel Kudruss.
// C++ implementation by Martin Huber.
class BaseGenerator
{
public:
    BaseGenerator(const std::string config_file_loc = "../libs/pattern_generator/configs.yaml");

    // Getters.
    inline const double&              G()               const { return g_;                 };
    inline const int&                 N()               const { return n_;                 };
    inline const double&              T()               const { return t_;                 };
    inline const double&              TStep()           const { return t_step_;            };
    inline const double&              InternalT()       const { return time_;              };
    inline const Eigen::Vector3d&     Ckx0()            const { return c_k_x_0_;           };
    inline const Eigen::Vector3d&     Cky0()            const { return c_k_y_0_;           };
    inline const Eigen::Vector3d&     Ckq0()            const { return c_k_q_0_;           };
    inline const Eigen::VectorXd&     Ckp1x()           const { return c_kp1_x_;           };
    inline const Eigen::VectorXd&     Ckp1y()           const { return c_kp1_y_;           };
    inline const Eigen::VectorXd&     Ckp1q()           const { return c_kp1_q_;           };
    inline const Eigen::VectorXd&     Dddckx()          const { return dddc_k_x_;          };
    inline const Eigen::VectorXd&     Dddcky()          const { return dddc_k_y_;          };
    inline const double&              Hcom()            const { return h_com_0_;           };
    inline const Eigen::Vector3d&     LocalVelRef()     const { return local_vel_ref_;     };
    inline const double&              Fkx0()            const { return f_k_x_0_;           };
    inline const double&              Fky0()            const { return f_k_y_0_;           };
    inline const double&              Fkq0()            const { return f_k_q_0_;           };
    inline const Eigen::VectorXd&     Fkx()             const { return f_k_x_;             };
    inline const Eigen::VectorXd&     Fky()             const { return f_k_y_;             };
    inline const Eigen::VectorXd&     Fkq()             const { return f_k_q_;             };
    inline const Eigen::VectorXd&     Dddfkql()         const { return dddf_k_ql_;         };
    inline const Eigen::VectorXd&     Dddfkqr()         const { return dddf_k_qr_;         };
    inline const Eigen::MatrixXd&     PPS()             const { return pps_;               };
    inline const Eigen::MatrixXd&     PPU()             const { return ppu_;               };
    inline const Eigen::MatrixXd&     PVS()             const { return pvs_;               };
    inline const Eigen::MatrixXd&     PVU()             const { return pvu_;               };
    inline const Eigen::MatrixXd&     PAS()             const { return pas_;               };
    inline const Eigen::MatrixXd&     PAU()             const { return pau_;               };
    inline const Eigen::MatrixXd&     PZS()             const { return pzs_;               };
    inline const Eigen::MatrixXd&     PZU()             const { return pzu_;               };
    inline const double&              SecurityMarginX() const { return security_margin_x_; };
    inline const double&              SecurityMarginY() const { return security_margin_y_; };
    inline const double&              FootDistance()    const { return foot_distance_;     };
    inline const BaseTypeSupportFoot& CurrentSupport()  const { return current_support_;   };
    inline const Eigen::VectorXi&     Vkp10()           const { return v_kp1_0_;           };
    inline const double&              XObs()            const { return x_obs_;             };
    inline const double&              YObs()            const { return y_obs_;             };
    inline const double&              RObs()            const { return r_obs_;             };
    inline const double&              RMargin()         const { return r_margin_;          };




public:
    void SetSecurityMargin(const double margin_x, const double margin_y);

    void SetInitialValues(PatternGeneratorState& initial_state);

    void SetVelocityReference(Eigen::Vector3d& local_vel_ref);

    void SetObstacle(Circle& circ);

    PatternGeneratorState Update();

    PatternGeneratorState Update(double dt);

    void InitializeConstantMatrices();

    void InitializeCopMatrices();

    void InitializeSelectionMatrices();

    void InitializeConvexHullSystems();

    void Reset();

    void UpdateHulls();

    void UpdateSelectionMatrices();

    void CalculateSupportOrder();

    void ComputeLinearSystem(Eigen::MatrixXd& hull, const std::string foot,
                             Eigen::MatrixXd& a0, Eigen::VectorXd& b0);

    void Simulate();

    void BuildConstraints();

    void BuildCopContraint();

    void BuildFootEqConstraint();

    void BuildFootIneqConstraint();

    void BuildFootRotationConstraints();

    void BuildRotIneqConstraint();

    void BuildObstacleConstraint();

    void UpdateCopConstraintTransformation();

    void UpdateFootSelectionMatrices();

    // Configurations.
    YAML::Node configs_;

    // Constants.
    const double g_;

    // Passed to constructor.
    const int n_;                 // #
    const double t_;              // s
    const double t_step_;         // s
    
    // For internal usage.
    const double t_window_;       // s
    const int nf_;                // #
    double time_;                 // s

    // Objective weights.
    const double alpha_;
    const double beta_;
    const double gamma_;

    // Center of mass initial values.
    Eigen::Vector3d c_k_x_0_;
    Eigen::Vector3d c_k_y_0_;
    Eigen::Vector3d c_k_q_0_;
    double h_com_0_;

    // Center of mass matrices.
    Eigen::VectorXd   c_kp1_x_;
    Eigen::VectorXd  dc_kp1_x_;
    Eigen::VectorXd ddc_kp1_x_;

    Eigen::VectorXd   c_kp1_y_;
    Eigen::VectorXd  dc_kp1_y_;
    Eigen::VectorXd ddc_kp1_y_;

    Eigen::VectorXd   c_kp1_q_;
    Eigen::VectorXd  dc_kp1_q_;
    Eigen::VectorXd ddc_kp1_q_;

    // Jerk controls for center of mass.
    Eigen::VectorXd dddc_k_x_;
    Eigen::VectorXd dddc_k_y_;
    Eigen::VectorXd dddc_k_q_;

    // Reference matrices.
    Eigen::VectorXd dc_kp1_x_ref_;
    Eigen::VectorXd dc_kp1_y_ref_;
    Eigen::VectorXd dc_kp1_q_ref_;
    Eigen::Vector3d local_vel_ref_;

    // Feet matrices.
    double f_k_x_0_;
    double f_k_y_0_;
    double f_k_q_0_;

    Eigen::VectorXd f_k_x_;
    Eigen::VectorXd f_k_y_;
    Eigen::VectorXd f_k_q_;

    // States for the foot orientation.
    Eigen::Vector3d  f_k_ql_0_;
    Eigen::Vector3d  f_k_qr_0_;

    Eigen::VectorXd    f_k_ql_;
    Eigen::VectorXd    f_k_qr_;

    Eigen::VectorXd   df_k_ql_;
    Eigen::VectorXd   df_k_qr_;

    Eigen::VectorXd  ddf_k_ql_;
    Eigen::VectorXd  ddf_k_qr_;

    Eigen::VectorXd dddf_k_ql_;
    Eigen::VectorXd dddf_k_qr_;

    Eigen::VectorXd   f_kp1_ql_;
    Eigen::VectorXd  df_kp1_ql_;
    Eigen::VectorXd ddf_kp1_ql_;

    Eigen::VectorXd   f_kp1_qr_;
    Eigen::VectorXd  df_kp1_qr_;
    Eigen::VectorXd ddf_kp1_qr_;

    Eigen::VectorXd   f_kp1_q_;

    // Foot angular velocity selection matrices objectives.
    Eigen::MatrixXd e_f_;
    Eigen::Ref<Eigen::MatrixXd> e_fl_;
    Eigen::Ref<Eigen::MatrixXd> e_fr_;

    // Foot angular velocity selection matrices objective.
    Eigen::MatrixXd e_f_bar_;
    Eigen::Ref<Eigen::MatrixXd> e_fl_bar_;
    Eigen::Ref<Eigen::MatrixXd> e_fr_bar_;

    // Zero moment point matrices.
    double z_k_x_0_;
    double z_k_y_0_;

    Eigen::VectorXd z_kp1_x_;
    Eigen::VectorXd z_kp1_y_;

    // Transformation matrices.
    Eigen::MatrixXd pps_;
    Eigen::MatrixXd ppu_;

    Eigen::MatrixXd pvs_;
    Eigen::MatrixXd pvu_;

    Eigen::MatrixXd pas_;
    Eigen::MatrixXd pau_;

    Eigen::MatrixXd pzs_;
    Eigen::MatrixXd pzu_;

    // Convex hulls used to bound the free placement of the foot.
    const int n_foot_pos_hull_edges_;

    // Support foot left and right.    
    Eigen::MatrixXd lf_pos_hull_;
    Eigen::MatrixXd rf_pos_hull_;
    
    // Set of cartesian equalities.
    Eigen::MatrixXd a0l_;
    Eigen::VectorXd ubb0l_;
    Eigen::MatrixXd a0r_;
    Eigen::VectorXd ubb0r_;

    // Linear constraints matrix.
    const int nc_fchange_eq_;
    Eigen::MatrixXd eq_a_foot_;
    Eigen::VectorXd eq_b_foot_;

    // Linear constraints vector.
    const int nc_foot_position_;
    Eigen::MatrixXd a_foot_;
    Eigen::VectorXd lbb_foot_;
    Eigen::VectorXd ubb_foot_;

    // Security margins for cop constraints.
    double security_margin_x_;
    double security_margin_y_;

    // Position of the foot in the local foot frame.
    const int n_foot_edge_;
    const double foot_width_;
    const double foot_height_;
    const double foot_distance_;

    // Position of the vertices of the feet in the foot coordinates.
    // Left foot.
    Eigen::MatrixXd lfoot_;
    Eigen::MatrixXd rfoot_;

    // Right foot.
    Eigen::MatrixXd lf_cop_hull_;
    Eigen::MatrixXd rf_cop_hull_;

    // double support.
    Eigen::MatrixXd ds_cop_hull_;

    // Corresponding linear system from polygonal set.
    // Right foot.
    Eigen::MatrixXd a0lf_;
    Eigen::VectorXd ubb0lf_;

    // Right foot.
    Eigen::MatrixXd a0rf_;
    Eigen::VectorXd ubb0rf_;

    // double support.
    Eigen::MatrixXd a0dlf_;
    Eigen::VectorXd ubb0dlf_;
    Eigen::MatrixXd a0drf_;
    Eigen::VectorXd ubb0drf_;

    // Transformation matrix for the constraints in BuildCopConstraint().
    // Constant in variables but varying in time, because of v_kp1_.
    Eigen::MatrixXd pzuv_;
    Eigen::Ref<Eigen::MatrixXd> pzuvx_;
    Eigen::Ref<Eigen::MatrixXd> pzuvy_;

    Eigen::VectorXd pzsc_;
    Eigen::Ref<Eigen::VectorXd> pzscx_;
    Eigen::Ref<Eigen::VectorXd> pzscy_;

    Eigen::VectorXd v_kp1fc_;
    Eigen::Ref<Eigen::VectorXd> v_kp1fc_x_;
    Eigen::Ref<Eigen::VectorXd> v_kp1fc_y_;

    Eigen::MatrixXd d_kp1_;
    Eigen::Ref<Eigen::MatrixXd> d_kp1_x_;
    Eigen::Ref<Eigen::MatrixXd> d_kp1_y_;
    Eigen::VectorXd b_kp1_;

    // Constraint matrices.
    const int nc_cop_;
    Eigen::MatrixXd a_cop_;
    Eigen::VectorXd lbb_cop_;
    Eigen::VectorXd ubb_cop_;

    // Foot rotation constraints.
    const int nc_fvel_eq_;
    Eigen::MatrixXd a_fvel_eq_;
    Eigen::VectorXd b_fvel_eq_;

    const int nc_fpos_ineq_;
    Eigen::MatrixXd a_fpos_ineq_;
    Eigen::VectorXd ubb_fpos_ineq_;
    Eigen::VectorXd lbb_fpos_ineq_;

    const int nc_fvel_ineq_;
    Eigen::MatrixXd a_fvel_ineq_;
    Eigen::VectorXd ubb_fvel_ineq_;
    Eigen::VectorXd lbb_fvel_ineq_;

    // Current support state.
    BaseTypeSupportFoot current_support_;
    std::vector<BaseTypeSupportFoot> support_deque_;

    // Matrices containing constraints representing a 
    // strictly convex obstacle in space.
    bool obstacle_;
    const int nc_obs_;
    const double x_obs_;
    const double y_obs_;
    const double r_obs_;
    const double r_margin_;
    Circle co_;

    Eigen::Tensor<double, 3> h_obs_;
    Eigen::MatrixXd a_obs_;
    Eigen::VectorXd b_obs_;
    Eigen::VectorXd lb_obs_;
    Eigen::VectorXd ub_obs_;

    Eigen::VectorXi v_kp1_0_;
    Eigen::MatrixXi v_kp1_;
};

#endif
