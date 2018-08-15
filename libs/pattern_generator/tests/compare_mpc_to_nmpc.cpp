#include "gtest/gtest.h"
#include <Eigen/Dense>

#include "nmpc_generator.h"
#include "mpc_generator.h"
#include "utils.h"


// The fixture for testing the class NMPCGenerator.
class CompareMPCToNMPC : public ::testing::Test   {
    protected:

    // Constrtuctor.
    CompareMPCToNMPC() {
      // Initialize NMPC and MPC generator.
      nmpc_generator_ = new NMPCGenerator();
      mpc_generator_ = new MPCGenerator();

      // Set security margin.
      nmpc_generator_->SetSecurityMargin(nmpc_generator_->SecurityMarginX(), 
                                         nmpc_generator_->SecurityMarginY());
      mpc_generator_->SetSecurityMargin(mpc_generator_->SecurityMarginX(), 
                                        mpc_generator_->SecurityMarginY());

      // Set initial values.
      pg_state_ = {nmpc_generator_->Ckx0(),
                   nmpc_generator_->Cky0(),
                   nmpc_generator_->Hcom(),
                   nmpc_generator_->Fkx0(),
                   nmpc_generator_->Fky0(),
                   nmpc_generator_->Fkq0(),
                   nmpc_generator_->CurrentSupport().foot,
                   nmpc_generator_->Ckq0()};
  
      nmpc_generator_->SetInitialValues(pg_state_);
      mpc_generator_->SetInitialValues(pg_state_);
    }

    // Destructor.
    virtual ~CompareMPCToNMPC() {
      delete nmpc_generator_;
      delete mpc_generator_;
    }

    // Member variables. 
    PatternGeneratorState pg_state_;

    // NMPC Generator.
    NMPCGenerator* nmpc_generator_;

    // MPC Generator.
    MPCGenerator* mpc_generator_;
};

// Compare submatrices to MPC generator.
TEST_F(CompareMPCToNMPC, Submatrices) {
    // Preprocess solutions.
    nmpc_generator_->PreprocessSolution();
    mpc_generator_->PreprocessSolution();

    // n_ and nf_ for convenience.
    const int mpc_n = mpc_generator_->n_;
    const int mpc_nf = mpc_generator_->nf_;

    // Extract reference mpc matrices.
    Eigen::MatrixXd mpc_q_k_x = mpc_generator_->pos_h_.topLeftCorner(mpc_n + mpc_nf, mpc_n + mpc_nf);
    Eigen::MatrixXd mpc_q_k_y = mpc_generator_->pos_h_.bottomRightCorner(mpc_n + mpc_nf, mpc_n + mpc_nf);

    Eigen::MatrixXd mpc_p_k_x = mpc_generator_->pos_g_.head(mpc_n + mpc_nf);
    Eigen::MatrixXd mpc_p_k_y = mpc_generator_->pos_g_.tail(mpc_n + mpc_nf);

    Eigen::MatrixXd mpc_q_k_ql = mpc_generator_->ori_h_.bottomRightCorner(mpc_n, mpc_n);
    Eigen::MatrixXd mpc_q_k_qr = mpc_generator_->ori_h_.topLeftCorner(mpc_n, mpc_n);

    Eigen::MatrixXd mpc_p_k_ql = mpc_generator_->ori_g_.tail(mpc_n);
    Eigen::MatrixXd mpc_p_k_qr = mpc_generator_->ori_g_.head(mpc_n);

    // Extract nmpc matrices.
    Eigen::MatrixXd nmpc_q_k_x = nmpc_generator_->q_k_x_;
    Eigen::MatrixXd nmpc_p_k_x = nmpc_generator_->p_k_x_;
    Eigen::MatrixXd nmpc_p_k_y = nmpc_generator_->p_k_y_;

    Eigen::MatrixXd nmpc_q_k_ql = nmpc_generator_->q_k_ql_;
    Eigen::MatrixXd nmpc_q_k_qr = nmpc_generator_->q_k_qr_;
    Eigen::MatrixXd nmpc_p_k_ql = nmpc_generator_->p_k_ql_;
    Eigen::MatrixXd nmpc_p_k_qr = nmpc_generator_->p_k_qr_;

    // Compare matrices.
    EXPECT_TRUE(nmpc_q_k_x.isApprox(mpc_q_k_x, 0.0001));
    EXPECT_TRUE(nmpc_q_k_x.isApprox(mpc_q_k_y, 0.0001));

    EXPECT_TRUE(nmpc_p_k_x.isApprox(mpc_p_k_x.transpose()));
    EXPECT_TRUE(nmpc_p_k_y.isApprox(mpc_p_k_y.transpose()));

    EXPECT_TRUE(nmpc_q_k_ql.isApprox(mpc_q_k_ql));
    EXPECT_TRUE(nmpc_q_k_qr.isApprox(mpc_q_k_qr));

    EXPECT_TRUE(nmpc_p_k_ql.isApprox(mpc_p_k_ql.transpose()));
    EXPECT_TRUE(nmpc_p_k_qr.isApprox(mpc_p_k_qr.transpose()));
}

// Compare constraint matrices to MPC generator.
TEST_F(CompareMPCToNMPC, CompareConstraintMatrices) {
    // Set security margin and velocity reference.
    Eigen::Vector3d velocity_reference(0.2, 0.2, 0.2);

    nmpc_generator_->SetSecurityMargin(0.02, 0.02);
    nmpc_generator_->SetVelocityReference(velocity_reference);

    mpc_generator_->SetSecurityMargin(0.02, 0.02);
    mpc_generator_->SetVelocityReference(velocity_reference);

    // Set initial values.
    nmpc_generator_->SetInitialValues(pg_state_);
    mpc_generator_->SetInitialValues(pg_state_);

    // Build up QP matrices.
    nmpc_generator_->PreprocessSolution();
    mpc_generator_->PreprocessSolution();

    // Extract mpc reference matrices.
    Eigen::MatrixXd mpc_pos_a   = mpc_generator_->pos_a_;
    Eigen::MatrixXd mpc_pos_lba = mpc_generator_->pos_lba_;
    Eigen::MatrixXd mpc_pos_uba = mpc_generator_->pos_uba_;

    Eigen::MatrixXd mpc_ori_a   = mpc_generator_->ori_a_;
    Eigen::MatrixXd mpc_ori_lba = mpc_generator_->ori_lba_;
    Eigen::MatrixXd mpc_ori_uba = mpc_generator_->ori_uba_;

    // Extract nmpc matrices.
    Eigen::MatrixXd nmpc_pos_a   = nmpc_generator_->a_pos_x_;
    Eigen::MatrixXd nmpc_pos_lba = nmpc_generator_->lba_pos_;
    Eigen::MatrixXd nmpc_pos_uba = nmpc_generator_->uba_pos_;

    Eigen::MatrixXd nmpc_ori_a   = nmpc_generator_->a_ori_;
    Eigen::MatrixXd nmpc_ori_lba = nmpc_generator_->lba_ori_;
    Eigen::MatrixXd nmpc_ori_uba = nmpc_generator_->uba_ori_;

    const int nmpc_n      = nmpc_generator_->n_;
    const int nmpc_nf     = nmpc_generator_->nf_;
    const int nmpc_nc_pos = nmpc_generator_->nc_pos_;
    const int nmpc_nc_ori = nmpc_generator_->nc_ori_;

    Eigen::MatrixXd nmpc_a_pos   = nmpc_generator_->qp_a_.topLeftCorner(nmpc_nc_pos, 2*(nmpc_n + nmpc_nf));
    Eigen::MatrixXd nmpc_lba_pos = nmpc_generator_->qp_lba_.head(nmpc_nc_pos);
    Eigen::MatrixXd nmpc_uba_pos = nmpc_generator_->qp_uba_.head(nmpc_nc_pos);

    Eigen::MatrixXd nmpc_a_ori   = nmpc_generator_->qp_a_.bottomRightCorner(nmpc_nc_ori, 2*nmpc_n);
    Eigen::MatrixXd nmpc_lba_ori = nmpc_generator_->qp_lba_.tail(nmpc_nc_ori);
    Eigen::MatrixXd nmpc_uba_ori = nmpc_generator_->qp_uba_.tail(nmpc_nc_ori);

    // Compare matrices.
    // Position common sub expressions.
    EXPECT_TRUE(nmpc_pos_a.isApprox(mpc_pos_a));
    EXPECT_TRUE(nmpc_pos_lba.isApprox(mpc_pos_lba.transpose()));
    EXPECT_TRUE(nmpc_pos_uba.isApprox(mpc_pos_uba.transpose()));

    EXPECT_TRUE(nmpc_a_pos.isApprox(mpc_pos_a));
    EXPECT_TRUE(nmpc_lba_pos.isApprox(mpc_pos_lba));
    EXPECT_TRUE(nmpc_uba_pos.isApprox(mpc_pos_uba));

    // Orientation common sub exprexssions.
    EXPECT_TRUE(nmpc_ori_a.isApprox(mpc_ori_a));
    EXPECT_TRUE(nmpc_ori_lba.isApprox(mpc_ori_lba.transpose()));
    EXPECT_TRUE(nmpc_ori_uba.isApprox(mpc_ori_uba.transpose()));

    EXPECT_TRUE(nmpc_a_ori.isApprox(mpc_ori_a));
    EXPECT_TRUE(nmpc_lba_ori.isApprox(mpc_ori_lba));
    EXPECT_TRUE(nmpc_uba_ori.isApprox(mpc_ori_uba));
}