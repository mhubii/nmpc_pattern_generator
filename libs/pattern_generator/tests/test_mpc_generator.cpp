#include "gtest/gtest.h"
#include <Eigen/Dense>
#include <qpOASES.hpp>

#include "mpc_generator.h"
#include "utils.h"

// The fixture for testing the class MPCGenerator.
class MPCGeneratorTest : public ::testing::Test   {
    protected:

    // Constrtuctor.
    MPCGeneratorTest() {
      // Initialize MPC generator.
      mpc_generator_ = new MPCGenerator();

      // Set security margin.
      mpc_generator_->SetSecurityMargin(mpc_generator_->SecurityMarginX(), 
                                        mpc_generator_->SecurityMarginY());

      // Set initial values.
      pg_state_ = {mpc_generator_->Ckx0(),
                   mpc_generator_->Cky0(),
                   mpc_generator_->Hcom(),
                   mpc_generator_->Fkx0(),
                   mpc_generator_->Fky0(),
                   mpc_generator_->Fkq0(),
                   mpc_generator_->CurrentSupport().foot,
                   mpc_generator_->Ckq0()};
  
      mpc_generator_->SetInitialValues(pg_state_);
    }

    // Destructor.
    virtual ~MPCGeneratorTest() {
      delete mpc_generator_;
    }

    // Member variables. 
    PatternGeneratorState pg_state_;

    // MPC Generator.
    MPCGenerator* mpc_generator_;
};


// Test to solve the quadratic problem.
TEST_F(MPCGeneratorTest, Solve) {
    // Set some initial velocity reference.
    Eigen::Vector3d velocity_reference(0.1, 0., 0.1);
    mpc_generator_->SetVelocityReference(velocity_reference);

    for (int i = 0; i < 20; i++) {
        double time = i*0.1;

        // Set reference velocity.
        mpc_generator_->SetVelocityReference(velocity_reference);

        // Solve QP.
        mpc_generator_->Solve();
        mpc_generator_->Simulate();

        // Expect a successful return.
        ASSERT_EQ(mpc_generator_->status_ori_, qpOASES::SUCCESSFUL_RETURN);
        ASSERT_EQ(mpc_generator_->status_pos_, qpOASES::SUCCESSFUL_RETURN);
        
        // Initial value embedding by internal states and simulation.
        pg_state_ = mpc_generator_->Update();
        mpc_generator_->SetInitialValues(pg_state_);
    }
}
