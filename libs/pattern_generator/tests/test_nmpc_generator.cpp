#include "gtest/gtest.h"
#include <Eigen/Dense>
#include <qpOASES.hpp>

#include "nmpc_generator.h"
#include "utils.h"

// The fixture for testing the class NMPCGenerator.
class NMPCGeneratorTest : public ::testing::Test   {
    protected:

    // Constrtuctor.
    NMPCGeneratorTest() {
      // Initialize NMPC generator.
      nmpc_generator_ = new NMPCGenerator();

      // Set security margin.
      nmpc_generator_->SetSecurityMargin(nmpc_generator_->SecurityMarginX(), 
                                         nmpc_generator_->SecurityMarginY());

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
    }

    // Destructor.
    virtual ~NMPCGeneratorTest() {
      delete nmpc_generator_;
    }

    // Member variables. 
    PatternGeneratorState pg_state_;

    // NMPC Generator.
    NMPCGenerator* nmpc_generator_;
};


// Test to solve the quadratic problem.
TEST_F(NMPCGeneratorTest, Solve) {
    // Set some initial velocity reference.
    Eigen::Vector3d velocity_reference(0.1, 0., 0.1);
    nmpc_generator_->SetVelocityReference(velocity_reference);

    for (int i = 0; i < 20; i++) {
        double time = i*0.1;

        // Set reference velocity.
        nmpc_generator_->SetVelocityReference(velocity_reference);

        // Solve QP.
        nmpc_generator_->Solve();
        nmpc_generator_->Simulate();

        // Expect a successful return.
        ASSERT_EQ(nmpc_generator_->status_, qpOASES::SUCCESSFUL_RETURN);
        
        // Initial value embedding by internal states and simulation.
        pg_state_ = nmpc_generator_->Update();
        nmpc_generator_->SetInitialValues(pg_state_);
    }
}
