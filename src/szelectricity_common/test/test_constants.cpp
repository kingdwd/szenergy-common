#include <szelectricity_common/SzenergyConstants.hpp>

#include <gtest/gtest.h>

/** REGRESSION: ensure that RAD_PER_S_RPM is equal to  9.5493
 * REGRESSION BUG: CONST_01
 * */

TEST(MathConstants, RadPerSRPM)
{
    ASSERT_NEAR(9.5493, szenergy::RAD_PER_S_RPM, 1e-3);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}