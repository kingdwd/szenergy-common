#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <gtest/gtest.h>

#include "VehicleParameters.hpp"

using namespace szenergy;

TEST(MathGeometryTestRotation, noRotationYaw0)
{
    ASSERT_DOUBLE_EQ(0.0, steerTransmissionPoly(0.0));
    ASSERT_NEAR(0.02617994, steerTransmissionPoly(10*M_PI/180.0),1e-3);
    ASSERT_NEAR(0.05244714, steerTransmissionPoly(20*M_PI/180.0),1e-3);
    ASSERT_NEAR(0.30473449, steerTransmissionPoly(111.4402911529*M_PI/180.0),1e-3);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}