#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <gtest/gtest.h>

#include <szelectricity_common/VehicleParameters.hpp>

using namespace szenergy;

TEST(SteerTransmissionSzelectricity, steerTransmissionPoly)
{
    ASSERT_DOUBLE_EQ(0.0, steerTransmissionPoly(0.0));
    ASSERT_NEAR(0.02617994, steerTransmissionPoly(10*M_PI/180.0),1e-3);
    ASSERT_NEAR(0.05244714, steerTransmissionPoly(20*M_PI/180.0),1e-3);
    ASSERT_NEAR(0.30473449, steerTransmissionPoly(111.4402911529*M_PI/180.0),1e-3);
}

TEST(SteerTransmissionSzelectricity, steerTransmissionDataPoly)
{
    ASSERT_DOUBLE_EQ(0.0, steerTransmissionDataPoly(0.0));
    ASSERT_NEAR(0.02617994, steerTransmissionDataPoly(10*M_PI/180.0),1e-3);
    ASSERT_NEAR(0.05244714, steerTransmissionDataPoly(20*M_PI/180.0),1e-3);
    ASSERT_NEAR(-0.333, steerTransmissionDataPoly(-111.4402911529*M_PI/180.0),1e-3);
    ASSERT_NEAR(0.275, steerTransmissionDataPoly(111.4402911529*M_PI/180.0),1e-3);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}