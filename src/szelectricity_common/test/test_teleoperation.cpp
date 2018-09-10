#include <gtest/gtest.h>

#include "VehicleParameters.hpp"
#include "Teleoperation.hpp"

using namespace szenergy;

TEST(TeleoperationParameterTest, EqualNull)
{
    RosTeleopPubState s0;
    RosTeleopPubState s1;
    s0.msg_torque.data = 0.0;
    s1.msg_torque.data = 0.0;
    s0.msg_steer.data = 0.0;
    s1.msg_steer.data = 0.0;
    s0.msg_brake.data = false;
    s1.msg_brake.data = false;
    EXPECT_TRUE(s0==s1);
}

TEST(TeleoperationParameterTest, EqualAll0)
{
    RosTeleopPubState s0;
    RosTeleopPubState s1;
    s0.msg_torque.data = 1.0;
    s1.msg_torque.data = 1.0;
    s0.msg_steer.data = 1.0;
    s1.msg_steer.data = 1.0;
    s0.msg_brake.data = false;
    s1.msg_brake.data = false;
    EXPECT_TRUE(s0==s1);
}

TEST(TeleoperationParameterTest, EqualAll1)
{
    RosTeleopPubState s0;
    RosTeleopPubState s1;
    s0.msg_torque.data = 1.0;
    s1.msg_torque.data = 1.0;
    s0.msg_steer.data = 1.0;
    s1.msg_steer.data = 1.0;
    s0.msg_brake.data = true;
    s1.msg_brake.data = true;
    EXPECT_TRUE(s0==s1);
}

TEST(TeleoperationParameterTest, DifferentTorque0)
{
    RosTeleopPubState s0;
    RosTeleopPubState s1;
    s0.msg_torque.data = 0.0;
    s1.msg_torque.data = 1.0;
    s0.msg_steer.data = 1.0;
    s1.msg_steer.data = 1.0;
    s0.msg_brake.data = true;
    s1.msg_brake.data = true;
    EXPECT_FALSE(s0==s1);
}

TEST(TeleoperationParameterTest, DifferentTorque1)
{
    RosTeleopPubState s0;
    RosTeleopPubState s1;
    s0.msg_torque.data = 1.0;
    s1.msg_torque.data = 0.0;
    s0.msg_steer.data = 1.0;
    s1.msg_steer.data = 1.0;
    s0.msg_brake.data = true;
    s1.msg_brake.data = true;
    EXPECT_FALSE(s0==s1);
}

TEST(TeleoperationParameterTest, DifferentSteer0)
{
    RosTeleopPubState s0;
    RosTeleopPubState s1;
    s0.msg_torque.data = 1.0;
    s1.msg_torque.data = 1.0;
    s0.msg_steer.data = 1.0;
    s1.msg_steer.data = 0.0;
    s0.msg_brake.data = true;
    s1.msg_brake.data = true;
    EXPECT_FALSE(s0==s1);
}

TEST(TeleoperationParameterTest, DifferentSteer1)
{
    RosTeleopPubState s0;
    RosTeleopPubState s1;
    s0.msg_torque.data = 1.0;
    s1.msg_torque.data = 1.0;
    s0.msg_steer.data = 0.0;
    s1.msg_steer.data = 1.0;
    s0.msg_brake.data = true;
    s1.msg_brake.data = true;
    EXPECT_FALSE(s0==s1);
}

TEST(TeleoperationParameterTest, DifferentBrake0)
{
    RosTeleopPubState s0;
    RosTeleopPubState s1;
    s0.msg_torque.data = 1.0;
    s1.msg_torque.data = 1.0;
    s0.msg_steer.data = 1.0;
    s1.msg_steer.data = 1.0;
    s0.msg_brake.data = false;
    s1.msg_brake.data = true;
    EXPECT_FALSE(s0==s1);
}

TEST(TeleoperationParameterTest, DifferentBrake1)
{
    RosTeleopPubState s0;
    RosTeleopPubState s1;
    s0.msg_torque.data = 1.0;
    s1.msg_torque.data = 1.0;
    s0.msg_steer.data = 1.0;
    s1.msg_steer.data = 1.0;
    s0.msg_brake.data = true;
    s1.msg_brake.data = false;
    EXPECT_FALSE(s0==s1);
}

TEST(TeleoperationParameterTest, DifferentSteerTorque)
{
    RosTeleopPubState s0;
    RosTeleopPubState s1;
    s0.msg_torque.data = 1.0;
    s1.msg_torque.data = 0.0;
    s0.msg_steer.data = 1.0;
    s1.msg_steer.data = 0.0;
    s0.msg_brake.data = true;
    s1.msg_brake.data = true;
    EXPECT_FALSE(s0==s1);
}

TEST(TeleoperationParameterTest, DifferentSteerBrake)
{
    RosTeleopPubState s0;
    RosTeleopPubState s1;
    s0.msg_torque.data = 1.0;
    s1.msg_torque.data = 1.0;
    s0.msg_steer.data = 0.0;
    s1.msg_steer.data = 0.0;
    s0.msg_brake.data = false;
    s1.msg_brake.data = true;
    EXPECT_FALSE(s0==s1);
}

TEST(TeleoperationParameterTest, DifferentAll)
{
    RosTeleopPubState s0;
    RosTeleopPubState s1;
    s0.msg_torque.data = 1.0;
    s1.msg_torque.data = 0.0;
    s0.msg_steer.data = 1.0;
    s1.msg_steer.data = 0.0;
    s0.msg_brake.data = true;
    s1.msg_brake.data = false;
    EXPECT_FALSE(s0==s1);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}