#include <szelectricity_common/MathCommon.hpp>
#include <szelectricity_common/TimeFunctions.hpp>

#include <gtest/gtest.h>

TEST(TimeFunctionTest, TestTimeFunctionBasic)
{
    int argc = 0;
    char* argv = NULL;
    ros::init(argc, &argv, "test_time");
    ros::NodeHandle nh;
    ASSERT_TRUE(szenergy::SynchronizeRosTime());
}

TEST(TimeFunctionTest, TestTimeFunctionBasicFailure)
{
    int argc = 0;
    char* argv = NULL;
    ros::init(argc, &argv, "test_time");
    ros::NodeHandle nh;
    ros::shutdown();
    ASSERT_FALSE(szenergy::SynchronizeRosTime());
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
