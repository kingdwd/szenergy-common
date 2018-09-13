#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <gtest/gtest.h>


#include <szelectricity_common/MathGeometry.hpp>

#include <chrono>

class YawRotationConversionP: public testing::TestWithParam<double>
{

};

TEST_P(YawRotationConversionP, RotationP)
{
    geometry_msgs::Pose ref_rotation;
    double yaw_rotation = GetParam()*M_PI/180.0;
    ref_rotation.orientation.w = cos(yaw_rotation/2.0);
    ref_rotation.orientation.x = 0.0;
    ref_rotation.orientation.y = 0.0;
    ref_rotation.orientation.z = sin(yaw_rotation/2.0);
    ASSERT_NEAR(szenergy::GetYawFromQuaternion(ref_rotation), yaw_rotation, 0.000001);
}



TEST(MathGeometryTestRotation, noRotationYaw0)
{
    geometry_msgs::Pose ref_rotation;
    ref_rotation.orientation.w = 1.0;
    ref_rotation.orientation.x = 0.0;
    ref_rotation.orientation.y = 0.0;
    ref_rotation.orientation.z = 0.0;
    geometry_msgs::Pose res = szenergy::RotateYaw(ref_rotation, 0.0);
    ASSERT_DOUBLE_EQ(res.orientation.w, 1.0);
    ASSERT_DOUBLE_EQ(res.orientation.x, 0.0);
    ASSERT_DOUBLE_EQ(res.orientation.y, 0.0);
    ASSERT_DOUBLE_EQ(res.orientation.z, 0.0);
    ASSERT_DOUBLE_EQ(szenergy::GetYawFromQuaternion(res), 0.0);
}

TEST(MathGeometryTestRotation, simpleStartZeroRotationYaw0)
{
    double ref_yaw = 45.0*M_PI/180.0;
    geometry_msgs::Pose ref_rotation;
    ref_rotation.orientation.w = 1.0;
    ref_rotation.orientation.x = 0.0;
    ref_rotation.orientation.y = 0.0;
    ref_rotation.orientation.z = 0.0;
    geometry_msgs::Pose res = szenergy::RotateYaw(ref_rotation, ref_yaw);
    ASSERT_DOUBLE_EQ(res.orientation.w, cos(ref_yaw/2));
    ASSERT_DOUBLE_EQ(res.orientation.x, 0.0);
    ASSERT_DOUBLE_EQ(res.orientation.y, 0.0);
    ASSERT_DOUBLE_EQ(res.orientation.z, sin(ref_yaw/2));
    ASSERT_DOUBLE_EQ(szenergy::GetYawFromQuaternion(res), ref_yaw);
}

TEST(MathGeometryTestRotation, simpleStartZeroRotationYaw1)
{
    double ref_yaw = 45.0*M_PI/180.0;
    geometry_msgs::Pose ref_rotation;
    ref_rotation.orientation.w = 0.8536;
    ref_rotation.orientation.x = 0.3536;
    ref_rotation.orientation.y = 0.3536;
    ref_rotation.orientation.z = -0.1464;
    geometry_msgs::Pose res = szenergy::RotateYaw(ref_rotation, ref_yaw);    
    ASSERT_NEAR(0.8446, res.orientation.w, 0.01);
    ASSERT_NEAR(0.1913, res.orientation.x, 0.01);
    ASSERT_NEAR(0.4619, res.orientation.y, 0.01);
    ASSERT_NEAR(0.1913, res.orientation.z, 0.01);
    ASSERT_NEAR(ref_yaw, szenergy::GetYawFromQuaternion(res),0.01);
}

TEST(MathGeometryTestRotation, simpleStartRotationYaw1)
{
    double ref_yaw = 45.0*M_PI/180.0;
    geometry_msgs::Pose ref_rotation;
    ref_rotation.orientation.w = cos(ref_yaw/2);
    ref_rotation.orientation.x = 0;
    ref_rotation.orientation.y = 0;
    ref_rotation.orientation.z = sin(ref_yaw/2);
    geometry_msgs::Pose res = szenergy::RotateYaw(ref_rotation, ref_yaw);    
    ASSERT_NEAR(res.orientation.w, sqrt(2)/2, 0.01);
    ASSERT_NEAR(res.orientation.x, 0, 0.01);
    ASSERT_NEAR(res.orientation.y, 0, 0.01);
    ASSERT_NEAR(res.orientation.z, sqrt(2)/2, 0.01);
    
    ASSERT_NEAR(M_PI_2, szenergy::GetYawFromQuaternion(res), 0.001);
}

class YawRotationStartPDouble: public testing::TestWithParam<double>
{
    
};

TEST_P(YawRotationStartPDouble, TestPDouble)
{
    double ref_yaw = GetParam()*M_PI/180.0;
    geometry_msgs::Pose ref_rotation;
    ref_rotation.orientation.w = cos(ref_yaw/2);
    ref_rotation.orientation.x = 0;
    ref_rotation.orientation.y = 0;
    ref_rotation.orientation.z = sin(ref_yaw/2);
    geometry_msgs::Pose res = szenergy::RotateYaw(ref_rotation, ref_yaw);    
    ASSERT_NEAR(cos(ref_yaw), res.orientation.w, 0.01);
    ASSERT_NEAR(0, res.orientation.x, 0.01);
    ASSERT_NEAR(0, res.orientation.y, 0.01);
    ASSERT_NEAR(sin(ref_yaw), res.orientation.z, 0.01);
    
    ASSERT_NEAR(ref_yaw*2.0, szenergy::GetYawFromQuaternion(res), 0.001);
}

const double ANGLE_SLICE_INCREMENT = 360.0/30.0;

INSTANTIATE_TEST_CASE_P(YawRotationConversion, YawRotationConversionP, 
        testing::Range<double>(-360,360+ANGLE_SLICE_INCREMENT,ANGLE_SLICE_INCREMENT)
        
);

INSTANTIATE_TEST_CASE_P(YawRotationDouble, YawRotationStartPDouble, 
        testing::Range<double>(-180,180+ANGLE_SLICE_INCREMENT,ANGLE_SLICE_INCREMENT)
        
);


TEST(YawRotationBenchmark, SimpleBenchmarkCycle)
{
    geometry_msgs::Pose ref_rotation;
    int steps = 200; //  Double of typical frequency in automative software
    double limit_threshold = 1e-5; // This is a critical operation, one should not spend too much time with calculating one rotation
    double angle_increment = M_PI*2/steps;
    ref_rotation.orientation.w = 1.0;
    ref_rotation.orientation.x = 0.0;
    ref_rotation.orientation.y = 0.0;
    ref_rotation.orientation.z = 0.0;

    geometry_msgs::Pose res = ref_rotation;
    auto start = std::chrono::system_clock::now();
    for (int i = 0; i < steps; i++)    
    {
        res = szenergy::RotateYaw(res, angle_increment);
    }
    auto end = std::chrono::system_clock::now();
    std::cout << "Total time spent (simple benchmark cycle): " << (end-start).count()/10e9 << std::endl;
    ASSERT_LE((end-start).count()/10e9, limit_threshold);        
}


TEST(YawRotationBenchmark, SimpleBenchmarkSingleOperation)
{
    geometry_msgs::Pose ref_rotation;
    int steps = 200; //  Double of typical frequency in automative software
    double limit_threshold = 1e-6; // This is a critical operation, one should not spend too much time with calculating one rotation
    double angle_increment = M_PI*2/steps;
    ref_rotation.orientation.w = 1.0;
    ref_rotation.orientation.x = 0.0;
    ref_rotation.orientation.y = 0.0;
    ref_rotation.orientation.z = 0.0;

    geometry_msgs::Pose res = ref_rotation;
    double sum_cnts = 0.0;

    for (int i = 0; i < steps; i++)    
    {
        auto start = std::chrono::system_clock::now();
        res = szenergy::RotateYaw(res, angle_increment);
        auto end = std::chrono::system_clock::now();
        ASSERT_LE((end-start).count()/10e9, limit_threshold);
        sum_cnts += (end-start).count()/10e9;
    }
    
    std::cout << "Total time spent (single operation): " << sum_cnts << std::endl;
    std::cout << "Average time spent with a single operation: " << sum_cnts/steps << std::endl;
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
