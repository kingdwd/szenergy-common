#include <gtest/gtest.h>

#include <ros/ros.h>

#include <szelectricity_common/VehicleParameters.hpp>

#include <szenergy_config/szenergy_config.hpp>

TEST(BasicSzenergyTest, SzenergyConfigTest)
{
    Configurer conf;
    const std::string filename = conf.GetConfigPath()+"szelectricity.xml";
    bool success = conf.ReadConfigFromFile(filename);
    ASSERT_TRUE(success);
    conf.PrintSummary();
    szenergy::VehicleParameters res = conf.VehicleParameters();
    ASSERT_STREQ("szelectricity",res.vehicle_name.c_str());
    ASSERT_DOUBLE_EQ(0.556/2.0, res.wheelradius);
    ASSERT_DOUBLE_EQ(1.6, res.wheelbase);
    ASSERT_DOUBLE_EQ(1.02, res.front_track);
    ASSERT_DOUBLE_EQ(0.8, res.rear_track);
}

TEST(BasicSzenergyTest, SzenergyConfigTestWrongPath)
{
    Configurer conf;
    const std::string filename = conf.GetConfigPath()+"dsfargeg.xml";
    bool success = conf.ReadConfigFromFile(filename);
    ASSERT_FALSE(success);
}

TEST(BasicSzenergyTest, SzenergyConfigTestMalformed)
{
    Configurer conf;
    const std::string malformed_config = "hhgldkfhgsolhsaw";
    bool success = conf.SetupFromString(malformed_config);
    ASSERT_FALSE(success);
}

TEST(BasicSzenergyTest, SzenergyConfigNullPointerVehicleElement)
{
    Configurer conf;
    try
    {
        conf.ReadElements(nullptr);
    }
    catch(const std::invalid_argument& e)
    {
        EXIT_SUCCESS;
    }
    EXIT_FAILURE;
    
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}