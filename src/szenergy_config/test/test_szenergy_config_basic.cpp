#include <gtest/gtest.h>

#include <ros/ros.h>

#include <szelectricity_common/VehicleParameters.hpp>

#include <szenergy_config/szenergy_config.hpp>

TEST(BasicSzenergyTest, SzenergyConfigTest)
{
    Configurer conf;
    const std::string filename = GetConfigPath()+"szelectricity.xml";
    std::cout << "Reading from: " << filename << std::endl;
    bool success = conf.ReadConfigFromFile(filename);
    ASSERT_TRUE(success);
    std::cout << "Successfully read configuration" << std::endl;
    try
    {
        conf.PrintSummary();
        std::shared_ptr<szenergy::VehicleParameters> res = conf.VehicleParameters();
        ASSERT_STREQ("szelectricity",res->vehicle_name.c_str());
        ASSERT_DOUBLE_EQ(0.556/2.0, res->wheelradius);
        ASSERT_DOUBLE_EQ(1.6, res->wheelbase);
        ASSERT_DOUBLE_EQ(1.02, res->front_track);
        ASSERT_DOUBLE_EQ(0.8, res->rear_track);

    }
    catch(const std::invalid_argument& e)
    {
        EXIT_FAILURE;
    }
    
}

TEST(BasicSzenergyTest, SzenergyConfigTestNullPath)
{
    Configurer conf;
    const std::string filename;
    bool success = conf.ReadConfigFromFile(filename);
    ASSERT_FALSE(success);
}

TEST(BasicSzenergyTest, SzenergyConfigTestWrongPath)
{
    Configurer conf;
    const std::string filename = GetConfigPath()+"dsfargeg.xml";
    bool success = conf.ReadConfigFromFile(filename);
    ASSERT_FALSE(success);
}

TEST(BasicSzenergyTest, SzenergyConfigTestMalformed)
{
    Configurer conf;
    const std::string malformed_config = "hhgldkfhgsolhsaw";
    bool success = conf.SetupFromString(malformed_config);
    ASSERT_FALSE(success);
    try
    {
        conf.VehicleParameters();
    }
    catch(const std::invalid_argument& e)
    {
        EXIT_SUCCESS;
    }
    EXIT_FAILURE;
}


TEST(BasicSzenergyTest, SzenergyConfigTestMalformedNoPrint)
{
    Configurer conf;
    const std::string malformed_config = "hhgldkfhgsolhsaw";
    bool success = conf.SetupFromString(malformed_config);
    ASSERT_FALSE(success);
    try
    {
        conf.PrintSummary();
    }
    catch(const std::invalid_argument& e)
    {
        EXIT_SUCCESS;
    }
    EXIT_FAILURE;
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

const std::string test_config_basic = "<?xml version=\"1.0\" ?>\
    <vehicle>\
        <name>car</name>\
        <kinematic>\
            <wheelbase>1.0</wheelbase>\
            <front_track>1.0</front_track>\
            <rear_track>0.5</rear_track>\
            <wheelparameters>\
                <radius>0.2</radius>\
            </wheelparameters>\
        </kinematic>\        
        <ros>\
            <vehicle>\
                <steer_topic>\
                    <topicname>steer</topicname>\
                    <frequency>100.0</frequency>\
                    <jointdefinition>0</jointdefinition>\
                    <jointdefinition>1</jointdefinition>\
                </steer_topic>\
            </vehicle>\
        </ros>\
    </vehicle>";

const std::string test_config_odom = "<?xml version=\"1.0\" ?>\
    <vehicle>\
        <name>car</name>\
        <kinematic>\
            <wheelbase>1.0</wheelbase>\
            <front_track>1.0</front_track>\
            <rear_track>0.5</rear_track>\
            <wheelparameters>\
                <radius>0.2</radius>\
            </wheelparameters>\
        </kinematic>\        
        <ros>\
            <control>\
                <steer_topic>/steer</steer_topic>\
                <throttle_topic>/throttle</throttle_topic>\
            </control>\
            <odom>\
                <odom_topic>\
                    <topicname>odom</topicname>\
                    <frequency>50.0</frequency>\
                </odom_topic>\
                <steer_topic>\
                    <topicname>steer</topicname>\
                    <frequency>100.0</frequency>\
                    <jointdefinition>0</jointdefinition>\
                    <jointdefinition>1</jointdefinition>\
                </steer_topic>\
                <throttle_topic>\
                    <topicname>throttle</topicname>\
                    <frequency>100.0</frequency>\
                    <jointdefinition>3</jointdefinition>\
                </throttle_topic>\
            </odom>\
        </ros>\
    </vehicle>";

void TestReadConfig(const std::string& test_config_basic)
{
    Configurer conf;
    EXPECT_THROW(conf.SetupFromString(test_config_basic), std::invalid_argument);
    try
    {
        conf.SetupFromString(test_config_basic);
        const std::shared_ptr<szenergy::VehicleParameters> param = conf.VehicleParameters();
    }
    catch(const std::invalid_argument& e)
    {
        std::cerr << e.what() << std::endl;
        EXIT_SUCCESS;
    }
    EXIT_FAILURE;
}

TEST(ConfigReadTest, SzenergyNullConfig)
{
    
    Configurer conf;
    ASSERT_FALSE(conf.SetupFromString(""));
    
}

TEST(ConfigReadTest, SzenergyConfigPointerVehicle)
{
    
    Configurer conf;
    conf.SetupFromString(test_config_basic);
    const std::shared_ptr<szenergy::VehicleParameters> param = conf.VehicleParameters();
    ASSERT_STREQ("car", param->vehicle_name.c_str());
    ASSERT_DOUBLE_EQ(0.2, param->wheelradius);
    ASSERT_DOUBLE_EQ(0.5, param->rear_track);
    ASSERT_DOUBLE_EQ(1.0, param->front_track);
    ASSERT_DOUBLE_EQ(1.0, param->wheelbase);
}

TEST(ConfigReadTestFailure, SzenergyConfigPointerVehicleNameMissing)
{
    
    const std::string test_config_basic = "<?xml version=\"1.0\" ?>\
    <vehicle>\
        <kinematic>\
            <wheelbase>1.0</wheelbase>\
            <front_track>1.0</front_track>\
            <rear_track>0.5</rear_track>\
            <wheelparameters>\
                <radius>0.2</radius>\
            </wheelparameters>\
        </kinematic>\        
        <ros>\
            <control>\
                <steer_topic>/steer</steer_topic>\
                <throttle_topic>/throttle</throttle_topic>\
            </control>\
        </ros>\
    </vehicle>";
    TestReadConfig(test_config_basic);
}

TEST(ConfigReadTestFailure, SzenergyConfigPointerVehicleKinematicElementMissing)
{
    
    const std::string test_config_basic = "<?xml version=\"1.0\" ?>\
    <vehicle>\
        <name>car</name>\
        <ros>\
            <control>\
                <steer_topic>/steer</steer_topic>\
                <throttle_topic>/throttle</throttle_topic>\
            </control>\
        </ros>\
    </vehicle>";
    TestReadConfig(test_config_basic);
}

TEST(ConfigReadTestFailure, SzenergyConfigPointerWheelParametersMissing)
{
    
    Configurer conf;
    const std::string test_config_basic = "<?xml version=\"1.0\" ?>\
    <vehicle>\
        <name>car</name>\
        <kinematic>\
            <wheelbase>1.0</wheelbase>\
            <front_track>1.0</front_track>\
            <rear_track>0.5</rear_track>\
        </kinematic>\
        <ros>\
            <control>\
                <steer_topic>/steer</steer_topic>\
                <throttle_topic>/throttle</throttle_topic>\
            </control>\
        </ros>\
    </vehicle>";
    TestReadConfig(test_config_basic);
}

TEST(ConfigReadTestFailure, SzenergyConfigPointerWheelParametersMissing2)
{
    
    Configurer conf;
    const std::string test_config_basic = "<?xml version=\"1.0\" ?>\
    <vehicle>\
        <name>car</name>\
        <kinematic>\            
        </kinematic>\
        <ros>\
            <control>\
                <steer_topic>/steer</steer_topic>\
                <throttle_topic>/throttle</throttle_topic>\
            </control>\
        </ros>\
    </vehicle>";
    TestReadConfig(test_config_basic);
}

TEST(ConfigReadTestFailure, SzenergyConfigRosPointer)
{
    
    Configurer conf;
    const std::string test_config_basic = "<?xml version=\"1.0\" ?>\
    <vehicle>\
        <name>car</name>\
        <kinematic>\ 
            <wheelbase>1.0</wheelbase>\
            <front_track>1.0</front_track>\
            <rear_track>0.5</rear_track>\
            <wheelparameters>\
                <radius>0.2</radius>\
            </wheelparameters>\           
        </kinematic>\
    </vehicle>";
    try {
        conf.SetupFromString(test_config_basic);
    }catch(std::invalid_argument &e){
        std::cout << e.what() << std::endl;
        EXIT_SUCCESS;
    }
}

TEST(ConfigReadTestFailure, SzenergyConfigPointerVehicleKinematicWheelRadiusMissing)
{
    const std::string test_config_basic = "<?xml version=\"1.0\" ?>\
    <vehicle>\
        <name>car</name>\
        <kinematic>\
            <wheelbase>1.0</wheelbase>\
            <front_track>1.0</front_track>\
            <rear_track>0.5</rear_track>\
            <wheelparameters>\
            </wheelparameters>\
        </kinematic>\        
        <ros>\
            <control>\
                <steer_topic>/steer</steer_topic>\
                <throttle_topic>/throttle</throttle_topic>\
            </control>\
        </ros>\
    </vehicle>";
    TestReadConfig(test_config_basic);
}

TEST(ConfigReadTestFailure, SzenergyConfigPointerFrontTrackMissing)
{
    
    const std::string test_config_basic = "<?xml version=\"1.0\" ?>\
    <vehicle>\
        <name>car</name>\
        <kinematic>\
            <wheelbase>1.0</wheelbase>\
            <rear_track>0.5</rear_track>\
        </kinematic>\
        <wheelparameters>\
            <radius>0.2</radius>\
        </wheelparameters>\
        <ros>\
            <control>\
                <steer_topic>/steer</steer_topic>\
                <throttle_topic>/throttle</throttle_topic>\
            </control>\
        </ros>\
    </vehicle>";
    TestReadConfig(test_config_basic);
}

TEST(ConfigReadTestFailure, SzenergyConfigPointerRearTrackMissing)
{
    
    const std::string test_config_basic = "<?xml version=\"1.0\" ?>\
    <vehicle>\
        <name>car</name>\
        <kinematic>\
            <wheelbase>1.0</wheelbase>\
            <front_track>1.0</front_track>\
            <wheelparameters>\
                <radius>0.2</radius>\
            </wheelparameters>\
        </kinematic>\        
        <ros>\
            <control>\
                <steer_topic>/steer</steer_topic>\
                <throttle_topic>/throttle</throttle_topic>\
            </control>\
        </ros>\
    </vehicle>";
    TestReadConfig(test_config_basic);
}


TEST(ConfigReadTestFailure, SzenergyConfigPointerWheelBaseMissing)
{
    
    const std::string test_config_basic = "<?xml version=\"1.0\" ?>\
    <vehicle>\
        <name>car</name>\
        <kinematic>\
            <front_track>1.0</front_track>\
            <rear_track>0.5</rear_track>\
            <wheelparameters>\
                <radius>0.2</radius>\
            </wheelparameters>\
        </kinematic>\
        <ros>\
            <control>\
                <steer_topic>/steer</steer_topic>\
                <throttle_topic>/throttle</throttle_topic>\
            </control>\
        </ros>\
    </vehicle>";
    TestReadConfig(test_config_basic);
}

TEST(ConfigReadOdomParameters, SzenergyConfigOdomParameters)
{
    Configurer conf;
    conf.SetupFromString(test_config_odom);
    conf.PrintSummary();
    if (conf.OdometryParameters()!=nullptr){
        std::shared_ptr<szenergy::OdometryParameters> odom_param =
            conf.OdometryParameters();
        ASSERT_STREQ(odom_param->odom_topic_name.c_str(), "odom");
        ASSERT_STREQ(odom_param->steer_topic_name.c_str(), "steer");
        ASSERT_STREQ(odom_param->throttle_topic_name.c_str(), "throttle");
        ASSERT_EQ(odom_param->steer_joint_ids[0], 0);
        ASSERT_EQ(odom_param->steer_joint_ids[1], 1);
        ASSERT_EQ(odom_param->throttle_joint_ids[0], 3);
        EXIT_SUCCESS;
    }
    EXIT_FAILURE;
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}