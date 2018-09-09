#include <gtest/gtest.h>

#include <szelectricity_common/VehicleParameters.hpp>

#include <szenergy_config/szenergy_config.hpp>

TEST(CreateConfigurationFromFactory, BasicConfigurerCreation)
{
    ConfigurerFactory config_factory;
    config_factory.createConfigurerFromPath(GetConfigPath()+"szelectricity.xml");
    EXIT_SUCCESS;
}

TEST(CreateConfigurationFromFactory, BasicConfigurerCreationNotFound)
{
    ConfigurerFactory config_factory;
    ASSERT_THROW(config_factory.createConfigurerFromPath(GetConfigPath()+"dsfargeg.xml"), std::invalid_argument);
}



int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}