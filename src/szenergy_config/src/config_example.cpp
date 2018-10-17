#include <ros/ros.h>

#include <szelectricity_common/VehicleParameters.hpp>

#include <szenergy_config/szenergy_config.hpp>

int main(int argc, char** argv)
{
    Configurer conf;
    const std::string filename = GetConfigPath()+"nissanleaf.xml";
    std::cout << "Reading from: " << filename << std::endl;
    bool success = conf.ReadConfigFromFile(filename);
    std::cout << "Print summary" << std::endl;
    conf.PrintSummary();
    return 0;
}