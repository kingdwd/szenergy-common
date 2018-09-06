#include <szenergy_config/szenergy_config.hpp>

using namespace tinyxml2;

/** 
 * Example to use the configurer package
 */
int main(int argc, char** argv)
{
    Configurer conf;
    // Read from location
    std::string filename = "/home/szakkor81/jkk_git/szenergy-control/src/szenergy_config/config/szelectricity.xml";
    // If the config is successfully read, we print it
    try
    {
        if(conf.ReadConfigFromFile(filename)){
            // Print the read configuration
            conf.PrintSummary();
            // Get an attribute directly from the vehicle parameters read
            std::cout << conf.VehicleParameters().vehicle_name << std::endl;
        }else{
            // Otherwise, FATAL ERROR
            std::cerr << "Unable to read configuration, exiting" << std::endl;
            return -1;
        }
    }
    catch(const std::invalid_argument& e)
    {
        std::cerr << e.what() << std::endl;
    }
    
    return 0;
}
