#ifndef SZENERGY_CONFIG_HPP
#define SZENERGY_CONFIG_HPP
#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <boost/program_options.hpp>

#include <szelectricity_common/VehicleParameters.hpp>

#include <tinyxml2.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <stdexcept>

// @brief: A class that reads the configuration and returns with
//          required defining parameters
class Configurer
{
private:
    tinyxml2::XMLDocument doc;
    std::unique_ptr<szenergy::VehicleParameters> vec_param;
public:
    bool ReadConfigFromFile(std::string filename)
    {
        std::ifstream configfile(filename.c_str());
        // Check if the file is available
        if (!configfile.is_open())
        {
            ROS_ERROR("Unable to open file: %s", filename.c_str());
            return false;
        }
        else
        {
            std::string line;
            std::stringstream ss;
            
            while(std::getline(configfile, line))
            {
                ss << line << '\n';
            }
            return SetupFromString(ss.str());
        }
    }

    bool SetupFromString(std::string config)
    {
        if (ParseConfig(config))
        {
            tinyxml2::XMLElement* vehicle_element = doc.FirstChildElement("vehicle");
            ReadElements(vehicle_element);
            return true;
        }
        else
        {
            ROS_ERROR("Unable to parse configuration file");
            return false;
        }
    }

    const void ReadElements(tinyxml2::XMLElement* vehicle_element)
    {
        if (vehicle_element!=nullptr)
        {
            std::string vehicle_name = vehicle_element->FirstChildElement("name")->GetText();
            tinyxml2::XMLElement* kinematic_element = vehicle_element->FirstChildElement("kinematic");
            double wheelbase = std::atof(kinematic_element->FirstChildElement("wheelbase")->GetText());
            double front_track = std::atof(kinematic_element->FirstChildElement("front_track")->GetText());
            double rear_track = std::atof(kinematic_element->FirstChildElement("rear_track")->GetText());
            tinyxml2::XMLElement* wheelp_element = vehicle_element->FirstChildElement("wheelparameters");
            double wheel_radius = std::atof(wheelp_element->FirstChildElement("radius")->GetText());
            vec_param.reset(new szenergy::VehicleParameters(
                vehicle_name,
                wheel_radius,
                wheelbase,
                front_track,
                rear_track
            ));
        }
        else
        {
            throw std::invalid_argument("NULL config XML element: no vehicle information available");
        }
    }

    bool ParseConfig(std::string conf)
    {
        // Parse and get its Get the result of parsing
        tinyxml2::XMLError errorParse = doc.Parse(conf.c_str());
        // If the configuration is successfully parsed
        //      we can move forward to use it
        return errorParse==tinyxml2::XML_SUCCESS;
    }

    const static std::string GetConfigPath()
    {
        return ros::package::getPath("szenergy_config")+"/config/";
    }

    szenergy::VehicleParameters VehicleParameters()
    {
        return szenergy::VehicleParameters(vec_param->vehicle_name,
            vec_param->wheelradius,
            vec_param->wheelbase,
            vec_param->front_track,
            vec_param->rear_track
        );
    }

    void PrintSummary()
    {
        if (vec_param!=nullptr)
        {
            std::cout << "---- Car parameters loaded --- " << std::endl;
            std::cout << "Vehicle name:" << '\t' << vec_param->vehicle_name << std::endl;
            std::cout << "Wheel radius:" << '\t' << vec_param->wheelradius << std::endl;
            std::cout << "Wheelbase:" << '\t' << vec_param->wheelbase << std::endl;
            std::cout << "Front track:" << '\t' << vec_param->front_track << std::endl;
            std::cout << "Rear track:" << '\t' << vec_param->rear_track << std::endl;
        }
    }
};

#endif