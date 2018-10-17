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

/**
 * @brief: Share the configuration path with other ROS based applications
 * 
 * */
const static std::string GetConfigPath()
{
    return ros::package::getPath("szenergy_config")+"/config/";
}

/**
 *  @brief: A class that reads the configuration and returns with
 *        required defining parameters
 * */
class Configurer
{
private:
    tinyxml2::XMLDocument doc; ///< Root of the XML configuration
    std::unique_ptr<szenergy::VehicleParameters> vec_param; ///< Vehicle parameters
    std::unique_ptr<szenergy::OdometryParameters> odom_param; ///< Odometry parameters
public:
    /**
     * @brief: Read config from file
     * 
     * */
    bool ReadConfigFromFile(std::string filename)
    {
        // If no filename is given, return with no success
        if (filename.length() == 0)
        {
            return false;
        }
        std::ifstream configfile(filename);
        // Check if the file is available
        if (!configfile.is_open())
        {
            return false;
        }
        else
        {
            std::string line;
            std::stringstream ss;
            // Read configuration line-by-line
            while(std::getline(configfile, line))
            {
                ss << line << '\n';
            }
            return SetupFromString(ss.str());
        }
    }

    /**
     * @brief: Setup XML from string
     * 
     * */
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
            return false;
        }
    }

    void ReadOdomParameters(tinyxml2::XMLElement* ros_element)
    {
        std::cout << "Getting odometry parameters" << std::endl;
        tinyxml2::XMLElement* odometry_element = ros_element->FirstChildElement("odom");
        std::vector<int> steer_joint_ids;
        std::vector<int> throttle_joint_ids;
        
        for (tinyxml2::XMLElement* jointdef_element = odometry_element->FirstChildElement("steer_topic")->FirstChildElement("jointdefinition");
            jointdef_element != nullptr; 
            jointdef_element = jointdef_element->NextSiblingElement("jointdefinition"))
        {
            steer_joint_ids.push_back(std::atol(jointdef_element->GetText()));
        }
        
        for (tinyxml2::XMLElement* jointdef_element = odometry_element->FirstChildElement("throttle_topic")->FirstChildElement("jointdefinition");
            jointdef_element != nullptr; 
            jointdef_element = jointdef_element->NextSiblingElement("jointdefinition"))
        {
            throttle_joint_ids.push_back(std::atol(jointdef_element->GetText()));
        }
        
        odom_param.reset (new szenergy::OdometryParameters(
            odometry_element->FirstChildElement("odom_topic")->FirstChildElement("topicname")->GetText(),
            odometry_element->FirstChildElement("steer_topic")->FirstChildElement("topicname")->GetText(),
            odometry_element->FirstChildElement("throttle_topic")->FirstChildElement("topicname")->GetText(),
            steer_joint_ids,
            throttle_joint_ids
        ));
        
        std::cout << "Returning everything" << std::endl;
        
    }

    /**
     * @brief: Read elements from a previously parsed XML element
     * */
    const void ReadElements(tinyxml2::XMLElement* vehicle_element)
    {
        if (vehicle_element!=nullptr)
        {
            // Read and set the vehicle name
            // if invalid, throw an error
            tinyxml2::XMLElement* vehicle_name_element = vehicle_element->FirstChildElement("name");
            if (vehicle_name_element==nullptr)
            {
                throw std::invalid_argument("No name element given");
            }
            std::string vehicle_name = vehicle_name_element->GetText();
            // Check if the kinematic element is valid            
            tinyxml2::XMLElement* kinematic_element = vehicle_element->FirstChildElement("kinematic");
            if (kinematic_element!=nullptr)
            {
                tinyxml2::XMLElement* wheelbase_element = kinematic_element->FirstChildElement("wheelbase");
                tinyxml2::XMLElement* fronttrack_element = kinematic_element->FirstChildElement("front_track");
                tinyxml2::XMLElement* reartrack_element = kinematic_element->FirstChildElement("rear_track");
                tinyxml2::XMLElement* wheelp_element = kinematic_element->FirstChildElement("wheelparameters");
                // If any elements are invalid raise error, otherwise set parameters
                if (wheelbase_element != nullptr &&
                    fronttrack_element != nullptr &&
                    reartrack_element != nullptr &&
                    wheelp_element != nullptr
                )
                {
                    // Set parameters stored in the configuration
                    double wheelbase = std::atof(wheelbase_element->GetText());
                    double front_track = std::atof(fronttrack_element->GetText());
                    double rear_track = std::atof(reartrack_element->GetText());
                    // Check if the wheel radius element exists
                    // otherwise throw an error
                    auto wheel_radius_element = wheelp_element->FirstChildElement("radius");
                    if (wheel_radius_element==nullptr)
                    {
                        throw std::invalid_argument("No wheel radius given");
                    }
                    double wheel_radius = std::atof(wheel_radius_element->GetText());
                    // Set new paremeter instance
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
                    throw std::invalid_argument("Some parameters are not well-defined");
                }
            }
            else
            {
                throw std::invalid_argument("Kinematic element not defined");
            }
            tinyxml2::XMLElement* ros_element = vehicle_element->FirstChildElement("ros");

            if (ros_element!=nullptr)
            {
                ReadOdomParameters(ros_element);
                std::cout << "Reading odometry parameters" << std::endl;
                /*
                odom_param.reset(new szenergy::OdometryParameters(odomp.odom_topic_name,
                    odomp.steer_topic_name,
                    odomp.throttle_topic_name,
                    odomp.steer_joint_ids,
                    odomp.throttle_joint_ids));
                */
            }
            else
            {
                throw std::invalid_argument("ROS element not defined");
            }
        }
        else
        {
            throw std::invalid_argument("NULL config XML element: no vehicle information available");
        }
    }

    /**
     * @brief: Parse XML config from string
     * */
    bool ParseConfig(std::string conf)
    {
        // Parse and get its Get the result of parsing
        tinyxml2::XMLError errorParse = doc.Parse(conf.c_str());
        // If the configuration is successfully parsed
        //      we can move forward to use it
        return errorParse==tinyxml2::XML_SUCCESS;
    }


    /**
     * @brief: Return with the most actual vehicle parameters
     * */
    const szenergy::OdometryParameters OdometryParameters()
    {
        // If the vehicle parameters are not available raise an error
        // otherwise return with a copy of it
        if (odom_param!=nullptr)
        {
            return szenergy::OdometryParameters(odom_param->odom_topic_name,
                odom_param->steer_topic_name,
                odom_param->throttle_topic_name,
                odom_param->steer_joint_ids,
                odom_param->throttle_joint_ids
            );
        }
        else
        {
            throw std::invalid_argument("NULL vec_param, cannot return");
        }
    }
    
    
    /**
     * @brief: Return with the most actual vehicle parameters
     * */
    const szenergy::VehicleParameters VehicleParameters()
    {
        // If the vehicle parameters are not available raise an error
        // otherwise return with a copy of it
        if (vec_param!=nullptr)
        {
            return szenergy::VehicleParameters(vec_param->vehicle_name,
                vec_param->wheelradius,
                vec_param->wheelbase,
                vec_param->front_track,
                vec_param->rear_track
            );
        }
        else
        {
            throw std::invalid_argument("NULL vec_param, cannot return");
        }
    }

    /**
     * @brief: print the summary of the vehicle parameters
     * 
     * */
    void PrintSummary()
    {
        // If the vehicle parameters are not available raise an error
        // otherwise print the summary
        if (vec_param!=nullptr)
        {
            std::cout << "---- Car parameters loaded --- " << std::endl;
            std::cout << "Vehicle name:" << '\t' << vec_param->vehicle_name << std::endl;
            std::cout << "Wheel radius:" << '\t' << vec_param->wheelradius << std::endl;
            std::cout << "Wheelbase:" << '\t' << vec_param->wheelbase << std::endl;
            std::cout << "Front track:" << '\t' << vec_param->front_track << std::endl;
            std::cout << "Rear track:" << '\t' << vec_param->rear_track << std::endl;
        }        
        else
        {
            throw std::invalid_argument("NULL vec_param, cannot print summary");
        }
        if (odom_param!=nullptr)
        {
            std::cout << "---- Odom parameters loaded --- " << std::endl;
            std::cout << "Odom topic name:" << '\t' << odom_param->odom_topic_name << std::endl;
            std::cout << "Steer topic name:" << '\t' << odom_param->steer_topic_name << std::endl;
            std::cout << "Throttle topic name:" << '\t' << odom_param->throttle_topic_name << std::endl;
            std::cout << "Throttle joint ids:" << '\t';
            for (const auto& v: odom_param->throttle_joint_ids){
                std::cout << v << '\t'; 
            }
            std::cout << std::endl;
            std::cout << "Steer joint ids:" << '\t';
            for (const auto& v: odom_param->steer_joint_ids){
                std::cout << v << '\t'; 
            }
            std::cout << std::endl;
        }
        
    }
    
};

/**
 * @brief: Class that generates an instance of configurer
 * 
 * */
class ConfigurerFactory
{
public:
    static std::unique_ptr<Configurer> createConfigurerFromPath(const std::string& path)
    {
        std::unique_ptr<Configurer> conf(new Configurer());
        try
        {
            if(conf->ReadConfigFromFile(path)){
                return std::move(conf);
            }else{
                // Otherwise, FATAL ERROR
                throw std::invalid_argument("Unable to read configuration, exiting");
            }
        }
        catch(const std::invalid_argument& e)
        {
            throw e;
        }
    }
};

#endif