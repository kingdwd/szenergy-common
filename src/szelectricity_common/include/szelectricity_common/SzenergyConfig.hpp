/*
 * SzenergyConfig.hpp
 *
 *  Created on: Aug 13, 2019
 *      Author: kyberszittya
 */

#ifndef INCLUDE_SZELECTRICITY_COMMON_SZENERGYCONFIG_HPP_
#define INCLUDE_SZELECTRICITY_COMMON_SZENERGYCONFIG_HPP_

#include <ros/ros.h>

#include <memory>
#include <string>
#include <exception>

#include "VehicleParameters.hpp"

namespace szenergy
{
	struct VehicleParametersFactoryException: public std::exception
	{
		const char * what() const throw() {
			return "Unable to produce vehicle parameters";
		}
	};

	std::unique_ptr<szenergy::VehicleParameters> getVehicleParametersFromPrivateNamespace(std::shared_ptr<ros::NodeHandle> private_nh);
}


#endif /* INCLUDE_SZELECTRICITY_COMMON_SZENERGYCONFIG_HPP_ */
