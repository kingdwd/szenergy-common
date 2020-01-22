/*
 * szenergy_configuration.cpp
 *
 *  Created on: Aug 13, 2019
 *      Author: kyberszittya
 */

#include <szelectricity_common/SzenergyConfig.hpp>

std::unique_ptr<szenergy::VehicleParameters> szenergy::getVehicleParametersFromPrivateNamespace(std::shared_ptr<ros::NodeHandle> private_nh)
{
	std::string vehicle_name;
	if (!private_nh->getParam("vehicle_name", vehicle_name))
	{
		throw new VehicleParametersFactoryException();
	}
	double wheelbase;
	if (!private_nh->getParam("wheelbase", wheelbase))
	{
		throw new VehicleParametersFactoryException();
	}
	double wheelradius;
	if (!private_nh->getParam("wheelradius", wheelradius))
	{
		throw new VehicleParametersFactoryException();
	}
	double front_track;
	if (!private_nh->getParam("front_track", front_track))
	{
		throw new VehicleParametersFactoryException();
	}
	double rear_track;
	if (!private_nh->getParam("rear_track", rear_track))
	{
		throw new VehicleParametersFactoryException();
	}
	double cog_ratio;
	if (!private_nh->getParam("cog_ratio", cog_ratio))
	{
		throw new VehicleParametersFactoryException();
	}
	return std::unique_ptr<szenergy::VehicleParameters>(new szenergy::VehicleParameters(
			vehicle_name,
			wheelradius,
			wheelbase,
			front_track,
			rear_track,
			cog_ratio)
	);
}


