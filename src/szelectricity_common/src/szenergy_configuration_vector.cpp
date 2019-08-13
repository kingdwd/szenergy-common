/*
 * szenergy_config_vector.cpp
 *
 *  Created on: Aug 13, 2019
 *      Author: kyberszittya
 */

#include <szelectricity_common/SzenergyConfigVector.hpp>


std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > szenergy::generateVectorFootprint(const double& wheelbase, const double& fronttrack, const double& reartrack)
{
	std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > footprint_points;
	Eigen::Vector2d pt_0(0, reartrack/2.0);
    footprint_points.push_back(std::move(pt_0));
    Eigen::Vector2d pt_1(0, -reartrack/2.0);
    footprint_points.push_back(std::move(pt_1));
    Eigen::Vector2d pt_2(wheelbase, -fronttrack/2.0);
    footprint_points.push_back(std::move(pt_2));
    Eigen::Vector2d pt_3(wheelbase, fronttrack/2.0);
    footprint_points.push_back(std::move(pt_3));

    return footprint_points;
}
