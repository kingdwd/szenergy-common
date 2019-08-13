/*
 * SzenergyConfigVector.hpp
 *
 *  Created on: Aug 13, 2019
 *      Author: kyberszittya
 */

#ifndef INCLUDE_SZELECTRICITY_COMMON_SZENERGYCONFIGVECTOR_HPP_
#define INCLUDE_SZELECTRICITY_COMMON_SZENERGYCONFIGVECTOR_HPP_

#include <Eigen/Dense>

#include <vector>

namespace szenergy
{

std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > generateVectorFootprint(
		const double& wheelbase, const double& fronttrack, const double& reartrack);

}

#endif /* INCLUDE_SZELECTRICITY_COMMON_SZENERGYCONFIGVECTOR_HPP_ */
