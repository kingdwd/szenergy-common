/*
 * Szenergy ROS-based software components
 * Széchenyi István University, Győr
 * Járműkutató Központ (Vehicle Research Center)
 * Autonóm Jármű Kutatórészleg (Autonomous Vehicle Research Division)
 * Copyright (C) 2018
 * 
 * These packages are not free and are not publicly distributed.
 * If you are not member of the Autonomous Vehicle Research Division,
 * or you are not explicitly invited to cooperate, you should not have
 * received these sources.
 * 
 * As a developer, you should not redistribute or make these source 
 * publicly available any way possible.
 * 
 * These restrictions apply until further revision.
 * 
 */

#ifndef SZELECTRICITY_MATH_GEOMETRY_HPP
#define SZELECTRICITY_MATH_GEOMETRY_HPP

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

namespace szenergy {
    /**
     * @brief: This is a simple function written to rotate a pose
     *         around the z-axis
     * */
    geometry_msgs::Pose RotateYaw(geometry_msgs::Pose &c, const double yaw);
    
    /**
     * @brief: utility function to derive yaw from a quaternion
     *         RPY used as an Euler-rotation convention
     * */
    double GetYawFromQuaternion(const geometry_msgs::Pose &c);
    /**
     *  @brief: utility function to convert an 
     *         RPY yaw (rotation around z-axis) to quaternion values
     * */
    geometry_msgs::Quaternion YawToQuaternion(const double yaw);

    /**
     * @brief: Distance between position
     */
    double DistanceBetweenTwoPose(const geometry_msgs::Pose &p0, const geometry_msgs::Pose &p1);
}

#endif
