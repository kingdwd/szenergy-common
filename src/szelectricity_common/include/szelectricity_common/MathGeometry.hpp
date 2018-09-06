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
    inline geometry_msgs::Pose RotateYaw(geometry_msgs::Pose &c, const double yaw)
    {
        geometry_msgs::Pose res(c);
        // This can be verified by simple math,
        // use quaternion conversion
        const double cyaw = cos(yaw/2.0);
        const double syaw = sin(yaw/2.0);
        // Get quaternion parameters
        const double w  = c.orientation.w;
        const double x  = c.orientation.x;
        const double y  = c.orientation.y;
        const double z  = c.orientation.z;
        // The resulting orientation is derived only using the yaw parameter
        res.orientation.w = w*cyaw - z*syaw;
        res.orientation.x = x*cyaw - y*syaw;
        res.orientation.y = x*syaw + y*cyaw;
        res.orientation.z = w*syaw + z*cyaw;
        return res;
    }

    
    /**
     * @brief: utility function to derive yaw from a quaternion
     *         RPY used as an Euler-rotation convention
     * */
    inline double GetYawFromQuaternion(const geometry_msgs::Pose &c)
    {
        const double x = c.orientation.x;
        const double y = c.orientation.y;
        const double w = c.orientation.w;
        const double z = c.orientation.z;
        double sgn_rot = (0.0 < z) - (z < 0.0);
        // These calcualtions are used from math textbooks
        const double xx = 2*(w*z+x*y);
        const double ww = 1-2*(y*y+z*z);
        const double res =  atan2(xx,ww);
        // Due to the very fragile nature of atan2,
        // some angles must be added at some points
        // Best dervied using the quaternion parameter
        // as quaternions define unique rotations
        if (w > 0.0)
        {
            return res;
        }
        else
        {
            return 2*sgn_rot*M_PI + res;
        }
        
    }

    /**
     *  @brief: utility function to convert an 
     *         RPY yaw (rotation around z-axis) to quaternion values
     * */
    inline geometry_msgs::Quaternion YawToQuaternion(const double yaw)
    {
        geometry_msgs::Quaternion q;
        // Quaternion parameters can be derived using simple arithmetics
        q.w = cos(yaw/2.0);
        q.x = 0;
        q.y = 0;
        q.z = sin(yaw/2.0);
        return q;
    }
}

#endif
