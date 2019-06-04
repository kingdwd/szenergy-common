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

#ifndef SZENERGY_TELEOPERATION_HPP
#define SZENERGY_TELEOPERATION_HPP
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include "SzenergyConstants.hpp"

namespace szenergy
{
    const double DEFAULT_MAX_BRAKE_RATE = 0.65; ///< Default brake rate for teleoperation purpose

    /**
     * @brief: control structure ready to publish messages on ROS topics
     * 
     * */
    struct RosTeleopPubState
    {
        std_msgs::Float64 msg_torque; ///< Torque message to be published
        std_msgs::Float64 msg_steer;  ///< Steer message to be published
        std_msgs::Bool msg_brake;     ///< Brake message to be published
    };

    bool operator==(const RosTeleopPubState& lhs, const RosTeleopPubState& rhs);

    /**
     * @brief: Structure to store required parameters for teleoperation
     * */
    struct TeleopState 
    {
        const double steerMin; ///< Minimal steering angle (in radians)
        const double steerMax; ///< Maximal steering angle (in radians)
        const double effortchange; ///< Maximal change of effort reference (used for keyboard teleoperation)
        const double maxBrakeRate; ///< Maximal brake rate (decay)
        const double update_rate; ///< Pre-defined update rate for teleoperation
        double effort; ///< Reference throttle
        double steer_angle; ///< Current reference steer angle
        double gaspedal; ///< Current gaspedal measurement
        
        bool reverse; ///< Switch forward/reverse
        ///      TRUE: reverse
        ///      FALSE (default): forward 

        /**
         * @param<steerMax>: max steer range (in radians)
         * @param<maxAcceleration>: maximal acceleration rate of the target vehicle
         * @param<maxBrakeRate>: maximal brake rate of the target vehicle
         * @param<updateRate>: update rate of the teleoperation
         * 
         * */
        TeleopState(const float steerMin,
            const double steerMax,
            const double maxAcceleration,
            const double maxBrakeRate = DEFAULT_MAX_BRAKE_RATE,
            const double updateRate = szenergy::TELEOP_UPDATE_RATE): 
                steerMin(steerMin), steerMax(steerMax), 
                effortchange(maxAcceleration),
                maxBrakeRate(maxBrakeRate),
                update_rate(updateRate),
                steer_angle(0.0),
                effort(0.0),
                reverse(false)
            {      
            }
    };
}
#endif
