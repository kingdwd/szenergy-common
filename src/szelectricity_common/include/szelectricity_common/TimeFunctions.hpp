#ifndef SZENERGY_TIME_FUNCTIONS_HPP
#define SZENERGY_TIME_FUNCTIONS_HPP

#include <ros/ros.h>

namespace szenergy
{
    bool SynchronizeRosTime()
    {
        // Wait for clock synchronization
        auto t_last_update = ros::Time::now();
        while(ros::ok())
        {
            t_last_update = ros::Time::now();            
            if (t_last_update.toSec()!=0.0)
            {
                return true;
            }
        }
        return false;
    }
}

#endif
