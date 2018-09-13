#ifndef SZENERGY_CONSTANTS_HPP
#define SZENERGY_CONSTANTS_HPP


namespace szenergy 
{
    const double RAD_PER_S_RPM = 60/2*M_PI; ///< Should be used as a constant
    const double TELEOP_QUEUE_SIZE = 100; ///< Queue size used for ROS publishers
    
    const unsigned int TELEOP_REFRESH_MS = 10; ///< Refresh rate of the teleoperation in milliseconds
    const unsigned int TELEOP_UPDATE_RATE = 50; ///< Refresh rate of the teleoperation in milliseconds
}

#endif
