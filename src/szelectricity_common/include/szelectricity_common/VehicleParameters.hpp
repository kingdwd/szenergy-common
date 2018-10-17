/*
* Header file containing the parameters related to vehicles
*/
#ifndef VEHICLE_PARAMETERS_HPP
#define VEHICLE_PARAMETERS_HPP

#include <string>

#include <szelectricity_common/MathCommon.hpp>
#include <szelectricity_common/Teleoperation.hpp>

namespace szenergy 
{

    
    namespace szelectricity 
    {
        const double MAX_STEER_DEG = 111.4402911529*M_PI/180.0; ///< Derived from physical vehicle parameter
        const double SZELECTRICITY_WHEEL_RADIUS = 0.556/2.0;    ///< Derived from vehicle parameter
        const double SZELECTRICITY_WHEELBASE = 1.6;             ///< Derived
        const double SZELECTRICITY_FRONT_WHEEL = 1.03;          ///< Derived
        const double SZELECTRICITY_REAR_WHEEL = 0.8;            ///< Derived
        const double VEHICLE_LENGTH = 2.0;                      ///< Derived
        const double KINGSPIN_WIDTH = 2.0;                      ///< Derived
    }

    namespace vehicle_dynamics_parameter
    {
        // These parameters are derived from drive-out measurements
        const double aerodyn = 0.1851;                      ///< Aerodynamical constant
        const double roll_mu = 2.46;                        ///< Rolling resistance
        const double motopad_x = 0.09228;                   ///< Derived linear parameter of mechanical parts
        const double motopad_p = 1.503;                     ///< Derived constant parameter of mechanical parts
        const double estimate_p = 2.073;                    ///< Estimation of mechanical friction
    }

    /**
     *  @brief: Quadratic approximation of measurement points
     *          of steer-front axle transmission used for Ackermann geometries
     * */
    double steerTransmissionPoly(double steer_angle)
    {
        const double s = Sgn<double>(steer_angle);
        const double sqr_angsteer = steer_angle*steer_angle;
        const static double c2 = 0.00143;
        const static double c1 = 0.1431;
        const static double c0 = 0.001;
        
        return steer_angle!=0.0 ? s*(c0+c1*sqrt(sqr_angsteer)+c2*sqr_angsteer): 0.0;
    }

    /**
     *  @brief: Quadratic approximation of measurement points
     *          of steer-front axle transmission based on measurements
     * */
    double steerTransmissionDataPoly(double steer_angle)
    {
        const double sqr_angsteer = steer_angle*steer_angle;
        const static double c3 = 0.001838;
        const static double c2 = -0.00772;
        const static double c1 = 0.14963;
        const static double c0 = 0.000504;
        
        return steer_angle != 0.0 ? c3*sqr_angsteer*steer_angle+c2*sqr_angsteer+c1*steer_angle+c0: 0.0;
    }

    /**
     * @brief: Odometry parameters
     * 
     * */
    struct OdometryParameters
    {
        const std::string odom_topic_name;
        const std::string steer_topic_name;
        const std::string throttle_topic_name;
        const std::vector<int> steer_joint_ids;
        const std::vector<int> throttle_joint_ids;

        OdometryParameters(
            const std::string odom_topic_name,
            const std::string steer_topic_name, 
            const std::string throttle_topic_name, 
            const std::vector<int> steer_joint_ids,
            const std::vector<int> throttle_joint_ids): 
                odom_topic_name(odom_topic_name),
                steer_topic_name(steer_topic_name), 
                throttle_topic_name(throttle_topic_name), 
                steer_joint_ids(steer_joint_ids),
                throttle_joint_ids(throttle_joint_ids)
        {
            
        }
    };

    /**
     * @brief: Vehicle parameters that are used for kinematic-based
     * algorithms, calculations
     * */
    struct VehicleParameters 
    {
        const std::string vehicle_name; ///< Referenced name of the vehicle
        const double wheelradius; ///< Radius of all of the wheels
        const double wheelbase;   ///< Distance between the front and rear axis
        const double front_track; ///< Front track width of a vehicle
        const double rear_track;  ///< Rear track width of a vehicle

        /**
         * @param<vehicle_name>: vehicle name
         * @param<wheelradius>: wheel radius
         * @param<wheelbase>: wheel base (axle-axle distance)
         * @param<front_track>: front track width
         * @param<rear_track>: rear track width
         * */
        VehicleParameters(const std::string vehicle_name,
            const double wheelradius,
            const double wheelbase,
            const double front_track,
            const double rear_track
        ): 
            vehicle_name(vehicle_name),
            wheelradius(wheelradius), 
            wheelbase(wheelbase), 
            front_track(front_track),
            rear_track(rear_track) 
            {}
    };

    /**
     * @brief: Utility structure to store control based parameters for ROS update
     * 
     * */
    struct ControlParameters {
        const std::string steer_angle_topic;            ///< ROS topic to advertise steer angle
        const std::string linear_controller_topic;      ///< ROS topic to advertise linear velocity

        /**
         * @param<steer_angle_topic>: ROS topic name to publish steer angle
         * @param<vtopic>>: ROS topic to publish the linear velocity
         * */

        ControlParameters(std::string steer_angle_topic, std::string vtopic):
            steer_angle_topic(steer_angle_topic),
            linear_controller_topic(vtopic){}
    };

    

    /**
     * @brief: Storage class to store control related parameters
     * 
     * */
    struct ControlState {
        const double update_rate;       ///< Update rate of control

        double steering_angle;          ///< Current steer angle
        double left_wheel_angle;        ///< Axial rotation of the left wheel (Z-axis)
        double right_wheel_angle;       ///< Axial rotation of the right wheel (Z-axis)

        double linear_velocity;         ///< Current linear velocity
        double effort;                  ///< Current reference effort
        double angular_velocity;        ///< Current angular velocity
        const double brake_rate;        ///< Brake rate
        const double acceleration_rate; ///< Acceleration rate, normally not needed
        double rc;                      ///< Radius of rotation circle

        /**
         * @param<update_rate>: update rate of the controller
         * @param<brake_rate>: brake rate of the target
         * @param<acceleration_rate>: acceleration rate of the target
         * 
         * */
        ControlState(const double update_rate, const double brake_rate, const double acceleration_rate): 
            update_rate(update_rate), brake_rate(brake_rate), acceleration_rate(acceleration_rate){
            steering_angle = 0.0;
            left_wheel_angle = 0.0;
            right_wheel_angle = 0.0;
            linear_velocity = 0.0;
            angular_velocity = 0.0;
            effort = 0.0;
            rc = 0.0;        
        }
    };
}
#endif
