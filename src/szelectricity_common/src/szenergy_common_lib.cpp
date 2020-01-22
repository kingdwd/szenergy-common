#include <szelectricity_common/Teleoperation.hpp>
#include <szelectricity_common/TimeFunctions.hpp>
#include <szelectricity_common/VehicleParameters.hpp>
#include <szelectricity_common/MathGeometry.hpp>

namespace szenergy
{
    bool operator==(const RosTeleopPubState& lhs, const RosTeleopPubState& rhs)
    {
        return 
            rhs.msg_torque.data == lhs.msg_torque.data &&
            rhs.msg_steer.data == lhs.msg_steer.data &&
            (rhs.msg_brake.data == lhs.msg_brake.data);
    }

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

    extern double steerTransmissionDataPoly(double steer_angle)
    {
        const double sqr_angsteer = steer_angle*steer_angle;
        const static double c3 = 0.001838;
        const static double c2 = -0.00772;
        const static double c1 = 0.14963;
        const static double c0 = 0.000504;
        
        return steer_angle != 0.0 ? c3*sqr_angsteer*steer_angle+c2*sqr_angsteer+c1*steer_angle+c0: 0.0;
    }

    extern double steerTransmissionPoly(double steer_angle)
    {
        const double s = Sgn<double>(steer_angle);
        const double sqr_angsteer = steer_angle*steer_angle;
        const static double c2 = 0.00143;
        const static double c1 = 0.1431;
        const static double c0 = 0.001;
        
        return steer_angle!=0.0 ? s*(c0+c1*sqrt(sqr_angsteer)+c2*sqr_angsteer): 0.0;
    }

    geometry_msgs::Quaternion YawToQuaternion(const double yaw)
    {
        geometry_msgs::Quaternion q;
        // Quaternion parameters can be derived using simple arithmetics
        q.w = cos(yaw/2.0);
        q.x = 0;
        q.y = 0;
        q.z = sin(yaw/2.0);
        return q;
    }

    double GetYawFromQuaternion(const geometry_msgs::Pose &c)
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


    geometry_msgs::Pose RotateYaw(geometry_msgs::Pose &c, const double yaw)
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

    double Clamp(const double& val, const double& min, const double& max)
    {
        const double t = val < min ? min: val;
        return t > max ? max : t; // This is minimal, yet efficient
    }

    double CutoffRange(const double& val, 
        const double& cutoffMin,     // Minimal value threshold
        const double& cutoffMinVal,  // Minimal returning value
        const double& cutoffMax,     // Maximal value threshold
        const double& cutoffMaxVal   // Maximal returning value
        )
    {
        const double t = val < cutoffMin ? cutoffMinVal: val;
        return t > cutoffMax ? cutoffMaxVal : t; // This is minimal, yet efficient
    }

    double CutoffMin(const double& val, const double& cutoff, const double& val_cutoff)
    {
        return val < cutoff ? val_cutoff : val;
    }

    double CutoffMax(const double& val, const double& cutoff, const double& val_cutoff)
    {
        return val > cutoff ? val_cutoff : val;
    }

    double ThresholdMax(const double& val, const double& threshold)
    {
        return val < threshold ? val : threshold;
    }

    double ThresholdMin(const double& val, const double& threshold)
    {
        return val > threshold ? val : threshold;
    }

    template <typename T> int Sgn(const T& val) {
        return (T(0) < val) - (val < T(0));  // No branches used, only simple arithmetics
    }

    template <> int Sgn(const double& val) {
        return (0.0 < val) - (val < 0.0);  // No branches used, only simple arithmetics
    }

    double DistanceBetweenTwoPose(const geometry_msgs::Pose &p0, const geometry_msgs::Pose &p1)
    {
    	double dx = p0.position.x - p1.position.x;
    	double dy = p0.position.y - p1.position.y;
    	double dz = p0.position.z - p1.position.z;
    	return sqrt(dx*dx + dy*dy + dz*dz);
    }
}
