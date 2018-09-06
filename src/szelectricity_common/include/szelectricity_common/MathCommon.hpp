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

#ifndef MATH_COMMON_HPP
#define MATH_COMMON_HPP


namespace szenergy {

    /**
     *  @brief: Minimal clamp function
     * */
    inline double Clamp(const double& val, const double& min, const double& max)
    {
        const double t = val < min ? min: val;
        return t > max ? max : t; // This is minimal, yet efficient
    }

    /**
     * @brief: Cutoff input value below a minimal value and above a maximal value
     * */
    inline double CutoffRange(const double& val, 
        const double& cutoffMin,     // Minimal value threshold
        const double& cutoffMinVal,  // Minimal returning value
        const double& cutoffMax,     // Maximal value threshold
        const double& cutoffMaxVal   // Maximal returning value
        )
    {
        const double t = val < cutoffMin ? cutoffMinVal: val;
        return t > cutoffMax ? cutoffMaxVal : t; // This is minimal, yet efficient
    }

    /**
     *  @brief: Cutoff input value below a defined value
     * */
    inline double CutoffMin(const double& val, const double& cutoff, const double& val_cutoff)
    {
        return val < cutoff ? val_cutoff : val;
    }

    /**
     * @brief: Cutoff input value above a defined value
     * */
    inline double CutoffMax(const double& val, const double& cutoff, const double& val_cutoff)
    {
        return val > cutoff ? val_cutoff : val;
    }

    /**
     * @brief: Maximal threshold
     * */
    inline double ThresholdMax(const double& val, const double& threshold)
    {
        return val < threshold ? val : threshold;
    }

    /**
     * @brief: Minimal threshold
     * */
    inline double ThresholdMin(const double& val, const double& threshold)
    {
        return val > threshold ? val : threshold;
    }

    /**
     * @brief: Very useful one-liner sign function
     * */
    template <typename T> int Sgn(const T& val) {
        return (T(0) < val) - (val < T(0));  // No branches used, only simple arithmetics
    }
}

#endif
