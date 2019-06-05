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
    double Clamp(const double& val, const double& min, const double& max);

    /**
     * @brief: Cutoff input value below a minimal value and above a maximal value
     * */
    template<typename T> T CutoffRange(const T& val, 
        const T& cutoffMin,     // Minimal value threshold
        const T& cutoffMinVal,  // Minimal returning value
        const T& cutoffMax,     // Maximal value threshold
        const T& cutoffMaxVal   // Maximal returning value
        )
    {
        const T t = val < cutoffMin ? cutoffMinVal: val;
        return t > cutoffMax ? cutoffMaxVal : t; // This is minimal, yet efficient    
    }

    template<> double CutoffRange<double>(const double&, 
        const double&,     // Minimal value threshold
        const double&,  // Minimal returning value
        const double&,     // Maximal value threshold
        const double&   // Maximal returning value
    );

    /**
     *  @brief: Cutoff input value below a defined value
     * */
    template<typename T> T CutoffMin(const T& val, const T& cutoff, const T& val_cutoff)
    {
        return val < cutoff ? val_cutoff : val;
    }
    template<> double CutoffMin<double>(const double&, const double&, const double&);

    /**
     * @brief: Cutoff input value above a defined value
     * */
    template<typename T> double CutoffMax(const T& val, const T& cutoff, const T& val_cutoff)
    {
        return val > cutoff ? val_cutoff : val;
    }
    template<> double CutoffMax<double>(const double&, const double&, const double&);

    /**
     * @brief: Maximal threshold
     * */
    template<typename T> T ThresholdMax(const T& val, const T& threshold)
    {
        return val < threshold ? val : threshold;
    }
    template<> double ThresholdMax<double>(const double&, const double&);

    /**
     * @brief: Minimal threshold
     * */
    template<typename T> T ThresholdMin(const T& val, const T& threshold)
    {
        return val > threshold ? val : threshold;
    }
    template<> double ThresholdMin<double>(const double&, const double&);

    /**
     * @brief: Very useful one-liner sign function
     * */
    template <typename T> int Sgn(const T& val) {
        return (T(0) < val) - (val < T(0));  // No branches used, only simple arithmetics
    }
    template<> int Sgn<double>(const double&);
}

#endif
