#include "MathCommon.hpp"

#include <gtest/gtest.h>


TEST(CommonMathTest, clampSimpleTest0)
{
    ASSERT_EQ(szenergy::Clamp(0.5, -1, 1), 0.5);
}

TEST(CommonMathTest, clampSimpleTest1)
{
    ASSERT_EQ(szenergy::Clamp(2.5, -1, 3), 2.5);
}

TEST(CommonMathTest, clampMinTest0)
{
    ASSERT_EQ(szenergy::Clamp(-1.5, -1, 1), -1);
}

TEST(CommonMathTest, clampMinTest1)
{
    ASSERT_EQ(szenergy::Clamp(-1.5, -2.5, 1), -1.5);
}

TEST(CommonMathTest, clampMaxTest0)
{
    ASSERT_EQ(szenergy::Clamp(4.0, -2.5, 1), 1);
}

TEST(CommonMathTest, sgnTestNegative)
{
    ASSERT_EQ(-1, szenergy::Sgn<double>(-3.44343534));
}

TEST(CommonMathTest, sgnTestZero)
{
    ASSERT_EQ(0, szenergy::Sgn<double>(0.0));
}

TEST(CommonMathTest, sgnTestPositive)
{
    ASSERT_EQ(1, szenergy::Sgn<double>(1.5654754));
}

TEST(CommonMathTest, thresholdMinUpThreshold)
{
    ASSERT_EQ(9.3, szenergy::ThresholdMin(9.3, 1.0));
}

TEST(CommonMathTest, thresholdMinUnderThreshold)
{
    ASSERT_EQ(1.0, szenergy::ThresholdMin(-1.0, 1.0));
}

TEST(CommonMathTest, thresholdMaxAboveThreshold)
{
    ASSERT_EQ(1.0, szenergy::ThresholdMax(9.3, 1.0));
}

TEST(CommonMathTest, thresholdMaxUnderThreshold)
{
    ASSERT_EQ(-1.0, szenergy::ThresholdMax(-1.0, 1.0));
}

TEST(CommonMathTest, cutoffMinBelow)
{
    ASSERT_EQ(0.0, szenergy::CutoffMin(4e-4,1e-3,0.0));
}

TEST(CommonMathTest, cutoffMinAbove)
{
    ASSERT_DOUBLE_EQ(4e-3, szenergy::CutoffMin(4e-3,1e-3,0.0));
}

TEST(CommonMathTest, cutoffMaxAbove)
{
    ASSERT_DOUBLE_EQ(10.0, szenergy::CutoffMax(11.0,10.0,10.0));
}

TEST(CommonMathTest, cutoffMaxBelow)
{
    ASSERT_DOUBLE_EQ(9.0, szenergy::CutoffMax(9.0,10.0,10.0));
}

TEST(CommonMathTest, cutoffRangeBetween)
{
    ASSERT_DOUBLE_EQ(8.0, szenergy::CutoffRange(8.0, 1.0, 0.0, 9.0, 10.0));
}

TEST(CommonMathTest, cutoffRangeMinBelow)
{
    ASSERT_DOUBLE_EQ(0.0, szenergy::CutoffRange(-1.0, 1.0, 0.0, 9.0, 10.0));
}

TEST(CommonMathTest, cutoffRangeMaxAbove)
{
    ASSERT_DOUBLE_EQ(10.0, szenergy::CutoffRange(11.0, 1.0, 0.0, 9.0, 10.0));
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}