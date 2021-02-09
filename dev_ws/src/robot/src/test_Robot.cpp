#include "rclcpp/rclcpp.hpp"
#include "gmock/gmock.h"

TEST(dummy, success)
{
    ASSERT_TRUE(true);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    int ret = RUN_ALL_TESTS();
    return ret;
}
