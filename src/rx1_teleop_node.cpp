#include "rx1_teleop/rx1_teleop.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rx1_teleop_node");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    rx1_teleop::Rx1Teleop rx1_teleop_node(nh, priv_nh);

    try
    {
        rx1_teleop_node.spin();
    }
    catch (std::runtime_error& ex)
    {
        ROS_FATAL_STREAM("[RX1_TELEOP] Runtime error: " << ex.what());
        return 1;
    }
    return 0;
}

