#ifndef RX1_TELEOP_H
#define RX1_TELEOP_H

#include "feetech_lib/SMS_STS.h"
#include "feetech_lib/SCSCL.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <string>
#include <vector>

namespace rx1_teleop
{

class Rx1Teleop
{
public:
    Rx1Teleop(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);
    ~Rx1Teleop();

    void spinOnce();
    void spin();
    void update();

protected:
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;

private:
    std::string servo_port_;
    SMS_STS sts_servo_;

    const int ARM_SERVO_NUM_ = 7;
    std::vector<int> right_servo_ids_ = {51, 52, 53, 54, 55, 56, 57};
    std::vector<int> right_servo_dirs_ = {-1, -1, 1, -1, 1, 1, -1};

    std::vector<int> left_servo_ids_ = {61, 62, 63, 64, 65, 66, 67};
    std::vector<int> left_servo_dirs_ = {-1, -1, 1, 1, 1, -1, -1};

    const u8 R_GRIPPER_CTRL_ID_ = 58;
    const u16 R_GRIPPER_TORQUE_ = 200; // %20 of maximum torque
    const s16 R_GRIPPER_OPEN_POS_ = 2400; // adjust based on actual intended value
    const s16 R_GRIPPER_CLOSE_POS_ = 2100; // adjust based on actual intended value

    const u8 L_GRIPPER_CTRL_ID_ = 68;
    const u16 L_GRIPPER_TORQUE_ = 200; // %20 of maximum torque
    const s16 L_GRIPPER_OPEN_POS_ = 1748; // adjust based on actual intended value
    const s16 L_GRIPPER_CLOSE_POS_ = 2048; // adjust based on actual intended value

    ros::Publisher right_arm_joint_state_pub_;
    sensor_msgs::JointState right_arm_joint_state_msg_;
    ros::Publisher right_gripper_pub_;

    ros::Publisher left_arm_joint_state_pub_;
    sensor_msgs::JointState left_arm_joint_state_msg_;
    ros::Publisher left_gripper_pub_;
};

} // namespace rx1_teleop

#endif // RX1_TELEOP_H
