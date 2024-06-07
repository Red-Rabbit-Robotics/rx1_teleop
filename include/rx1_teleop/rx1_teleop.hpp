#ifndef RX1_TELEOP_H
#define RX1_TELEOP_H

#include "feetech_lib/SMSBL.h"
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
    SMSBL sts_servo_;

    //std::vector<int> sts_servo_ids_ = {0, 1, 2, 3, 4, 5};
    const int ARM_SERVO_NUM_ = 7;
    std::vector<int> sts_servo_ids_ = {51, 52, 53, 54, 55, 56, 57};
    std::vector<int> sts_servo_dirs_ = {-1, -1, 1, 1, 1, 1, -1};

    const u8 R_GRIPPER_CTRL_ID_ = 58;
    const u16 R_GRIPPER_TORQUE_ = 200; // %20
    const s16 R_GRIPPER_OPEN_POS_ = 2400;
    const s16 R_GRIPPER_CLOSE_POS_ = 2100;

    ros::Publisher joint_state_pub_;
    sensor_msgs::JointState joint_state_msg_;
    
    ros::Publisher right_gripper_pub_;
};

} // namespace rx1_teleop

#endif // RX1_TELEOP_H
