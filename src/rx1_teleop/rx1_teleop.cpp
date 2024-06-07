#include "rx1_teleop/rx1_teleop.hpp"

#include <std_msgs/Float32.h>

namespace rx1_teleop
{
Rx1Teleop::Rx1Teleop(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
    : nh_(nh),
      priv_nh_(priv_nh)
{
    servo_port_ = "/dev/ttyUSB-arduino4.3";

    if (!sts_servo_.begin(1000000, servo_port_.c_str()))
    {
        ROS_ERROR("[RX1_TELEOP] Failed initialize servo!");
    }

    // Set initial values for arm joint states
    joint_state_msg_.header.stamp = ros::Time::now();
    joint_state_msg_.name.resize(ARM_SERVO_NUM_);
    joint_state_msg_.position.resize(ARM_SERVO_NUM_);
    std::vector<std::string> joint_name = {"right_shoul_base2shoul_joint", "right_shoul2shoul_rot_joint", "right_arm2armrot_joint", "right_armrot2elbow_joint", "right_forearm2forearmrot_joint", 
                                           "right_forearmrot2forearm_pitch_joint", "right_forearm_pitch2forearm_roll_joint"};
    for (int i = 0; i < ARM_SERVO_NUM_; ++i)
    {
        joint_state_msg_.position[i] = 0;
        joint_state_msg_.name[i] = joint_name[i];
    }
    joint_state_msg_.position[3] = -1.57;
    joint_state_pub_.publish(joint_state_msg_);
    ROS_INFO("[RX1_TELEOP] Joint state initialized");

    // Turn off torque for all arm joints servo
    for (int i = 0; i < ARM_SERVO_NUM_; i ++)
    {
        u8 id = sts_servo_ids_[i];
        sts_servo_.EnableTorque(id, 0); 
    }

    // Set initial value for the gripper controller servo
    sts_servo_.WriteTorqueLimit(R_GRIPPER_CTRL_ID_, R_GRIPPER_TORQUE_); 
    sts_servo_.WritePosEx(R_GRIPPER_CTRL_ID_, R_GRIPPER_OPEN_POS_, 0, 0); 

    // Initialize publishers
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);
    right_gripper_pub_ = nh_.advertise<std_msgs::Float32>("right_gripper_pos", 10);

    ROS_INFO("[RX1_TELEOP] Teleop arm initialized");
}

Rx1Teleop::~Rx1Teleop()
{
    sts_servo_.end();
}

void Rx1Teleop::spinOnce()
{
    ros::spinOnce();
    update();
}

void Rx1Teleop::spin()
{
    ros::Rate rate(30);
    while(ros::ok())
    {
        spinOnce();
        rate.sleep();
    }
}

void Rx1Teleop::update()
{
    // Read all teleop arm servo value
    for (int i = 0; i < 7; i ++)
    {
        //if (sts_servo_.FeedBack(sts_servo_ids_[i]) == -1)
        //{ 
        //    ROS_INFO("servo %d not responding", i);
        //}
        //else{
        //    ROS_INFO("servo %d pos: %d", i, sts_servo_.ReadPos(sts_servo_ids_[i]));
            joint_state_msg_.position[i] = sts_servo_dirs_[i] * (static_cast<float>(sts_servo_.ReadPos(sts_servo_ids_[i])) - 2048.0) / 2048.0 * 3.14;
        //}
    }

    // Publish joint states
    joint_state_msg_.header.stamp = ros::Time::now();
    joint_state_pub_.publish(joint_state_msg_);

    // Read gripper controller value and publish
    int r_gripper_var = sts_servo_.ReadPos(R_GRIPPER_CTRL_ID_);
    if (r_gripper_var < R_GRIPPER_CLOSE_POS_)
    {
        r_gripper_var = R_GRIPPER_CLOSE_POS_;
    }
    else if (r_gripper_var > R_GRIPPER_OPEN_POS_)
    {
        r_gripper_var = R_GRIPPER_OPEN_POS_;
    }
    std_msgs::Float32 r_gripper_pos_msg;
    r_gripper_pos_msg.data = static_cast<float>(r_gripper_var-R_GRIPPER_CLOSE_POS_)/static_cast<float>(R_GRIPPER_OPEN_POS_-R_GRIPPER_CLOSE_POS_);
    right_gripper_pub_.publish(r_gripper_pos_msg);

    s16 pos_1, pos_2;
    pos_1 = 2300 + r_gripper_pos_msg.data*100;
    pos_2 = 2000 + r_gripper_pos_msg.data*300;
    sts_servo_.WritePosEx(32, pos_1, 0, 100);
    sts_servo_.WritePosEx(33, pos_2, 0, 100);
}

} // namespace rx1_teleop
