#include "rx1_teleop/rx1_teleop.hpp"

#include <std_msgs/Float32.h>

namespace rx1_teleop
{
Rx1Teleop::Rx1Teleop(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
    : nh_(nh),
      priv_nh_(priv_nh)
{
    nh_.param<std::string>("rx1_teleop_node/servo_port", servo_port_, "/dev/ttyUSB-arduino4.2");

    if (!sts_servo_.begin(1000000, servo_port_.c_str()))
    {
        ROS_ERROR("[RX1_TELEOP] Failed initialize servo!");
    }

    // Set initial values for arm joint states
    right_arm_joint_state_msg_.header.stamp = ros::Time::now();
    right_arm_joint_state_msg_.name.resize(ARM_SERVO_NUM_);
    right_arm_joint_state_msg_.position.resize(ARM_SERVO_NUM_);
    std::vector<std::string> right_joint_name = {"right_shoul_base2shoul_joint", 
                                         "right_shoul2shoul_rot_joint", 
                                         "right_arm2armrot_joint", 
                                         "right_armrot2elbow_joint", 
                                         "right_forearm2forearmrot_joint", 
                                         "right_forearmrot2forearm_pitch_joint", 
                                         "right_forearm_pitch2forearm_roll_joint"};
    for (int i = 0; i < ARM_SERVO_NUM_; ++i)
    {
        right_arm_joint_state_msg_.position[i] = 0;
        right_arm_joint_state_msg_.name[i] = right_joint_name[i];
    }

    left_arm_joint_state_msg_.header.stamp = ros::Time::now();
    left_arm_joint_state_msg_.name.resize(ARM_SERVO_NUM_);
    left_arm_joint_state_msg_.position.resize(ARM_SERVO_NUM_);
    std::vector<std::string> left_joint_name = {"left_shoul_base2shoul_joint", 
                                         "left_shoul2shoul_rot_joint", 
                                         "left_arm2armrot_joint", 
                                         "left_armrot2elbow_joint", 
                                         "left_forearm2forearmrot_joint", 
                                         "left_forearmrot2forearm_pitch_joint", 
                                         "left_forearm_pitch2forearm_roll_joint"};
    for (int i = 0; i < ARM_SERVO_NUM_; ++i)
    {
        left_arm_joint_state_msg_.position[i] = 0;
        left_arm_joint_state_msg_.name[i] = left_joint_name[i];
    }

    ROS_INFO("[RX1_TELEOP] Joint state initialized");

    // Turn off torque for all arm joints servo
    for (int i = 0; i < ARM_SERVO_NUM_; i ++)
    {
        u8 id = right_servo_ids_[i];
        sts_servo_.EnableTorque(id, 0); 
        
        id = left_servo_ids_[i];
        sts_servo_.EnableTorque(id, 0); 
    }

    // Set initial value for the gripper controller servo
    sts_servo_.WriteTorqueLimit(R_GRIPPER_CTRL_ID_, R_GRIPPER_TORQUE_); 
    sts_servo_.WritePosEx(R_GRIPPER_CTRL_ID_, R_GRIPPER_OPEN_POS_, 0, 0); 
    sts_servo_.WriteTorqueLimit(L_GRIPPER_CTRL_ID_, L_GRIPPER_TORQUE_); 
    sts_servo_.WritePosEx(L_GRIPPER_CTRL_ID_, L_GRIPPER_OPEN_POS_, 0, 0); 

    // Initialize publishers
    right_arm_joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("right_arm_joint_states", 100);
    right_gripper_pub_ = nh_.advertise<std_msgs::Float32>("right_gripper", 100);
    left_arm_joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("left_arm_joint_states", 100);
    left_gripper_pub_ = nh_.advertise<std_msgs::Float32>("left_gripper", 100);

    ROS_INFO("[RX1_TELEOP] Teleop arm initialized");
}

Rx1Teleop::~Rx1Teleop()
{
    sts_servo_.end();
}

void Rx1Teleop::spinOnce()
{
    ros::spinOnce();

    ros::Time update_time_start = ros::Time::now(); 
    update();
    double update_time = (ros::Time::now() - update_time_start).toSec();
    ROS_INFO("[RX1_TELEOP] teleop update time is %f sec", update_time );
}

void Rx1Teleop::spin()
{
    ros::Rate rate(100);
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
        right_arm_joint_state_msg_.position[i] = right_servo_dirs_[i] * (static_cast<float>(sts_servo_.ReadPos(right_servo_ids_[i])) - 2048.0) / 2048.0 * 3.14;
        left_arm_joint_state_msg_.position[i] = left_servo_dirs_[i] * (static_cast<float>(sts_servo_.ReadPos(left_servo_ids_[i])) - 2048.0) / 2048.0 * 3.14;
    }

    // Publish joint states
    right_arm_joint_state_msg_.header.stamp = ros::Time::now();
    right_arm_joint_state_pub_.publish(right_arm_joint_state_msg_);
    left_arm_joint_state_msg_.header.stamp = ros::Time::now();
    left_arm_joint_state_pub_.publish(left_arm_joint_state_msg_);

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
    
    r_gripper_pos_msg.data = static_cast<float>(R_GRIPPER_OPEN_POS_ - r_gripper_var)/static_cast<float>(R_GRIPPER_OPEN_POS_-R_GRIPPER_CLOSE_POS_);
    right_gripper_pub_.publish(r_gripper_pos_msg);

    int l_gripper_var = sts_servo_.ReadPos(L_GRIPPER_CTRL_ID_);
    if (l_gripper_var < L_GRIPPER_OPEN_POS_)
    {
        l_gripper_var = L_GRIPPER_OPEN_POS_;
    }
    else if (l_gripper_var > L_GRIPPER_CLOSE_POS_)
    {
        l_gripper_var = L_GRIPPER_CLOSE_POS_;
    }
    std_msgs::Float32 l_gripper_pos_msg;
    
    l_gripper_pos_msg.data = static_cast<float>(L_GRIPPER_OPEN_POS_ - l_gripper_var)/static_cast<float>(L_GRIPPER_OPEN_POS_-L_GRIPPER_CLOSE_POS_);
    left_gripper_pub_.publish(l_gripper_pos_msg);
}

} // namespace rx1_teleop
