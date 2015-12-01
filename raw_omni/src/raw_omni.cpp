#include <vector>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

#include "raw_omni_driver.h"

class RawOmniNode {
private:
    std::string name_;

    RawOmniDriver driver_;

    ros::Publisher joint_pub_;
    sensor_msgs::JointState joint_state_;

    ros::Publisher pose_pub_;
    geometry_msgs::PoseStamped pose_stamped_;

    ros::Timer timer_;

public:
    RawOmniNode(const std::string& name, const std::string& serial)
            : name_(name), driver_(serial)
    {
        ros::NodeHandle n;

        // Prepare joint state publisher.
        {
            std::ostringstream topic_name;
            topic_name << name_ << "_joint_states";
            joint_pub_ = n.advertise<sensor_msgs::JointState>(
                    topic_name.str(), 10);

            joint_state_.name.resize(6);
            joint_state_.position.resize(6);
            joint_state_.name[0] = name_ + "_waist";
            joint_state_.name[1] = name_ + "_shoulder";
            joint_state_.name[2] = name_ + "_elbow";
            joint_state_.name[3] = name_ + "_wrist1";
            joint_state_.name[4] = name_ + "_wrist2";
            joint_state_.name[5] = name_ + "_wrist3";
        }

        // Prepare pose publisher.
        {
            std::ostringstream topic_name;
            topic_name << name_ << "_pose";
            pose_pub_ = n.advertise<geometry_msgs::PoseStamped>(
                    topic_name.str(), 10);

            pose_stamped_.header.frame_id = name_ + "_stylus";
        }

        // Create timer for communication.
        timer_ = n.createTimer(ros::Duration(0.0025),
                &RawOmniNode::timerHandler, this);
    }

    ~RawOmniNode()
    {
        driver_.close();
    }

    void timerHandler(const ros::TimerEvent& event)
    {
        if (driver_.opened() && !driver_.calibrated()) {
            // Phantom Omni is not open or calibrated. Don't publish.
            return;
        }

        // Get the joint angles from the omni.
        std::vector<double> joint_angles;
        driver_.get_current_joint_angles(joint_angles);

        // Publish the joint state.
        joint_state_.header.stamp = ros::Time::now();
        for (int i = 0; i < 6; i++) {
            joint_state_.position[i] = joint_angles[i];
        }
        joint_pub_.publish(joint_state_);

        // Publish the end effector pose.
        pose_stamped_.header.stamp = ros::Time::now();
        pose_stamped_.pose.position.x = 0.0;
        pose_stamped_.pose.orientation.w = 1.0;
        pose_pub_.publish(pose_stamped_);
    }

    bool run()
    {
        // Open Phantom Omni.
        if (!driver_.open()) {
            ROS_ERROR("Failed to open Phantom Omni.");
            return false;
        }
        ROS_INFO("%s (%s): opened.",
                name_.c_str(), driver_.serial_number().c_str());

        // Spin.
        ros::spin();

        return true;
    }
};

/**
 * Entry point.
 */
int main(int argc, char** argv)
{
    // Initialize ROS.
    ros::init(argc, argv, "raw_omni");

    // Get the omni parameters.
    std::string omni_name;
    ros::param::param<std::string>("~omni_name", omni_name, "omni1");
    std::string omni_serial;
    ros::param::param<std::string>("~omni_serial", omni_serial, "60604300004");

    // List the serial number of each Phantom Omni connected.
    const std::vector<RawOmniDriver::OmniInfo>& omnis = RawOmniDriver::enumerate_omnis();
    std::ostringstream serial_numbers;
    for (size_t i = 0; i < omnis.size(); i++) {
        serial_numbers << omnis[i].serial << " ";
    }
    ROS_INFO("Phantom Omni Serial Numbers: %s", serial_numbers.str().c_str());

    // Start the node.
    RawOmniNode node(omni_name, omni_serial);
    node.run();

    return 0;
}
