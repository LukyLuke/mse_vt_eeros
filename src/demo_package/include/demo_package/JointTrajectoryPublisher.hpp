#pragma once

#include <eeros/control/ros/RosPublisher.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "JointState.hpp"

namespace eeros {
namespace control {

/**
 * This block allows to subscribe to a ROS message of type trajectory_msgs::msg::JointTrajectory::Type
 * and publishes it as a signal of type double.
 *
 * @since v2.0
 */
class JointTrajectoryPublisher : public RosPublisher<trajectory_msgs::msg::JointTrajectory::Type, double> {
  typedef trajectory_msgs::msg::JointTrajectory::Type TRosMsg;
 public:
    /**
     * Creates an instance of a publisher block which publishes a input signal
     * of type double as a ROS message of type std_msgs::msg::Float64.
     *
     * @param node - ROS Node as a SharedPtr
     * @param topic - Name of the topic
     * @param joint_name - Name of the joint to publishthe position
     */
    JointTrajectoryPublisher(const rclcpp::Node::SharedPtr node, const std::string& topic, const std::string joint_name)
      : RosPublisher<TRosMsg, double>(node, topic),
      jointName(joint_name) { }

    /**
     * Disabling use of copy constructor because the block should never be copied unintentionally.
     */
    JointTrajectoryPublisher(const JointTrajectoryPublisher& other) = delete;

    /**
     * Sets the message to be published by this block.
     *
     * @param msg - message content
     */
    virtual void setRosMsg(TRosMsg& msg) {
      auto position = getIn().getSignal().getValue();

      std::vector<std::string> names = { jointName };
      std::vector<double> positions = { position };
      std::vector<double> velocities = { 0.0 };
      std::vector<double> accelerations = { 0.0 };
      std::vector<double> efforts = { 0.0 };

      eeros::logger::Logger::getLogger().info() << "PUBLISHER: " << jointName << " = " << position;

      trajectory_msgs::msg::JointTrajectoryPoint::Type points;
      points.set__positions(positions)
        .set__velocities(velocities)
        .set__accelerations(accelerations)
        .set__effort(efforts);
      msg.set__joint_names(names)
        .set__points({points});
      msg.header.set__frame_id("map");
    }

  private:
    std::string jointName;
};

}
}
