#pragma once

#include <eeros/control/ros/RosPublisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "JointState.hpp"

namespace eeros {
namespace control {

/**
 * This block allows to subscribe to a ROS message of type sensor_msgs::msg::JointState::Type
 * and publishes it as a signal of type double.
 *
 * @since v2.0
 */
class JointStatePublisher : public RosPublisher<sensor_msgs::msg::JointState::Type, double> {
  typedef sensor_msgs::msg::JointState::Type TRosMsg;
 public:
    /**
     * Creates an instance of a publisher block which publishes a input signal
   * of type double as a ROS message of type std_msgs::msg::Float64.
     *
     * @param node - ROS Node as a SharedPtr
     * @param topic - Name of the topic
     * @param joint_name - Name of the joint to publishthe position
     */
    JointStatePublisher(const rclcpp::Node::SharedPtr node, const std::string& topic, const std::string joint_name)
      : RosPublisher<TRosMsg, double>(node, topic),
      jointName(joint_name) { }

    /**
     * Disabling use of copy constructor because the block should never be copied unintentionally.
     */
    JointStatePublisher(const JointStatePublisher& other) = delete;

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
      std::vector<double> efforts = { 0.0 };

      msg.set__name(names)
        .set__position(positions)
        .set__velocity(velocities)
        .set__effort(efforts);
    }

  private:
    std::string jointName;
};

}
}
