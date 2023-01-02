#pragma once

#include <eeros/control/ros/RosSubscriber.hpp>
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
class JointStateSubscriber : public RosSubscriber<sensor_msgs::msg::JointState::Type, JointStateSignal> {
  typedef sensor_msgs::msg::JointState::Type TRosMsg;
 public:
    /**
     * Creates an instance of a ROS subscriber block.
     * The block reads ROS messages of type sensor_msgs::msg::JointState::Type
     * from a given topic and outputs its values onto a signal of type double.
     * If several messages are pending for a given topic you can choose if the
     * block simply consumes the oldest message or processes all pending messages.
     * If no ROS master can be found, the block does not do anything.
     *
     * @param node - ROS Node as a SharedPtr
     * @param topic - Name of the topic
     * @param joint_name - Only react if a new Value for the given joint_name is received
     */
    JointStateSubscriber(const rclcpp::Node::SharedPtr node, const std::string& topic, const std::string joint_name)
        : RosSubscriber<TRosMsg, JointStateSignal>(node, topic, 1000, true, true),
        jointName(joint_name) { };

    /**
     * Disabling use of copy constructor because the block should never be copied unintentionally.
     */
    JointStateSubscriber(const JointStateSubscriber& other) = delete;

    /**
     * This function is called whenever the run function reads a pending ROS message.
     * It sets all states of all joints in the output.
     *
     * @param msg - message content
     */
    virtual void rosCallbackFct(const TRosMsg& msg) {
      for (size_t i = 0; i < msg.name.size(); i++) {
        std::string name = msg.name.at(i);
        double position = 0.0;
        double velocity = 0.0;
        double effort   = 0.0;
        if (msg.position.size() >= i) { position = msg.position.at(i); }
        if (msg.velocity.size() >= i) { position = msg.velocity.at(i); }
        if (msg.effort.size() >= i)   { position = msg.effort.at(i); }

        auto joint = states->findJoint(name);
        if (joint == nullptr) {
          states->add(name, position, velocity, effort);
        } else {
          joint->name = name;
          joint->position = position;
          joint->velocity = velocity;
          joint->effort = effort;
        }
      }

      auto time = eeros::System::getTimeNs();
      getOut().getSignal().setTimestamp(time);
      getOut().getSignal().setValue(states->getSignalStates());
    }

    /**
     * Get all JointStates as a shared pointer
     *
     * @return Shared Pointer to JointStates
     */
    std::shared_ptr<JointState> getJointStates() {
      return states;
    }

  private:
    std::string jointName;
    std::shared_ptr<JointState> states = std::make_shared<JointState>();
};

}
}
