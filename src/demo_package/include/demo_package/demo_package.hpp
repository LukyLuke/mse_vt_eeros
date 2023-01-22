#pragma once

#include <cmath>

#include <eeros/logger/Logger.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/control/ros/EerosRosTools.hpp>
#include <eeros/control/ros/RosPublisherSafetyLevel.hpp>

#include <ros-eeros/RosNodeDevice.hpp>

#include "JointState.hpp"
#include "JointStateSubscriber.hpp"
#include "JointTrajectoryPublisher.hpp"

using namespace eeros;
using namespace eeros::control;
using namespace eeros::safety;
using namespace eeros::logger;

/**
 * This class holds all inputs and outputs from the robot.
 */
class DemoRobot {
  public:
    DemoRobot(std::shared_ptr<roseeros::RosNodeDevice> node_device, const std::string root_topic, const double period) :
      wheelJoint("wheel_joint"),
      publishJoint(0.0),
      jointTrajectoryPublisher(node_device->getRosNodeHandle(), root_topic + "/set_joint_trajectory", wheelJoint),
      jointStateSubscriber(node_device->getRosNodeHandle(), root_topic + "/joint_states", wheelJoint),
      safetyPublisher(node_device->getRosNodeHandle(), root_topic + "/safetyLevel"),
      timedomain(node_device->getRosNodeHandle(), "Time Domain for " + root_topic, period, true)
    {
      jointTrajectoryPublisher.getIn().connect(publishJoint.getOut());

      timedomain.addBlock(publishJoint);
      timedomain.addBlock(jointTrajectoryPublisher);
      timedomain.addBlock(jointStateSubscriber);
      Executor::instance().add(timedomain);
    }

    std::string wheelJoint;

    Constant<double> publishJoint;
    JointTrajectoryPublisher jointTrajectoryPublisher;
    JointStateSubscriber jointStateSubscriber;
    RosPublisherSafetyLevel safetyPublisher;
    TimeDomain timedomain;

    double step = 0.5;
};

/**
 * Every change of the movement is made in here
 */
class DemoRobotEvents : public SafetyProperties {
public:
  DemoRobotEvents(DemoRobot &robot) :
    state("JointState"),
    log(Logger::getLogger())
  {
    addLevel(state);
    setEntryLevel(state);

    // Rotate the wheel - this should be the motor in reality
    state.setLevelAction([&](SafetyContext* context) {
      (void) context;

      auto states = robot.jointStateSubscriber.getJointStates();
      auto wheel = states->findJoint(robot.wheelJoint);

      // Set the angle of the motor between -PI < x < PI as soon as the wheel is on the same position
      if (wheel != nullptr) {
        //double angle_new = wheel->position + robot.step;
        //if (angle_new > M_PI) {
        //  angle_new = 0 - M_PI + robot.step;
        //}
        //robot.publishJoint.setValue(angle_new);

        // Round the current position so we not run away too much with the value we want to move to
        double position_new = round(wheel->position) + robot.step;
        robot.publishJoint.setValue(position_new);
      }
    });
  }

  SafetyLevel state;
  Logger log;
};


/**
 * The main Robot-Class which is started and visible by the user
 */
class DemoSimulationRobot
{
public:
  DemoSimulationRobot(const std::string node_name = "DemoSimuRobot", const std::string root_topic = "", const double period = 0.1);
  void run();
  static void stop(int sigNum) {
    (void) sigNum;
    Executor::stop();
  };

private:
  std::shared_ptr<roseeros::RosNodeDevice> node;
  DemoRobot robot;
  DemoRobotEvents events;
  SafetySystem* safetySystem;
};
