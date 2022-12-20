#pragma once

#include <eeros/logger/Logger.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/control/ros/EerosRosTools.hpp>
#include <eeros/control/ros/RosPublisherSafetyLevel.hpp>

#include <ros-eeros/RosNodeDevice.hpp>

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
      safetyPublisher(node_device->getRosNodeHandle(), root_topic + "/safetyLevel"),
      timedomain(node_device->getRosNodeHandle(), "Time Domain for " + root_topic, period, true)
    {
      Executor::instance().add(timedomain);
    }

    RosPublisherSafetyLevel safetyPublisher;
    TimeDomain timedomain;
};

/**
 * Every change of the movement is made in here
 */
class DemoRobotEvents : public SafetyProperties {
public:
  DemoRobotEvents(DemoRobot &robot) :
  twist("Twist"),
  log(Logger::getLogger())
  {
    addLevel(twist);
    setEntryLevel(twist);

    twist.setLevelAction([&](SafetyContext* context) {
      // TODO: Do something here likemove the robot randomly...
      (void) context;
      (void) robot;
    });
  }

  SafetyLevel twist;
  Logger log;
};


/**
 * The main Robot-Class which is started and visible by the user
 */
class DemoSimulationRobot
{
public:
  DemoSimulationRobot(const std::string node_name = "DemoSimuRobot", const std::string root_topic = "/demo", const double period = 0.1);
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
