
#include "demo_package/demo_package.hpp"
#include <signal.h>

DemoSimulationRobot::DemoSimulationRobot(const std::string node_name, const std::string root_topic, const double period) :
  node(roseeros::RosNodeDevice::getDevice(node_name)),
  robot(node, root_topic, period),
  events(robot)
{
  safetySystem = new SafetySystem(events, period);
  robot.safetyPublisher.setSafetySystem(*safetySystem);
}

void DemoSimulationRobot::run() {
  Executor::instance().setMainTask(*safetySystem);
  Executor::instance().run();
}


int main() {
  Logger::setDefaultStreamLogger(std::cout);
  Logger log = Logger::getLogger();
  log.show(LogLevel::TRACE);
  log.info() << "DemoSimulationRobot started";

  auto demo = DemoSimulationRobot();
  signal(SIGINT, DemoSimulationRobot::stop);

  demo.run();

  log.info() << "DemoSimulationRobot ended";
  return 0;
}
