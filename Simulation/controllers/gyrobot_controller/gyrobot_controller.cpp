#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <limits>

using namespace webots;

int main() {
  Robot *robot = new Robot();
  const int dt = static_cast<int>(robot->getBasicTimeStep());  // 5 ms

  Motor *motG = robot->getMotor("MotG");
  Motor *motD = robot->getMotor("MotD");
  motG->setPosition(std::numeric_limits<double>::infinity());  // mode vitesse
  motD->setPosition(std::numeric_limits<double>::infinity());

  while (robot->step(dt) != -1) {
    motG->setVelocity(2.0);
    motD->setVelocity(2.0);
  }

  delete robot;
  return 0;
}