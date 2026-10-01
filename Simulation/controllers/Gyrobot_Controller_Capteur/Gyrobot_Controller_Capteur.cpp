#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>
#include <cstdio>
#include <limits>

using namespace webots;

int main() {
  Robot *robot = new Robot();  //creation du robot
  const int dt = static_cast<int>(robot->getBasicTimeStep()); //recuperation pas de temps en entier

  Motor *motG = robot->getMotor("MotG");  //creation outils moteur
  Motor *motD = robot->getMotor("MotD");
  motG->setPosition(std::numeric_limits<double>::infinity());  //mode vitesse et non position
  motD->setPosition(std::numeric_limits<double>::infinity());

  InertialUnit *imu = robot->getInertialUnit("imu");  //creation imu
  Gyro *gyro = robot->getGyro("gyro");  //creation gyro
  PositionSensor *codG = robot->getPositionSensor("CodG");  //creation encodeur
  PositionSensor *codD = robot->getPositionSensor("CodD");
  imu->enable(dt);  //activation des capteur initialise
  gyro->enable(dt);
  codG->enable(dt);
  codD->enable(dt);

  int i = 0;  //compteur
  while (robot->step(dt) != -1) {  //faire avancer la simule en pas par pas
    const double *rpy = imu->getRollPitchYaw();  ///lecture imu
    const double *g = gyro->getValues();  //lecture gyro
    const double roll = rpy[0];   // inclinaison avant-arrière (roues sur l'axe x)
    const double gx = g[0];       // vitesse angulaire autour de x

    if (i % 2 == 0) {            // un affichage toutes les 10 ms pour ne pas trop ralentir
      printf("roll=%.3f rad | gyro_x=%.3f rad/s | codG=%.2f rad | codD=%.2f rad\n",
             roll, gx, codG->getValue(), codD->getValue());
    }
    i++;

    motG->setVelocity(0.0);  //Roue immobile
    motD->setVelocity(0.0);
  }

  delete robot;
  return 0;
}