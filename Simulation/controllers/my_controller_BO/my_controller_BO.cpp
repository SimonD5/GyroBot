#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/InertialUnit.hpp>
#include <cmath>
#include <cstdio>
#include <limits>

using namespace webots;

int main() {
  Robot *robot = new Robot();
  const int dt_ms = static_cast<int>(robot->getBasicTimeStep());

  Motor *motG = robot->getMotor("MotG");
  Motor *motD = robot->getMotor("MotD");
  motG->setPosition(std::numeric_limits<double>::infinity());
  motD->setPosition(std::numeric_limits<double>::infinity());

  InertialUnit *imu = robot->getInertialUnit("imu");  //Init
  imu->enable(dt_ms);

  const double U_BO = 1;     // vitesse imposée aux roues (rad/s)
  const double LAMBDA = 6.2;   // taux de chute prévu par le modèle (1/s)

  double theta0 = 0.0, t0 = 0.0;
  bool init = false;
  int i = 0;

  while (robot->step(dt_ms) != -1) {
    const double t = robot->getTime();                   // temps simulé (s)
    const double roll = imu->getRollPitchYaw()[0];       // mesure de l'axe X

    if (!init) { theta0 = roll; t0 = t; init = true; }   // angle de départ

    // BOUCLE OUVERTE : la commande ne dépend PAS de la mesure
    motG->setVelocity(U_BO);
    motD->setVelocity(U_BO);

    if (i % 4 == 0) {   // t ; roll mesuré ; roll prévu par le modèle
      printf("%.3f;%.4f;%.4f\n", t, roll, theta0 * std::cosh(LAMBDA * (t - t0)));
    }
    i++;
  }

  delete robot;
  return 0;
}