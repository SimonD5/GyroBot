#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

using namespace webots;

//Classe du PID pour plus simple à utiliser
class PID {
 public:
  PID(double kp, double ki, double kd, double sortieMax)
      : kp_(kp), ki_(ki), kd_(kd), max_(sortieMax) {}

  // erreur  : écart à la consigne (ici : le roll, car on veut roll = 0)
  // derivee : dérivée de l'erreur (ici : la vitesse angulaire donnée par le gyro)
  double update(double erreur, double derivee, double dt) {
    // Terme P : réagit à l'erreur ACTUELLE
    const double terme_p = kp_ * erreur;

    // Terme I : cumule l'erreur au cours du temps (somme de erreur × dt)
    integrale_ += erreur * dt;
    if (ki_ > 0.0)  // anti-windup : borne l'intégrale pour qu'elle ne grossisse pas sans fin
      integrale_ = clamp(integrale_, -max_ / ki_, max_ / ki_);
    const double terme_i = ki_ * integrale_;

    // Terme D : réagit à la VITESSE de variation de l'erreur
    const double terme_d = kd_ * derivee;

    return clamp(terme_p + terme_i + terme_d, -max_, max_);
  }
  void reset() { integrale_ = 0.0; }

 private:
  static double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
  }
  double kp_, ki_, kd_, max_;
  double integrale_ = 0.0;
};





int main() {
  Robot *robot = new Robot();
  const int dt_ms = static_cast<int>(robot->getBasicTimeStep());
  const double dt = dt_ms / 1000.0;

  Motor *motG = robot->getMotor("MotG");
  Motor *motD = robot->getMotor("MotD");
  motG->setPosition(std::numeric_limits<double>::infinity());  // mode vitesse
  motD->setPosition(std::numeric_limits<double>::infinity());

  InertialUnit *imu = robot->getInertialUnit("imu");
  Gyro *gyro = robot->getGyro("gyro");
  imu->enable(dt_ms);
  gyro->enable(dt_ms);

  // --------->>> SEULE LIGNE À CHANGER D'UN ESSAI À L'AUTRE <<<-------------
  const double KP = 40.0, KI = 400.0, KD = 1.0;
  // --------->>> SEULE LIGNE À CHANGER D'UN ESSAI À L'AUTRE <<<-------------
  const double VMAX = 82.6;   // vitesse max des moteurs en rad/s

  PID pid(KP, KI, KD, VMAX);
  int i = 0;

  while (robot->step(dt_ms) != -1) {
    const double t = robot->getTime();
    const double roll = imu->getRollPitchYaw()[0];   // mesure d'inclinaison
    const double gx = gyro->getValues()[0];          // vitesse de variation du roll

    if (std::fabs(roll) > 0.6) {   // robot tombé : on coupe tout
      motG->setVelocity(0.0);
      motD->setVelocity(0.0);
      pid.reset();
      continue;
    }

    // Consigne = 0 (robot droit) donc erreur = roll
    const double cmd = pid.update(roll, gx, dt);

    motG->setVelocity(cmd);   // même commande sur les deux roues
    motD->setVelocity(cmd);

    if (i % 4 == 0) printf("%.3f;%.4f;%.4f;%.2f\n", t, roll, gx, cmd);   // t;roll;gyro;commande
    i++;
  }

  delete robot;
  return 0;
}