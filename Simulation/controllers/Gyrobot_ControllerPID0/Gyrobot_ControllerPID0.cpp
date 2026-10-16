#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>
#include <webots/PositionSensor.hpp>
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

  // erreur  : écart à la consigne (ici : roll - roll_ref)
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

static double borne(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

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
  PositionSensor *codG = robot->getPositionSensor("CodG");
  PositionSensor *codD = robot->getPositionSensor("CodD");
  imu->enable(dt_ms);
  gyro->enable(dt_ms);
  codG->enable(dt_ms);
  codD->enable(dt_ms);

  // --------->>> PID D'ÉQUILIBRAGE (boucle interne, inchangée) <<<-------------
  const double KP = 40.0, KI = 400.0, KD = 0.0;
  const double VMAX = 82.6;   // vitesse max des moteurs en rad/s

  // --------->>> BOUCLE DE POSITION (boucle externe, à régler ensuite) <<<-----
  const double KPOS = 0.003;          // rad d'inclinaison demandée par rad de roue d'écart
  const double KVEL = 0.02;          // rad d'inclinaison demandée par rad/s de vitesse roue
  const double ROLL_REF_MAX = 0.04;  // inclinaison max demandée (~4.5°)
  const double SENS_POS = 1.0;       // passer à -1.0 si le robot s'éloigne au lieu de revenir

  PID pid(KP, KI, KD, VMAX);
  int i = 0;

  bool origine_init = false;
  double origine = 0.0, pos_prec = 0.0, vel_filtre = 0.0;

  while (robot->step(dt_ms) != -1) {
    const double t = robot->getTime();
    const double roll = imu->getRollPitchYaw()[0];   // mesure d'inclinaison
    const double gx = gyro->getValues()[0];          // vitesse de variation du roll

    // Position moyenne des deux roues (rad), l'origine = position au démarrage
    const double pos_brute = 0.5 * (codG->getValue() + codD->getValue());
    if (!origine_init) {
      origine = pos_brute;
      pos_prec = 0.0;
      vel_filtre = 0.0;
      origine_init = true;
    }
    const double pos = pos_brute - origine;
    const double vel_brute = (pos - pos_prec) / dt;
    vel_filtre = 0.9 * vel_filtre + 0.1 * vel_brute;   // filtre passe-bas simple
    pos_prec = pos;

    if (std::fabs(roll) > 0.6) {   // robot tombé : on coupe tout
      motG->setVelocity(0.0);
      motD->setVelocity(0.0);
      pid.reset();
      origine_init = false;        // nouvelle origine quand il sera redressé
      continue;
    }

    // Boucle externe : position/vitesse -> inclinaison désirée
    // Robot trop en avant (pos > 0) -> on demande de pencher vers l'arrière
    const double roll_ref = borne(-SENS_POS * (KPOS * pos + KVEL * vel_filtre),
                                  -ROLL_REF_MAX, ROLL_REF_MAX);

    // Boucle interne : le PID suit maintenant roll_ref au lieu de 0
    const double cmd = pid.update(roll - roll_ref, gx, dt);

    motG->setVelocity(cmd);   // même commande sur les deux roues
    motD->setVelocity(cmd);

    // t;roll;gyro;commande;position;roll_ref
    if (i % 4 == 0) printf("%.3f;%.4f;%.4f;%.2f;%.3f;%.4f\n", t, roll, gx, cmd, pos, roll_ref);
    i++;
  }

  delete robot;
  return 0;
}