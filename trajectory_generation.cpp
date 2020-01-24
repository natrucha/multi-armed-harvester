#include <Arduino.h>

// see algorithm in https://sam.ensam.eu/bitstream/handle/10985/11974/LSIS_CEP_2017_BEAREE.pdf?sequence=1&isAllowed=y

class Axis {
  public:
    float q0;    // initial location
    float qe;    // end location (position set point)
    float v0;    // adapted initial conditions for accel limited profile
    int s;       // direction (sign) of trajectory
    // kinematic bounds
    float v_max;
    float a_max;
    float d_max;
    // minimum stopping distance
    float Dq_stop;
    // minimum stopping velocity
    float Dq_v0;
    // min distance for which max velocity is reached
    float Dq_v_max;
    //
    float Dq;

    // calculate the sign of v0 for the trapezoidal profile function
    int8_t checkSign(float value) {
      int8_t sign = 1;
      if (value < 0) {
        sign = -1;
      }
      return sign;
    }

    void trapProfile() {
      ar = s * a_max;
      dr = s * d_max;
      vr = s * v_max;

      Ta = (vr - v0) / ar;
      Td = vr / dr;
      Tv = (Dq - s*Dq_v_max) / vr;
    }

    void triangProfile() {
      ar = s * a_max;
      dr = s * d_max;
      vr = s * sqrt( (2*a_max*d_max*Dq - d_max*sq(v0)) / (a_max + d_max) );

      Ta = (vr - v0) / ar;
      Td = vr / dr;
      Tv = 0;
    }

  public:
    // reached values of velocity and acceleration
    float vr;
    float ar;
    float dr;
    // duration time of each of the three stages
    float Tv;
    float Ta;
    float Td;

    // caluculate the trapezoidal profile
    void noJerkProfile(float q0, float qe, float v0) {
      float v0_new, q0_new;
      int8_t sign;

      Dq_stop = sq(v0) / (2 * d_max);
      Dq = qe - q0;

      // check if a full stop trajectory is needed
      if (abs(Dq) <= Dq_stop) {
        sign = checkSign(v0);
        v0_new = 0;
        q0_new = q0 + sign*Dq_stop;
      }
      // special case found when v_max is changed on the fly (shouldn't happen...)
      if (abs(v0) > v_max) {
        sign = checkSign(v0);
        v0_new = sign*v_max;
        Dq_v0 = sq(abs(v0) - v_max) / (2*d_max);
        q0_new = q0 + sign*Dq_v0;
      }

      Dq_v_max = (sq(v_max)-sq(v0)) / (2*a_max) + sq(v_max) / (2*d_max);
      s = checkSign(Dq);
      // check if the profile is trapezoidal or triangular
      if (abs(Dq) >= Dq_v_max) { // trapezoidal
        trapProfile();
      } else {                   // triangular
        triangProfile();
      }
    }


};


void setup() {
  Serial.begin(38400);
}

void loop() {

}
